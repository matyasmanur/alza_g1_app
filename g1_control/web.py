"""Local FastAPI console with one motion loop and one session per browser socket."""

import argparse
import asyncio
import json
import os
import subprocess
import sys
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from time import monotonic
from uuid import uuid4

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from g1_control.controller import Motion, MotionController, Rejected
from g1_control.gestures import GESTURES
from g1_control.joints import JOINT_LIMITS, JOINT_NAMES, PARKED, TARGET_MARGIN
from g1_control.latency import LatencyTrace
from g1_control.protocol import COMMAND_PERIOD
from g1_control.trajectory import PlanRequest, UnsafePath

WEB_DIRECTORY = Path(__file__).resolve().parent / "static"
ASSET_DIRECTORY = Path("/opt/web-assets")
STATE_PERIOD = 0.04
LOCAL_HOSTS = {f"{host}:{port}" for host in ("localhost", "127.0.0.1") for port in (8000, 8001)}


async def run_controller(controller: MotionController) -> None:
    """Run the sole writer independently of browser traffic."""
    loop = asyncio.get_running_loop()
    deadline = loop.time()
    while True:
        controller.tick()
        deadline = max(deadline + COMMAND_PERIOD, loop.time())
        await asyncio.sleep(max(0, deadline - loop.time()))


def state_for(controller: MotionController, session: str) -> dict:
    """Report ownership from this socket's perspective and preserve measured pose."""
    state = controller.status
    return {
        "type": "state", "connection": state.connection, "owned": state.owner == session,
        "control_taken": state.owner is not None, "enabled": state.enabled, "motion": state.motion.value,
        "stop_latched": state.stop_latched, "fault": state.fault, "target": state.target, "gesture": state.gesture,
        "pose": None if state.measured is None else state.measured.lower_position + state.measured.position,
    }


async def receive_command(socket: WebSocket) -> tuple[str, float]:
    """Capture receipt before queuing or validating the command."""
    raw = await socket.receive_text()
    return raw, monotonic()


def handle_command(controller: MotionController, session: str, message: tuple[str, float],
                   plans: dict[asyncio.Task, tuple[int, PlanRequest]], trace: LatencyTrace | None) -> dict | None:
    """Apply one browser command and acknowledge it. Successful heartbeats are silent."""
    command_id = None
    try:
        raw, received_at = message
        command = json.loads(raw)
        if not isinstance(command, dict) or not isinstance(command.get("type"), str):
            raise Rejected("Invalid command")
        action = command["type"]
        command_id = command.get("id")
        if action == "heartbeat":
            controller.heartbeat(session)
            return None
        if type(command_id) is not int:
            raise Rejected("Invalid command ID")
        if action in ("gesture", "ready", "release_control"):
            if action == "gesture":
                gesture_id = command.get("gesture")
                if not isinstance(gesture_id, str) or gesture_id not in GESTURES:
                    raise Rejected("Unknown gesture")
                gesture = GESTURES[gesture_id]
                request = controller.prepare_gesture(session, gesture.name, gesture.frames)
            elif action == "ready":
                request = controller.prepare_gesture(session, "Ready pose", ())
            else:
                request = controller.release(session)
            if trace is not None:
                trace.cancel(session)
            if request is not None:
                task = asyncio.create_task(asyncio.to_thread(controller.plan_gesture, request))
                plans[task] = command_id, request
                return None
        elif action == "set_joint":
            if not isinstance(command.get("joint"), str):
                raise Rejected("Missing arm joint")
            joint = command["joint"]
            angle = command.get("angle")
            before = controller.status if trace is not None else None
            controller.set_joint(session, joint, angle)
            if trace is not None:
                if before.motion is Motion.IDLE and before.measured is not None:
                    trace.start(session, command_id, JOINT_NAMES.index(joint), angle, before.measured, received_at)
                else:
                    trace.cancel(session)
        else:
            actions = {
                "take_control": controller.take_control, "enable": controller.enable,
                "stop": controller.stop, "reset_stop": controller.reset_stop,
            }
            if action not in actions:
                raise Rejected("Unknown command")
            actions[action](session)
            if trace is not None and action == "stop":
                trace.cancel(session)
    except (ValueError, TypeError, RecursionError) as error:
        return {"type": "ack", "id": command_id if type(command_id) is int else None,
                "ok": False, "message": str(error) if isinstance(error, Rejected) else "Invalid command"}
    return {"type": "ack", "id": command_id, "ok": True}


def finish_gesture(controller: MotionController, session: str, task: asyncio.Task,
                   command_id: int, request: PlanRequest) -> dict:
    """Commit a completed plan on the controller loop and return command feedback."""
    try:
        try:
            segments = task.result()
        except UnsafePath as error:
            controller.finish_plan(session, request, ())
            raise Rejected(str(error)) from error
        controller.finish_plan(session, request, segments)
    except Rejected as error:
        return {"type": "ack", "id": command_id, "ok": False, "message": str(error)}
    return {"type": "ack", "id": command_id, "ok": True}


def create_app(backend: str) -> FastAPI:
    """Build the same local console for the mock or the isolated DDS simulator."""
    trace = LatencyTrace() if backend == "sim" and os.environ.get("G1_MEASURE_LATENCY") == "1" else None

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        simulator = worker = None
        clearance = None
        try:
            if backend == "sim":
                from g1_control.clearance import MujocoClearance
                from g1_control.dds_transport import DdsTransport

                clearance = MujocoClearance()
                simulator = subprocess.Popen([sys.executable, "-m", "g1_control.simulator"])
                transport = DdsTransport(trace)
            else:
                from g1_control.mock import MockTransport

                transport = MockTransport(position=PARKED)
            app.state.controller = MotionController(transport, clearance=clearance)
            worker = asyncio.create_task(run_controller(app.state.controller))
            yield
        finally:
            if worker is not None:
                worker.cancel()
                with suppress(asyncio.CancelledError):
                    await worker
            if simulator is not None and simulator.poll() is None:
                simulator.terminate()
                simulator.wait(timeout=5)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.mount("/static", StaticFiles(directory=WEB_DIRECTORY), name="static")
    app.mount("/assets", StaticFiles(directory=ASSET_DIRECTORY), name="assets")

    @app.get("/")
    async def index():
        return FileResponse(WEB_DIRECTORY / "index.html", headers={"Cache-Control": "no-cache"})

    @app.get("/config")
    async def config():
        return {
            "backend": backend,
            "joints": [{"name": name, "min": low + TARGET_MARGIN, "max": high - TARGET_MARGIN}
                       for name, (low, high) in JOINT_LIMITS.items()],
            "gestures": [{"id": slug, "name": gesture.name, "sdk_id": gesture.sdk_id}
                         for slug, gesture in GESTURES.items()],
        }

    @app.websocket("/control")
    async def control(socket: WebSocket):
        host = socket.headers.get("host")
        if host not in LOCAL_HOSTS or socket.headers.get("origin") != f"http://{host}":
            await socket.close(code=1008)
            return
        session = uuid4().hex
        controller = app.state.controller
        await socket.accept()
        incoming = asyncio.create_task(receive_command(socket))
        plans = {}
        loop = asyncio.get_running_loop()
        next_state = loop.time()
        try:
            while True:
                if loop.time() >= next_state:
                    await socket.send_json(state_for(controller, session))
                    if trace is not None and (timing := trace.take(session)) is not None:
                        await socket.send_json(timing)
                    next_state = loop.time() + STATE_PERIOD
                ready, _ = await asyncio.wait({incoming, *plans}, timeout=max(0, next_state - loop.time()),
                                              return_when=asyncio.FIRST_COMPLETED)
                if incoming in ready:
                    ready.remove(incoming)
                    acknowledgment = handle_command(controller, session, incoming.result(), plans, trace)
                    incoming = asyncio.create_task(receive_command(socket))
                    if acknowledgment is not None:
                        await socket.send_json(acknowledgment)
                for task in ready:
                    command_id, request = plans.pop(task)
                    await socket.send_json(finish_gesture(controller, session, task, command_id, request))
        except (WebSocketDisconnect, OSError):
            pass
        finally:
            controller.disconnect(session)
            if trace is not None:
                trace.cancel(session)
            for task in plans:
                task.cancel()
            await asyncio.gather(*plans, return_exceptions=True)
            incoming.cancel()
            with suppress(asyncio.CancelledError, WebSocketDisconnect, OSError):
                await incoming

    return app


def main() -> None:
    """Serve one controller process in its Docker container."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mock", "sim"), required=True)
    uvicorn.run(create_app(parser.parse_args().backend), host="0.0.0.0", port=8000, ws_max_size=4096)


if __name__ == "__main__":
    main()

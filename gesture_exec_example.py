"""Play a local gesture over Unitree DDS in the running web-sim container."""

import argparse
from time import monotonic, sleep

from g1_control.gestures import GESTURES
from g1_control.joints import PARKED
from g1_control.protocol import COMMAND_PERIOD, COMMAND_TIMEOUT
from g1_control.trajectory import PlanRequest, UnsafePath, plan
    
from g1_control.clearance import MujocoClearance
from g1_control.dds_transport import DdsTransport

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gesture", nargs="?", default="wave", choices=GESTURES)
    args = parser.parse_args()



    gesture = GESTURES[args.gesture]
    clearance = MujocoClearance()
    transport = DdsTransport()
    deadline = monotonic() + 5
    while (initial := transport.read()) is None:
        if monotonic() >= deadline:
            raise SystemExit("No G1 feedback. Run this inside make shell with web-sim running.")
        sleep(COMMAND_PERIOD)

    try:
        segments = plan(
            PlanRequest(gesture.name, initial, initial.position, gesture.frames, PARKED),
            clearance,
        )
    except UnsafePath as error:
        raise SystemExit(str(error)) from error

    def fresh_feedback():
        feedback = transport.read()
        if feedback is None or monotonic() - feedback.received_at >= COMMAND_TIMEOUT:
            raise SystemExit("Lost simulator feedback.")
        return feedback

    before = initial.lower_position + initial.position
    feedback = fresh_feedback()
    after = feedback.lower_position + feedback.position
    if max(abs(a - b) for a, b in zip(before, after)) > 0.035:
        raise SystemExit("The pose changed during planning. Let the arms settle and try again.")

    print(f"Playing local gesture: {gesture.name}", flush=True)
    try:
        for segment in segments:
            started = last_write = monotonic()
            while True:
                fresh_feedback()
                now = monotonic()
                if now - last_write >= COMMAND_TIMEOUT:
                    raise SystemExit("Missed the simulator command deadline.")
                elapsed = now - started
                position, velocity = segment.sample(elapsed)
                transport.write(position, velocity, enabled=True)
                last_write = now
                if elapsed >= segment.seconds:
                    break
                sleep(COMMAND_PERIOD)
    finally:
        feedback = transport.read()
        if feedback is not None:
            transport.write(feedback.position, (0.0,) * len(feedback.position), enabled=False)


if __name__ == "__main__":
    main()

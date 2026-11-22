"""Check the read-only hardware contract with the SDK installed."""

import json
from time import monotonic
from unittest.mock import patch

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC

from g1_control.controller import MotionController
from g1_control.dds_transport import DdsObservation
from g1_control.protocol import ARM_START, MODE_MACHINE
from g1_control.web import handle_command, state_for


def main() -> None:
    with (
        patch("g1_control.dds_transport.ChannelFactoryInitialize") as initialize,
        patch("g1_control.dds_transport.ChannelSubscriber") as subscriber,
        patch("g1_control.dds_transport.ChannelPublisher") as publisher,
    ):
        transport = DdsObservation(0, "enp2s0")
        initialize.assert_called_once_with(0, "enp2s0")
        publisher.assert_not_called()
        receive, _ = subscriber.return_value.Init.call_args.args

        message = unitree_hg_msg_dds__LowState_()
        message.mode_machine = (MODE_MACHINE + 1) % 256
        message.tick = 1
        message.motor_state[ARM_START].q = 0.25
        message.crc = CRC().Crc(message)
        receive(message)

        controller = MotionController(transport)
        controller.tick()
        state = state_for(controller, "browser")
        assert state["connection"] == "connected"
        assert len(state["pose"]) == 29 and state["pose"][ARM_START] == 0.25

        for action in ("take_control", "enable", "set_joint", "gesture", "ready", "release_control", "stop", "reset_stop"):
            command = json.dumps({"type": action, "id": 1}), monotonic()
            acknowledgment = handle_command(controller, "browser", command, {}, None, [], read_only=True)
            assert acknowledgment == {
                "type": "ack", "id": 1, "ok": False, "message": "Hardware observation is read-only",
            }
        assert not controller.status.enabled and controller.status.owner is None
        try:
            transport.write((0.0,) * 14, (0.0,) * 14, enabled=True)
        except OSError:
            pass
        else:
            raise AssertionError("The hardware transport accepted an arm write")
        publisher.assert_not_called()
    print("Hardware mode accepts G1 feedback and cannot publish arm commands.")


if __name__ == "__main__":
    main()

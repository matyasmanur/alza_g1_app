"""Move the local MuJoCo G1 through Unitree SDK2, independently of the web app."""

from threading import Event
from time import monotonic, sleep

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


ARMS = slice(15, 29)
ARM_WEIGHT = 29
LEFT_SHOULDER_PITCH = 0  # Offsets within the arm slice.
LEFT_ELBOW = 3
LEFT_WRIST_ROLL = 4
PERIOD = 0.02


def main():
    ChannelFactoryInitialize(1, "lo")
    crc = CRC()
    ready = Event()
    initial = None

    def receive_state(message):
        nonlocal initial
        if not ready.is_set() and message.mode_machine == 5 and message.crc == crc.Crc(message):
            initial = tuple(motor.q for motor in message.motor_state[ARMS])
            ready.set()

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(receive_state, 1)
    publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    publisher.Init()
    if not ready.wait(timeout=5):
        raise SystemExit("No G1 feedback on local DDS. Start with make up web-sim.")

    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_machine = 5
    command.motor_cmd[ARM_WEIGHT].q = 1.0
    motors = command.motor_cmd[ARMS]
    for index, motor in enumerate(motors):
        motor.kp, motor.kd = (8.0, 1.0) if index % 7 >= 4 else (45.0, 4.0)

    def publish():
        command.crc = crc.Crc(command)
        if not publisher.Write(command):
            raise RuntimeError("Failed to publish the arm command")

    current = initial

    def move(target, duration):
        nonlocal current
        started = monotonic()
        while True:
            progress = min(1.0, (monotonic() - started) / duration)
            blend = progress**2 * (3 - 2 * progress)
            rate = 6 * progress * (1 - progress) / duration
            for motor, start, end in zip(motors, current, target):
                motor.q = start + (end - start) * blend
                motor.dq = (end - start) * rate
            publish()
            if progress == 1.0:
                break
            sleep(PERIOD)
        current = tuple(target)

    try:
        print("Manual move: bend the left elbow", flush=True)
        elbow = list(initial)
        elbow[LEFT_ELBOW] += 0.25
        move(elbow, 1.2)
        move(elbow, 0.5)
        move(initial, 1.2)

        print("Gesture: raise the left arm and wave", flush=True)
        raised = list(initial)
        raised[LEFT_SHOULDER_PITCH] -= 0.65
        move(raised, 1.8)
        for offset in (0.35, -0.35, 0.35, -0.35, 0.0):
            wave = raised.copy()
            wave[LEFT_WRIST_ROLL] += offset
            move(wave, 0.8)
        move(initial, 1.8)
        move(initial, 0.5)
    finally:
        command.motor_cmd[ARM_WEIGHT].q = 0.0
        publish()


if __name__ == "__main__":
    main()

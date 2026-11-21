"""Deterministic controller checks with a mock and manually advanced clock."""

import unittest

from g1_control.controller import Motion, MotionController, Rejected
from g1_control.joints import JOINT_LIMITS, JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED, TARGET_MARGIN
from g1_control.mock import MockTransport
from g1_control.protocol import COMMAND_PERIOD, COMMAND_TIMEOUT

ELBOW = JOINT_NAMES.index("left_elbow")


class Clock:
    """Keep tests independent of wall-clock sleeps and Docker scheduling."""

    now = 0.0

    def __call__(self):
        return self.now


class RecordingMock(MockTransport):
    """Record controller writes and allow one explicit transport failure."""

    def __init__(self, clock):
        super().__init__(clock)
        self.writes = []
        self.fail_writes = False

    def write(self, position, velocity, *, enabled):
        if self.fail_writes:
            raise OSError("simulated write failure")
        super().write(position, velocity, enabled=enabled)
        self.writes.append((position, velocity, enabled))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.mock = RecordingMock(self.clock)
        self.controller = MotionController(self.mock, self.clock)

    def advance(self, count=1, *, keepalive=True):
        for _ in range(count):
            if keepalive and (owner := self.controller.status.owner):
                self.controller.heartbeat(owner)
            self.clock.now += COMMAND_PERIOD
            self.controller.tick()

    def enable(self):
        self.advance()
        self.controller.take_control("operator")
        self.controller.enable("operator")

    def pose(self, elbow):
        return tuple(elbow if i == ELBOW else 0.0 for i in range(len(JOINT_NAMES)))

    def test_observe_claim_enable_and_exclusive_writes(self):
        self.assertEqual(self.controller.status.connection, "waiting")
        with self.assertRaises(Rejected):
            self.controller.take_control("operator")
        self.advance()
        self.assertEqual(self.controller.status.connection, "connected")
        self.controller.take_control("operator")
        for action in (self.controller.take_control, self.controller.enable, self.controller.stop):
            with self.assertRaises(Rejected):
                action("observer")
        with self.assertRaises(Rejected):
            self.controller.set_joint("operator", "left_elbow", 0.30)
        self.advance()
        self.assertEqual(self.mock.writes, [])
        self.controller.enable("operator")
        self.advance()
        self.assertEqual(len(self.mock.writes), 1)
        self.assertTrue(self.mock.writes[-1][2])

    def test_reject_invalid_targets_and_keep_latest(self):
        self.enable()
        for value in (float("nan"), float("inf"), -float("inf"), True, "0.2", 10**1000, -2.0, 2.09):
            with self.subTest(value=value), self.assertRaises(Rejected):
                self.controller.set_joint("operator", "left_elbow", value)
        with self.assertRaises(Rejected):
            self.controller.set_joint("operator", "waist_yaw", 0.0)
        self.controller.set_joint("operator", "left_elbow", 0.40)
        self.controller.set_joint("operator", "left_elbow", -0.20)
        self.advance()
        self.assertLess(self.mock.writes[-1][0][ELBOW], 0.0)
        self.assertEqual(self.controller.status.target[ELBOW], -0.20)

    def test_angle_speed_and_acceleration_limits_under_retargeting(self):
        self.enable()
        low, high = JOINT_LIMITS["left_elbow"]
        for target, ticks in ((high - TARGET_MARGIN, 154), (low + TARGET_MARGIN, 313),
                              (high - TARGET_MARGIN, 380)):
            self.controller.set_joint("operator", "left_elbow", target)
            self.advance(ticks)
        previous_position = previous_speed = 0.0
        for position, velocity, _ in self.mock.writes:
            angle = position[ELBOW]
            speed = velocity[ELBOW]
            self.assertGreaterEqual(angle, low)
            self.assertLessEqual(angle, high)
            self.assertLessEqual(abs(speed), MAX_SPEED + 1e-9)
            self.assertLessEqual(abs(angle - previous_position), MAX_SPEED * COMMAND_PERIOD + 1e-9)
            self.assertLessEqual(abs(speed - previous_speed), MAX_ACCELERATION * COMMAND_PERIOD + 1e-9)
            previous_position, previous_speed = angle, speed
        self.assertIsNone(self.controller.status.fault)
        self.assertAlmostEqual(previous_position, high - TARGET_MARGIN, places=3)

    def test_gesture_exclusion_and_completion(self):
        self.enable()
        self.controller.set_joint("operator", "left_elbow", 0.25)
        gesture = (self.pose(0.15), self.pose(0.0))
        with self.assertRaises(Rejected):
            self.controller.start_gesture("operator", gesture)
        self.advance(80)
        with self.assertRaises(Rejected):
            self.controller.start_gesture("operator", (self.pose(0.1), self.pose(99)))
        self.assertEqual(self.controller.status.motion, Motion.IDLE)
        self.controller.start_gesture("operator", gesture)
        with self.assertRaises(Rejected):
            self.controller.set_joint("operator", "left_elbow", 0.5)
        with self.assertRaises(Rejected):
            self.controller.start_gesture("operator", gesture)
        self.advance(100)
        self.assertEqual(self.controller.status.motion, Motion.IDLE)
        self.assertAlmostEqual(self.controller.status.measured.position[ELBOW], 0.0, places=2)

    def test_stop_latches_and_discards_gesture_without_return(self):
        self.enable()
        self.controller.start_gesture("operator", (self.pose(0.8), self.pose(0.0)))
        self.advance(30)
        self.controller.stop("operator")
        self.assertEqual(self.controller.status.motion, Motion.STOPPING)
        self.assertTrue(self.controller.status.stop_latched)
        self.assertIsNone(self.controller.status.target)
        with self.assertRaises(Rejected):
            self.controller.reset_stop("operator")
        self.advance(20)
        self.assertEqual(self.controller.status.motion, Motion.IDLE)
        self.assertFalse(self.mock.writes[-1][2])
        self.assertGreater(self.controller.status.measured.position[ELBOW], 0.05)
        self.assertLess(self.controller.status.measured.position[ELBOW], 0.50)
        with self.assertRaises(Rejected):
            self.controller.enable("operator")
        self.controller.reset_stop("operator")
        self.assertFalse(self.controller.status.enabled)
        self.controller.enable("operator")
        self.assertEqual(self.controller.status.target, self.controller.status.measured.position)

    def test_stale_feedback_disarms_preserves_snapshot_and_needs_new_claim(self):
        self.enable()
        self.controller.set_joint("operator", "left_elbow", 0.4)
        self.advance(10)
        sample = self.controller.status.measured
        self.mock.feedback_enabled = False
        self.advance(15)
        writes = len(self.mock.writes)
        self.assertEqual(self.controller.status.connection, "stale")
        self.assertIs(self.controller.status.measured, sample)
        self.assertIsNone(self.controller.status.owner)
        self.assertFalse(self.controller.status.enabled)
        self.assertIsNone(self.controller.status.target)
        self.advance(20)
        self.assertEqual(len(self.mock.writes), writes)
        self.assertFalse(self.mock.authority)
        self.mock.feedback_enabled = True
        self.advance()
        self.assertEqual(self.controller.status.connection, "connected")
        with self.assertRaises(Rejected):
            self.controller.enable("operator")
        self.controller.take_control("operator")
        self.controller.enable("operator")
        self.assertEqual(self.controller.status.target, self.controller.status.measured.position)

    def test_closed_session_cancels_motion_and_observer_can_claim_after_hold(self):
        self.enable()
        self.controller.set_joint("operator", "left_elbow", 0.8)
        self.advance(15)
        self.controller.disconnect("observer")
        self.assertEqual(self.controller.status.owner, "operator")
        self.controller.disconnect("operator")
        with self.assertRaises(Rejected):
            self.controller.take_control("observer")
        self.advance(15)
        self.controller.take_control("observer")
        self.assertFalse(self.controller.status.enabled)
        self.assertIsNone(self.controller.status.target)

    def test_expired_session_revokes_control(self):
        self.enable()
        self.controller.set_joint("operator", "left_elbow", 0.8)
        self.advance(60, keepalive=False)
        self.assertIsNone(self.controller.status.owner)
        self.assertFalse(self.controller.status.enabled)
        self.assertFalse(self.mock.authority)
        with self.assertRaises(Rejected):
            self.controller.heartbeat("operator")

    def test_transport_failure_revokes_control(self):
        self.enable()
        self.mock.fail_writes = True
        self.advance()
        self.assertEqual(self.controller.status.fault, "simulated write failure")
        self.assertIsNone(self.controller.status.owner)
        self.assertFalse(self.controller.status.enabled)

    def test_loop_stall_does_not_resume_old_motion(self):
        self.enable()
        self.controller.set_joint("operator", "left_elbow", 0.8)
        self.advance(10)
        writes = len(self.mock.writes)
        self.clock.now += COMMAND_TIMEOUT + COMMAND_PERIOD
        self.controller.tick()
        self.assertEqual(self.controller.status.connection, "connected")
        self.assertIsNone(self.controller.status.owner)
        self.assertIsNone(self.controller.status.target)
        self.assertFalse(self.controller.status.enabled)
        self.assertFalse(self.mock.authority)
        self.assertEqual(len(self.mock.writes), writes)

    def test_stop_latch_survives_operator_disconnect(self):
        self.enable()
        self.controller.stop("operator")
        self.controller.disconnect("operator")
        self.advance(2)
        self.controller.take_control("replacement")
        with self.assertRaises(Rejected):
            self.controller.enable("replacement")
        self.controller.reset_stop("replacement")
        self.assertFalse(self.controller.status.enabled)

    def test_late_feedback_cannot_hide_a_stale_interval(self):
        self.enable()
        self.mock.feedback_enabled = False
        self.advance(12)
        self.assertTrue(self.controller.status.enabled)
        self.mock.feedback_enabled = True
        self.advance()
        self.assertEqual(self.controller.status.connection, "connected")
        self.assertIsNone(self.controller.status.owner)
        self.assertFalse(self.controller.status.enabled)

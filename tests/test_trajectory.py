"""Check path selection, bounds, and cancellation without simulator dependencies."""

import unittest

from g1_control.controller import Motion, MotionController, Rejected
from g1_control.joints import JOINT_NAMES, MAX_ACCELERATION, MAX_SPEED
from g1_control.mock import MockTransport
from g1_control.trajectory import READY, Issue, Keyframe, PlanRequest, Segment, path_issue, plan
from g1_control.transport import Feedback

LEFT = JOINT_NAMES.index("left_elbow")
RIGHT = JOINT_NAMES.index("right_elbow")


def elbows(left, right):
    """Create a fixture pose with both elbow angles specified."""
    return tuple(left if i == LEFT else right if i == RIGHT else 0 for i in range(len(READY)))


class TrajectoryTests(unittest.TestCase):
    def test_segment_respects_velocity_and_acceleration(self):
        segment = Segment.between(READY, Keyframe(elbows(1.25, -0.91), 0.25))
        dt = segment.seconds / 1000
        previous = segment.sample(0)[1]
        for tick in range(1, 1001):
            _, velocity = segment.sample(tick * dt)
            self.assertLessEqual(max(map(abs, velocity)), MAX_SPEED + 1e-10)
            self.assertLessEqual(max(abs(a - b) / dt for a, b in zip(previous, velocity)), MAX_ACCELERATION)
            previous = velocity
        self.assertEqual(segment.sample(segment.seconds)[0], segment.end)

    def test_fallback_through_ready(self):
        start = elbows(0.4, 0)
        frame = Keyframe(elbows(0, 0.4), 0.25)
        request = PlanRequest("Test", Feedback(0, start, READY, ()), start, (frame,))

        def clearance(pose, lower):
            return "Elbows interfere" if pose[LEFT] > 0.12 and pose[RIGHT] > 0.12 else None

        result = plan(request, clearance)
        self.assertEqual(len(result), 3)
        self.assertEqual(result[0].end, READY)
        self.assertEqual(result[1].end, frame.pose)

    def test_path_may_leave_the_margin_but_not_enter_or_approach(self):
        def clearance(pose, lower):
            """An obstacle at elbow 0.5 rad with a 0.012 m margin, scaled at 0.1 m per radian."""
            depth = 0.012 - abs(pose[LEFT] - 0.5) * 0.1
            return Issue("Too close", depth) if depth > 0 else None

        inside = elbows(0.45, 0)
        self.assertIsNone(path_issue(clearance, inside, READY, ()))
        self.assertIsNone(path_issue(clearance, elbows(0.5, 0), READY, ()))
        self.assertEqual(path_issue(clearance, inside, elbows(0.42, 0), ()), "Too close")
        self.assertEqual(path_issue(clearance, inside, elbows(0.9, 0), ()), "Too close")
        self.assertEqual(path_issue(clearance, READY, inside, ()), "Too close")

    def test_fallback_joint_by_joint(self):
        start = elbows(0.4, 0.4)
        request = PlanRequest("Test", Feedback(0, start, READY, ()), start, ())

        def clearance(pose, lower):
            return "Elbows interfere" if 0.1 < pose[LEFT] < 0.3 and 0.1 < pose[RIGHT] < 0.3 else None

        result = plan(request, clearance)
        self.assertEqual(result[0].end, elbows(0, 0.4))
        self.assertEqual(result[-1].end, READY)

    def test_manual_and_gesture_rejection_keep_previous_target(self):
        mock = MockTransport()
        controller = MotionController(mock, clearance=lambda pose, lower: "Blocked" if pose[LEFT] > 0.2 else None)
        controller.tick()
        controller.take_control("operator")
        controller.enable("operator")
        with self.assertRaisesRegex(Rejected, "Blocked"):
            controller.set_joint("operator", "left_elbow", 0.5)
        with self.assertRaisesRegex(Rejected, "No clear route"):
            controller.start_gesture("operator", (elbows(0.5, 0),))
        self.assertEqual(controller.status.motion, Motion.IDLE)
        self.assertEqual(controller.status.target, READY)
        self.assertEqual(mock.target, READY)

    def test_stop_invalidates_a_pending_plan(self):
        controller = MotionController(MockTransport())
        controller.tick()
        controller.take_control("operator")
        controller.enable("operator")
        request = controller.prepare_gesture("operator", "Test", (Keyframe(elbows(0.5, 0), 0.25),))
        with self.assertRaises(Rejected):
            controller.set_joint("operator", "left_elbow", 0.1)
        segments = controller.plan_gesture(request)
        controller.stop("operator")
        with self.assertRaisesRegex(Rejected, "cancelled"):
            controller.finish_plan("operator", request, segments)
        self.assertTrue(controller.status.stop_latched)
        self.assertIsNone(controller.status.target)

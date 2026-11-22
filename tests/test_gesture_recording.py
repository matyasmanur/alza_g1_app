"""Check that measured poses can be saved and loaded as ordinary gestures."""

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from g1_control.controller import MotionController, Rejected
from g1_control.gestures import load_gestures, save_recording
from g1_control.mock import MockTransport


class GestureRecordingTests(unittest.TestCase):
    def test_capture_requires_enabled_operator_and_saves_measured_pose(self) -> None:
        angles = (0.25,) + (0.0,) * 13
        controller = MotionController(MockTransport(position=angles))
        controller.tick()
        controller.take_control("operator")
        with self.assertRaises(Rejected):
            controller.capture_pose("operator")
        controller.enable("operator")
        with self.assertRaises(Rejected):
            controller.capture_pose("observer")
        first_pose = controller.capture_pose("operator")
        self.assertEqual(first_pose, angles)

        with TemporaryDirectory() as directory, patch("g1_control.gestures.RECORDINGS_DIR", Path(directory)):
            slug, gesture = save_recording("Elbow test", [first_pose, (0.0,) * 14])
            paths = list(Path(directory).glob("*.json"))
            self.assertEqual(len(paths), 1)
            self.assertEqual(json.loads(paths[0].read_text())["poses"][0], list(angles))
            catalog = load_gestures()
            self.assertEqual(list(catalog), [slug, "alza"])
            self.assertEqual(catalog[slug], gesture)

    def test_invalid_recordings_do_not_enter_the_catalog(self) -> None:
        with TemporaryDirectory() as directory, patch("g1_control.gestures.RECORDINGS_DIR", Path(directory)):
            invalid = [100.0] + [0.0] * 13
            with self.assertRaises(ValueError):
                save_recording("Invalid", [tuple(invalid)])
            self.assertFalse(list(Path(directory).iterdir()))
            (Path(directory) / "invalid.json").write_text(json.dumps({"name": "Invalid", "poses": [invalid]}))
            with self.assertLogs("g1_control.gestures", level="WARNING"):
                self.assertNotIn("recorded-invalid", load_gestures())


if __name__ == "__main__":
    unittest.main()

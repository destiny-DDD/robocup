import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
import collect_dataset


class FakeCapture:
    def __init__(self):
        self.calls = []

    def set(self, property_id, value):
        self.calls.append((property_id, value))
        return True


class CameraConfigurationTest(unittest.TestCase):
    def test_manual_exposure_is_set_before_exposure_value(self):
        capture = FakeCapture()
        collect_dataset.configure_camera(capture, exposure=100, brightness=None)
        self.assertEqual(
            capture.calls,
            [(collect_dataset.cv2.CAP_PROP_AUTO_EXPOSURE, 1),
             (collect_dataset.cv2.CAP_PROP_EXPOSURE, 100)],
        )


if __name__ == "__main__":
    unittest.main()

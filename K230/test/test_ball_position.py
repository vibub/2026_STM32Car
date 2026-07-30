"""钢球位置坐标转换和可配置标定保持周期测试。"""

import pathlib
import sys
import unittest


K230_DIR = pathlib.Path(__file__).resolve().parents[1]
if str(K230_DIR) not in sys.path:
    sys.path.insert(0, str(K230_DIR))

from ball_position import BallPositionDetector, scale_detection_box  # noqa: E402


class BallPositionOptimizationTest(unittest.TestCase):
    def test_ai_detection_is_scaled_to_cv_coordinates(self):
        detection = (80, 30, 20, 10, 0.875)

        scaled = scale_detection_box(
            detection,
            source_size=(320, 180),
            target_size=(640, 360),
        )

        self.assertEqual(scaled, (160, 60, 40, 20, 0.875))

    def test_missing_detection_stays_missing(self):
        self.assertIsNone(
            scale_detection_box(None, (320, 180), (640, 360))
        )

    def test_invalid_source_size_is_rejected(self):
        with self.assertRaises(ValueError):
            scale_detection_box((0, 0, 1, 1, 1.0), (0, 180), (640, 360))

    def test_scale_hold_window_is_configurable(self):
        detector = BallPositionDetector(
            scale_refresh_frames=1,
            scale_hold_frames=12,
        )

        self.assertEqual(detector.scale_refresh_frames, 1)
        self.assertEqual(detector.scale_hold_frames, 12)


if __name__ == "__main__":
    unittest.main()

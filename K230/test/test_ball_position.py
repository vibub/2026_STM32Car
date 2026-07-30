"""钢球位置坐标转换和可配置标定保持周期测试。"""

import pathlib
import sys
import unittest


K230_DIR = pathlib.Path(__file__).resolve().parents[1]
if str(K230_DIR) not in sys.path:
    sys.path.insert(0, str(K230_DIR))

from ball_position import (  # noqa: E402
    BallPositionDetector,
    SCALE_THRESHOLDS,
    scale_detection_box,
    select_marker_calibration,
)

class FakeBlob:
    def __init__(self, rect, pixels=None):
        self._rect = rect
        self._pixels = rect[2] * rect[3] if pixels is None else pixels

    def rect(self):
        return self._rect

    def pixels(self):
        return self._pixels

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

    def test_marker_grid_prefers_spacing_over_bright_center_noise(self):
        blobs = [
            FakeBlob((100, 184, 4, 12), pixels=16),
            FakeBlob((250, 184, 4, 12), pixels=16),
            FakeBlob((398, 185, 4, 12), pixels=15),
            FakeBlob((547, 184, 4, 12), pixels=16),
            FakeBlob((317, 181, 8, 8), pixels=80),
        ]

        calibration = select_marker_calibration(blobs)

        self.assertIsNotNone(calibration)
        self.assertEqual(calibration.marker_count, 4)
        self.assertEqual([round(column.center_x) for column in calibration.marker_columns], [102, 252, 400, 549])

    def test_marker_grid_ignores_same_row_noise(self):
        blobs = [
            FakeBlob((42, 186, 5, 8), pixels=32),
            FakeBlob((100, 184, 4, 12), pixels=16),
            FakeBlob((250, 184, 4, 12), pixels=16),
            FakeBlob((398, 185, 4, 12), pixels=15),
            FakeBlob((547, 184, 4, 12), pixels=16),
        ]

        calibration = select_marker_calibration(blobs)

        self.assertIsNotNone(calibration)
        self.assertEqual(calibration.marker_count, 4)
        self.assertEqual([round(column.center_x) for column in calibration.marker_columns], [102, 252, 400, 549])
    def test_sloped_marker_line_rejects_black_ruler_tick(self):
        blobs = [
            FakeBlob((96, 174, 5, 12), pixels=18),
            FakeBlob((246, 183, 5, 12), pixels=18),
            FakeBlob((396, 192, 5, 12), pixels=18),
            FakeBlob((546, 201, 5, 12), pixels=18),
            FakeBlob((318, 151, 14, 2), pixels=28),
        ]

        calibration = select_marker_calibration(blobs)

        self.assertIsNotNone(calibration)
        self.assertEqual(calibration.marker_count, 4)
        self.assertEqual([round(column.center_x) for column in calibration.marker_columns], [98, 248, 398, 548])
    def test_three_visible_markers_project_missing_fourth(self):
        blobs = [
            FakeBlob((100, 184, 4, 12), pixels=16),
            FakeBlob((398, 185, 4, 12), pixels=15),
            FakeBlob((547, 184, 4, 12), pixels=16),
        ]

        calibration = select_marker_calibration(blobs)

        self.assertIsNotNone(calibration)
        self.assertEqual(calibration.marker_count, 4)
        self.assertEqual([round(column.center_x) for column in calibration.marker_columns], [102, 251, 400, 549])

    def test_marker_values_are_fixed_left_to_right(self):
        blobs = [
            FakeBlob((100, 184, 4, 12), pixels=16),
            FakeBlob((250, 184, 4, 12), pixels=16),
            FakeBlob((398, 185, 4, 12), pixels=15),
            FakeBlob((547, 184, 4, 12), pixels=16),
        ]

        calibration = select_marker_calibration(blobs)
        centers = [column.center_x for column in calibration.marker_columns]

        self.assertEqual(calibration.marker_values_mm, [50.0, 0.0, -50.0, -100.0])
        self.assertAlmostEqual(calibration.pixel_to_mm(centers[0]), 50.0)
        self.assertAlmostEqual(calibration.pixel_to_mm(centers[1]), 0.0)
        self.assertAlmostEqual(calibration.pixel_to_mm(centers[2]), -50.0)
        self.assertAlmostEqual(calibration.pixel_to_mm(centers[3]), -100.0)
    def test_scale_thresholds_include_pale_red_markers(self):
        self.assertIn((24, 100, 8, 127, -10, 127), SCALE_THRESHOLDS)
        self.assertNotIn((0, 100, 6, 127, -64, 127), SCALE_THRESHOLDS)

    def test_scale_hold_window_is_configurable(self):
        detector = BallPositionDetector(
            scale_refresh_frames=1,
            scale_hold_frames=12,
        )

        self.assertEqual(detector.scale_refresh_frames, 1)
        self.assertEqual(detector.scale_hold_frames, 12)


if __name__ == "__main__":
    unittest.main()

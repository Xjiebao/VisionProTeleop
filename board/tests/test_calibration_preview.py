"""无硬件检查标定取景、双眼判定及候选计数。"""
import base64
import json
from tempfile import TemporaryDirectory
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibration_preview as preview


def poses(x=0., angle=0.):
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(np.array([0., 0., np.radians(angle)]))[0]
    transform[:3, 3] = [x, 0., 1.]
    return [transform.copy(), transform.copy()]


class CounterTest(unittest.TestCase):
    def test_missing_board_never_counts(self):
        counter = preview.PoseCounter()
        for timestamp in range(10):
            seconds, _ = counter.update(timestamp, [None, None], timestamp)
            self.assertEqual(seconds, 0)
        self.assertEqual(len(counter.accepted), 0)

    def test_both_eyes_must_detect(self):
        counter = preview.PoseCounter()
        for timestamp in range(4):
            counter.update(timestamp, [poses()[0], None], timestamp)
        self.assertEqual(len(counter.accepted), 0)
        self.assertIsNone(counter.reference)

    def test_stable_pose_counts_once_and_new_pose_counts(self):
        counter = preview.PoseCounter()
        for timestamp in range(4):
            counter.update(timestamp, poses(), timestamp)
        self.assertEqual(len(counter.accepted), 1)
        counter.update(4., poses(x=.02), 4.)
        counter.update(5., poses(x=.02), 5.)
        self.assertEqual(len(counter.accepted), 2)

    def test_small_change_is_duplicate(self):
        counter = preview.PoseCounter()
        for timestamp, x in ((0., 0.), (1., 0.), (2., .006), (3., .006)):
            counter.update(timestamp, poses(x=x), timestamp)
        self.assertEqual(len(counter.accepted), 1)

    def test_rotation_creates_new_candidate(self):
        counter = preview.PoseCounter()
        for timestamp, angle in ((0., 0.), (1., 0.), (2., 4.), (3., 4.)):
            counter.update(timestamp, poses(angle=angle), timestamp)
        self.assertEqual(len(counter.accepted), 2)

    def test_movement_resets_stability(self):
        counter = preview.PoseCounter()
        counter.update(0., poses(), 0.)
        counter.update(.5, poses(), .5)
        seconds, _ = counter.update(1., poses(x=.004), 1.)
        self.assertEqual(seconds, 0)
        self.assertEqual(len(counter.accepted), 0)

    def test_tracking_loss_resets_stability(self):
        counter = preview.PoseCounter()
        counter.update(0., poses(), 0.)
        counter.update(.5, [None, None], .5)
        seconds, _ = counter.update(1., poses(), 1.)
        self.assertEqual(seconds, 0)
        self.assertEqual(len(counter.accepted), 0)

    def test_stale_frame_and_long_gap_never_finish_window(self):
        counter = preview.PoseCounter()
        counter.update(0., poses(), 0.)
        seconds, _ = counter.update(1., poses(), 4.)
        self.assertEqual(seconds, 0)
        seconds, _ = counter.update(4., poses(), 4.)
        self.assertEqual(seconds, 0)
        self.assertEqual(len(counter.accepted), 0)

    def test_same_frame_cannot_count_twice(self):
        counter = preview.PoseCounter()
        counter.update(0., poses(), 0.)
        counter.update(0., poses(), 1.)
        self.assertEqual(len(counter.accepted), 0)


class ImageTest(unittest.TestCase):
    def setUp(self):
        # Synthetic camera parameters keep these image tests independent of device calibration.
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        eye = {"fx": 1000, "fy": 1000, "cx": 960, "cy": 600, "dist": [0] * 5}
        intrinsics = {
            "image_size": [1920, 1200], "convert_meta": {"convention": "opencv_raw"},
            "recording_geometry": {"sbs_order": "imu_right_left", "rotation_deg": 0,
                                   "left_crop": [2080, 0, 1920, 1200], "right_crop": [160, 0, 1920, 1200]},
            "left_intrinsics": eye, "right_intrinsics": eye,
            "stereo_extrinsics": {"R": np.eye(3).tolist(), "T": [60, 0, 0]},
        }
        (root / "intrinsics.json").write_text(json.dumps(intrinsics))
        (root / "extrinsics.json").write_text(json.dumps({
            "board": {"inner_corners": [11, 8], "square_mm": 30}}))
        self.processor = preview.Preview(root / "intrinsics.json", root / "extrinsics.json")

    def test_no_chessboard_returns_new_image_without_counting(self):
        raw = np.full((1200, 4000, 3), 80, dtype=np.uint8)
        result = self.processor.process(raw, 1., now=1.)
        self.assertFalse(result["left_detected"])
        self.assertFalse(result["right_detected"])
        self.assertEqual(result["pose_count"], 0)
        self.assertIsNone(result["error"])
        image = cv2.imdecode(np.frombuffer(base64.b64decode(result["image_base64"]), np.uint8), 1)
        self.assertEqual(image.shape, (300, 960, 3))

    def test_split_order_orientation_and_scaled_intrinsics(self):
        raw = np.zeros((1200, 4000, 3), dtype=np.uint8)
        raw[:600, 2080:] = 40
        raw[600:, 2080:] = 80
        raw[:600, 160:2080] = 120
        raw[600:, 160:2080] = 160
        observed = []

        def detect(image, pattern, points, matrix, distortion):
            observed.append((image[100, 100, 0], image[500, 100, 0], matrix.copy()))
            raise ValueError("未检出完整棋盘")

        with patch.object(preview.ego, "board_pose", side_effect=detect):
            self.processor.process(raw, 1., now=1.)
        self.assertEqual([row[:2] for row in observed], [(40, 80), (120, 160)])
        for index, side in enumerate(("left", "right")):
            expected = self.processor.cameras[side][0].copy()
            expected[:2] *= .5
            np.testing.assert_array_equal(observed[index][2], expected)

    def test_real_detector_requires_complete_board_in_each_eye(self):
        eye = np.full((1200, 1920, 3), 180, dtype=np.uint8)
        cols, rows = self.processor.pattern
        square = 80
        for row in range(rows + 1):
            for col in range(cols + 1):
                y, x = 220 + row * square, 480 + col * square
                eye[y:y + square, x:x + square] = 255 if (row + col) % 2 else 0
        raw = np.zeros((1200, 4000, 3), dtype=np.uint8)
        raw[:, 160:2080] = eye
        raw[:, 2080:] = eye
        result = self.processor.process(raw, 0., now=0.)
        self.assertTrue(result["left_detected"])
        self.assertTrue(result["right_detected"])
        raw[:, 160:2080] = 180
        result = self.processor.process(raw, .5, now=.5)
        self.assertTrue(result["left_detected"])
        self.assertFalse(result["right_detected"])
        self.assertEqual(result["stable_seconds"], 0.)
        self.assertEqual(result["pose_count"], 0)


if __name__ == "__main__":
    cv2.setNumThreads(1)
    unittest.main()

"""逐帧标注的时间、坐标和有效性约束；不依赖采集硬件。"""
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import process_recording as processing


def stream(times, xyz=None):
    count = len(times)
    return {"times": np.asarray(times, dtype=float),
            "matrices": np.repeat(np.eye(4)[None], count, axis=0),
            "tracked": np.ones(count, dtype=bool),
            "xyz": np.repeat(np.array([[[0.04, 0, 1.0]] * 21]), count, axis=0)
                   if xyz is None else xyz,
            "mask": np.ones((count, 21), dtype=bool),
            "present": np.ones((count, 21), dtype=bool)}


class ProcessingTests(unittest.TestCase):
    def test_gap_limits_and_no_extrapolation(self):
        hand = stream([0, 0.05])
        self.assertEqual(processing.sample_hand(hand, 0.025)[4], "tracked")
        self.assertEqual(processing.sample_hand(stream([0, 0.051]), 0.025)[4], "gap_too_large")
        self.assertEqual(processing.sample_head(stream([0, 0.15]), 0.075)[1], "tracked")
        self.assertEqual(processing.sample_head(stream([0, 0.151]), 0.075)[1], "gap_too_large")
        self.assertEqual(processing.sample_hand(hand, -0.01)[4], "no_data")
        self.assertEqual(processing.sample_hand(hand, 0.06)[4], "no_data")
        self.assertEqual(processing.sample_hand(hand, 0.05)[5], 0)

    def test_head_rotation_and_translation_interpolation(self):
        head = stream([0, 0.1])
        head["matrices"][1, :3, :3] = cv2.Rodrigues(np.array([0.0, 0.0, np.pi / 2]))[0]
        head["matrices"][1, 0, 3] = 2
        matrix, status, gap = processing.sample_head(head, 0.05)
        self.assertEqual(status, "tracked")
        self.assertEqual(gap, 100)
        np.testing.assert_allclose(matrix[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(matrix[:3, :3], cv2.Rodrigues(np.array([0.0, 0.0, np.pi / 4]))[0], atol=1e-12)

    def test_raw_pixels_and_independent_in_frame_mask(self):
        xyz = np.array([[0.04, 0, 1.0]] * 21)
        xyz[1, 2] = -1
        xyz[2, 0] = 2
        valid = np.ones(21, dtype=bool)
        valid[3] = False
        reasons = [None] * 21
        reasons[3] = "joint_not_tracked"
        camera = (np.array([[1000.0, 0, 960], [0, 1000, 600], [0, 0, 1]]), np.zeros(5))
        output = processing.project_hand(xyz, valid, reasons, np.eye(4), "tracked", np.eye(4), camera)
        self.assertEqual(output["pixels"][0], [1000.0, 600.0])
        self.assertTrue(output["valid"][0])
        self.assertFalse(output["valid"][1])
        self.assertEqual(output["invalid_reasons"][1], "behind_or_near_camera")
        self.assertTrue(output["valid"][2])
        self.assertFalse(output["in_frame"][2])
        self.assertFalse(output["valid"][3])
        self.assertTrue(output["in_frame"][3])

    def test_missing_head_preserves_world_hands(self):
        streams = {"head": stream([]), "leftHand": stream([0, 0.04]), "rightHand": stream([0, 0.04])}
        camera = (np.eye(3), np.zeros(5))
        row = processing.annotate_frame(
            {"frame_index": 3, "pts_seconds": 0.1, "systemTime": 123}, 0.02, streams,
            {"left": camera, "right": camera}, {"left": np.eye(4), "right": np.eye(4)})
        self.assertFalse(row["head"]["valid"])
        for hand in row["hands"].values():
            self.assertTrue(all(hand["world_valid"]))
            self.assertEqual(hand["joints_world_m"][0], [0.04, 0.0, 1.0])
            self.assertFalse(any(hand["cameras"]["left"]["valid"]))
            self.assertEqual(hand["cameras"]["left"]["pixels"][0], [None, None])
            self.assertEqual(hand["cameras"]["left"]["invalid_reasons"][0], "head_no_data")
        json.dumps(row, allow_nan=False)

    def test_estimated_and_missing_joints_remain_invalid(self):
        hand = stream([0, 0.04])
        hand["mask"][1, 0] = False
        hand["present"][0, 1] = False
        hand["xyz"][0, 1] = np.nan
        xyz, tracked, valid, reasons, _, _ = processing.sample_hand(hand, 0.02)
        self.assertTrue(np.isfinite(xyz[0]).all())
        self.assertFalse(tracked[0])
        self.assertFalse(valid[0])
        self.assertEqual(reasons[0], "joint_not_tracked")
        self.assertEqual(reasons[1], "missing_joint")
        self.assertEqual(processing.finite_list(xyz)[1], [None, None, None])
        hand["tracked"][1] = False
        self.assertEqual(processing.sample_hand(hand, 0.02)[4], "untracked")

    def test_duplicate_anchor_timestamp_retains_changed_pose(self):
        with TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "metadata.json").write_text(json.dumps({"poseData": {
                "matrixLayout": "column_major", "translationUnit": "meters",
                "headPoseSource": "queryDeviceAnchor_current_time"}}))
            records = []
            for time, x in ((0, 0), (0.04, 2)):
                transform = np.eye(4)
                transform[0, 3] = x
                records.append({"source": "leftHand", "systemTime": time, "timestamp": 10,
                    "trackingSessionId": "one-session", "isTracked": True,
                    "originFromAnchorTransform": transform.flatten(order="F").tolist(),
                    "joints": [{"name": "wrist", "isTracked": True,
                                "anchorFromJointTransform": np.eye(4).flatten(order="F").tolist()}]})
            path = directory / "tracking_events.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in records))
            with self.assertRaisesRegex(ValueError, "至少需要两条 head"):
                processing.load_streams(directory)
            for time in (0, 0.04):
                records.append({"source": "head", "systemTime": time,
                    "recordingTimestamp": time, "trackingSessionId": "one-session", "isTracked": True,
                    "originFromAnchorTransform": np.eye(4).flatten(order="F").tolist()})
                if time == 0:
                    path.write_text("\n".join(json.dumps(row) for row in records))
                    with self.assertRaisesRegex(ValueError, "至少需要两条 head"):
                        processing.load_streams(directory)
            path.write_text("\n".join(json.dumps(row) for row in records))
            streams, continuity, session = processing.load_streams(directory)
            self.assertEqual(len(streams["leftHand"]["times"]), 2)
            xyz, _, _, _, _, _ = processing.sample_hand(streams["leftHand"], 0.02)
            np.testing.assert_allclose(xyz[0], [1, 0, 0])
            self.assertEqual(continuity["max_abs_residual_ms"], 0)
            self.assertEqual(session, "one-session")
            records[1]["trackingSessionId"] = "second-session"
            path.write_text("\n".join(json.dumps(row) for row in records))
            with self.assertRaises(ValueError):
                processing.load_streams(directory)


if __name__ == "__main__":
    unittest.main()

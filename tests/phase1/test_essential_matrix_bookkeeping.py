#!/usr/bin/env python3
"""
Synthetic Regression Test: Essential Matrix Bookkeeping vs. recoverPose Cheirality Filter

Verifies:
1. num_inliers_E (findEssentialMat RANSAC epipolar inliers) vs. num_inliers_pose (recoverPose cheirality inliers)
2. Demonstrates the distinction under:
   - 1.5 cm translation (0 deg roll)
   - 10 deg roll + 1.5 cm translation
   - 20 cm translation (0 deg roll)
3. Ensures pose matrix R and unit-scale translation vector t recovery are evaluated.
"""

import sys
import unittest
import cv2
import numpy as np


class TestEssentialMatrixBookkeeping(unittest.TestCase):
    def setUp(self):
        # Camera Intrinsics Matrix K (matching minimal_vo.py)
        self.fx = 539.9363
        self.fy = 539.9364
        self.cx = 640.0
        self.cy = 480.0
        self.K = np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0]
        ], dtype=np.float64)

        # Generate 500 synthetic 3D points in front of camera (Z in [2.0, 10.0] meters)
        np.random.seed(42)
        self.N = 500
        X = np.random.uniform(-3.0, 3.0, self.N)
        Y = np.random.uniform(-3.0, 3.0, self.N)
        Z = np.random.uniform(2.0, 10.0, self.N)
        self.pts_3d = np.vstack([X, Y, Z]).T

        # Project points to Frame 1 (Identity pose)
        pts1_h = (self.K @ self.pts_3d.T).T
        self.pts1 = (pts1_h[:, :2] / pts1_h[:, 2:]).reshape(-1, 1, 2).astype(np.float32)

    def _simulate_motion(self, roll_deg, t_vec):
        # Create rotation around optical axis Z
        rad = np.radians(roll_deg)
        R_gt = np.array([
            [np.cos(rad), -np.sin(rad), 0.0],
            [np.sin(rad),  np.cos(rad), 0.0],
            [0.0,          0.0,         1.0]
        ], dtype=np.float64)
        t_gt = np.array(t_vec, dtype=np.float64).reshape(3, 1)

        # Transform 3D points to Frame 2
        pts_3d_2 = (R_gt @ self.pts_3d.T + t_gt).T
        pts2_h = (self.K @ pts_3d_2.T).T
        pts2 = (pts2_h[:, :2] / pts2_h[:, 2:]).reshape(-1, 1, 2).astype(np.float32)

        # 1. findEssentialMat
        E, mask_E = cv2.findEssentialMat(
            self.pts1, pts2, self.K,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        num_inliers_E = int(np.sum(mask_E == 1)) if mask_E is not None else 0

        # 2. Legacy recoverPose (default distanceThresh=50.0)
        num_inliers_pose_legacy = 0
        if E is not None and E.shape == (3, 3):
            res_legacy = cv2.recoverPose(
                E, self.pts1, pts2, self.K,
                mask=mask_E.copy() if mask_E is not None else None
            )
            num_inliers_pose_legacy = int(res_legacy[0])

        # 3. Repaired production recoverPose (distanceThresh=1000.0)
        num_inliers_pose_repaired = 0
        R_opt, t_opt = None, None
        r_valid, t_valid = False, False
        if E is not None and E.shape == (3, 3):
            res_repaired = cv2.recoverPose(
                E, self.pts1, pts2, self.K,
                distanceThresh=1000.0,
                mask=mask_E.copy() if mask_E is not None else None
            )
            num_inliers_pose_repaired = int(res_repaired[0])
            R_opt = res_repaired[1]
            t_opt = res_repaired[2]
            r_valid = (R_opt is not None and R_opt.shape == (3, 3) and np.abs(np.linalg.det(R_opt) - 1.0) < 1e-3)
            t_valid = (t_opt is not None and t_opt.shape == (3, 1) and np.abs(np.linalg.norm(t_opt) - 1.0) < 1e-3)

        return num_inliers_E, num_inliers_pose_legacy, num_inliers_pose_repaired, r_valid, t_valid, E, R_opt, t_opt

    def test_small_translation_0deg(self):
        """Test Case 1: 1.5 cm translation, 0 deg roll"""
        num_E, num_legacy, num_repaired, r_valid, t_valid, E, R, t = self._simulate_motion(0.0, [0.015, 0.0, 0.0])
        print(f"\n[TEST 1] 1.5 cm Translation (0 deg roll):")
        print(f"  num_inliers_E (findEssentialMat)            : {num_E} / {self.N}")
        print(f"  legacy recoverPose (distanceThresh=50.0)    : {num_legacy} / {self.N}")
        print(f"  repaired recoverPose (distanceThresh=1000.0): {num_repaired} / {self.N}")

        # Assertions matching user requirements
        self.assertGreaterEqual(num_E, 450, "findEssentialMat failed to find epipolar inliers!")
        self.assertLess(num_legacy, 50, "Legacy recoverPose unexpectedly passed unit-scale points under default threshold!")
        self.assertGreaterEqual(num_repaired, 450, "Repaired recoverPose failed to recover valid unit-scale points!")

    def test_small_translation_10deg_roll(self):
        """Test Case 2: 10 deg roll + 1.5 cm translation (Experimental Baseline)"""
        num_E, num_legacy, num_repaired, r_valid, t_valid, E, R, t = self._simulate_motion(10.0, [0.015, 0.0, 0.0])
        print(f"\n[TEST 2] 10 deg Roll + 1.5 cm Translation (Experimental Baseline):")
        print(f"  num_inliers_E (findEssentialMat)            : {num_E} / {self.N}")
        print(f"  legacy recoverPose (distanceThresh=50.0)    : {num_legacy} / {self.N}")
        print(f"  repaired recoverPose (distanceThresh=1000.0): {num_repaired} / {self.N}")

        self.assertGreaterEqual(num_E, 450, "findEssentialMat failed under 10 deg roll + 1.5 cm translation!")
        self.assertLess(num_legacy, 50, "Legacy recoverPose unexpectedly passed unit-scale points under default threshold!")
        self.assertGreaterEqual(num_repaired, 450, "Repaired recoverPose failed to recover valid unit-scale points!")

    def test_large_translation_20cm(self):
        """Test Case 3: 20 cm translation, 0 deg roll"""
        num_E, num_legacy, num_repaired, r_valid, t_valid, E, R, t = self._simulate_motion(0.0, [0.20, 0.0, 0.0])
        print(f"\n[TEST 3] 20 cm Translation (0 deg roll):")
        print(f"  num_inliers_E (findEssentialMat)            : {num_E} / {self.N}")
        print(f"  legacy recoverPose (distanceThresh=50.0)    : {num_legacy} / {self.N}")
        print(f"  repaired recoverPose (distanceThresh=1000.0): {num_repaired} / {self.N}")

        self.assertGreaterEqual(num_E, 450, "findEssentialMat failed under 20 cm translation!")
        self.assertGreaterEqual(num_repaired, 450, "Repaired recoverPose failed under 20 cm translation!")
        self.assertTrue(r_valid, "Recovered rotation matrix det != 1.0")
        self.assertTrue(t_valid, "Recovered translation norm != 1.0")


if __name__ == '__main__':
    unittest.main()

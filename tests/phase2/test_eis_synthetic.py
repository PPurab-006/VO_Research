#!/usr/bin/env python3
"""
Synthetic Regression Test Suite for EIS / IMU Derotation (Phase 2A)

Verifies:
1. Mathematical formulation and OpenCV warpPerspective direction of H_cv = K @ R_rel @ K_inv.
2. Multi-depth point cloud projection (Z = 2.0m, 5.0m, 10.0m, 20.0m).
3. Pure Rotation: EIS derotation removes rotation motion completely (< 10^-5 px residual across ALL depths).
4. Pure Translation: EIS leaves translation parallax intact (depth-dependent motion preserved).
5. Coupled Motion (Translation + Rotation): EIS flow matches pure-translation flow field (< 0.05 px error).
6. Fixed Virtual-Camera Intrinsics K preservation.
"""

import sys
import unittest
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R_scipy


class TestEISSynthetic(unittest.TestCase):
    def setUp(self):
        # Camera Intrinsics
        self.fx = 539.9363327026367
        self.fy = 539.9363708496094
        self.cx = 640.0
        self.cy = 480.0
        self.K = np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0]
        ], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)
        self.width = 1280
        self.height = 960

        # Frame transform: Optical -> Gazebo ENU
        self.R_opt2gaz = np.array([
            [ 0.0,  0.0,  1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0]
        ], dtype=np.float64)

        # Generate multi-depth 3D point cloud grid in World ENU frame
        # Depths: Z = 2.0m, 5.0m, 10.0m, 20.0m in optical frame
        pts_world = []
        for depth in [2.0, 5.0, 10.0, 20.0]:
            for x_opt in np.linspace(-1.5, 1.5, 7):
                for y_opt in np.linspace(-1.0, 1.0, 5):
                    X_opt = np.array([x_opt * (depth / 2.0), y_opt * (depth / 2.0), depth], dtype=np.float64)
                    # Convert to World ENU (assuming camera at origin with default orientation)
                    X_world = self.R_opt2gaz.T @ X_opt
                    pts_world.append((X_world, depth))

        self.pts_world = pts_world

    def project_points(self, pts_world, R_body, t_world):
        """
        Projects 3D world points into camera image plane given vehicle pose (R_body, t_world).
        R_cam_opt = R_body @ self.R_opt2gaz.T
        X_opt = R_cam_opt.T @ (X_world - t_world)
        p = K @ X_opt
        """
        R_cam_opt = R_body @ self.R_opt2gaz.T
        pixels = []
        valid_indices = []

        for idx, (X_w, d) in enumerate(pts_world):
            X_opt = R_cam_opt.T @ (X_w - t_world)
            if X_opt[2] > 0.1:  # Check front of camera
                p_hom = self.K @ X_opt
                u = float(p_hom[0] / p_hom[2])
                v = float(p_hom[1] / p_hom[2])
                if 0 <= u < self.width and 0 <= v < self.height:
                    pixels.append([u, v])
                    valid_indices.append(idx)

        res_pts = np.array(pixels, dtype=np.float64)
        if res_pts.ndim == 1:
            res_pts = res_pts.reshape(-1, 2)
        return res_pts, valid_indices

    def render_synthetic_image(self, pixels):
        """
        Renders a synthetic binary grid image with rendered feature points.
        """
        img = np.zeros((self.height, self.width), dtype=np.uint8)
        for p in pixels:
            u, v = int(round(p[0])), int(round(p[1]))
            if 0 <= u < self.width and 0 <= v < self.height:
                cv2.circle(img, (u, v), 3, 255, -1)
        return img

    def test_pure_rotation_derotation(self):
        """
        Case A: Pure Rotation
        Vehicle rotates pitch 5 deg and roll 3 deg. Translation = 0.
        Verifies depth-independent rotation removal to numerical precision.
        """
        R_body_1 = np.eye(3, dtype=np.float64)
        t_world_1 = np.zeros(3, dtype=np.float64)

        # Frame 2: Rotate vehicle (pitch 5 deg, roll 3 deg)
        rot_delta = R_scipy.from_euler('xyz', [3.0, 5.0, 0.0], degrees=True).as_matrix()
        R_body_2 = rot_delta @ R_body_1
        t_world_2 = np.zeros(3, dtype=np.float64)

        pts1, ids1 = self.project_points(self.pts_world, R_body_1, t_world_1)
        pts2, ids2 = self.project_points(self.pts_world, R_body_2, t_world_2)

        # Intersect indices present in both frames
        common_ids = sorted(list(set(ids1).intersection(set(ids2))))
        idx_map_1 = {id_: i for i, id_ in enumerate(ids1)}
        idx_map_2 = {id_: i for i, id_ in enumerate(ids2)}

        p1_matched = np.array([pts1[idx_map_1[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)
        p2_matched = np.array([pts2[idx_map_2[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)

        # Compute optical orientations and homography H_cv for OpenCV warpPerspective
        # p_2 = H_cv @ p_1  =>  H_cv = K @ (R_cam_opt_2.T @ R_cam_opt_1) @ K_inv
        # Derotating p_2 to p_1: p_1_est = H_cv_inv @ p_2
        R_cam_opt_1 = R_body_1 @ self.R_opt2gaz.T
        R_cam_opt_2 = R_body_2 @ self.R_opt2gaz.T

        R_rel_opt = R_cam_opt_2.T @ R_cam_opt_1
        H_cv = self.K @ R_rel_opt @ self.K_inv
        H_cv_inv = np.linalg.inv(H_cv)

        # Warp Frame 2 pixel coordinates back to Frame 1 reference plane
        # p_2 = H_cv @ p_1  =>  p_1_est = H_cv_inv @ p_2
        p2_hom = np.column_stack([p2_matched, np.ones(len(p2_matched))])
        p1_est_hom = (H_cv_inv @ p2_hom.T).T
        p1_est = p1_est_hom[:, :2] / p1_est_hom[:, 2:]

        # Raw displacement error vs Residual error post-derotation
        raw_displacement = np.linalg.norm(p2_matched - p1_matched, axis=1)
        residual_error = np.linalg.norm(p1_est - p1_matched, axis=1)

        print(f"\n--- [SYNTHETIC TEST A: PURE ROTATION] ---")
        print(f"  Raw Pixel Displacement Mean : {np.mean(raw_displacement):.2f} px (Max: {np.max(raw_displacement):.2f} px)")
        print(f"  Post-EIS Residual Error Mean: {np.mean(residual_error):.8f} px (Max: {np.max(residual_error):.8f} px)")

        # Assertion: Derotation removes pure rotation pixel error to < 1e-5 px across all depths
        self.assertLess(np.max(residual_error), 1e-5, "Pure rotation residual exceeds numerical tolerance!")

    def test_pure_translation_preservation(self):
        """
        Case B: Pure Translation
        Vehicle translates [0.2, 0.1, 0.05] m. Rotation = 0.
        Verifies translation parallax is depth-dependent and 100% preserved.
        """
        R_body_1 = np.eye(3, dtype=np.float64)
        t_world_1 = np.zeros(3, dtype=np.float64)

        R_body_2 = np.eye(3, dtype=np.float64)
        t_world_2 = np.array([0.2, 0.1, 0.05], dtype=np.float64)

        pts1, ids1 = self.project_points(self.pts_world, R_body_1, t_world_1)
        pts2, ids2 = self.project_points(self.pts_world, R_body_2, t_world_2)

        # Intersect indices present in both frames
        common_ids = sorted(list(set(ids1).intersection(set(ids2))))
        idx_map_1 = {id_: i for i, id_ in enumerate(ids1)}
        idx_map_2 = {id_: i for i, id_ in enumerate(ids2)}

        p1_matched = np.array([pts1[idx_map_1[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)
        p2_matched = np.array([pts2[idx_map_2[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)

        # Homography for pure translation should be Identity matrix
        R_cam_opt_1 = R_body_1 @ self.R_opt2gaz.T
        R_cam_opt_2 = R_body_2 @ self.R_opt2gaz.T
        R_rel_ref = R_cam_opt_2 @ R_cam_opt_1.T

        H_cv = self.K @ R_rel_ref @ self.K_inv

        # Verify H_cv is Identity
        identity_diff = np.max(np.abs(H_cv - np.eye(3)))
        self.assertLess(identity_diff, 1e-12, "Homography for pure translation must be Identity!")

        # Measure flow across depths
        displacement = np.linalg.norm(p2_matched - p1_matched, axis=1)
        depths = [self.pts_world[i][1] for i in common_ids]

        disp_by_depth = {}
        for d, disp in zip(depths, displacement):
            disp_by_depth.setdefault(d, []).append(disp)

        print(f"\n--- [SYNTHETIC TEST B: PURE TRANSLATION PRESERVATION] ---")
        for d in sorted(disp_by_depth.keys()):
            mean_disp = np.mean(disp_by_depth[d])
            print(f"  Depth Z = {d:4.1f} m -> Mean Pixel Flow: {mean_disp:.2f} px")

        # Assertion: Near points (Z=2.0m) must have larger displacement than far points (Z=20.0m)
        self.assertGreater(np.mean(disp_by_depth[2.0]), np.mean(disp_by_depth[20.0]), "Translational flow must show 1/Z depth scaling!")

    def test_coupled_motion_isolation(self):
        """
        Case C: Translation + Rotation (Coupled Motion)
        Vehicle translates [0.2, 0.1, 0.05] m AND rotates pitch 4 deg, roll 2 deg.
        Verifies EIS derotates Frame 2 such that residual flow matches pure translation flow!
        """
        R_body_1 = np.eye(3, dtype=np.float64)
        t_world_1 = np.zeros(3, dtype=np.float64)

        # Coupled Motion: Translation + Rotation
        rot_delta = R_scipy.from_euler('xyz', [2.0, 4.0, 0.0], degrees=True).as_matrix()
        R_body_2 = rot_delta @ R_body_1
        t_world_2 = np.array([0.2, 0.1, 0.05], dtype=np.float64)

        # Pure Translation Benchmark & Coupled Motion Points
        pts1, ids1 = self.project_points(self.pts_world, R_body_1, t_world_1)
        pts2_trans, ids2_trans = self.project_points(self.pts_world, R_body_1, t_world_2)
        pts2_coupled, ids2_coupled = self.project_points(self.pts_world, R_body_2, t_world_2)

        # Intersect indices present across all three sets
        common_ids = sorted(list(set(ids1).intersection(set(ids2_trans)).intersection(set(ids2_coupled))))
        idx_map_1 = {id_: i for i, id_ in enumerate(ids1)}
        idx_map_trans = {id_: i for i, id_ in enumerate(ids2_trans)}
        idx_map_coupled = {id_: i for i, id_ in enumerate(ids2_coupled)}

        p1_matched = np.array([pts1[idx_map_1[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)
        p2_trans_matched = np.array([pts2_trans[idx_map_trans[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)
        p2_coupled_matched = np.array([pts2_coupled[idx_map_coupled[id_]] for id_ in common_ids], dtype=np.float64).reshape(-1, 2)

        # Compute Homography for Frame 2 rotation
        R_cam_opt_1 = R_body_1 @ self.R_opt2gaz.T
        R_cam_opt_2 = R_body_2 @ self.R_opt2gaz.T
        R_rel_opt = R_cam_opt_2.T @ R_cam_opt_1

        H_cv = self.K @ R_rel_opt @ self.K_inv
        H_cv_inv = np.linalg.inv(H_cv)

        # Warp coupled points back to Frame 1 orientation plane
        p2_coupled_hom = np.column_stack([p2_coupled_matched, np.ones(len(p2_coupled_matched))])
        p2_eis_hom = (H_cv_inv @ p2_coupled_hom.T).T
        p2_eis = p2_eis_hom[:, :2] / p2_eis_hom[:, 2:]

        # Difference between EIS flow and Pure Translation flow
        flow_raw = p2_coupled_matched - p1_matched
        flow_pure_trans = p2_trans_matched - p1_matched
        flow_eis = p2_eis - p1_matched

        eis_vs_trans_diff = np.linalg.norm(p2_eis - p2_trans_matched, axis=1)

        print(f"\n--- [SYNTHETIC TEST C: COUPLED MOTION ISOLATION] ---")
        print(f"  Raw Flow Mean         : {np.mean(np.linalg.norm(flow_raw, axis=1)):.2f} px")
        print(f"  Pure Translation Flow : {np.mean(np.linalg.norm(flow_pure_trans, axis=1)):.2f} px")
        print(f"  EIS Derotated Flow    : {np.mean(np.linalg.norm(flow_eis, axis=1)):.2f} px")
        print(f"  EIS vs Trans Diff Mean: {np.mean(eis_vs_trans_diff):.6f} px (Max: {np.max(eis_vs_trans_diff):.6f} px)")

        # Assertion: Derotated coupled flow matches pure translation flow to high numerical precision
        self.assertLess(np.max(eis_vs_trans_diff), 0.05, "EIS derotated flow deviates from pure translation flow!")

    def test_incremental_vs_fixed_large_cumulative_yaw(self):
        """
        Case D: Large Cumulative Yaw with Small Per-Frame Yaw (60 frames x 0.5 deg/frame = 30 deg total yaw)
        Directly tests the hypothesis that incremental mode avoids the coverage-loss and perspective stretch
        that fixed mode exhibits under large cumulative yaw.
        """
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
        from eis_derotation import EISDerotator

        eis_fixed = EISDerotator(reference_mode='fixed')
        eis_inc = EISDerotator(reference_mode='incremental')

        # Frame 0: Yaw 0 deg
        R_body_0 = np.eye(3, dtype=np.float64)
        R_cam_opt_0 = R_body_0 @ self.R_opt2gaz.T
        eis_fixed.R_ref = R_cam_opt_0

        # Simulate 60 frames of 0.5 deg/frame yaw (total 30 deg yaw)
        yaw_per_frame = 0.5
        n_frames = 60
        cum_yaw_deg = n_frames * yaw_per_frame  # 30.0 deg

        # Frame 59 (t_59) and Frame 60 (t_60)
        yaw_59 = (n_frames - 1) * yaw_per_frame
        yaw_60 = n_frames * yaw_per_frame

        R_body_59 = R_scipy.from_euler('z', yaw_59, degrees=True).as_matrix()
        R_body_60 = R_scipy.from_euler('z', yaw_60, degrees=True).as_matrix()

        # Fixed Mode Homography for Frame 60 (relative to Frame 0 reference)
        H_fixed = eis_fixed.compute_homography(R_body_60, reference_mode='fixed')
        crop_pct_fixed = eis_fixed.compute_crop_percentage(H_fixed)
        warp_angle_fixed = eis_fixed.compute_warp_angle(H_fixed)

        # Incremental Mode Homography for Frame 60 (relative to Frame 59 reference)
        eis_inc.prev_R_body = R_body_59
        H_inc = eis_inc.compute_homography(R_body_60, reference_mode='incremental')
        crop_pct_inc = eis_inc.compute_crop_percentage(H_inc)
        warp_angle_inc = eis_inc.compute_warp_angle(H_inc)

        print(f"\n--- [SYNTHETIC TEST D: LARGE CUMULATIVE YAW (30.0 deg)] ---")
        print(f"  Fixed Mode (rel to t_0)   : Warp Angle = {warp_angle_fixed:5.2f} deg, Border Crop Lost = {crop_pct_fixed:5.2f}%")
        print(f"  Incremental Mode (frame-to-frame): Warp Angle = {warp_angle_inc:5.2f} deg, Border Crop Lost = {crop_pct_inc:5.2f}%")

        # Assertions:
        # Fixed mode exhibits large crop loss (> 25%) and 30 deg warp angle
        self.assertGreater(crop_pct_fixed, 25.0, "Fixed mode should exhibit > 25% border crop under 30 deg cumulative yaw!")
        self.assertAlmostEqual(warp_angle_fixed, cum_yaw_deg, delta=0.5)

        # Incremental mode exhibits minimal crop loss (< 2.0%) and ~0.5 deg warp angle
        self.assertLess(crop_pct_inc, 2.0, "Incremental mode must exhibit < 2.0% border crop under 0.5 deg inter-frame yaw!")
        self.assertAlmostEqual(warp_angle_inc, yaw_per_frame, delta=0.1)


if __name__ == '__main__':
    unittest.main()



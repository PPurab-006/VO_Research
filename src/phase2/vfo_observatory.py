#!/usr/bin/env python3
"""
Visual Field Observatory (VFO) Module (Phase 2C)

A non-intrusive, diagnostic-only pipeline that extracts time-series of visual-condition
variables from raw camera sequences and synchronized attitude telemetry.

VFO Variables & Theoretical Failure Linkage:
--------------------------------------------
1. feature_count: Total GFTT corners detected in previous frame.
   - Proxy for: Local scene keypoint richness.
   - Linkage to VO: Low feature count directly reduces RANSAC hypothesis quality and pose solver stability.

2. feature_survival_rate: Fraction of features successfully tracked from frame k-1 to k (N_tracked / N_prev).
   - Proxy for: Frame-to-frame temporal tracking continuity.
   - Linkage to VO: Sudden drops indicate illumination jumps, motion blur, or rapid FOV shifts.

3. mean_lk_err: Mean Lucas-Kanade optical flow residual tracking error.
   - Proxy for: Image patch appearance constancy / warp alignment quality.
   - Linkage to VO: High LK error indicates sub-pixel tracking degradation, non-rigid motion, or blur.

4. feature_vel_mean: Mean optical flow displacement magnitude ||V_i||_2 in pixels/frame.
   - Proxy for: Apparent image velocity across the focal plane.
   - Linkage to VO: Excessive displacement (>15-20 px/frame) causes KLT search window overflow and tracking loss.

5. num_inliers_E: Essential matrix RANSAC inlier count (5-point algorithm).
   - Proxy for: Geometric epipolar constraint compliance.
   - Linkage to VO: Low E-inliers signal non-epipolar motion, occlusion, or corrupt feature matches.

6. num_inliers_pose: Cheirality/depth verification inlier count from cv2.recoverPose.
   - Proxy for: Valid 3D point structure in front of both camera focal centers.
   - Linkage to VO: Primary metric for monocular VO pose validity (N_pose >= 8 threshold).

7. pose_e_ratio: Ratio of pose inliers to Essential matrix inliers (num_inliers_pose / max(1, num_inliers_E)).
   - Proxy for: 3D point cheirality agreement relative to epipolar geometry.
   - Linkage to VO: Drops occur when points lie behind camera or when pure rotation causes ill-conditioned baseline.

8. flow_direction_entropy: Normalized entropy of 16-bin optical flow direction histogram (-sum P_i log_16 P_i).
   - Proxy for: Directional dispersion of image motion.
   - Linkage to VO: 0.0 = coherent linear translation (unidirectional flow); 1.0 = isotropic/radial rotational or chaotic flow.

9. flow_coherence: Ratio of mean displacement vector magnitude to mean scalar displacement (||sum V_i|| / sum ||V_i||).
   - Proxy for: Global spatial alignment of optical flow vectors.
   - Linkage to VO: 1.0 = perfectly parallel translational flow; near 0.0 = rotational flow (vectors cancel out) or noise.

10. spatial_distribution_score: Normalized entropy of feature counts across a 4x4 image grid (16 cells).
    - Proxy for: Spatial uniform distribution of tracked features across focal plane.
    - Linkage to VO: Low score (<0.5) indicates feature clustering in small image patches, reducing geometric leverage for pose.

11. texture_density: Mean image gradient magnitude sqrt(I_x^2 + I_y^2) across the grayscale frame.
    - Proxy for: Environmental visual texture and high-frequency edge content.
    - Linkage to VO: Low texture density (e.g. featureless ground/walls) limits corner detection density.

12. border_loss_pct: Fraction of features in frame k-1 located near border (outer 10% margin) that were lost in frame k.
    - Proxy for: FOV border egress caused by camera rotation or aggressive translational motion.
    - Linkage to VO: High border loss isolates rotation/yaw sweeping crop losses from general feature degradation.

13. estimated_rotational_flow_magnitude: Mean magnitude of optical flow ||V_i_rot||_2 predicted purely by attitude rotation.
    - Proxy for: Pure rotation-induced image motion (from IMU/attitude telemetry).
    - Linkage to VO: Direct theoretical predictor of EIS geometric warping cost and parallax attenuation.

14. estimated_translational_flow_magnitude: Mean magnitude of residual optical flow ||V_i_obs - V_i_rot||_2.
    - Proxy for: Physical translation-induced parallax flow.
    - Linkage to VO: Necessary signal for 3D translation estimation and monocular VO pose recovery.

15. rotational_flow_ratio: V_rot / (V_rot + V_trans + eps).
    - Proxy for: Fraction of total optical flow driven by rotation vs translation.
    - Linkage to VO: High ratio (>0.7) signals rotational dominance and high risk of ill-conditioned pose estimation.
"""

import os
import math
import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy


class VisualFieldObservatory:
    def __init__(self, K=None, width=1280, height=960, eis_derotator=None):
        if K is None:
            # Default intrinsics from Phase 0A verification
            self.fx = 539.9363327026367
            self.fy = 539.9363708496094
            self.cx = 640.0
            self.cy = 480.0
            self.K = np.array([
                [self.fx, 0.0,     self.cx],
                [0.0,     self.fy, self.cy],
                [0.0,     0.0,     1.0]
            ], dtype=np.float64)
        else:
            self.K = np.array(K, dtype=np.float64)
            self.fx = self.K[0, 0]
            self.fy = self.K[1, 1]
            self.cx = self.K[0, 2]
            self.cy = self.K[1, 2]

        self.K_inv = np.linalg.inv(self.K)
        self.width = width
        self.height = height
        self.eis_derotator = eis_derotator

        # CV Optical to Gazebo ENU transformation matrix
        self.R_opt2gaz = np.array([
            [ 0.0,  0.0,  1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0]
        ], dtype=np.float64)

        # Internal state
        self.prev_img = None
        self.prev_pts = None
        self.prev_time_sec = None
        self.frame_idx = 0

    def reset_state(self):
        """Resets internal feature tracking state."""
        self.prev_img = None
        self.prev_pts = None
        self.prev_time_sec = None
        self.frame_idx = 0

    def process_frame(self, cv_img, total_sec):
        """
        Processes raw unwarped grayscale frame at timestamp total_sec.
        Returns a dictionary of per-frame VFO metrics.
        """
        # 1. Texture density (mean image gradient magnitude)
        sobel_x = cv2.Sobel(cv_img, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(cv_img, cv2.CV_64F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobel_x**2 + sobel_y**2)
        texture_density = float(np.mean(grad_mag))

        # Initial frame or feature re-detection
        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) < 100:
            pts = cv2.goodFeaturesToTrack(cv_img, maxCorners=2000, qualityLevel=0.001, minDistance=5)
            self.prev_pts = pts
            self.prev_img = cv_img
            self.prev_time_sec = total_sec
            self.frame_idx += 1

            num_det = len(pts) if pts is not None else 0
            spatial_dist = self._compute_spatial_distribution(pts, self.width, self.height)

            return {
                'frame_idx': self.frame_idx - 1,
                'timestamp_total_sec': total_sec,
                'feature_count': num_det,
                'feature_survival_rate': 1.0,
                'mean_lk_err': 0.0,
                'feature_vel_mean': 0.0,
                'feature_vel_max': 0.0,
                'num_inliers_E': 0,
                'num_inliers_pose': 0,
                'pose_e_ratio': 0.0,
                'flow_direction_entropy': 0.0,
                'flow_coherence': 1.0,
                'spatial_distribution_score': spatial_dist,
                'texture_density': texture_density,
                'border_loss_pct': 0.0,
                'estimated_rotational_flow_magnitude': 0.0,
                'estimated_translational_flow_magnitude': 0.0,
                'rotational_flow_ratio': 0.0
            }

        n_prev = len(self.prev_pts)

        # 2. Pyramidal Lucas-Kanade Optical Flow
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_img, cv_img, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        status_flat = status.ravel() == 1
        good_prev = self.prev_pts[status_flat].reshape(-1, 2)
        good_curr = curr_pts[status_flat].reshape(-1, 2)
        n_tracked = len(good_curr)

        survival_rate = float(n_tracked / n_prev) if n_prev > 0 else 0.0
        mean_lk_err = float(np.mean(err[status_flat])) if n_tracked > 0 else 0.0

        # 3. Border loss calculation (outer 10% margin)
        margin_x = 0.10 * self.width
        margin_y = 0.10 * self.height
        prev_pts_2d = self.prev_pts.reshape(-1, 2)
        is_border_prev = (
            (prev_pts_2d[:, 0] < margin_x) | (prev_pts_2d[:, 0] > (self.width - margin_x)) |
            (prev_pts_2d[:, 1] < margin_y) | (prev_pts_2d[:, 1] > (self.height - margin_y))
        )
        num_border_prev = int(np.sum(is_border_prev))
        lost_border_count = int(np.sum(is_border_prev & (~status_flat)))
        border_loss_pct = float(lost_border_count / num_border_prev * 100.0) if num_border_prev > 0 else 0.0

        # Default values if tracking collapses
        num_inliers_E = 0
        num_inliers_pose = 0
        pose_e_ratio = 0.0
        vel_mean = 0.0
        vel_max = 0.0
        flow_entropy = 0.0
        flow_coherence = 0.0
        rot_flow_mag = 0.0
        trans_flow_mag = 0.0
        rot_flow_ratio = 0.0

        if n_tracked >= 5:
            # Displacement vectors V = good_curr - good_prev
            V = good_curr - good_prev
            v_mags = np.linalg.norm(V, axis=1)
            vel_mean = float(np.mean(v_mags))
            vel_max = float(np.max(v_mags))

            # Flow coherence: ||sum V|| / sum ||V||
            sum_V_mag = float(np.linalg.norm(np.sum(V, axis=0)))
            sum_v_mags = float(np.sum(v_mags))
            flow_coherence = float(sum_V_mag / (sum_v_mags + 1e-6))

            # Flow direction histogram (16 bins in [-pi, pi])
            angles = np.arctan2(V[:, 1], V[:, 0])
            hist_counts, _ = np.histogram(angles, bins=16, range=(-np.pi, np.pi))
            P_hist = hist_counts / np.sum(hist_counts) if np.sum(hist_counts) > 0 else np.zeros(16)
            # Normalized entropy: -sum P_i log_16(P_i)
            nonzero_P = P_hist[P_hist > 0]
            flow_entropy = float(-np.sum(nonzero_P * np.log(nonzero_P)) / np.log(16)) if len(nonzero_P) > 0 else 0.0

            # Essential Matrix RANSAC & Pose Recovery
            E, mask_E = cv2.findEssentialMat(
                good_prev, good_curr, self.K,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )

            if E is not None and E.shape == (3, 3):
                num_inliers_E = int(np.sum(mask_E))
                if num_inliers_E >= 5:
                    res_pose = cv2.recoverPose(
                        E, good_prev, good_curr, self.K,
                        mask=mask_E.copy(), distanceThresh=1000.0
                    )
                    mask_pose = res_pose[3]
                    num_inliers_pose = int(res_pose[0])

            pose_e_ratio = float(num_inliers_pose / max(1, num_inliers_E))

            # Rotational vs Translational Flow Decomposition using attitude telemetry
            if self.eis_derotator is not None and self.prev_time_sec is not None:
                rot_flow_mag, trans_flow_mag, rot_flow_ratio = self._decompose_optical_flow(
                    good_prev, good_curr, self.prev_time_sec, total_sec
                )

        # Compute spatial distribution score on current tracked/detected points
        spatial_dist = self._compute_spatial_distribution(self.prev_pts, self.width, self.height)

        # Prepare record
        record = {
            'frame_idx': self.frame_idx,
            'timestamp_total_sec': total_sec,
            'feature_count': n_prev,
            'feature_survival_rate': survival_rate,
            'mean_lk_err': mean_lk_err,
            'feature_vel_mean': vel_mean,
            'feature_vel_max': vel_max,
            'num_inliers_E': num_inliers_E,
            'num_inliers_pose': num_inliers_pose,
            'pose_e_ratio': pose_e_ratio,
            'flow_direction_entropy': flow_entropy,
            'flow_coherence': flow_coherence,
            'spatial_distribution_score': spatial_dist,
            'texture_density': texture_density,
            'border_loss_pct': border_loss_pct,
            'estimated_rotational_flow_magnitude': rot_flow_mag,
            'estimated_translational_flow_magnitude': trans_flow_mag,
            'rotational_flow_ratio': rot_flow_ratio
        }

        # Update state for next frame
        # If tracked points drops below 100, re-detect features
        if n_tracked < 100:
            pts = cv2.goodFeaturesToTrack(cv_img, maxCorners=2000, qualityLevel=0.001, minDistance=5)
            self.prev_pts = pts
        else:
            self.prev_pts = good_curr.reshape(-1, 1, 2)

        self.prev_img = cv_img
        self.prev_time_sec = total_sec
        self.frame_idx += 1

        return record

    def _compute_spatial_distribution(self, pts, width, height):
        """
        Computes 4x4 grid spatial entropy score (1.0 = perfectly uniform distribution).
        """
        if pts is None or len(pts) == 0:
            return 0.0

        pts_2d = pts.reshape(-1, 2)
        grid_x = np.clip((pts_2d[:, 0] / width * 4).astype(int), 0, 3)
        grid_y = np.clip((pts_2d[:, 1] / height * 4).astype(int), 0, 3)

        cell_indices = grid_y * 4 + grid_x
        counts = np.bincount(cell_indices, minlength=16)
        probs = counts / np.sum(counts)

        nonzero_probs = probs[probs > 0]
        if len(nonzero_probs) == 0:
            return 0.0

        entropy = -np.sum(nonzero_probs * np.log(nonzero_probs))
        normalized_entropy = float(entropy / np.log(16))
        return normalized_entropy

    def _decompose_optical_flow(self, pts_prev, pts_curr, t_prev, t_curr):
        """
        Decomposes observed optical flow into expected rotational flow (from attitude telemetry)
        and residual translational flow.
        """
        try:
            R_body_prev = self.eis_derotator.get_attitude_at_timestamp(t_prev)
            R_body_curr = self.eis_derotator.get_attitude_at_timestamp(t_curr)

            # R_cam_opt = R_body @ R_opt2gaz.T
            R_cam_opt_prev = R_body_prev @ self.R_opt2gaz.T
            R_cam_opt_curr = R_body_curr @ self.R_opt2gaz.T

            # Relative rotation from frame k-1 to frame k in optical frame:
            # R_rel = R_cam_opt_curr.T @ R_cam_opt_prev
            R_rel = R_cam_opt_curr.T @ R_cam_opt_prev

            # Homography for pure rotation: H_rot = K @ R_rel @ K_inv
            H_rot = self.K @ R_rel @ self.K_inv

            # Transform pts_prev through H_rot
            N = len(pts_prev)
            ones = np.ones((N, 1), dtype=np.float64)
            homog_prev = np.hstack([pts_prev, ones]) # (N, 3)

            # p_rot = (H_rot @ p_prev^T)^T
            p_rot_homog = (H_rot @ homog_prev.T).T
            p_rot_2d = p_rot_homog[:, :2] / p_rot_homog[:, 2:3]

            # Rotational flow vector V_rot = p_rot_2d - pts_prev
            V_rot = p_rot_2d - pts_prev
            rot_mags = np.linalg.norm(V_rot, axis=1)
            rot_flow_mag = float(np.mean(rot_mags))

            # Observed total flow V_obs = pts_curr - pts_prev
            V_obs = pts_curr - pts_prev

            # Residual translational flow V_trans = V_obs - V_rot
            V_trans = V_obs - V_rot
            trans_mags = np.linalg.norm(V_trans, axis=1)
            trans_flow_mag = float(np.mean(trans_mags))

            rot_ratio = float(rot_flow_mag / (rot_flow_mag + trans_flow_mag + 1e-6))
            return rot_flow_mag, trans_flow_mag, rot_ratio

        except Exception as e:
            return 0.0, 0.0, 0.0

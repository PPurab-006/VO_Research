#!/usr/bin/env python3
"""
Electronic Image Stabilization (EIS) / IMU Derotation Module (Phase 2A & Phase 2C)

Formulation: Global Reference-Frame and Pairwise Derotation
Modes supported:
  - 'fixed': Warps every frame relative to initial attitude t_0.
  - 'incremental': Warps every frame relative to previous frame attitude t_{k-1}.
  - 'null': Computes exact Identity homography H_cv = I (null warp control).
  - 'gated': Reactive threshold gating based on current frame body yaw rate |omega_z|.
             If |omega_z| > threshold (e.g. 15 deg/s), bypasses derotation (H_cv = I).
  - 'scaled': Reactive smooth linear scaling based on current frame body yaw rate |omega_z|.
              Scales relative rotation matrix R_rel_opt down to I as |omega_z| rises.

Frame Conventions:
- Vehicle body orientation in Gazebo ENU world frame: R_body(t) (3x3 rotation matrix from quaternion).
- Gazebo ENU to CV Optical frame transform: R_opt2gaz = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
- Camera Optical orientation in World ENU: R_world_cam_opt(t) = R_body(t) @ R_opt2gaz.T
- Relative rotation from reference orientation to frame k: R_rel_ref(k) = R_world_cam_opt(t_k) @ R_ref.T
- Homography for OpenCV warpPerspective(src_img_k, H_cv, (w, h)):
  p_k = H_cv @ p_ref  =>  H_cv = K @ R_rel_ref(k) @ K_inv
"""

import os
import math
import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
from scipy.spatial.transform import Slerp


class EISDerotator:
    def __init__(self, K=None, width=1280, height=960, reference_mode='fixed'):
        if K is None:
            # Default camera intrinsics from Phase 0A verification
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
        self.reference_mode = reference_mode.lower()

        # CV Optical to Gazebo ENU frame transformation matrix
        self.R_opt2gaz = np.array([
            [ 0.0,  0.0,  1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0]
        ], dtype=np.float64)

        self.gt_times = None
        self.gt_quats = None
        self.slerp = None
        self.R_ref = None
        self.prev_R_body = None
        self.prev_t_sec = None

    def reset_state(self):
        """
        Resets internal tracking state for a new sequence processing run.
        """
        self.prev_R_body = None
        self.prev_t_sec = None

    def load_attitude_telemetry(self, gt_csv_path):
        """
        Loads GT attitude CSV containing timestamp_total_sec and quaternion (rot_x, rot_y, rot_z, rot_w).
        Initializes scipy Slerp for attitude interpolation on common simulation clock timeline.
        """
        df_gt = pd.read_csv(gt_csv_path)

        t_col = 'timestamp_total_sec' if 'timestamp_total_sec' in df_gt.columns else 'timestamp'
        self.gt_times = df_gt[t_col].values.astype(np.float64)

        if self.gt_times[0] > 1e12:
            self.gt_times = self.gt_times / 1e9

        # Ensure strict monotonicity for Slerp
        unique_indices = np.where(np.diff(self.gt_times) > 1e-7)[0]
        unique_indices = np.append(unique_indices, len(self.gt_times) - 1)

        self.gt_times = self.gt_times[unique_indices]

        qx = df_gt['rot_x'].values[unique_indices]
        qy = df_gt['rot_y'].values[unique_indices]
        qz = df_gt['rot_z'].values[unique_indices]
        qw = df_gt['rot_w'].values[unique_indices]

        quats = np.column_stack([qx, qy, qz, qw])
        quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)
        self.gt_quats = quats

        rotations = R_scipy.from_quat(self.gt_quats)
        self.slerp = Slerp(self.gt_times, rotations)

        # Set default reference orientation to initial attitude sample
        r_body_0 = rotations[0].as_matrix()
        self.R_ref = r_body_0 @ self.R_opt2gaz.T
        self.prev_R_body = None
        self.prev_t_sec = None

    def get_attitude_at_timestamp(self, t_sec):
        """
        Interpolates vehicle body rotation matrix R_body(t) at time t_sec via SLERP.
        """
        if self.slerp is None:
            raise RuntimeError("Attitude telemetry not loaded. Call load_attitude_telemetry() first.")

        t_clamped = np.clip(t_sec, self.gt_times[0], self.gt_times[-1])
        r_interp = self.slerp(t_clamped)
        return r_interp.as_matrix()

    def get_optical_attitude(self, R_body):
        """
        Converts vehicle body orientation (Gazebo ENU) to camera optical orientation in World ENU.
        """
        return R_body @ self.R_opt2gaz.T

    def compute_homography(self, R_body_k, R_ref=None, reference_mode=None, t_sec=None, prev_t_sec=None, gate_thresh_deg=15.0, gate_max_deg=45.0):
        """
        Computes OpenCV perspective homography H_cv for warping frame k.
        Returns tuple: (H_cv, scale_factor, yaw_rate_deg)
        """
        ref_mode = reference_mode if reference_mode is not None else self.reference_mode
        scale_factor = 1.0
        yaw_rate_deg = 0.0

        if ref_mode == 'fixed':
            if R_ref is None:
                if self.R_ref is None:
                    raise RuntimeError("Reference orientation R_ref is not set.")
                R_ref_target = self.R_ref
            else:
                R_ref_target = R_ref
            R_cam_opt_k = self.get_optical_attitude(R_body_k)
            R_rel_opt = R_cam_opt_k.T @ R_ref_target
        elif ref_mode in ['incremental', 'gated', 'scaled']:
            if R_ref is not None:
                R_ref_target = R_ref
            elif self.prev_R_body is not None:
                R_ref_target = self.get_optical_attitude(self.prev_R_body)
            else:
                R_ref_target = self.get_optical_attitude(R_body_k)
            R_cam_opt_k = self.get_optical_attitude(R_body_k)
            R_rel_opt = R_cam_opt_k.T @ R_ref_target

            # Compute body yaw rate |omega_z|
            if self.prev_R_body is not None and t_sec is not None and prev_t_sec is not None:
                dt = max(1e-4, t_sec - prev_t_sec)
                R_rel_body = self.prev_R_body.T @ R_body_k
                rotvec = R_scipy.from_matrix(R_rel_body).as_rotvec()
                omega_body_deg = np.degrees(rotvec / dt)
                yaw_rate_deg = float(abs(omega_body_deg[2]))
            else:
                yaw_rate_deg = 0.0

            if ref_mode == 'gated':
                if yaw_rate_deg > gate_thresh_deg:
                    R_rel_opt = np.eye(3, dtype=np.float64)
                    scale_factor = 0.0
                else:
                    scale_factor = 1.0
            elif ref_mode == 'scaled':
                if yaw_rate_deg <= gate_thresh_deg:
                    scale_factor = 1.0
                elif yaw_rate_deg >= gate_max_deg:
                    scale_factor = 0.0
                else:
                    scale_factor = float(1.0 - (yaw_rate_deg - gate_thresh_deg) / (gate_max_deg - gate_thresh_deg))

                scale_factor = float(np.clip(scale_factor, 0.0, 1.0))

                if scale_factor < 1.0:
                    rotvec_opt = R_scipy.from_matrix(R_rel_opt).as_rotvec()
                    scaled_rotvec = scale_factor * rotvec_opt
                    R_rel_opt = R_scipy.from_rotvec(scaled_rotvec).as_matrix()

        elif ref_mode == 'null':
            R_rel_opt = np.eye(3, dtype=np.float64)
            scale_factor = 0.0
            yaw_rate_deg = 0.0
        else:
            raise ValueError(f"Unknown reference_mode: {ref_mode}")

        H_cv = self.K @ R_rel_opt @ self.K_inv
        return H_cv, scale_factor, yaw_rate_deg

    def derotate_image(self, img, t_sec, R_ref=None, reference_mode=None, gate_thresh_deg=15.0, gate_max_deg=45.0):
        """
        Warps input image at timestamp t_sec to virtual reference camera orientation.
        Returns:
            derotated_img (np.ndarray), H_cv (np.ndarray), dt_ms (float), scale_factor (float), yaw_rate_deg (float)
        """
        R_body_k = self.get_attitude_at_timestamp(t_sec)
        nearest_gt_t = self.gt_times[np.argmin(np.abs(self.gt_times - t_sec))]
        dt_ms = abs(t_sec - nearest_gt_t) * 1000.0

        prev_t = self.prev_t_sec if hasattr(self, 'prev_t_sec') else None
        H_cv, scale_factor, yaw_rate_deg = self.compute_homography(
            R_body_k, R_ref=R_ref, reference_mode=reference_mode,
            t_sec=t_sec, prev_t_sec=prev_t,
            gate_thresh_deg=gate_thresh_deg, gate_max_deg=gate_max_deg
        )

        derotated = cv2.warpPerspective(
            img, H_cv, (self.width, self.height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0
        )

        ref_mode = reference_mode if reference_mode is not None else self.reference_mode
        if ref_mode in ['incremental', 'null', 'gated', 'scaled']:
            self.prev_R_body = R_body_k
            self.prev_t_sec = t_sec

        return derotated, H_cv, dt_ms, scale_factor, yaw_rate_deg

    def unwarp_points(self, pts, H_cv):
        """
        Transforms pixel coordinates pts (N, 1, 2) from warped image space back to raw image space:
        p_raw = H_cv @ p_derotated
        """
        if pts is None or len(pts) == 0:
            return pts
        pts_2d = pts.reshape(-1, 2)
        pts_hom = np.column_stack([pts_2d, np.ones(len(pts_2d))])
        raw_hom = (H_cv @ pts_hom.T).T
        pts_raw = raw_hom[:, :2] / raw_hom[:, 2:]
        return pts_raw.reshape(-1, 1, 2).astype(np.float32)

    def compute_crop_percentage(self, H_cv):
        """
        Computes percentage of frame area lost to border (zero-source-pixel regions) under homography warp H_cv.
        """
        mask_src = np.ones((self.height, self.width), dtype=np.uint8) * 255
        mask_dst = cv2.warpPerspective(
            mask_src, H_cv, (self.width, self.height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0
        )
        crop_pct = float(np.sum(mask_dst == 0)) / float(self.width * self.height) * 100.0
        return crop_pct

    def compute_warp_angle(self, H_cv):
        """
        Extracts equivalent rotation angle in degrees from homography H_cv = K R_rel K_inv.
        """
        R_rel = self.K_inv @ H_cv @ self.K
        rotvec = R_scipy.from_matrix(R_rel).as_rotvec()
        return math.degrees(np.linalg.norm(rotvec))

# Forensic Audit: EIS Reference-Frame Strategy & Mechanism Analysis

## Executive Summary

This forensic code audit resolves the reference-frame strategy used by the Phase 2A EIS implementation (`src/eis_derotation.py`, `src/run_offline_vo.py`).

> [!IMPORTANT]
> **Audit Finding**: The current Phase 2A EIS implementation is strictly **FIXED-REFERENCE (Global Reference Frame Mode)**. 
> Every image frame $k$ at timestamp $t_k$ is warped to match the initial virtual camera orientation at $t_0$ ($R_{\text{ref}} = R_{\text{world\_cam\_opt}}(t_0)$).
>
> **Consequence for F9 Yaw Results**: In F9 (Yaw + Translation $\pm 30^\circ$), the warp magnitude at frame $k$ is determined by the **accumulated absolute yaw relative to $t_0$** (up to $30^\circ$), NOT the small frame-to-frame yaw delta ($\approx 0.5^\circ$). A $30^\circ$ absolute homography warp causes $>40\%$ valid image loss and extreme perspective stretching.
>
> **Status of Previous Conclusions**: The previous claim that *"wide yaw is an intrinsic boundary condition for EIS"* is **PREMATURE and UNVERIFIED FOR INCREMENTAL EIS**. What Phase 2A actually established is that **FIXED-REFERENCE EIS degrades under large accumulated yaw**. Incremental/pairwise derotation remains an un-evaluated experimental variable.

---

## 1. Code Path & Implementation Trace

### Reference Initialization (`src/eis_derotation.py`, lines 94–96)
```python
# Set default reference orientation to initial attitude sample (t_0)
r_body_0 = rotations[0].as_matrix()
self.R_ref = r_body_0 @ self.R_opt2gaz.T
```
- **Finding**: `self.R_ref` is set ONCE when `load_attitude_telemetry()` is called at the start of dataset processing.

### Per-Frame Warp Computation (`src/eis_derotation.py`, lines 127–130)
```python
def compute_homography(self, R_body_k, R_ref=None):
    if R_ref is None:
        R_ref = self.R_ref  # Uses fixed t_0 reference
    R_cam_opt_k = self.get_optical_attitude(R_body_k)
    R_rel_opt = R_cam_opt_k.T @ R_ref
    H_cv = self.K @ R_rel_opt @ self.K_inv
    return H_cv
```
- **Finding**: `derotate_image(img, t_sec)` is called inside `run_offline_vo.py` for every frame $k$ with default `R_ref=None`.
- `self.R_ref` is NEVER updated during the sequence.
- The rotation matrix $R_{\text{rel\_opt}} = R_{\text{cam\_opt\_k}}^T R_{\text{ref}}$ measures the full orientation delta between frame $k$ and initial frame 0.

---

## 2. Point-by-Point Audit Questions

| Audit Question | Code Audit Determination |
| :--- | :--- |
| **1. Reference initialization location** | `EISDerotator.load_attitude_telemetry()` in `src/eis_derotation.py` (lines 94–96). |
| **2. Reference variable name** | `self.R_ref` |
| **3. Fixed for entire sequence?** | **YES.** Initialized at $t_0$ and remains constant across all $N$ frames. |
| **4. Updated after every frame?** | **NO.** Never updated during sequence processing. |
| **5. Warp relative basis** | $R(\text{frame}_k)$ relative to $R(\text{reference at } t_0)$. |
| **6. Exact rotation matrix** | $R_{\text{rel\_opt}} = R_{\text{cam\_opt}}(t_k)^T R_{\text{cam\_opt}}(t_0)$ |
| **7. Exact homography for `warpPerspective`** | $H_{\text{cv}} = K R_{\text{rel\_opt}} K^{-1} = K (R_{\text{cam\_opt}}(t_k)^T R_{\text{cam\_opt}}(t_0)) K^{-1}$ |
| **8. Inverse rotation used?** | $R_{\text{cam\_opt\_k}}^T R_{\text{ref}}$ maps output reference coordinates to source frame $k$ coordinates for OpenCV perspective pulling. |
| **9. Coordinate convention** | World ENU ($+X$ East, $+Y$ North, $+Z$ Up), Optical ($+Z$ Fwd, $+X$ Right, $+Y$ Down), $R_{\text{opt2gaz}} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$. |
| **10. Warping both frames or single frame?** | Every frame $k$ is warped into the same fixed virtual reference frame $R_{\text{ref}}$. Thus any consecutive pair $(I_k, I_{k+1})$ are both in plane $R_{\text{ref}}$. |
| **11. Downstream VO virtual orientation** | **YES.** All frames presented to VO share the identical virtual camera orientation $R_{\text{ref}}$. |
| **12. Intrinsics $K$ constant after warp?** | **YES.** $H_{\text{cv}}$ maps through $K^{-1}$ and back through $K$, maintaining $(f_x, f_y, c_x, c_y)$ on output $1280 \times 960$ canvas. |
| **13. Cropping / Resizing?** | Output size is fixed at $1280 \times 960$. |
| **14. Border / FOV handling?** | `borderMode=cv2.BORDER_CONSTANT` with `borderValue=0` (black padding). |
| **15. Warp magnitude dependency** | **ACCUMULATED ABSOLUTE YAW RELATIVE TO $t_0$**. |

---

## 3. Synthetic Test Audit (`tests/test_eis_synthetic.py`)

- **Scope**: The synthetic regression test evaluated a 2-frame pair ($R_1 = I$, $R_2 = \text{rot}(3^\circ, 5^\circ)$).
- **Limitation**: It validated pairwise homography linear math for small angles ($\le 5^\circ$), but **did NOT simulate multi-hundred-frame trajectories with accumulated yaw up to $30^\circ$**.

---

## 4. Re-Interpretation of Phase 2A Findings

### What Phase 2A Evidence ESTABLISHES:
1. **Fixed-Reference EIS for Pitch/Roll Tilts (`F5`)**: When accumulated rotation relative to $t_0$ remains small ($\approx 2^\circ \dots 5^\circ$), Fixed-Reference EIS works exceptionally well and reproducibly improves VO metrics (+1.31% pose validity, lower tracking residual).
2. **Fixed-Reference EIS for Large Yaw (`F9`)**: When accumulated rotation relative to $t_0$ reaches $30^\circ$, Fixed-Reference EIS applies a massive $30^\circ$ homography warp, causing $>40\%$ border pixel loss and severe perspective stretching that degrades VO.

### What Phase 2A Evidence CANNOT Establish:
1. It does **NOT** establish whether **Incremental / Pairwise EIS** (where frame $k$ is warped relative to frame $k-1$) would succeed or fail on F9.
2. It does **NOT** establish that yaw derotation is an intrinsic mathematical impossibility.

---

## 5. Candidate Maneuver Space for Phase 2B (Pirouette)

- **Status**: **RE-OPENED / UNBOUNDED BY YAW PHOBIA**.
- Because the F9 failure in Phase 2A was caused by Fixed-Reference accumulation rather than proven incremental failure, Pirouette candidate maneuver design does NOT need to artificially exclude yaw-based active motion.

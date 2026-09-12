# Scientific Report: Forensic Audit of the Essential Matrix ($) Pipeline

## Executive Summary

This forensic audit investigates the monocular Visual Odometry (VO) pipeline in 
======================================================================
PHASE 0E — MONOCULAR VO EXECUTION REPORT
======================================================================
Tracking Pipeline      : KLT
Total Processed Frames : 300 (Achieved FPS: 30.40 Hz over 9.87s)
Output CSV Path        : /home/purab/Purab/Projects/ROS/results/vo_trajectory.csv
Streak Failure (Old)   : True (First at Frame #9)
Window Failure (New)   : True (First at Frame #0)
----------------------------------------------------------------------
INTERMEDIATE METRICS SUMMARY:
  Detected Features/Frame : Mean = 945.5, Min = 109, Max = 2000
  Matched Features/Frame  : Mean = 901.6, Min = 0, Max = 1991
  Essential Inliers/Frame : Mean = 61.9, Min = 0, Max = 1339
  Homography Inliers/Frame: Mean = 889.8, Min = 0, Max = 1718
  Inlier Ratio            : Mean = 9.20%, Min = 0.00%, Max = 97.73%
  Feature Survival Rate   : Mean = 96.37%, Min = 0.00%, Max = 100.00%
  Feature Velocity (Mean) : Mean = 4.41 px/fr, Max = 62.42 px/fr
  LK Tracking Error (Mean): Mean = 0.5718 px, Max = 6.8109 px
======================================================================


======================================================================
PHASE 0E — MONOCULAR VO EXECUTION REPORT
======================================================================
Tracking Pipeline      : KLT
Total Processed Frames : 300 (Achieved FPS: 30.40 Hz over 9.87s)
Output CSV Path        : /home/purab/Purab/Projects/ROS/results/vo_trajectory.csv
Streak Failure (Old)   : True (First at Frame #9)
Window Failure (New)   : True (First at Frame #0)
----------------------------------------------------------------------
INTERMEDIATE METRICS SUMMARY:
  Detected Features/Frame : Mean = 945.5, Min = 109, Max = 2000
  Matched Features/Frame  : Mean = 901.6, Min = 0, Max = 1991
  Essential Inliers/Frame : Mean = 61.9, Min = 0, Max = 1339
  Homography Inliers/Frame: Mean = 889.8, Min = 0, Max = 1718
  Inlier Ratio            : Mean = 9.20%, Min = 0.00%, Max = 97.73%
  Feature Survival Rate   : Mean = 96.37%, Min = 0.00%, Max = 100.00%
  Feature Velocity (Mean) : Mean = 4.41 px/fr, Max = 62.42 px/fr
  LK Tracking Error (Mean): Mean = 0.5718 px, Max = 6.8109 px
====================================================================== to identify why Essential Matrix inliers ($) collapsed to median 0–1 in both  (~81% collapse) and  (~79% collapse) during 0^\circ$ roll oscillation maneuvers (bash.5	ext{ Hz}$), despite healthy KLT feature tracking ($> 1800$ matched features, $> 99.6\%$ survival, $< 2.1	ext{ px}$ LK error, $> 1800$ Homography inliers).

### Key Discovery: The   Rejection Bug

The audit identified a **critical implementation artifact** in how OpenCV's  filters triangulated 3D point depths:

1. Monocular VO estimates relative translation normalized to unit length ($\|\hat{t}\| = 1.0	ext{ m}$).
2. In low-translation maneuvers (e.g. roll oscillation with per-frame physical baseline {	ext{true}} pprox 1.5	ext{ cm} = 0.015	ext{ m}$ at hover altitude {	ext{true}} pprox 2.5 - 6.0	ext{ m}$), unit-scale triangulated point depths scale proportionally:
   53127Z_{	ext{unit}} = Z_{	ext{true}} 	imes rac{\|\hat{t}\|}{\|t_{	ext{true}}\|} = 6.0	ext{ m} 	imes rac{1.0	ext{ m}}{0.015	ext{ m}} = 400.0	ext{ meters}53127
3. OpenCV's  has an internal default parameter **** meters (which caps allowed triangulated point depth at 0.0	ext{ meters}$).
4. Because {	ext{unit}} pprox 400.0	ext{ m} > 50.0	ext{ m}$, ** rejects 100% of the triangulated points during its cheirality/depth check**, returning .
5.  sets  (from ), overwriting the valid epipolar RANSAC inliers found by  (which had 500+ valid matches).

This created an **artificial bookkeeping collapse** in logged $ inliers.

---

## 1. Exact $ Pipeline Trace

File inspected: [src/minimal_vo.py](file:///home/purab/Purab/Projects/ROS/src/minimal_vo.py)

1. **Feature Tracking (, Lines 286–296)**:
   -  tracks points.
   - Points filtered by  to produce  and  (arrays of shape , ).
2. **Homography Diagnostic (Lines 310–316)**:
   - 
   - 
3. **Essential Matrix Estimation (Lines 320–325)**:
   - 
4. **Relative Pose Recovery & Inlier Bookkeeping (Lines 328–330)**:
   - 
   - **** (Assigns returned  count)
5. **State Update Condition (Line 332)**:
   -  accumulate pose, else keep previous points.

---

## 2. Audit of  Invocation

* **OpenCV Function**: 
* **Arguments**:
  - :  (, shape , raw pixel coordinates $)
  - :  (, shape , raw pixel coordinates $)
  - :  ( 	imes 3$  matrix)
  - :  ($)
  - : 
  - :  (pixel reprojection threshold)
* **Mathematical Consistency**: **CONSISTENT**. Raw pixel coordinates match the passed .  returns valid $ matrices and  inliers.

---

## 3. Audit of Camera Intrinsics ($)

* **Matrix $ Values**:
  53127\mathbf{{K}} = egin{{bmatrix}} 539.9363 & 0.0 & 640.0 \ 0.0 & 539.9364 & 480.0 \ 0.0 & 0.0 & 1.0 \end{{bmatrix}}53127
* **Resolution**: 280 	imes 960$ pixels.
* **Principal Point**:  = (640.0, 480.0)$ (Exact image center).
* **Focal Length Verification**:
  SDF horizontal FOV $	heta = 1.74	ext{ rad} pprox 99.7^\circ$.
  53127f = rac{W / 2}{	an(	heta / 2)} = rac{640}{	an(0.87)} = 539.9363	ext{ px}53127
* **Image Scaling / Resizing**: None.
* **Intrinsics Status**: **NO ISSUE FOUND** (Perfectly matches Gazebo camera sensor physics).

---

## 4. Audit of Camera Distortion

* **Distortion Model**: Ideal pinhole camera model in Gazebo  SDF (=k_2=p_1=p_2=k_3=0$).
* **Distortion Status**: **NO ISSUE FOUND** (Distortion is zero; raw pixel coordinates require no undistortion).

---

## 5. Audit of Mask Handling & Inlier Bookkeeping

*  returns  (uint8 array,  for epipolar inliers,  for outliers).
*  receives  and modifies it or returns .
* **Bookkeeping Flaw**:  is assigned from  returned by , **AFTER**  applies its internal  depth check.
* **Impact**:  inliers are overwritten by  cheirality/depth failures.

---

## 6. Audit of  Overload & 

* **Invocation**: 
* **OpenCV Default Parameter**:  (meters).
* **Mechanism**:
  For small translation baselines ( pprox 1.5	ext{ cm}$), unit scale normalization ($\|\hat{t}\| = 1.0$) inflates triangulated point depth {	ext{unit}}$ to $\sim 400	ext{ m}$.
  Because 00	ext{ m} > 50	ext{ m}$,  filters out 100% of valid points as exceeding , returning .

---

## 7. Camera Motion Geometry Check

* **Mounting**: Forward-facing camera (, link pose ).
* **Frame Alignment**: Optical axis {	ext{opt}}$ aligns with vehicle body $+X$ (forward).
* **Roll Dynamics**: Vehicle roll ($\phi$) rotates around body $+X$, causing pure in-plane optical axis rotation ({	ext{opt}}$) in the image plane around $.
* **Geometry Status**: **NO ISSUE FOUND** (Camera mounting matches expected physical motion).

---

## 8. Offline Re-estimation Test

* **CSV Log Inspection**: Pixel coordinates $ were **not logged** in  or .
* **Status**: **Replay cannot be performed directly from existing CSV logs**.

---

## 9. Numerical Synthetic Sanity Test

Synthetic 3D point cloud (=500$,  \in [2, 10]	ext{ m}$) under 0^\circ$ roll and .5	ext{ cm}$ baseline:

| Test Configuration |  Inliers |  Inliers (Default ) |  Inliers () |  Inliers |
| :--- | :---: | :---: | :---: | :---: |
| **Pure Translation (	ext{ cm}$, bash^\circ$)** | 500 / 500 | 33 / 500 | 500 / 500 | 372 / 500 |
| **Pure Translation (.5	ext{ cm}$, bash^\circ$)** | 500 / 500 | **0 / 500** | 500 / 500 | 447 / 500 |
| **Pure Roll (0^\circ$, bash	ext{ cm}$)** | 500 / 500 | **0 / 500** | 500 / 500 | 500 / 500 |
| **Roll 0^\circ$ + Trans .5	ext{ cm}$ (Baseline)** | 500 / 500 | **0 / 500** | **500 / 500** | 440 / 500 |
| **Roll 0^\circ$ + Trans 0	ext{ cm}* | 500 / 500 | 500 / 500 | 500 / 500 | 200 / 500 |

* **Synthetic Result**: Synthetic test **conclusively proved** that  finds 500/500 inliers, but  with default  drops inliers to 0 when translation is small (.5	ext{ cm}$). Setting  completely recovers all 500 inliers.

---

## 10. Compare Agriculture and 3dmap Implementation Conditions

* **Verification**:  and  runs used identical code, resolution (280 	imes 960$), intrinsics $, KLT parameters, and ROS node binaries.

---

## 11. Scientific Verdict

| Finding / Component | Status | Evidence |
| :--- | :---: | :--- |
| **A. KLT Implementation** | **NO ISSUE FOUND** | $> 1800$ matched features, $> 99.6\%$ survival rate, $< 2.1	ext{ px}$ LK error. |
| **B. Camera Intrinsics ($)** | **NO ISSUE FOUND** | =539.94, f_y=539.94, c_x=640, c_y=480$ exactly matches SDF physics. |
| **C. Distortion Handling** | **NO ISSUE FOUND** | Pinhole camera model in SDF has zero distortion; undistortion unnecessary. |
| **D. findEssentialMat Config** | **NO ISSUE FOUND** | Raw pixel coordinates $ with $ matrix correctly compute epipolar inliers. |
| **E. findEssentialMat Mask Handling** | **POSSIBLE ISSUE** | Epipolar inliers from  were overwritten by  count. |
| **F. recoverPose Handling** | **SUPPORTED (BUG FOUND)** | Default  rejects unit-scale triangulated points ({	ext{unit}} pprox 400	ext{m}$) during small-baseline flight, producing artificial $ inlier collapse (). |
| **G. Camera-Axis Geometry** | **NO ISSUE FOUND** | Forward-facing camera translates 0^\circ$ body roll into in-plane image rotation around $. |
| **H. Estimator Config Artificial Collapse** | **SUPPORTED** | Synthetic sanity test proved   causes 100% inlier rejection under small-baseline roll motion. |

---

## 12. Next Experiment Decision

### **PATH 1: Implementation/Configuration Issue Found**

* **Justification**: The forensic audit identified an explicit implementation bug (  depth rejection under unit-scale translation) that artificially zeroed $ inlier counts during small-baseline roll maneuvers.
* **Next Steps**:
  1. Fix the  invocation in  by passing an appropriate  parameter (or logging  epipolar inliers separately from  cheirality inliers).
  2. Verify the fix offline using a synthetic test script before conducting any future characterization or validation flights.

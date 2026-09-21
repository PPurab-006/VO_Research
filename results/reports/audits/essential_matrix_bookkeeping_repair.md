# Research Note: Essential Matrix ($E$) Inlier Bookkeeping Repair

## 1. Previous Metric Semantics

Previously in `src/minimal_vo.py`, the metric logged as `num_inliers` was recorded as:

```python
E, mask_E = cv2.findEssentialMat(...)
if E is not None and E.shape == (3, 3):
    inliers_count, R_opt, t_opt, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask_E)
    num_inliers = int(inliers_count)  # <-- Assigned from recoverPose()
```

Under this implementation, `num_inliers` recorded the number of points passing **`cv2.recoverPose()` cheirality/depth filtering**, rather than the RANSAC epipolar inliers computed by `cv2.findEssentialMat()`.

---

## 2. The Bug / Artifact Mechanism

OpenCV's `cv2.recoverPose()` normalizes translation to unit vector length ($\|\hat{t}\| = 1.0\text{ m}$). For flight maneuvers with small physical baselines ($t_{\text{true}} \approx 1.5\text{ cm} = 0.015\text{ m}$ at hover altitude $Z_{\text{true}} \approx 2.5 - 6.0\text{ m}$):

1. **Unit-Scale Depth Inflation**:
   $$Z_{\text{unit}} = Z_{\text{true}} \times \frac{\|\hat{t}\|}{\|t_{\text{true}}\|} = 6.0\text{ m} \times \frac{1.0\text{ m}}{0.015\text{ m}} = 400.0\text{ meters}$$
2. **OpenCV Default Depth Threshold**:
   OpenCV's `recoverPose()` has an internal default parameter **`distanceThresh = 50.0`** meters (which caps allowed triangulated point depth at $50.0\text{ m}$).
3. **100% Inlier Rejection**:
   Because $Z_{\text{unit}} \approx 400.0\text{ m} > 50.0\text{ m}$, `recoverPose()` rejects **100% of points during depth filtering**, returning `inliers_count = 0`.
4. **Bookkeeping Overwrite**:
   `minimal_vo.py` set `num_inliers = int(inliers_count)` (0), completely overwriting the $500+$ valid epipolar RANSAC inliers computed by `findEssentialMat()`.

---

## 3. New Metric Semantics

The VO pipeline in [src/minimal_vo.py](../../../src/minimal_vo.py) has been updated with strict metric separation:

1. **`num_inliers_E`**:
   The exact count of entries in `mask_E` equal to 1 returned directly by `cv2.findEssentialMat()`:
   $$\text{num\_inliers\_E} = \sum (\text{mask\_E} == 1)$$
2. **`num_inliers_pose`**:
   The integer count returned by `cv2.recoverPose()`, representing points passing cheirality ($Z > 0$) and depth threshold filtering ($Z < \text{distanceThresh}$).
3. **`num_inliers` (Backward Compatibility)**:
   Explicitly mapped to `num_inliers_E`:
   $$\text{num\_inliers} \equiv \text{num\_inliers\_E}$$

Both `num_inliers_E` and `num_inliers_pose` are now recorded as separate columns in the VO CSV output.

---

## 4. Why `recoverPose` Count Must Not Be Conflated with `findEssentialMat` Count

* **`findEssentialMat()` RANSAC count (`num_inliers_E`)**: Evaluates geometric algebraic consistency ($x_2^T E x_1 \approx 0$) under noise. It measures whether a valid 3D epipolar geometry exists for the point correspondences.
* **`recoverPose()` count (`num_inliers_pose`)**: Evaluates physical cheirality ($Z > 0$) and metric depth constraints ($Z < \text{distanceThresh}$). Under unit scale ($\|\hat{t}\| = 1.0$), low physical translation inflates unit depth beyond default bounds, causing `recoverPose()` to return 0 even when `findEssentialMat()` successfully found a valid $E$ matrix with hundreds of inliers.

Conflating these metrics hid the fact that `findEssentialMat()` was succeeding mathematically while `recoverPose()` was rejecting the pose due to unit-scale depth thresholding.

---

## 5. Synthetic Regression Test Results

Test File: [tests/test_essential_matrix_bookkeeping.py](../../../tests/test_essential_matrix_bookkeeping.py)

Command: `python3 -m unittest tests/test_essential_matrix_bookkeeping.py`

| Test Case | `num_inliers_E` (`findEssentialMat`) | `num_inliers_pose` (`recoverPose` default `distanceThresh=50`) | Pose $R$ Valid | Pose $t$ Valid | Result |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **1. 1.5 cm Translation (0° roll)** | **500 / 500** | **0 / 500** | True | True | **PASSED** |
| **2. 10° Roll + 1.5 cm Trans (Baseline)** | **500 / 500** | **0 / 500** | True | True | **PASSED** |
| **3. 20 cm Translation (0° roll)** | **500 / 500** | **500 / 500** | True | True | **PASSED** |

The regression test proves that `num_inliers_E` and `num_inliers_pose` are now cleanly decoupled and distinguishable.

---

## 6. What Remains Unresolved About `recoverPose` `distanceThresh`

While metric bookkeeping is repaired, **pose accumulation logic in `minimal_vo.py` still relies on `num_inliers_pose >= 8`**:

```python
if num_inliers_pose >= 8:
    self.curr_pos += self.curr_rot @ t_gaz
    self.curr_rot = self.curr_rot @ R_gaz
```

Because default `distanceThresh=50.0` rejects unit-scale triangulated points for baselines $t < 2\text{ cm}$, pose accumulation is skipped when `num_inliers_pose < 8`.

Future Phase 0E steps should address whether to pass an appropriately scaled `distanceThresh` parameter to `recoverPose()` or adjust the pose accumulation condition, after establishing baseline metrics with the repaired bookkeeping.

---

## 7. Reinterpretation of Historical Experimental Datasets

* Historical CSV datasets (`results/roll_validation_vo.csv`, `results/roll_3dmap_vo.csv`) **MUST NOT** be modified or rewritten.
* Previous reported `num_inliers` values in those datasets represent `recoverPose()` cheirality counts (`num_inliers_pose`).
* These historical figures **MUST NOT** be interpreted as evidence that `findEssentialMat()` itself failed or produced zero epipolar inliers.

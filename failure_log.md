# Failure Log & Technical Journal

## Template
| Date | Problem / Goal | Hypothesis | What I Changed | Result | What I Learned |
| :--- | :--- | :--- | :--- | :--- | :--- |

---

## Master Log Summary Index

| Entry # | Date | Topic / Focus | Original Finding | Final Reinterpreted Status / Verdict |
| :---: | :---: | :--- | :--- | :--- |
| **1** | 2026-08-20 | Environment Scaffolding | Base infrastructure setup | **PASS** — Environment frozen |
| **2** | 2026-08-22 | Phase 0A Camera Sensor | 30.4 Hz, zero-distortion pinhole | **PASS** — Intrinsics verified |
| **3** | 2026-08-22 | Phase 0A Ground Truth Pose | 50 Hz World ENU pose | **PASS** — ENU coordinates verified |
| **4** | 2026-08-22 | Phase 0B Synchronization | SimTime NN matching ($\le 20\text{ms}$) | **PASS** — 99.02% matched within 20ms |
| **5** | 2026-08-22 | Phase 0C Minimal Monocular VO | GFTT + KLT + 5-pt Essential RANSAC | **PASS** — Poses recovered at unit scale |
| **6** | 2026-08-31 | World Texture Upgrade | Low inliers on featureless world | **RESOLVED** — Added textured materials |
| **7** | 2026-08-31 | Phase 0D Trajectory Alignment | Sim(3) Umeyama alignment | **PASS** — Scale $s=0.0745$, ATE $0.676\text{m}$ |
| **8** | 2026-09-01 | Z-Axis Sawtooth Artifact | Teleportation without thrust | **RESOLVED** — Switched to PX4 offboard |
| **9** | 2026-09-01 | OFFBOARD Handshake | Altitude plateau at 1.0m | **RESOLVED** — Pre-stream setpoints + mode cmd |
| **10** | 2026-09-01 | Planar RANSAC & Distance Sweet-spot | Non-monotonic inlier ratio vs range | **DOCUMENTED** — 3–5m sweet spot in textured.sdf |
| **11** | 2026-09-02 | `recoverPose` Translation Sign | Negative Pearson correlation | **RESOLVED** — Negated step translation |
| **12** | 2026-09-02 | Physics Engine Collision Test | Visual proximity pass-through | **PASS** — Wall collision physics active |
| **13** | 2026-09-02 | `boxworld_obstacles_tight` Migration | Rendering black in OGRE2 | **RESOLVED** — Material scripts upgraded to PBR |
| **14** | 2026-09-03 | CPU Rendering Jitter | Software rendering frame drops | **RESOLVED** — Enabled NVIDIA GPU offload |
| **15** | 2026-09-03 | Inlier Ratio vs Count Distinction | Apparent low inliers in bounded arena | **SUPERSEDED** — Metric conflation (`dt=50` rejection) |
| **16** | 2026-09-03 | Sweep A: Pure Yaw Escalation | Yaw rate vs frame drop & inliers | **SUPERSEDED** — E-RANSAC healthy; $t=0$ depth filter artifact |
| **17** | 2026-09-04 | Forensic Pipeline Repair & Phase 0E Finalization | Conflated metrics & `dt=50` rejection | **PASS / REPAIRED** — `num_inliers_E` decoupled; `dt=1000` adopted; validated in real 10° flight; Phase 0E complete |
| **18** | 2026-09-05 | `recoverPose` Envelope Calibration | `dt=50` rejection under small baselines | **CALIBRATED** — `dt=1000` adopted for project envelope ($Z/t \le 1000$); project-specific calibration caveat preserved |
| **19** | 2026-09-05 | GT Timestamp Provenance | GT wall-clock fallback (`time.time()`) | **RESOLVED** — Repaired to ROS sim time; historical GT aligned via relative time offset |
| **20** | 2026-09-05 | GT Derivative Micro-Timing Spikes | $45\text{m/s}$ speed & $721^\circ/\text{s}$ yaw spikes | **RESOLVED** — Masked $dt < 2\text{ms}$ & unwrapped angles; removes timing artifacts, not real motion |
| **21** | 2026-09-05 | Historical HOVER Active Window Bug | Mixed clocks produced $+1.289\text{s}$ offset | **CORRECTED** — Canonical sim-time window selection applied; historical baseline retained for provenance |
| **22** | 2026-09-05 | HOVER Zero-Parallax Degeneracy | Low pose validity ($53\%$) in hover | **DIAGNOSTIC BASELINE** — Static hover lacks translation parallax; static baseline, not moving-flight Pose/E failure |
| **23** | 2026-09-05 | P01/F1 Short-Duration Anomaly | P01 had only $3.29\text{s}$ active motion | **RESOLVED** — P01 marked historical duration anomaly; confirmation F1 R1–R3 (20s) provided full replacement |
| **24** | 2026-09-05 | P03/P04 Attitude & Braking Excursions | Peak roll/pitch spikes ($19^\circ$) | **CAVEAT** — Excursions caused by terminal position-hold braking, not cruise severity; preserved as analysis caveat |
| **25** | 2026-09-06 | Confirmation F9 R3 Execution Anomaly | F9 R3 hit 8s takeoff timeout ($1.70\text{m}$) | **EXECUTION ANOMALY** — Active motion never started; VO metrics must NOT be interpreted as F9 VO degradation |
| **26** | 2026-09-06 | Confirmation F10 R1–R3 Execution Issue | F10 R1–R3 hit 8s takeoff timeout ($1.7\text{m}$) | **EXECUTION CAVEAT** — Intended L3 aggressive motion never executed; metrics reflect climb/hover, not L3 VO degradation |
| **27** | 2026-09-06 | Sequential Multi-Run Degradation Audit | Tested chronological batch ($1 \to 21$) | **NO SEQUENTIAL DEGRADATION** — Bounded: No detectable sequential VO degradation under tested 21-run SITL conditions |
| **28** | 2026-09-06 | Within-Run Temporal Degradation Audit | Tested active window quartiles ($Q1 \to Q4$) | **NO WITHIN-RUN DEGRADATION** — Inlier ratio stable ($\ge 99.5\%$) across quartiles; no temporal degradation in valid runs |
| **29** | 2026-09-06 | Takeoff Timeout Action Item | 8s takeoff timeout too tight for SITL | **ACTION ITEM** — Forensic audit recommends `TAKEOFF_TIMEOUT_SEC` $8\text{s} \to 12\text{s}$ before Phase 2; documented |
| **30** | 2026-09-06 | Phase 1 Programmatic Closure | Baseline characterization complete | **COMPLETE WITH DOCUMENTED CAVEATS** — Phase 1 closed; Phase 2 **READY** |

---

## Log Entries

### Entry 1: 2026-08-20 — Workspace & Environment Scaffolding
- **Date**: 2026-08-20
- **Problem / Goal**: Set up Phase 0 infrastructure for Robust Monocular VO project based on `SSRF_Research_Plan.docx`. Detect and freeze environment specs, create directory layout, install missing dependencies (`evo`, `scipy`), and establish baseline repository.
- **Hypothesis**: Documenting exact environment specifications (Ubuntu 26.04, ROS2 Lyrical, Gazebo 10.4.0, PX4 SITL `g8aba32c862`) and structuring Phase 0 around strict hard-gate exit criteria (0A–0E) prevents configuration drift and non-reproducible bugs.
- **What I Changed**: Created initial project structure (`research_question.md`, `methodology.md`, `environment.md`, `failure_log.md`, and directory placeholders for `src/`, `configs/`, `experiments/`, `results/`, `plots/`, `report/`). Installed and verified python dependencies (`evo` 1.37.0, `scipy` 1.18.0, `colcon-common-extensions`). Integrated explicit design principles and 0A–0E Phase 0 milestones from `SSRF_Research_Plan.docx`.
- **Result**: Phase 0 scaffolding established, dependencies installed, environment frozen, and repository aligned with SSRF Research Plan.
- **What I Learned**: Environment versions pinned: Ubuntu 26.04 LTS, ROS2 Lyrical, Gazebo 10.4.0, PX4 SITL `v1.18.0-beta1-209-g8aba32c862`, Python 3.14.4, `evo` 1.37.0, `scipy` 1.18.0.

### Entry 2: 2026-08-22 — Phase 0A Sensor Verification (Camera)
- **Date**: 2026-08-22
- **Problem / Goal**: Phase 0A sensor verification — verify Gazebo simulated camera publication on ROS 2, capture topic metadata (encoding, resolution, rate, timestamp monotonicity), extract camera intrinsics, determine frame convention, and create reproducible verification script.
- **Hypothesis**: The PX4 SITL `gz_x500_mono_cam` model publishes Gazebo transport camera topics which can be bridged to ROS 2 via `ros_gz_bridge` with ~30 Hz publish rate, zero-distortion pinhole intrinsics, and monotonic timestamps.
- **What I Changed**:
  - Executed PX4 SITL + Gazebo simulation with `gz_x500_mono_cam` model.
  - Configured `ros_gz_bridge parameter_bridge` for `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image` and `/camera_info`.
  - Created `src/verify_camera.py` script to subscribe to camera topic, save 20 frames to disk, record frame timestamps to CSV, and evaluate rate/monotonicity.
  - Created `src/launch_camera_sim.sh` launch helper script.
- **Result**:
  - **Topic Name**: `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`
  - **Message Type**: `sensor_msgs/msg/Image`
  - **Encoding**: `rgb8`
  - **Resolution**: 1280 x 960
  - **Publish Rate**: ~30.4 Hz (measured via `ros2 topic hz` over 300+ window and via verification script).
  - **Timestamps**: Strictly monotonic simulation time from Gazebo clock.
  - **Intrinsics**: Found on `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info`. Zero distortion (`plumb_bob`, D=[0,0,0,0,0]), fx=539.936, fy=539.936, cx=640.0, cy=480.0 (derived from `horizontal_fov`=1.74 rad).
  - **Frame Convention**: Drone body `base_link` and `camera_link` share Gazebo frame orientation (+X Forward, +Y Left, +Z Up). Standard CV optical frame transform is $X_{opt}=-Y, Y_{opt}=-Z, Z_{opt}=X$.
- **What I Learned**:
  - Gazebo Sim requires `ros_gz_bridge` to bridge transport topics to ROS 2.
  - `camera_info` is natively generated by Gazebo camera sensor.
  - Verification script successfully captured 20 frames to `results/camera_verification` with `frame_timestamps.csv`.

### Entry 3: 2026-08-22 — Phase 0A Ground-Truth Pose Verification
- **Date**: 2026-08-22
- **Problem / Goal**: Phase 0A ground-truth pose verification — identify Gazebo simulator ground-truth pose topic for `x500_mono_cam_0`, verify static stability, perform motion sanity test, determine precise coordinate conventions (ENU/NED, quaternion order, camera-to-world transform), and write reproducible verification script.
- **Hypothesis**: Gazebo publishes true simulator pose on `/world/default/dynamic_pose/info`, which bridges via `ros_gz_bridge` as `tf2_msgs/msg/TFMessage` at 50 Hz in World ENU coordinates with `(x, y, z, w)` quaternion structure.
- **What I Changed**:
  - Identified Gazebo ground-truth pose topic `/world/default/dynamic_pose/info`.
  - Added parameter bridge `/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` to `src/launch_camera_sim.sh`.
  - Conducted static sanity test (stationary drone on ground: $x=0.0000\text{m}, y=0.0000\text{m}, z=-0.0130\text{m}$, zero rotation, std dev = 0.000000m).
  - Conducted motion sanity test (+2.0m forward shift along World $+X$ axis and MAVLink flight command). Confirmed $+X$ increases during forward motion.
  - Derived exact coordinate convention and body-to-camera optical transformation matrix $R_{\text{gazebo}\rightarrow\text{optical}}$.
  - Implemented `src/verify_ground_truth.py` script to subscribe to pose topic, log $N$ seconds to CSV, and output summary metrics.
- **Result**:
  - **Topic Name**: `/world/default/dynamic_pose/info`
  - **Message Type**: `tf2_msgs/msg/TFMessage` (bridged from `gz.msgs.Pose_V`)
  - **Publish Rate**: 50.0 Hz (measured 50.97 Hz over 255 samples)
  - **World Frame**: ENU (East-North-Up). $+X$ = Forward/East, $+Y$ = Left/North, $+Z$ = Up.
  - **Quaternion Order**: `(x, y, z, w)` in ROS 2 `geometry_msgs/msg/Quaternion` / `TransformStamped`.
  - **Static Sanity**: $x=0.0000\text{m}, y=0.0000\text{m}, z=-0.0130\text{m}$, $q=(0, 0, 0, 1)$, noise/drift = 0.000000m.
  - **Motion Sanity**: Moving drone forward increases $+X$. Altitude gain increases $+Z$.
  - **Camera Transform**: $R_{\text{gazebo}\rightarrow\text{optical}} = \begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & -1 \\ 1 & 0 & 0 \end{bmatrix}$, $q_{\text{opt}} = (-0.5, 0.5, -0.5, 0.5)$.
- **What I Learned**:
  - Simulator ground-truth pose uses World ENU convention, whereas PX4 internal flight controller operates in Local NED. Ground-truth evaluation must maintain ENU convention.
  - ROS 2 quaternion fields are ordered `(x, y, z, w)`.
  - Verification script successfully logged 255 pose samples to `results/ground_truth_verification/ground_truth_poses.csv`.

### Entry 4: 2026-08-22 — Phase 0B Synchronization Verification
- **Date**: 2026-08-22
- **Problem / Goal**: Phase 0B synchronization verification — measure and document actual timestamp alignment between 30 Hz camera frames and 50 Hz ground-truth pose stream during dynamic motion, verify clock source alignment, establish a justified sync tolerance threshold, and write reproducible script `src/verify_sync.py`.
- **Hypothesis**: Both streams are driven by Gazebo Simulation Time (`world->SimTime()`). Nearest-neighbor timestamp matching between 30 Hz camera frames and 50 Hz ground-truth poses has a theoretical maximum quantization gap of $\le 10.0\text{ ms}$ ($\frac{20\text{ ms}}{2}$). Setting a tolerance threshold of $\le 20.0\text{ ms}$ (one ground-truth sampling period) accounts for inter-thread phase drift and pairs >99% of frames cleanly.
- **What I Changed**:
  - Implemented `src/verify_sync.py` to record both topics simultaneously, execute nearest-neighbor matching $|t_{\text{cam}} - t_{\text{pose}}|$, calculate gap distribution statistics (min/max/mean/median/std), generate gap histogram, and export CSV log to `results/sync_verification/sync_matches.csv`.
  - Verified clock alignment: confirmed both topics use Gazebo Simulation Time when `use_sim_time: True` is enabled with `/clock` bridged.
  - Recorded 10s motion run (305 camera frames, 1024 ground-truth poses).
- **Result**:
  - **Clock Source**: Both streams are driven by Gazebo Simulation Time (`gz::sim::World::SimTime()`).
  - **Matched Pairs**: 305 / 305 camera frames matched.
  - **Gap Statistics**:
    - Minimum Gap: $0.0000\text{ ms}$
    - Maximum Gap: $84.0000\text{ ms}$ (initial sim startup transient)
    - Mean Gap: $6.1115\text{ ms}$
    - Median Gap: $4.0000\text{ ms}$
    - Standard Deviation: $6.6224\text{ ms}$
  - **Sync Tolerance Threshold**: **$20.0\text{ ms}$** (one 50 Hz ground-truth period).
  - **Tolerance Pass Rate**: **99.02%** (302 / 305 frames matched within $\le 20.0\text{ ms}$).
- **What I Learned**:
  - Nearest-neighbor timestamp matching using simulation time cleanly handles frequency mismatch (~30 Hz vs ~50 Hz) without relying on arrival-order assumptions.
  - A tolerance threshold of 20.0 ms matches 99%+ of valid frames while detecting any sim pauses or step glitches.

### Entry 5: 2026-08-22 — Phase 0C Minimal Visual Odometry (VO) Implementation
- **Date**: 2026-08-22
- **Problem / Goal**: Phase 0C minimal VO implementation — implement an un-fused monocular Visual Odometry ROS 2 node (`src/minimal_vo.py`), process camera stream end-to-end (feature detection, tracking/matching, 5-point Essential Matrix RANSAC, relative pose recovery, frame transformation from CV Optical to World ENU, and trajectory accumulation), log per-frame intermediate metrics (detected features, matched features, RANSAC inliers, inlier ratio, relative motion vectors), handle monocular scale ambiguity explicitly, and export raw unaligned trajectory to `results/vo_trajectory.csv`.
- **Hypothesis**: Monocular VO recovers 6-DoF relative motion direction $\|t\| = 1.0$ and rotation $R$ per frame. While scale ambiguity prevents metric distance recovery without alignment (Phase 0D), the accumulated unit-scale trajectory will qualitatively follow the motion profile.
- **What I Changed**:
  - Developed `src/minimal_vo.py` supporting both Pyramidal KLT Optical Flow tracking and ORB feature descriptor matching.
  - Implemented 5-point RANSAC Essential Matrix estimation (`cv2.findEssentialMat`) and relative pose recovery (`cv2.recoverPose`).
  - Implemented explicit coordinate transform $R_{\text{opt2gaz}} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$ to map CV optical frame relative steps into World ENU coordinates.
  - Configured publishing of `/vo/pose` (`geometry_msgs/msg/PoseStamped`) and `/vo/path` (`nav_msgs/msg/Path`).
  - Configured logging of all intermediate metrics (`num_detected`, `num_matched`, `num_inliers`, `inlier_ratio`, `rel_tx`, `rel_ty`, `rel_tz`, `rel_rot_deg`) to `results/vo_trajectory.csv`.
  - Executed end-to-end run on 250 frames of motion (straight flight + curve).
- **Result**:
  - **Processed Frames**: 250 frames.
  - **Intermediate Metrics Summary**:
    - Detected Features: Mean = 177.0 / frame (min: 74, max: 2000)
    - Matched Features: Mean = 82.6 / frame (min: 0, max: 204)
    - RANSAC Inliers: Mean = 10.4 / frame (min: 0, max: 71)
    - Inlier Ratio: Mean = 6.19% (max: 48.30%)
  - **Raw Output Trajectory**: Saved to `results/vo_trajectory.csv`.
  - **Visual Motion Tracking Evaluation**: The integrated unit-scale VO trajectory tracks the forward heading along $+X$ ($+0.69\text{ units}$) with lateral drift along $-Y$ ($-5.57\text{ units}$) and $-Z$ ($-3.11\text{ units}$). As expected for un-aligned monocular VO, drift accumulates over time and scale is arbitrary (unit-norm steps), but the pipeline successfully converts raw image stream into estimated 6-DoF poses.
- **What I Learned**:
  - Pure monocular VO with `cv2.recoverPose` produces unit-length translation vectors ($\|t\| = 1.0$). Attempts to hardcode or guess scale are mathematically invalid; scale must be resolved via Sim(3) alignment against ground truth (Phase 0D).
  - Degenerate motion (zero parallax when drone is stationary) yields 0 Essential Matrix RANSAC inliers. Handling zero-motion frames by preserving previous pose prevents unbounded numeric divergence.
  - GFTT + Pyramidal KLT Optical Flow provides smoother, more continuous feature tracking across consecutive frames in synthetic simulation environments than standalone ORB descriptor matching.

### Entry 6: 2026-08-31 — Phase 0C World-Texture Fix & Resumption
- **Date**: 2026-08-31
- **Problem / Goal**: Fix low inlier ratio in Phase 0C (mean 6.19%, 141 zero-inlier frames out of 250) caused by featureless default Gazebo world (flat texture-less cubes on flat texture-less ground plane). Add real surface textures and visually varied objects without tuning VO algorithm parameters. Note: Session was interrupted by a system memory crash (unrelated to simulation) and resumed cleanly after baseline verification (no orphaned processes, clean git status).
- **Hypothesis**: Replacing the flat featureless ground plane and monochrome cubes with lightweight textured materials (high-contrast checkerboard/noise grid ground plane, geometric block textures on objects, ArUco tag, helipad, and rover model at varying distances) will provide rich corners for optical flow tracking, increasing matched features and RANSAC inliers dramatically without altering VO algorithm parameters.
- **What I Changed**:
  - Verified clean baseline: checked for orphaned processes (`ps aux | grep -E "gz|px4"` — none found), verified file consistency, and checked `git status`.
  - Created lightweight texture generator `configs/textures/generate_textures.py` generating high-contrast 512x512 ground plane texture (`ground_texture.png`, 629 KB) and 256x256 object texture (`box_texture.png`, 35 KB).
  - Created world SDF `configs/textured.sdf` and installed to Gazebo worlds directory (`Tools/simulation/gz/worlds/textured.sdf`).
  - Added textured materials to ground plane and existing cubes, plus additional visually varied objects at varying distances (`arucotag`, `helipad`, `r1_rover`).
  - Maintained identical drone, camera intrinsics, and VO pipeline (`minimal_vo.py`) for clean A/B comparison.
  - Executed VO pipeline over 250 frames on the new textured world and saved output trajectory to `results/vo_trajectory_textured.csv`.
- **Result**:
  - **A/B Comparison (Original vs. Textured World)**:
    - **Total Frames**: 250 vs. 250
    - **Mean Inlier Ratio**: **6.19%** $\rightarrow$ **39.80%** (+33.61% absolute gain, **6.4x improvement**)
    - **Max Inlier Ratio**: 48.30% $\rightarrow$ **100.00%**
    - **Mean Detected Features**: 176.98 $\rightarrow$ **1053.32 / frame** (**5.9x improvement**)
    - **Mean Matched Features**: 82.59 $\rightarrow$ **661.31 / frame** (**8.0x improvement**)
    - **Mean RANSAC Inliers**: 10.42 $\rightarrow$ **309.16 / frame** (**29.6x improvement**)
    - **Zero-Match Frames**: 128 (51.2%) $\rightarrow$ **41 (16.4%)**
    - **Zero-Inlier Frames**: 141 (56.4%) $\rightarrow$ **83 (33.2%)**
  - Output saved to `results/vo_trajectory_textured.csv`.
- **What I Learned**:
  - The primary bottleneck of the initial low inlier ratio was environmental feature scarcity (flat monochrome geometry), not VO algorithm parameters.
  - Adding lightweight high-contrast surface textures and multi-distance visual objects increases mean RANSAC inliers per frame from ~10 to ~309 and mean inlier ratio from 6.19% to 39.80%.
  - Zero-inlier frames only occur during initial stationary/takeoff phases where inter-frame motion is sub-pixel (zero parallax degeneracy for 5-point Essential Matrix RANSAC). Once in motion, frame tracking reaches 95–100% inlier ratios.

### Entry 7: 2026-08-31 — Phase 0D Trajectory Alignment & Evaluation Pipeline
- **Date**: 2026-08-31
- **Problem / Goal**: Phase 0D trajectory alignment and quantitative evaluation. Develop a reusable evaluation script (`src/evaluate_trajectory.py`) using `evo` 1.37.0 to perform timestamp synchronization (20ms tolerance threshold from Phase 0B), execute Sim(3)/Umeyama 7-degree-of-freedom trajectory alignment (rotation, translation, and metric scale recovery), compute Absolute Trajectory Error (ATE) and Relative Pose Error (RPE), and export overlaid trajectory and error-over-time plots to `plots/`.
- **Hypothesis**: Since un-fused monocular VO recovers relative translation only up to an arbitrary unit scale ($\|t\| = 1.0$), Sim(3) Umeyama alignment will estimate a positive metric scale factor $s > 0$ and map the VO trajectory into World ENU coordinates, enabling rigorous ATE/RPE benchmarking.
- **What I Changed**:
  - Implemented `src/evaluate_trajectory.py` supporting command-line arguments (`--vo-csv`, `--gt-csv`, `--output-dir`, `--tolerance-ms`).
  - Integrated nearest-neighbor timestamp matching logic from `src/verify_sync.py` with 20.0 ms tolerance threshold (250 / 250 frames synchronized, 0.00 ms mean gap).
  - Executed Sim(3) Umeyama alignment via `evo` (`evo.core.geometry.umeyama_alignment` and `PoseTrajectory3D.align`).
  - Computed ATE translation metrics (RMSE, mean, median, std, min, max) and RPE metrics for delta = 1 frame and delta = 10 frames via `evo.core.metrics`.
  - Generated and saved two plots: `plots/trajectory_overlay.png` (overlaid X-Y top-down trajectory view) and `plots/ate_error_over_time.png` (per-frame ATE error over time).
- **Result**:
  - **Estimated Sim(3) Alignment Parameters**:
    - **Scale Factor ($s$)**: **0.074521** (Sanity Check: **PASS** — valid positive metric scale factor)
    - **Translation Vector ($t$)**: $[1.0381, -0.3830, 1.8447]\text{ m}$
    - **Rotation Matrix ($R$)**: Valid 3x3 orthogonal rotation matrix
  - **Absolute Trajectory Error (ATE)**:
    - **RMSE**: **0.6758 m**
    - **Mean**: **0.6089 m**
    - **Median**: **0.6018 m**
    - **StdDev**: **0.2931 m**
    - **Min / Max**: **0.0386 m / 1.3653 m**
  - **Relative Pose Error (RPE)**:
    - **Delta = 1 Frame**: RMSE = **0.0678 m**, Mean = **0.0626 m**, Median = **0.0785 m**, StdDev = **0.0261 m**
    - **Delta = 10 Frames**: RMSE = **0.4440 m**, Mean = **0.4320 m**, Median = **0.4060 m**, StdDev = **0.1027 m**
  - **Visual Alignment Evaluation**: The Sim(3) aligned VO trajectory cleanly tracks the ground-truth flight path profile along the forward motion (+X) and curved lateral displacement (+Y). The evaluation pipeline functions correctly and produces reproducible plots in `plots/`.
- **What I Learned**:
  - Sim(3) Umeyama alignment accurately estimates the scale factor ($s = 0.074521$) required to map raw un-scaled monocular VO poses into metric coordinates.
  - The reusable evaluation pipeline (`src/evaluate_trajectory.py`) provides automated ATE and RPE calculation and plot generation for all future experiments from Phase 1 onward.
  - All Phase 0 exit criteria (0A–0D) are now fully satisfied.

### Entry 8: 2026-09-01 — Bug 3: Z-Axis Sawtooth from Teleportation & Gravity
- **Date**: 2026-09-01
- **Problem / Goal**: Resolve Z-axis sawtooth pattern in ground-truth trajectory logs during Phase 0 trajectory generation.
- **Hypothesis**: `animate_drone.py` used Gazebo `set_pose` teleportation on an un-armed drone model. Between teleportation service calls, Gazebo physics and gravity caused freefall acceleration, producing severe sawtooth altitude drops in ground truth.
- **What I Changed**: Switched trajectory generation from script-based model teleportation to real PX4 offboard flight with armed motors via `fly_trajectory.py`.
- **Result**: Ground-truth Z-axis smoothness restored without sawtooth drops (max step change $\Delta z = 0.011\text{ m}$, std dev $0.0028\text{ m}$).
- **What I Learned**: Teleporting Gazebo models without active flight controller thrust causes freefall physics artifacts between frames; all VO evaluation flights must use real PX4 offboard flight commands.

### Entry 9: 2026-09-01 — Bug 4: OFFBOARD Mode Handshake & MAVLink `type_mask` Fix
- **Date**: 2026-09-01
- **Problem / Goal**: Fix offboard trajectory flight plateauing at $\sim 1.0\text{ m}$ altitude instead of climbing to the commanded $\sim 2.0\text{ m}$ cruise altitude.
- **Hypothesis**: The flight controller failed to transition into OFFBOARD mode because `fly_trajectory.py` sent `MAV_CMD_NAV_TAKEOFF` but never issued `MAV_CMD_DO_SET_MODE` to request OFFBOARD mode. PX4 remained in `AUTO.TAKEOFF` $\rightarrow$ `AUTO.LOITER` and ignored external velocity/position setpoints.
- **What I Changed**: Updated `fly_trajectory.py` to pre-stream setpoints for $\sim 1.5\text{ s}$ before requesting mode transition, explicitly sent MAVLink command `MAV_CMD_DO_SET_MODE` (`param2=6` for OFFBOARD), and corrected position/velocity `type_mask` bitmask to `1507` (`0x05E3`).
- **Result**: Drone successfully climbed to and sustained commanded cruise altitude ($1.91\text{ m} \text{--} 1.98\text{ m}$). OFFBOARD mode confirmed via MAVLink heartbeat telemetry (`custom_mode=393216`, `main_mode=6`).
- **What I Learned**: PX4 requires continuous streaming of setpoints prior to accepting OFFBOARD mode transition requests; proper MAVLink `type_mask` bitmask definitions are mandatory for position/velocity control.

### Entry 10: 2026-09-01 — Bug 5: Planar RANSAC Degeneracy & Proximity Sweet-Spot Finding
- **Date**: 2026-09-01
- **Problem / Goal**: Resolve low mean inlier ratio ($6.94\%$, $\sim 80\%$ near-zero frames) during straight-line flight over a flat tiled floor.
- **Hypothesis**: 5-point Essential Matrix RANSAC is ill-conditioned when all tracked keypoints lie on a planar surface (flat ground plane), causing planar RANSAC degeneracy during straight flight.
- **What I Changed**: Replaced straight-line flight over flat ground with a circular trajectory surrounding a 3D object cluster (`textured.sdf`). Conducted a systematic radial distance sweep from the cluster centroid.
- **Result**: SUBSTANTIVE RESEARCH FINDING — Discovered a non-monotonic distance sweet-spot in `textured.sdf`: a radial distance of $3.0\text{ m} \text{--} 5.0\text{ m}$ from the cluster centroid yields a **57.10% mean inlier ratio**; $<3.0\text{ m}$ drops to **18.87%** (due to motion blur and FOV clipping); $5.0\text{ m} \text{--} 7.0\text{ m}$ drops to **18.41%** (due to feature pixel sparsity). Closer proximity is not strictly better. Note: This finding was specific to `textured.sdf` and did not transfer to `boxworld_obstacles_tight` (see Entry 13).
- **What I Learned**: Monocular VO feature tracking in textured environments exhibits a non-monotonic distance sweet-spot; proximity to object clusters is non-linearly bounded by FOV/motion blur at close range and feature resolution at far range.

### Entry 11: 2026-09-02 — Bug 6: `cv2.recoverPose` Translation Sign Convention Fix
- **Date**: 2026-09-02
- **Problem / Goal**: Fix negative / wrong-sign Pearson correlation between estimated VO trajectory and ground truth ($r_x = -0.46$, $r_z = -0.05$).
- **Hypothesis**: OpenCV `cv2.recoverPose` returns relative translation vector $t$ satisfying $p_1 = R p_2 + t$, which represents the transformation from camera 2 to camera 1 frame—the exact opposite of camera motion direction. The code accumulated $t$ without negation.
- **What I Changed**: Negated translation vector during optical-to-Gazebo frame transformation: $t_{\text{gaz}} = -(R_{\text{opt2gaz}} t_{\text{opt}})$.
- **Result**: Verified without re-running flight data: Pearson correlation immediately flipped to strong positive alignment ($r_x = +0.97$, $r_y = +0.71$, $r_z = +0.69$).
- **What I Learned**: OpenCV `cv2.recoverPose` translation output is defined in point transformation coordinates; converting to camera ego-motion vectors requires explicit negation when accumulating trajectory step vectors.

### Entry 12: 2026-09-02 — Collision Test: Simulator Physics Engine Verification
- **Date**: 2026-09-02
- **Problem / Goal**: Resolve discrepancy between a visually observed near-collision during circular flight near `textured_cylinder` (GT showed smooth trajectory, max step $0.033\text{ m}$, $1.10\text{ m}$ minimum clearance) and verify if Gazebo physics collision geometry is active.
- **Hypothesis**: Verify whether Gazebo physics collision engine is active or if models permit uncaptured pass-through artifacts.
- **What I Changed**: Created a dedicated collision test world `collision_test_world.sdf` with a large wall obstacle ($10\text{m} \times 5\text{m} \times 0.5\text{m}$ at $X=10.0\text{m}$, front face at $X=9.75\text{m}$) and executed head-on flight at $1.2\text{ m/s}$.
- **Result**: COLLISION REGISTERED — Drone struck wall face at $X=9.565\text{m}$, experienced violent velocity reversal ($1.2\text{ m/s} \rightarrow 5.2\text{ m/s}$ recoil), bounced back to $X=5.29\text{m}$, with zero wall penetration. Confirmed Gazebo collision physics system is fully active; earlier cylinder flight was a genuine non-contact pass.
- **What I Learned**: Gazebo rigid-body collision handling is functional; visual proximity without trajectory disruption in simulation reflects valid spatial clearance.

### Entry 13: 2026-09-02 — World Migration to `boxworld_obstacles_tight`, Rendering Fix, & Feature Density Analysis
- **Date**: 2026-09-02
- **Problem / Goal**: Migrate from hand-built `textured.sdf` to standardized benchmark world `engcang/gazebo_maps` `boxworld_obstacles_tight` ($20\text{m} \times 20\text{m}$ arena, 4 bounding walls, 200 obstacle spheres, ROLAND ICCAS 2021) to eliminate scene-construction confounds. Fix rendering failure in GZ Sim PBR pipeline and analyze feature density.
- **Hypothesis**: `boxworld_obstacles_tight` rendered unlit/black in GZ Sim OGRE2 PBR pipeline (mean pixel brightness 0.0) due to legacy Gazebo 11 OGRE1 material scripts (`grass_plane`, `bbox_wall`, `obstacle_r`).
- **What I Changed**:
  - Updated material scripts by adding explicit `pbr`, `albedo_map`, `ambient`, `diffuse`, and `specular` tags for all materials, and corrected `sun_2` light intensity. Verified visually via screenshot (mean pixel brightness increased from 0.0 to 37.84).
  - Evaluated feature density contribution of 200 obstacle spheres ($r=0.025\text{m}$).
- **Result**:
  - Benchmark world successfully migrated and rendered with full PBR illumination.
  - FEATURE DENSITY FINDING: The 200 obstacle spheres ($r=0.025\text{m}$) project to small $\sim 3 \times 3\text{ px}$ targets with weak corner gradients, contributing $<15\%$ of total keypoints. However, arena ground and wall textures alone saturate GFTT/ORB feature detectors to 2000/2000 keypoints uniformly across the arena.
  - Bug 5's proximity-based sweet-spot finding from `textured.sdf` does NOT transfer to `boxworld_obstacles_tight`; trajectory design in this world should optimize for wall clearance and flight dynamics rather than obstacle proximity.
- **What I Learned**: Benchmark world migration requires PBR material translation for Gazebo Sim OGRE2 engine; feature distribution in bounded textured arenas is dominated by ground/wall surfaces rather than small scattered obstacles.

### Entry 14: 2026-09-03 — CPU Rendering Frame-Rate Jitter & GPU Offload Fix
- **Date**: 2026-09-03
- **Problem / Goal**: Resolve severe VO tracking collapse in v3 circular flight (`boxworld_obstacles_tight`), which produced a mean inlier ratio of 0.51% (vs 41.96% in v1) and collapsed Pearson correlation ($r_y = +0.0993$, $r_z = +0.1310$).
- **Hypothesis**: Evaluated and REJECTED texture aliasing and specular instability hypotheses (ORB inliers $>90\%$, KLT inliers $98.93\%$ at true 30 FPS, brightness variation $<3.55/255$). True root cause: CPU-starved software rendering (Mesa/llvmpipe on integrated Iris Xe) caused severe ROS 2 camera frame-rate jitter (inter-frame deltas up to 1.52s vs 32ms target), pushing inter-frame pixel displacement (up to 53.6 px) beyond KLT's 20 px search window.
- **What I Changed**:
  - Enabled NVIDIA PRIME render offload (`__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `GZ_SIM_RENDER_ENGINE=ogre2`) for Gazebo Sim on NVIDIA RTX 3050.
  - Updated `src/launch_camera_sim.sh` process cleanup handler: added missing `gz-sim-main` and `gz-sim-gui-client` process names with a 2-stage SIGTERM $\rightarrow$ SIGKILL fallback to eliminate orphaned process accumulation.
- **Result**: GPU utilization verified at 34–38% (up from 0%), frame timing restored to $\sim 36\text{ ms}$ mean delta. IDE lag resolved via clean process termination. CAVEAT: v1/v2 baselines were recorded under CPU rendering and may retain minor unquantified jitter degradation.
- **What I Learned**: Monocular VO optical flow is highly sensitive to camera frame-rate jitter; software rendering frame drops cause inter-frame motion to exceed the KLT tracking search window. GPU rendering hardware acceleration and clean process lifecycle management are essential for reproducible VO benchmarks.

### Entry 15: 2026-09-03 — Inlier Ratio vs Inlier Count Distinction in Bounded Obstacle Arenas
- **Date**: 2026-09-03
- **Problem / Goal**: Resolve apparent contradiction in v3-rerun post-GPU-fix: Pearson correlation recovered strongly ($r_y: 0.0993 \rightarrow 0.9073$), but mean inlier ratio remained low (0.11% raw, 0.12% corrected excluding re-detection resets).
- **Hypothesis**: Investigated whether low mean inlier ratio was an artifact of `minimal_vo.py` feature re-detection resets (which reset tracked points to 0 when $<100$). HYPOTHESIS REJECTED: excluding 24 re-detection reset frames (6.17%) shifted the mean only from 0.11% to 0.12%.
- **What I Changed**: Modified `src/analyze_v3_flight_metrics.py` to implement Approach A (excluding `num_matched == 0` reset frames) and conducted deep breakdown of matched vs inlier feature counts across all 4 flights.
- **Result**:
  - True explanation: KLT tracks $\sim 1870$ features per frame across the entire image (large denominator), but 5-point Essential Matrix RANSAC accepts only 1–42 points (mean 3.67 inliers) due to high 3D parallax and depth variation in `boxworld_obstacles_tight`.
  - Dividing mean 3.67 inliers by $\sim 1870$ matches yields $\sim 0.197\%$ on non-zero frames.
  - Crucially, Essential Matrix `recoverPose` requires only 5–8 inliers to solve for relative camera rotation and unit translation direction. The small absolute inlier count was sufficient for accurate pose recovery ($r_y = +0.9073$).
  - **[SUPERSEDED / LEGACY OBSERVATION]**: Note: The logged inlier count (3.67 inliers) was recorded using the legacy `cv2.recoverPose()` count under default `distanceThresh=50.0`. Forensic investigation (Entry 17) proved `findEssentialMat()` actually found $>500$ valid RANSAC inliers (`num_inliers_E`), and the low pose count was caused by small-baseline unit-scale depth truncation ($Z_{\text{unit}} = 166.7\text{m} > 50.0\text{m}$).

### Entry 16: 2026-09-03 — Phase 0E Sweep A: Pure Yaw-Rate Escalation & Precursor Signal Characterization
- **Date**: 2026-09-03
- **Problem / Goal**: Execute Phase 0E Sweep A to characterize how monocular VO degrades under increasingly aggressive rotational motion, isolated from translation, in `boxworld_obstacles_tight`, and identify observable precursor signals (feature-track survival rate, image-space feature velocity) prior to tracking failure.
- **Design**: In-place hover at $(5.0, 5.0, 2.5\text{m})$, pure yaw rotation across 6 levels ($10, 20, 40, 80, 120, 180\text{ deg/s}$). No translation, isolating rotation as a single independent variable.
- **v1 Confounds Identified & Resolved**:
  1. `minimal_vo.py` `--max-frames=500` cap truncated data non-uniformly across levels when combined with camera FPS dropping at high yaw rates, producing inconsistent active-frame counts (257 frames at 10 deg/s vs 94 at 180 deg/s) that cut off data collection early relative to the full $\sim 20.5\text{s}$ hover.
  2. Consecutive-streak failure definition (`num_inliers < 5` for 10+ consecutive frames) was fragile: a single lucky high-inlier frame (e.g. matching a distant, low-parallax wall) reset the streak counter, causing 180 deg/s to falsely register "no failure" despite 67% frame-level tracking failure (28.7% re-detection resets + 38.3% zero-inlier frames).
  - **Fixes Applied**: Raised `--max-frames` to 2000 (eliminating truncation artifacts) and replaced the failure definition with a 2.0s sliding window (failure = $>70\%$ of frames in any 2.0s window have `num_inliers < 5`).
- **v2 Results (Clean, Monotonic, Trustworthy)**:
  - Feature-track survival rate ($S_f$): 91.74% (10 deg/s) $\rightarrow$ 66.07% (180 deg/s), demonstrating progressive degradation.
  - Mean image-space feature velocity ($\bar{v}_{px}$) crosses the $\sim 10.5\text{ px}$ Pyramidal KLT search-window bound between 20–40 deg/s ($7.85\text{ px} \rightarrow 14.56\text{ px/frame}$).
  - All 6 levels correctly triggered failure under the 2.0s sliding-window definition (resolving the false-pass anomaly at 180 deg/s).
- **Open Limitation (Documented & Audit Verified)**:
  - Achieved camera FPS still drops with yaw rate even after the truncation fix ($30.34\text{ Hz}$ at 10–80 deg/s $\rightarrow 26.66\text{ Hz}$ at 120 deg/s $\rightarrow 19.15\text{ Hz}$ at 180 deg/s).
  - Investigated motion blur as a possible cause (Purab's hypothesis): audit of camera sensor SDF confirmed NO `motion_blur`, `shutter_speed`, `exposure`, or TAA tags exist in this GZ Sim camera pipeline; ruled out with direct config inspection.
  - True cause: GPU rendering throughput bottleneck — Ogre2 frustum culling and PBR texture rebinding cost scales with how much visible scene content changes per frame, which increases with rotation speed; compounded by physics-render thread sync forcing dropped frames under real-time PX4 SITL execution.
  - Net effect: Yaw rate and camera FPS remain partially entangled variables in this dataset — degradation magnitude at 120–180 deg/s is likely somewhat overstated relative to a true FPS-locked test, though the underlying trend and mechanism (faster rotation $\rightarrow$ more image-space motion $\rightarrow$ worse tracking) are real and directionally sound. Accepted as a documented limitation rather than pursued further, given project deadline.
- **Data References**:
  - Datasets: `results/yaw_sweep_v2_{10,20,40,80,120,180}dps_{gt,vo}.csv`
  - Plot: `plots/yaw_sweep_v2_characterization.png`
- **What I Learned**: Experimental protocol confounds (frame caps and fragile streak counters) must be rigorously validated before interpreting failure thresholds; GPU rendering load during rapid sensor rotation can introduce camera frame rate drops in physics simulators, which should be explicitly audited and logged.
- **[SUPERSEDED / REINTERPRETED BY REPAIRED PIPELINE]**: The failure triggers recorded in v2 were evaluated against `num_inliers` (the conflated `recoverPose` count under `distanceThresh=50.0`). As established in Entry 17, `findEssentialMat()` remained 100% healthy across all yaw rates ($819.1 - 1037.0$ mean inliers, `strict_E_fail = FALSE`). Pure yaw produces near-zero physical translation baseline ($t_{\text{true}} \approx 0$), inflating unit-scale triangulated point depth ($Z_{\text{unit}} = Z_{\text{true}} / \|t\| \to \infty$) beyond depth filters. Therefore, low pose counts under pure yaw are unit-scale zero-baseline depth filtering artifacts, NOT Essential Matrix RANSAC or epipolar correspondence failures.

### Entry 17: 2026-09-04 — Phase 0E Forensic Pipeline Repair, Metric Separation & Finalization Synthesis
- **Date**: 2026-09-04
- **Problem / Goal**: Conduct a comprehensive forensic audit of the Essential Matrix and pose estimation pipeline (`src/minimal_vo.py`), resolve metric conflation between epipolar RANSAC inliers and pose depth-filter inliers, fix small-baseline point rejection in OpenCV `recoverPose()`, validate the repair on physical flight telemetry in `agriculture.world`, re-evaluate Phase 0E Sweep A and Sweep B under the corrected pipeline, and establish forward experimental world policies for Phase 1+.
- **Forensic Discovery & Mechanism**:
  1. **Conflated Metric Discovery**: Historically, `minimal_vo.py` logged `num_inliers = int(inliers_count)` directly from `cv2.recoverPose()`. The logged `num_inliers` was **NOT equivalent** to `cv2.findEssentialMat()` RANSAC inliers.
  2. **OpenCV `recoverPose()` Depth Filtering Rejection**: OpenCV's `cv2.recoverPose(E, pts1, pts2, K, mask=mask_E)` dispatches to an internal C++ overload that enforces an implicit default parameter **`distanceThresh = 50.0` meters**.
  3. **Unit-Scale Triangulated Depth Inflation**: Monocular VO normalizes estimated translation to unit norm ($\|\hat{t}\| = 1.0\text{ m}$). For a camera hovering at physical altitude $Z_{\text{true}}$ with per-frame physical translation baseline $t_{\text{true}}$, the unit-scale triangulated depth is:
     $$Z_{\text{unit}} = Z_{\text{true}} \cdot \frac{\|\hat{t}\|}{\|t_{\text{true}}\|} = \frac{Z_{\text{true}}}{\|t_{\text{true}}\|}$$
     During small-baseline maneuvers ($Z_{\text{true}} \approx 2.50\text{ m}, t_{\text{true}} \approx 0.015\text{ m}$), $Z_{\text{unit}} = 2.50 / 0.015 = 166.7\text{ meters}$. Because $166.7 > 50.0$, `recoverPose()` rejected **100%** of valid triangulated points as exceeding `distanceThresh`, returning `num_inliers_pose = 0` despite `findEssentialMat()` finding $>500$ valid epipolar RANSAC inliers.
- **Pipeline Instrumentations & Repairs**:
  1. **Metric Separation**: `src/minimal_vo.py` was instrumented to separate and log:
     - `num_inliers_E`: `findEssentialMat()` 5-point RANSAC inlier count (algebraic epipolar correspondence quality).
     - `num_inliers_pose`: `recoverPose()` accepted pose inlier count (cheirality $Z > 0$ and depth bounds).
     - `num_inliers`: set equal to `num_inliers_E` for backward-compatible telemetry logging.
  2. **Operating Envelope Parameter Adoption**: `cv2.recoverPose()` was updated to explicitly pass `distanceThresh=1000.0`. A 42-case synthetic parameter matrix ($N=500$ points) verified that `distanceThresh=1000.0` accommodates unit-scale depths for hover altitudes up to $10.0\text{ m}$ and baselines down to $1.0\text{ cm}$ ($Z/t = 1000.0$), restoring 500/500 point acceptance.
- **Real-Flight 10° Roll Validation (`agriculture.world`)**:
  - Executed a $10^\circ$ roll validation flight ($0.5\text{ Hz}$, $20.0\text{s}$ maneuver) in `agriculture.world` at hover target $Z \approx 2.41\text{ m}$.
  - **Results**:
    - Epipolar RANSAC failure (`strict_E_fail`): **FALSE** (Max 2.0s window low-E fraction: **14.52%** $\ll 70\%$).
    - Pose recovery failure (`strict_pose_fail`): **FALSE** (Max 2.0s window low-pose fraction: **16.13%** $\ll 70\%$).
    - Usable pose updates (`num_inliers_pose >= 8`): **90.43%** of frames (up from 18.60% under legacy `dt=50`).
    - Low pose inlier frames (`num_inliers_pose < 5`): **9.57%** of frames (down from 81.40% under legacy `dt=50`).
    - Agreement ratio (`num_inliers_pose / num_inliers_E`): Median **0.9906** (99.06% inlier recovery).
- **Phase 0E Sweep A Reinterpretation (Pure Yaw Escalation, 10–180 deg/s in `boxworld_obstacles_tight`)**:
  - Epipolar correspondence RANSAC (`num_inliers_E`) remained extremely healthy across all six levels, averaging **$819.1 - 1037.0$ inliers per frame**.
  - `strict_E_fail = FALSE` across all 6 yaw rate levels ($10, 20, 40, 80, 120, 180\text{ deg/s}$).
  - **Reinterpretation**: The previous interpretation of Sweep A as Essential Matrix failure is **RETRACTED**. Pure yaw rotation generates a near-zero physical translation baseline ($t_{\text{true}} \approx 0$), inflating unit-scale triangulated depth ($Z_{\text{unit}} \to \infty$) beyond `distanceThresh=1000.0`. Low pose inlier counts during pure yaw holds are unit-scale zero-baseline depth filtering artifacts, and must be interpreted separately from E-RANSAC correspondence quality.
- **Phase 0E Sweep B Reinterpretation (Roll Attitude Escalation, 5–50 deg in `agriculture.world`)**:
  - Achieved dominant attitude axis was verified as **ROLL** across all six target levels ($p95 \text{ Roll}: 6.30^\circ - 47.51^\circ$, parasitic pitch $\le 0.82^\circ$, yaw $\le 1.82^\circ$).
  - Epipolar RANSAC (`num_inliers_E`) remained healthy, averaging **$576.3 - 655.7$ inliers per frame** (`strict_E_fail = FALSE` across all 6 levels).
  - Pose recovery (`num_inliers_pose`) did NOT trigger strict failure (`strict_pose_fail = FALSE` across all 6 levels). Usable pose updates reached **$87.44\% - 93.56\%$**.
  - KLT feature tracking remained strong across the full range (**$99.81\% - 100.0\%$ survival rate**, feature velocity up to $21.56\text{ px/fr}$ under `maxLevel=3` pyramidal LK).
  - **Mechanism**: Dynamic roll oscillation generates lateral acceleration ($a_y = g \tan(\phi)$), producing a non-zero lateral translation baseline ($t_{\text{true}} > 0$). This translation baseline lowers unit-scale depth ($Z_{\text{unit}} < 1000.0\text{m}$), enabling `recoverPose()` to accept $88.70\% - 99.91\%$ of true epipolar inliers.
- **Experimental World Policy (Forward-Looking)**:
  - **Phase 0**: Controlled diagnostic characterization may use `boxworld_obstacles_tight` and other synthetic worlds where appropriate.
  - **Phase 1+**: `configs/gazebo_maps/agriculture.world` is designated as the **PRIMARY** research and testing environment for baseline characterization, failure prediction intervention testing, and comparative evaluations.
- **Data & Documentation References**:
  - Audit & Repair: `results/essential_matrix_pipeline_audit.md`, `results/essential_matrix_bookkeeping_repair.md`
  - Synthetic Matrix: `results/recoverpose_distance_threshold_investigation.md`
  - Real Flight Validation: `results/recoverpose_1000_10deg_agriculture_validation.md`
  - Phase 0E Summaries: `results/yaw_sweep_phase0e_v2_summary.md`, `results/tilt_sweep_phase0e_v3_summary.md`
- **What I Learned**: In monocular visual odometry, metric depth filtering in non-metric unit-scale pose estimation can inadvertently truncate valid correspondences when physical translation baselines are small. Rigorous metric separation between algebraic epipolar RANSAC inliers (`num_inliers_E`) and pose-recovery cheirality inliers (`num_inliers_pose`) is essential to prevent false failure diagnoses.

### Entry 18: 2026-09-05 — `recoverPose` Operating-Envelope Calibration & Scope Caveat
- **Date**: 2026-09-05
- **Problem / Goal**: Formalize operating-envelope calibration for OpenCV `cv2.recoverPose()` depth threshold (`distanceThresh`) and document epistemic scope boundaries.
- **Hypothesis**: Default OpenCV `distanceThresh=50.0` rejected valid unit-scale triangulated points during small-baseline flight maneuvers ($Z_{\text{unit}} = Z_{\text{true}} / \|t\| > 50\text{m}$), whereas `distanceThresh=1000.0` accommodates unit-scale depths for project operating envelopes ($Z \le 10\text{m}$, baselines $t \ge 1\text{cm}$).
- **What I Changed**: Adopted `distanceThresh=1000.0` in `src/minimal_vo.py` following 42-case synthetic matrix regression testing and real-flight $10^\circ$ roll validation in `agriculture.world`. Documented calibration scope in forensic audit.
- **Result**: Restored 500/500 point acceptance in synthetic regressions and enabled $>90\%$ pose recovery in real flights.
- **What I Learned**: `distanceThresh=1000.0` is an operating-envelope calibration specific to this project's unit-scale geometry ($Z/t \le 1000$), NOT a universal mathematical constant or general OpenCV patch.

### Entry 19: 2026-09-05 — Ground-Truth Timestamp Provenance & Clock Disconnect Repair
- **Date**: 2026-09-05
- **Problem / Goal**: Resolve timestamp disconnect between ground-truth pose recording (`src/record_ground_truth.py`) and visual odometry (`src/minimal_vo.py`).
- **Hypothesis**: Historical ground-truth logging fell back to system wall-clock Unix timestamps ($\sim 1.788 \times 10^9\text{ s}$) because Gazebo `TFMessage` header timestamps were zero ($0,0$), whereas VO logged Gazebo ROS simulation time ($t_{\text{sim}} \approx 6.6\text{s} - 45.5\text{s}$).
- **What I Changed**: Repaired ground-truth subscriber node to enforce `use_sim_time: True` and explicitly subscribe to ROS `/clock` simulation time headers.
- **Result**: Synchronized future telemetry streams directly to simulation time.
- **What I Learned**: Historical dataset GT logs must be aligned using relative elapsed time ($\Delta t$) from takeoff completion rather than direct absolute timestamp equality. Historical data should NOT be silently treated as directly timestamp-synchronized with VO.

### Entry 20: 2026-09-05 — Ground-Truth Micro-Timing Jitter & Derivative Filtering
- **Date**: 2026-09-05
- **Problem / Goal**: Eliminate artificial kinetic derivative spikes ($45.01\text{ m/s}$ speed, $721.3^\circ/\text{s}$ yaw rate in $P07$) in ground-truth analysis scripts (`src/analyze_phase1_gt.py`).
- **Hypothesis**: Thread dispatch jitter in Gazebo TF bridging produced clustered micro-interval sample pairs ($dt < 2.0\text{ ms}$, up to $15.5\%$ of samples) and raw angle wraparound ($-\pi \to +\pi$), causing finite difference division artifacts.
- **What I Changed**: Updated canonical analysis pipeline to filter out sample intervals with $dt < 2.0\text{ ms}$ and unwrap angular orientation series before computing spatial derivatives ($\mathbf{v}, \boldsymbol{\omega}$).
- **Result**: Artificial velocity and yaw-rate spikes were fully eliminated (clean P95 speed $2.28\text{ m/s}$, clean P95 yaw rate $140.2^\circ/\text{s}$ in $P07$).
- **What I Learned**: Derivative masking of micro-timing noise ($dt < 2.0\text{ ms}$) removes numerical and timing artifacts created by thread dispatch jitter, NOT genuine high-speed physical motion.

### Entry 21: 2026-09-05 — Historical HOVER Baseline Active-Window Alignment Bug
- **Date**: 2026-09-05
- **Problem / Goal**: Audit active window selection in historical HOVER baseline analysis.
- **Hypothesis**: The historical baseline script mixed GT-relative wall-clock time and VO-relative simulation time, shifting the evaluated active motion window by approximately $+1.289\text{ s}$.
- **What I Changed**: Corrected canonical active window selection logic in `src/analyze_phase1_gt.py` to use unified simulation timestamps for both GT and VO telemetry.
- **Result**: Active window selection brought into strict temporal alignment.
- **What I Learned**: Historical baseline reports must be retained as historical provenance for auditability rather than silently overwritten; canonical analysis explicitly documents the correction.

### Entry 22: 2026-09-05 — HOVER Zero-Parallax Degeneracy Diagnostic Classification
- **Date**: 2026-09-05
- **Problem / Goal**: Diagnose low valid pose update rates ($52.45\% - 53.81\%$) during stationary HOVER ($L0$) runs despite high Essential Matrix RANSAC inlier ratios ($99.9\%$).
- **Hypothesis**: Stationary hovering exhibits near-zero translation ($v \approx 0.05\text{ m/s}$), creating insufficient baseline parallax for 5-point Essential Matrix triangulation.
- **What I Changed**: Conducted diagnostic breakdown across HOVER runs $R1 - R3$. Evaluated feature tracking survival ($72.87\% - 75.71\%$) vs pose valid rates.
- **Result**: Confirmed that near-zero translation baseline causes parallel epipolar ray degeneracy, triggering cheirality/depth filtering rejection in `recoverPose()` while epipolar feature matching remains healthy.
- **What I Learned**: HOVER $L0$ is a static/zero-parallax diagnostic baseline, NOT a direct moving-flight Pose/E health threshold. Pose/E metrics during pure hover reflect geometric baseline limits rather than VO algorithm degradation.

### Entry 23: 2026-09-05 — P01/F1 Forward Translation Short-Duration Anomaly & Resolution
- **Date**: 2026-09-05
- **Problem / Goal**: Address duration imbalance in pilot trajectory $P01$ ($F1$ Forward Translation $L2$).
- **Hypothesis**: $P01$ completed its forward translation target ($14.0\text{m}$) rapidly, yielding only $3.29\text{ s}$ of canonical cruise motion (100 VO frames) compared to $22.4 - 24.2\text{ s}$ ($679 - 732$ VO frames) for pilots $P02 - P08$.
- **What I Changed**: Audit flagged $P01$ as a historical duration anomaly. Confirmation batch executed 20.0s replacement runs ($F1$ $R1 - R3$, $749 - 754$ frames).
- **Result**: Replicate runs $F1$ $R1 - R3$ provided full-duration, time-normalized baseline comparisons ($80.54\% - 82.69\%$ valid pose rate).
- **What I Learned**: Original $P01$ must be documented as a historical duration anomaly; full-duration confirmation runs resolved the cross-pilot comparability issue.

### Entry 24: 2026-09-05 — P03/P04 Attitude Excursion & Braking Trajectory Interpretation
- **Date**: 2026-09-05
- **Problem / Goal**: Interpret peak attitude excursions ($19.4^\circ$ roll in $P03$, $18.8^\circ$ pitch in $P04$) observed in Phase 1 pilot telemetry.
- **Hypothesis**: High peak attitude values reflected hard position-hold braking at active trajectory termination rather than steady-state cruise severity.
- **What I Changed**: Analyzed time-series attitude profiles and body/ENU frame transformations across active motion windows.
- **Result**: Confirmed cruise-phase roll/pitch remained moderate ($4.2^\circ - 6.8^\circ$ P95), with peak excursions isolated strictly to terminal braking maneuvers.
- **What I Learned**: Terminal position-hold braking causes transient attitude spikes; evaluation must distinguish terminal braking artifacts from steady-state motion severity as an analysis caveat, NOT a VO degradation failure.

### Entry 25: 2026-09-06 — Confirmation F9 R3 Takeoff Timeout Execution Anomaly
- **Date**: 2026-09-06
- **Problem / Goal**: Investigate execution anomaly in confirmation run 18 (`confirmation_018_F9_L2_R3`).
- **Hypothesis**: F9 R3 hit the 8.0s takeoff timeout prior to reaching the $2.0\text{m}$ altitude threshold due to SITL climb rate fluctuation, triggering an early fallback return.
- **What I Changed**: Audited execution log `results/confirmation_018_F9_L2_R3_exec.log`. Confirmed active trajectory motion was never initialized ($N_{\text{VO}} = 295$ vs $626-627$ in R1/R2).
- **Result**: Identified F9 R3 as an execution timeout anomaly rather than an algorithmic VO failure.
- **What I Learned**: F9 R3 VO metrics MUST NOT be interpreted as F9 VO degradation; the run is classified strictly as an execution anomaly.

### Entry 26: 2026-09-06 — Confirmation F10 R1–R3 Takeoff Execution Incomplete Caveat
- **Date**: 2026-09-06
- **Problem / Goal**: Audit performance of confirmation runs 19–21 (`confirmation_019_F10_L3_R1` to `confirmation_021_F10_L3_R3`).
- **Hypothesis**: All three F10 runs hit the 8.0s takeoff timeout at altitude $Z \approx 1.65 - 1.72\text{m}$, failing to reach the $2.0\text{m}$ threshold required to start Level-3 aggressive coupled active motion setpoints.
- **What I Changed**: Examined execution logs (`confirmation_019-021_F10_L3_exec.log`). Verified vehicle remained in hover/climb fallback throughout the active window.
- **Result**: Confirmed intended aggressive Level-3 motion profile was never executed by PX4 SITL.
- **What I Learned**: F10 R1–R3 VO metrics do NOT characterize the intended F10 aggressive trajectory and must be classified as lower-achieved-severity / execution caveats rather than VO degradation results.

### Entry 27: 2026-09-06 — 21-Run Sequential Batch Degradation Audit Finding
- **Date**: 2026-09-06
- **Problem / Goal**: Evaluate whether monocular VO systematically degrades over consecutive chronologically executed flight runs ($1 \to 21$).
- **Hypothesis**: Cumulative software state, memory leaks, or simulator timing drift might cause progressive VO performance loss over sequential batch execution.
- **What I Changed**: Analyzed chronological sequence metrics across 21 consecutive SITL runs. Evaluated replicate consistency ($R1 \to R2 \to R3$) across identical trajectory families.
- **Result**: Replicate runs demonstrated high consistency ($\Delta\text{E-Ratio} < 0.001$, $\Delta\text{Valid Pose Rate} < 1.8\%$). VO metrics clustered by motion family rather than chronological batch index.
- **What I Learned**: Bounded Finding: "Under the tested 21-run SITL/Gazebo sequence and conditions, no detectable sequential VO degradation was observed." This does NOT constitute proof that cumulative/thermal/hardware degradation is impossible in physical real-world systems.

### Entry 28: 2026-09-06 — Within-Run Temporal Stability Audit Finding
- **Date**: 2026-09-06
- **Problem / Goal**: Evaluate whether VO performance degrades intra-run across active window duration.
- **Hypothesis**: Feature drift or frame-to-frame error accumulation during a 20-second flight profile could cause within-run temporal degradation.
- **What I Changed**: Divided active motion windows into four equal temporal quartiles ($Q1: 0-25\%$, $Q2: 25-50\%$, $Q3: 50-75\%$, $Q4: 75-100\%$) across all 21 confirmation runs.
- **Result**: Essential Matrix inlier ratios remained stable at $\ge 99.5\%$ across all quartiles in translational runs; feature tracking reached steady-state equilibrium.
- **What I Learned**: No detectable within-run temporal degradation occurred in valid test trajectories under the evaluated simulation conditions.

### Entry 29: 2026-09-06 — Operational Infrastructure Action Item: Takeoff Readiness Timeout Adjustment
- **Date**: 2026-09-06
- **Problem / Goal**: Address execution timeouts observed in F9 R3 and F10 R1–R3 where SITL climb variability caused valid runs to hit the 8.0s takeoff threshold cap.
- **Hypothesis**: The closed-loop altitude takeoff state machine concept is sound, but the 8.0s timeout (`TAKEOFF_TIMEOUT_SEC`) is overly restrictive for observed SITL initialization variability.
- **What I Changed**: Recorded formal forensic audit recommendation to increase `TAKEOFF_TIMEOUT_SEC` from 8.0s to 12.0s prior to Phase 2 execution.
- **Result**: Documented as an infrastructure configuration action item. No code changes or reruns were executed at this time.
- **What I Learned**: Simulation environment initialization jitter requires conservative state-machine timeout margins ($\ge 12.0\text{ s}$) to prevent premature fallback aborts.

### Entry 30: 2026-09-06 — Phase 1 Programmatic Closure & Phase 2 Transition Readiness
- **Date**: 2026-09-06
- **Problem / Goal**: Finalize Phase 1 experimental closure and declare Phase 2 readiness.
- **Hypothesis**: Baseline characterization across 8 pilot trajectories ($P01-P08$) and 21 confirmation runs provides a complete, trustworthy foundation for Phase 2 fault injection.
- **What I Changed**: Consolidated findings from Phase 1 Pilot Audit (`results/phase1_pilot_audit.md`) and Confirmation Batch Report (`results/phase1_confirmation_batch_report.md`).
- **Result**: Formally closed Phase 1 with documented caveats and declared Phase 2 readiness.
- **What I Learned**: Final Phase 1 Status: **COMPLETE WITH DOCUMENTED CAVEATS**. Phase 2 Status: **READY**.


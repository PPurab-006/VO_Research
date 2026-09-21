"""Fig 12 - System diagram. Drawn from the code (src/core/eis_preprocessor.py, src/pipelines/run_offline_vo.py), not from memory.
Gate semantics taken from compute_homography(): mode 'gated' sets R_rel = I (no derotation) when |yaw rate| > threshold,
otherwise applies the incremental relative rotation. Attitude comes from load_attitude_telemetry(gt_csv): simulator ground truth.
Camera rate is measured from camera_frames.csv timestamps (33.0 ms), not assumed.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, FancyArrowPatch
import style as S, common as c

dts = []
for r in c.RUNS:
    df = pd.read_csv(c.run_dir("F9_L2", r) / "camera_frames.csv"); dts.append(np.diff(df["timestamp_total_sec"].values))
dt = np.concatenate(dts); dt = dt[dt > 0]; HZ = 1.0 / np.median(dt)
W, H_ = [int(v) for v in pd.read_csv(c.run_dir("F9_L2", 1) / "camera_frames.csv").iloc[0][["width", "height"]]]

fig, ax = plt.subplots(figsize=(S.COL2 + 0.6, 4.7)); ax.set_xlim(0, 120); ax.set_ylim(0, 72); ax.axis("off")
def box(x, y, w, h, txt, fc="#F5F5F5", ec="#333333", fs=7.2, bold_first=True, lw=1.0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25,rounding_size=1.4", fc=fc, ec=ec, lw=lw))
    lines = txt.split("\n")
    if bold_first:
        ax.text(x + w / 2, y + h - 1.6, lines[0], ha="center", va="top", fontsize=fs + 0.4, fontweight="bold", color=ec)
        ax.text(x + w / 2, y + h - 4.3, "\n".join(lines[1:]), ha="center", va="top", fontsize=fs, linespacing=1.25)
    else:
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs, linespacing=1.25)
def arrow(x0, y0, x1, y1, color="#333333", ls="-", lw=1.3, rad=0.0):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=9, color=color, lw=lw, ls=ls, connectionstyle=f"arc3,rad={rad}", shrinkA=0, shrinkB=0))

# ---- main pipeline (row A)
yA, hA, w = 50, 17, 20
xs = [1, 25.5, 50, 74.5, 99]
box(xs[0], yA, w, hA, f"Camera\nGazebo, simulated\n{W}×{H_} px\n{HZ:.1f} Hz (measured)", fc="#EEF3FA", ec="#1F4E79", fs=6.5)
box(xs[1], yA, w, hA, "EIS stage\nwarps each frame by H\n(H = I when bypassed)", fc="#EAF6F0", ec=S.C["green"], fs=6.5)
box(xs[2], yA, w, hA, "KLT tracking\npyramidal LK\noptical flow", fc="#F5F5F5", fs=6.5)
box(xs[3], yA, w, hA, "Essential matrix\n5-point + RANSAC\n(num_inliers_E)", fc="#F5F5F5", fs=6.5)
box(xs[4], yA, w, hA, "recoverPose\ndistanceThresh = 1000\nVO units, not metres\nvalid: inliers ≥ 8", fc="#FDF1E7", ec=S.C["vermilion"], fs=6.3)
for i in range(4): arrow(xs[i] + w + 0.3, yA + hA / 2, xs[i + 1] - 0.3, yA + hA / 2)
ax.text(xs[4] + w / 2, yA - 1.6, "relative pose → integrated trajectory\n(unit scale; Sim(3) alignment at evaluation)", ha="center", va="top", fontsize=6.3, style="italic")
ax.text(xs[0], yA + hA + 1.8, "RAW = the EIS stage removed: frames pass straight through", ha="left", fontsize=6.8, color=S.RAW_C, fontweight="bold")

# ---- gate module (inset)
gx0, gy0, gx1, gy1 = 1, 5, 119, 37
ax.add_patch(FancyBboxPatch((gx0, gy0), gx1 - gx0, gy1 - gy0, boxstyle="round,pad=0.3,rounding_size=1.6", fc="#FBFBFB", ec=S.C["green"], lw=1.2, ls="--"))
ax.text(gx0 + 1.5, gy1 - 1.4, "EIS-GATED: per-frame gate", ha="left", va="top", fontsize=8, fontweight="bold", color=S.C["green"])
box(4, 9, 25, 19, "Attitude\nsimulator ground-truth\nquaternion, SLERP to\nframe time\n(not an estimated IMU)", fc="#FFF9E5", ec=S.C["orange"], fs=6.4)
box(36, 9, 25, 19, "Yaw rate\n|ω_z| = |Δψ| / Δt\nbetween consecutive\nframes (body z-axis)", fc="#F5F5F5", fs=6.4)
arrow(29.3, 18.5, 35.7, 18.5)
cx, cy, dw, dh = 79, 18.5, 13.5, 11.5
ax.add_patch(Polygon([(cx - dw, cy), (cx, cy + dh), (cx + dw, cy), (cx, cy - dh)], closed=True, fc="#FFFFFF", ec="#333333", lw=1.1))
ax.text(cx, cy + 1.2, "|ω_z| > τ ?", ha="center", va="center", fontsize=7.6, fontweight="bold")
ax.text(cx, cy - 3.4, "τ = 15 deg/s\n(chosen)", ha="center", va="center", fontsize=6.1)
arrow(61.3, 18.5, cx - dw - 0.3, 18.5)
box(97, 21.5, 21, 12, "yes: bypass\nH = I  (no warp)", fc="#FDECEC", ec=S.RAW_C, fs=6.4)
box(97, 7.5, 21, 12, "no: derotate\nH = K R_rel K⁻¹\n(incremental)", fc="#EAF6F0", ec=S.C["green"], fs=6.4)
arrow(cx + dw * 0.55, cy + dh * 0.45, 96.8, 27.5, color=S.RAW_C)
arrow(cx + dw * 0.55, cy - dh * 0.45, 96.8, 13.5, color=S.C["green"])
ax.text(cx + 8.8, cy + 9.2, "yes", fontsize=6.5, color=S.RAW_C, fontweight="bold"); ax.text(cx + 8.8, cy - 10.0, "no", fontsize=6.5, color=S.C["green"], fontweight="bold")
arrow(xs[1] + w / 2, gy1 + 0.35, xs[1] + w / 2, yA - 0.35, color=S.C["green"], lw=1.6)
ax.text(xs[1] + w / 2 + 1.2, (gy1 + yA) / 2, "warp matrix H_cv (this frame)", ha="left", va="center", fontsize=6.6, color=S.C["green"], fontweight="bold")
ax.text(60, 1.2, "Derotation is applied only when the yaw rate is at or below τ; bypassed frames are identical to RAW's. Other variants: FIXED (reference = first frame), INCREMENTAL (always on), NULL (H = I always).",
        ha="center", va="bottom", fontsize=5.9)
S.save(fig, "fig_12_system_diagram")
pd.DataFrame([dict(item="camera_rate_hz_measured", value=HZ), dict(item="median_frame_dt_ms", value=np.median(dt) * 1000), dict(item="image_width", value=W), dict(item="image_height", value=H_),
              dict(item="gate_threshold_deg_per_s", value=15.0), dict(item="recoverPose_distanceThresh_vo_units", value=1000.0), dict(item="validity_min_inliers", value=8)]).to_csv(S.DATA / "fig_12_system_diagram.csv", index=False)
print("camera rate:", round(HZ, 2), "Hz  frame size:", W, "x", H_)

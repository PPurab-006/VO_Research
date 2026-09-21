"""Fig 7 - F9 (yaw + translation): ground truth vs RAW vs EIS-GATED, Sim(3)-aligned with evo, all three runs.
Alignment and ATE use the repo's own evaluation code path (same GT interpolation, same active window).
ATE printed on each panel is read from common.per_run_metrics(), and the plotted alignment is asserted to reproduce it.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp
from evo.core import trajectory, metrics
import style as S, common as c
import evaluate_phase3_evo as ev

M = c.per_run_metrics()

def aligned(vo_csv, gt_csv):
    gt = pd.read_csv(gt_csv); vo = pd.read_csv(vo_csv)
    t0, t1 = c.active_window(gt)
    va = c.in_window(vo, t0, t1)
    tg = gt["timestamp_total_sec"].values.astype(float)
    tu, ui = np.unique(tg, return_index=True)
    gp = gt[["pos_x", "pos_y", "pos_z"]].values[ui]
    gq = R.from_quat(gt[["rot_x", "rot_y", "rot_z", "rot_w"]].values[ui]); sl = Slerp(tu, gq)
    ts = va["timestamp_total_sec"].values.astype(float)
    gtpos = np.column_stack([np.interp(ts, tu, gp[:, k]) for k in range(3)])
    gtq = sl(np.clip(ts, tu[0], tu[-1])).as_quat()
    vpos = va[["pos_x", "pos_y", "pos_z"]].values.astype(float)
    vq = va[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)
    tgt = trajectory.PoseTrajectory3D(positions_xyz=gtpos, orientations_quat_wxyz=ev.gt_quats_to_wxyz(gtq), timestamps=ts)
    tvo = trajectory.PoseTrajectory3D(positions_xyz=vpos, orientations_quat_wxyz=ev.gt_quats_to_wxyz(vq), timestamps=ts)
    tvo.align(tgt, correct_scale=True)
    ape = metrics.APE(metrics.PoseRelation.translation_part); ape.process_data((tgt, tvo))
    return tgt.positions_xyz, tvo.positions_xyz, ape.get_statistic(metrics.StatisticsType.rmse)

fig, axes = plt.subplots(1, 3, figsize=(S.COL2 + 0.4, 3.0), sharey=False)
rows = []
for j, r in enumerate(c.RUNS):
    d = c.run_dir("F9_L2", r); g = d / "dataset_gt.csv"
    gp, rp, ar = aligned(d / "raw_vo.csv", g)
    _, ep, ag = aligned(d / "eis_gated_vo.csv", g)
    ref_r = M.query("family=='F9_L2' and run==@r and mechanism=='RAW'").ate_rmse.iloc[0]
    ref_g = M.query("family=='F9_L2' and run==@r and mechanism=='EIS-GATED'").ate_rmse.iloc[0]
    assert abs(ar - ref_r) < 1e-6 and abs(ag - ref_g) < 1e-6, (r, ar, ref_r, ag, ref_g)
    a = axes[j]
    a.plot(gp[:, 0], gp[:, 1], color=S.C["black"], lw=1.6, label="Ground truth")
    a.plot(rp[:, 0], rp[:, 1], color=S.RAW_C, lw=0.9, ls="--", label=f"RAW  (ATE {ar:.2f} m)")
    a.plot(ep[:, 0], ep[:, 1], color=S.GATED_C, lw=0.9, ls="-.", label=f"EIS-GATED  (ATE {ag:.2f} m)")
    a.set_xlabel("x (m)"); a.set_aspect("equal", adjustable="datalim")
    allp = np.vstack([gp[:, :2], rp[:, :2], ep[:, :2]]); pad = 0.6
    a.set_xlim(allp[:, 0].min() - pad, allp[:, 0].max() + pad); a.set_ylim(allp[:, 1].min() - pad, allp[:, 1].max() + pad)
    a.set_title(f"Run {r}", fontsize=8.5, fontweight="bold"); S.panel(a, "abc"[j], dx=-0.12)
    a.legend(loc="lower center", bbox_to_anchor=(0.5, -0.62), fontsize=6.6, ncol=1)
    rows.append(dict(run=r, source_dir=d.name, ate_raw=ar, ate_gated=ag, gated_better=bool(ag < ar)))
axes[0].set_ylabel("y (m)")
fig.subplots_adjust(wspace=0.28, bottom=0.36)
S.save(fig, "fig_07_f9_trajectories")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_07_f9_trajectories.csv", index=False)
print(out.round(4).to_string(index=False))

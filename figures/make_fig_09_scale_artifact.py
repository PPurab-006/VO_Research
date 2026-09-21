"""Fig 9 - Why meter-valued RPE misleads for monocular VO: the Sim(3) scale differs between runs.
F9 run 1 ONLY. Panel (a): meter RPE for RAW, EIS-GATED, and RAW after re-scaling its trajectory by s_GATED BEFORE a rigid (scale-free) alignment.
(b): scale-normalised RPE (meter RPE / Sim(3) scale) for RAW and EIS-GATED. Normalised RPE is scale-invariant by construction,
so no third 'rescaled' bar is shown. (c): per-run Sim(3) scale, all 3 F9 runs, RAW vs EIS-GATED, to show the scale varies run to run.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp
from evo.core import trajectory, metrics
import style as S, common as c
import evaluate_phase3_evo as ev

M = c.per_run_metrics()
def traj_pair(vo_csv, gt_csv):
    gt = pd.read_csv(gt_csv); vo = pd.read_csv(vo_csv); t0, t1 = c.active_window(gt)
    va = c.in_window(vo, t0, t1); tg = gt["timestamp_total_sec"].values.astype(float); tu, ui = np.unique(tg, return_index=True)
    gp = gt[["pos_x", "pos_y", "pos_z"]].values[ui]; sl = Slerp(tu, R.from_quat(gt[["rot_x", "rot_y", "rot_z", "rot_w"]].values[ui]))
    ts = va["timestamp_total_sec"].values.astype(float)
    gtpos = np.column_stack([np.interp(ts, tu, gp[:, k]) for k in range(3)]); gtq = sl(np.clip(ts, tu[0], tu[-1])).as_quat()
    vpos = va[["pos_x", "pos_y", "pos_z"]].values.astype(float); vq = va[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)
    return (trajectory.PoseTrajectory3D(gtpos, ev.gt_quats_to_wxyz(gtq), ts), vpos, ev.gt_quats_to_wxyz(vq), ts)

def evaluate(gt_traj, vpos, vq, ts, prescale=1.0, correct_scale=True):
    t = trajectory.PoseTrajectory3D(vpos * prescale, np.copy(vq), np.copy(ts))
    _, _, s = t.align(gt_traj, correct_scale=correct_scale)
    ape = metrics.APE(metrics.PoseRelation.translation_part); ape.process_data((gt_traj, t))
    rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames); rpe.process_data((gt_traj, t))
    return ape.get_statistic(metrics.StatisticsType.rmse), rpe.get_statistic(metrics.StatisticsType.mean), s

d = c.run_dir("F9_L2", 1); g = d / "dataset_gt.csv"
gtT, rp, rq, rts = traj_pair(d / "raw_vo.csv", g); gtG, gp_, gq, gts = traj_pair(d / "eis_gated_vo.csv", g)
ate_r, rpe_r, s_r = evaluate(gtT, rp, rq, rts)
ate_g, rpe_g, s_g = evaluate(gtG, gp_, gq, gts)
ate_x, rpe_x, _ = evaluate(gtT, rp, rq, rts, prescale=s_g, correct_scale=False)   # RAW pre-scaled by s_GATED, rigid align only
ref = M.query("family=='F9_L2' and run==1")
assert abs(ate_r - ref[ref.mechanism == "RAW"].ate_rmse.iloc[0]) < 1e-6 and abs(ate_g - ref[ref.mechanism == "EIS-GATED"].ate_rmse.iloc[0]) < 1e-6
norm_r, norm_g = rpe_r / s_r, rpe_g / s_g
# Method 1A (archived test_scale_causal.py): rescale the ALREADY-ALIGNED RAW trajectory about the alignment translation.
_t = trajectory.PoseTrajectory3D(np.copy(rp), np.copy(rq), np.copy(rts)); _, _tr, _s = _t.align(gtT, correct_scale=True)
_t2 = trajectory.PoseTrajectory3D(s_g * (_t.positions_xyz - _tr) / _s + _tr, np.copy(_t.orientations_quat_wxyz), np.copy(rts))
_ap = metrics.APE(metrics.PoseRelation.translation_part); _ap.process_data((gtT, _t2)); ate_1a = _ap.get_statistic(metrics.StatisticsType.rmse)
ate_ctrl = evaluate(gtT, rp, rq, rts, prescale=s_g, correct_scale=True)[0]   # scale re-fit: must equal RAW's own ATE
assert abs(ate_ctrl - ate_r) < 1e-6

fig, ax = plt.subplots(1, 3, figsize=(S.COL2 + 0.4, 2.7), gridspec_kw=dict(width_ratios=[1.1, 0.8, 1.1]))
a = ax[0]; v = [rpe_r, rpe_g, rpe_x]; lab = ["RAW", "EIS-\nGATED", "RAW\nrescaled"]
b = a.bar(range(3), v, color=[S.RAW_C, S.GATED_C, S.C["sky"]], width=0.62)
for i, x in enumerate(v): a.text(i, x + 0.002, f"{x:.4f}", ha="center", fontsize=7)
a.set_xticks(range(3)); a.set_xticklabels(lab, fontsize=7.6); a.set_ylabel("Meter RPE (m/step)"); a.set_ylim(0, 0.115); a.grid(axis="x", visible=False); S.panel(a, "a", dx=-0.25)
a = ax[1]; v = [norm_r, norm_g]
a.bar(range(2), v, color=[S.RAW_C, S.GATED_C], width=0.55)
for i, x in enumerate(v): a.text(i, x + 0.02, f"{x:.4f}", ha="center", fontsize=7)
a.set_xticks(range(2)); a.set_xticklabels(["RAW", "EIS-GATED"], fontsize=7.5); a.set_ylabel("Scale-normalised RPE"); a.set_ylim(0, 1.15); a.grid(axis="x", visible=False); S.panel(a, "b", dx=-0.3)
a = ax[2]
sc = M[(M.family == "F9_L2") & M.mechanism.isin(["RAW", "EIS-GATED"])]
for m_, colr, off in (("RAW", S.RAW_C, -0.14), ("EIS-GATED", S.GATED_C, 0.14)):
    y = sc[sc.mechanism == m_].sort_values("run").sim3_scale.values
    a.scatter(np.arange(1, 4) + off, y, s=22, color=colr, zorder=3, edgecolor="white", lw=0.3, label=m_)
a.set_xticks([1, 2, 3]); a.set_xticklabels(["R1", "R2", "R3"]); a.set_ylabel("Sim(3) scale factor"); a.set_xlabel("F9 run"); a.legend(fontsize=7, loc="upper right"); a.grid(axis="x", visible=False); S.panel(a, "c", dx=-0.3)
fig.subplots_adjust(wspace=0.55)
S.save(fig, "fig_09_scale_artifact")
rows = [dict(item="s_RAW", value=s_r), dict(item="s_GATED", value=s_g), dict(item="meter_RPE_RAW", value=rpe_r), dict(item="meter_RPE_GATED", value=rpe_g),
        dict(item="meter_RPE_RAW_prescaled_by_sGATED", value=rpe_x), dict(item="normRPE_RAW", value=norm_r), dict(item="normRPE_GATED", value=norm_g),
        dict(item="ATE_RAW", value=ate_r), dict(item="ATE_GATED", value=ate_g), dict(item="ATE_RAW_prescaled_by_sGATED_rigid_realign_(method_1B)", value=ate_x),
        dict(item="ATE_RAW_rescaled_after_alignment_(method_1A_archived_test)", value=ate_1a),
        dict(item="ATE_RAW_prescaled_by_sGATED_scale_refit_(control_equals_RAW)", value=ate_ctrl)]
for r in c.RUNS:
    for m_ in ("RAW", "EIS-GATED"):
        rows.append(dict(item=f"sim3_scale_{m_}_R{r}", value=sc[(sc.mechanism == m_) & (sc.run == r)].sim3_scale.iloc[0]))
out = pd.DataFrame(rows); out["scope"] = np.where(out.item.str.startswith("sim3_scale"), "F9 R1-R3", "F9 R1 only"); out.to_csv(S.DATA / "fig_09_scale_artifact.csv", index=False)
print(out.round(5).to_string(index=False))

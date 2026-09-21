"""Fig 5 - Does the 15 deg/s gate threshold have an empirical basis?
(a) RAW pose-loss rate vs |yaw rate| bin on F9 (3 runs pooled), 95% bootstrap CI (1000 resamples, seed 0).
    Yaw rate = VO-logged eis_yaw_rate_deg (attitude-derived, same signal the gate uses), active window.
(b) Offline replay sweep of the gate threshold on F9, per-run points and mean, with RAW reference.
    Panel (b) needs results/analysis/threshold_sweep.csv (frame replay); it is a required input.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

SW = c.REPO / "results" / "analysis" / "threshold_sweep.csv"
if not SW.exists():
    raise SystemExit("results/analysis/threshold_sweep.csv missing: Fig 5(b) needs the frame-replay sweep (make rerun-vo).")

# (a) binned pose loss, pooled over F9 R1-R3
recs = []
for r in c.RUNS:
    d = c.run_dir("F9_L2", r)
    gt = pd.read_csv(d / "dataset_gt.csv"); t0, t1 = c.active_window(gt)
    raw = c.in_window(pd.read_csv(d / "raw_vo.csv"), t0, t1)
    gat = c.in_window(pd.read_csv(d / "eis_gated_vo.csv"), t0, t1)
    n = min(len(raw), len(gat))
    recs.append(pd.DataFrame(dict(yr=gat["eis_yaw_rate_deg"].abs().values[:n], bad=(raw["num_inliers_pose"].values[:n] < 8).astype(float), run=r)))
A = pd.concat(recs, ignore_index=True)
edges = [0, 5, 10, 15, 20, 30, 50, np.inf]
labels = ["0-5", "5-10", "10-15", "15-20", "20-30", "30-50", ">50"]
A["bin"] = pd.cut(A.yr, edges, right=False, labels=labels)
rng = np.random.RandomState(0)
brow = []
for lab in labels:
    v = A[A.bin == lab].bad.values
    boots = [v[rng.randint(0, len(v), len(v))].mean() * 100 for _ in range(1000)]
    brow.append(dict(bin=lab, n_frames=len(v), loss_pct=v.mean() * 100, ci_lo=np.percentile(boots, 2.5), ci_hi=np.percentile(boots, 97.5)))
B = pd.DataFrame(brow)

sw = pd.read_csv(SW)
_need = {"gate_thresh_deg", "repeat", "valid_pose_pct", "ate_rmse"}
if not _need <= set(sw.columns):
    raise SystemExit(f"threshold_sweep.csv must contain columns {sorted(_need)}; found {list(sw.columns)}")
ref_valid = c.per_run_metrics().query("family=='F9_L2' and mechanism=='RAW'").valid_pose_pct.mean()
ref_ate = c.per_run_metrics().query("family=='F9_L2' and mechanism=='RAW'").ate_rmse.mean()

fig = plt.figure(figsize=(S.COL2 + 0.5, 3.4))
gs = fig.add_gridspec(2, 3, height_ratios=[4, 1.05], hspace=0.1, wspace=0.55, width_ratios=[1.15, 1, 1])
a = fig.add_subplot(gs[0, 0]); ab = fig.add_subplot(gs[1, 0], sharex=a)
x = np.arange(len(B))
a.errorbar(x, B.loss_pct, yerr=[B.loss_pct - B.ci_lo, B.ci_hi - B.loss_pct], fmt="o-", color=S.RAW_C, capsize=3, lw=1.2, ms=4)
a.axvline(2.5, color=S.C["black"], ls=":", lw=0.9)
a.text(2.55, a.get_ylim()[1] if False else 13.3, "15 deg/s", fontsize=7, va="top")
a.set_ylim(0, 14); a.set_ylabel("RAW pose-loss rate (%)"); plt.setp(a.get_xticklabels(), visible=False)
ab.bar(x, B.n_frames, color=S.C["grey"], width=0.6)
for xi, n in zip(x, B.n_frames):
    ab.text(xi, n + 30, str(n), ha="center", fontsize=6.3)
ab.set_ylim(0, 820); ab.set_yticks([0, 400, 800]); ab.set_ylabel("Frames", fontsize=8)
ab.set_xticks(x); ab.set_xticklabels(labels, fontsize=7.3, rotation=35); ab.set_xlabel("|Yaw rate| bin (deg/s)")
ab.grid(False); a.grid(axis="x", visible=False); S.panel(a, "a", dx=-0.25)

thr = sorted(sw.gate_thresh_deg.unique()); xt = np.arange(len(thr))
for j, (col, ylab, ref, refl, let) in enumerate((("valid_pose_pct", "Pose validity (%)", ref_valid, "RAW mean", "b"),
                                                   ("ate_rmse", "ATE RMSE (m)", ref_ate, "RAW mean", "c"))):
    ax_ = fig.add_subplot(gs[:, 1 + j])
    for i, t in enumerate(thr):
        v = sw[sw.gate_thresh_deg == t].sort_values("repeat")[col].values
        ax_.scatter(i + np.array([-0.1, 0, 0.1]), v, s=16, color=S.GATED_C, zorder=3, edgecolor="white", lw=0.3)
        ax_.hlines(v.mean(), i - 0.24, i + 0.24, color=S.GATED_C, lw=2, zorder=4)
    ax_.axhline(ref, color=S.RAW_C, ls="--", lw=1.0, label=f"{refl} ({ref:.2f})")
    ax_.set_xticks(xt); ax_.set_xticklabels([f"{t:g}" for t in thr]); ax_.set_xlabel("Gate threshold (deg/s)")
    ax_.set_ylabel(ylab); ax_.legend(loc="lower left" if col == "valid_pose_pct" else "upper right", fontsize=6.8)
    ax_.grid(axis="x", visible=False); S.panel(ax_, let, dx=-0.28)
S.save(fig, "fig_05_threshold")
B.to_csv(S.DATA / "fig_05_threshold_bins.csv", index=False)
out = sw.copy(); out.to_csv(S.DATA / "fig_05_threshold_sweep.csv", index=False)
print(B.round(2).to_string(index=False))
print(sw.groupby("gate_thresh_deg")[["valid_pose_pct", "ate_rmse"]].agg(["mean", "std"]).round(3))
print("RAW ref valid, ate:", round(ref_valid, 2), round(ref_ate, 3))

# ---- how much of the run has EIS ON at each threshold, and what a uniform-harm interpolation would predict ---------------
inc_valid = []
for r in c.RUNS:
    d = c.run_dir("F9_L2", r); gt = pd.read_csv(d / "dataset_gt.csv"); t0, t1 = c.active_window(gt)
    inc_valid.append(c.valid_pct(c.in_window(pd.read_csv(d / "eis_inc_vo.csv"), t0, t1)))
inc_valid = float(np.mean(inc_valid))
on_rows = []
for t in sorted(sw.gate_thresh_deg.unique()):
    f_on = float((A.yr <= t).mean())            # gate: derotation applied when |yaw rate| <= threshold, bypassed above it
    on_rows.append(dict(gate_thresh_deg=t, frac_frames_eis_on=f_on, uniform_harm_prediction_valid_pct=ref_valid + f_on * (inc_valid - ref_valid),
                        measured_valid_pct_mean=float(sw[sw.gate_thresh_deg == t].valid_pose_pct.mean()), raw_valid_pct=float(ref_valid), incremental_valid_pct=inc_valid))
pd.DataFrame(on_rows).to_csv(S.DATA / "fig_05_gate_on_fraction.csv", index=False)
print(pd.DataFrame(on_rows).round(3).to_string(index=False))

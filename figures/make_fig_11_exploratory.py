"""Fig 11 - EXPLORATORY families (HOVER, F3, F7, F8). Never aggregated with the core matrix (Fig 6).
Same paired layout as Fig 6. p = unadjusted paired t-test (n=3). HOVER is a near-stationary condition (positional extent ~0.1-0.3 m).
ATE is on a log axis because HOVER/F3 (~0.05-0.15 m) and F7/F8 (~2.9 m) differ by ~50x.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

df = c.per_run_metrics(); FAM = c.EXPLORATORY
METRICS = [("valid_pose_pct", "Pose validity (%)", "a", False), ("ate_rmse", "ATE RMSE (m, log)", "b", True), ("rpe_t_norm", "Scale-normalised RPE", "c", False)]
fig, axes = plt.subplots(3, 1, figsize=(S.COL1 + 1.6, 6.4), sharex=True)
rows = []
for ax, (met, ylab, let, logy) in zip(axes, METRICS):
    lo, hi = np.inf, -np.inf
    for i, fam in enumerate(FAM):
        p = c.paired(df, fam, met, exploratory=True)
        for r in range(3): ax.plot([i - 0.14, i + 0.14], [p["raw"][r], p["gated"][r]], color=S.C["grey"], lw=0.7, alpha=0.7, zorder=2)
        ax.scatter(np.full(3, i - 0.14), p["raw"], s=20, color=S.RAW_C, zorder=3, edgecolor="white", lw=0.3)
        ax.scatter(np.full(3, i + 0.14), p["gated"], s=20, color=S.GATED_C, zorder=3, edgecolor="white", lw=0.3)
        ax.hlines(p["raw"].mean(), i - 0.26, i - 0.02, color=S.RAW_C, lw=2, zorder=4); ax.hlines(p["gated"].mean(), i + 0.02, i + 0.26, color=S.GATED_C, lw=2, zorder=4)
        lo = min(lo, p["raw"].min(), p["gated"].min()); hi = max(hi, p["raw"].max(), p["gated"].max())
        rows.append(dict(metric=met, family=fam, k_gated_better=p["k"], paired_t_p=p["t_p"], mean_diff_gated_minus_raw=p["mean_diff"],
                         **{f"raw_r{r+1}": p["raw"][r] for r in range(3)}, **{f"gated_r{r+1}": p["gated"][r] for r in range(3)}))
    if logy:
        ax.set_yscale("log"); ax.set_ylim(lo / 1.6, hi * 3.2); top = hi * 1.35
    else:
        span = hi - lo; ax.set_ylim(lo - 0.08 * span, hi + 0.34 * span); top = hi + 0.07 * span
    for i, fam in enumerate(FAM):
        r_ = [x for x in rows if x["metric"] == met and x["family"] == fam][0]; star = r_["paired_t_p"] < 0.05
        ax.text(i, top, f"{r_['k_gated_better']}/3\np={r_['paired_t_p']:.3f}{'*' if star else ''}", ha="center", va="bottom", fontsize=6.6,
                color=(S.C["vermilion"] if star else "black"), fontweight=("bold" if star else "normal"))
    ax.set_ylabel(ylab); S.panel(ax, let, dx=-0.2); ax.grid(axis="x", visible=False)
axes[-1].set_xticks(range(4)); axes[-1].set_xticklabels([f.replace("_L2", "").replace("_L0", "") for f in FAM]); axes[-1].set_xlabel("Exploratory family")
h = [plt.Line2D([], [], marker="o", ls="", color=S.RAW_C, label="RAW"), plt.Line2D([], [], marker="o", ls="", color=S.GATED_C, label="EIS-GATED")]
axes[0].legend(handles=h, loc="lower right", ncol=1, fontsize=7)
fig.text(0.5, 0.945, "EXPLORATORY — not aggregated with the core matrix", ha="center", fontsize=7.5, fontweight="bold", color=S.C["vermilion"])
fig.text(0.5, 0.008, "Unadjusted paired-t p, n = 3 runs. 36 comparisons are reported across Figs 6 and 11; * marks p < 0.05 before any multiplicity correction.", ha="center", fontsize=6.3)
fig.subplots_adjust(hspace=0.15, top=0.9, bottom=0.09)
S.save(fig, "fig_11_exploratory")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_11_exploratory.csv", index=False)
print(out[["metric", "family", "k_gated_better", "paired_t_p", "mean_diff_gated_minus_raw"]].round(4).to_string(index=False))

# HOVER: how large is the yaw rate the gate sees? (gate threshold = 15 deg/s)
hy = []
for r in c.RUNS:
    d_ = c.run_dir("HOVER_L0", r, True); gt = pd.read_csv(d_ / "dataset_gt.csv"); t, hd, _ = c.heading_series(gt)
    _, _, yr, nbad, mj = c.yaw_rate_grid(t, hd)
    hy.append(dict(run=r, source_dir=d_.name, peak_rate_degps=float(yr.max()), p99_rate_degps=float(np.percentile(yr, 99)), n_heading_jumps_gt10deg=nbad))
pd.DataFrame(hy).to_csv(S.DATA / "fig_11_hover_yaw.csv", index=False); print(pd.DataFrame(hy).round(3).to_string(index=False))

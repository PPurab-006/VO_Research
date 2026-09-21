"""Fig 6 - Eight core families, RAW vs EIS-GATED, three metrics, paired per run (n=3).
Lines join the same run under the two mechanisms. k = runs (of 3) where EIS-GATED is better.
p = paired t-test; the n=3 Wilcoxon p can only take the values 0.25/0.5/0.75/1.0, so it is not annotated.
All statistics come from common.paired(); no test is computed in this script.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

df = c.per_run_metrics()
core = df[~df.exploratory]
FAM = c.CORE
METRICS = [("valid_pose_pct", "Pose validity (%)", "a"), ("ate_rmse", "ATE RMSE (m)", "b"),
           ("rpe_t_norm", "Scale-normalised RPE", "c")]
phase2a = {f for f in FAM if c.source_of(f, 1) == "phase2a"}
ga = []
for fam_, expl_ in [(f, False) for f in c.CORE] + [(f, True) for f in c.EXPLORATORY]:
    n_, b_ = c.gate_bypass_stats(fam_, expl_); ga.append(dict(family=fam_, exploratory=expl_, active_frames=n_, frames_bypassed=b_, pct_bypassed=100.0 * b_ / n_))
GA = pd.DataFrame(ga); GA.to_csv(S.DATA / "fig_06_gate_activity.csv", index=False)
byp = {r.family: r.pct_bypassed for r in GA.itertuples()}

fig, axes = plt.subplots(3, 1, figsize=(S.COL2, 7.0), sharex=True)
rows = []
for ax, (met, ylab, let) in zip(axes, METRICS):
    ylo, yhi = np.inf, -np.inf
    for i, fam in enumerate(FAM):
        p = c.paired(df, fam, met)
        for r in range(3):
            ax.plot([i - 0.14, i + 0.14], [p["raw"][r], p["gated"][r]], color=S.C["grey"], lw=0.7, alpha=0.7, zorder=2)
        ax.scatter(np.full(3, i - 0.14), p["raw"], s=20, color=S.RAW_C, zorder=3, edgecolor="white", lw=0.3)
        ax.scatter(np.full(3, i + 0.14), p["gated"], s=20, color=S.GATED_C, zorder=3, edgecolor="white", lw=0.3)
        ax.hlines(p["raw"].mean(), i - 0.26, i - 0.02, color=S.RAW_C, lw=2, zorder=4)
        ax.hlines(p["gated"].mean(), i + 0.02, i + 0.26, color=S.GATED_C, lw=2, zorder=4)
        ylo = min(ylo, p["raw"].min(), p["gated"].min()); yhi = max(yhi, p["raw"].max(), p["gated"].max())
        rows.append(dict(metric=met, family=fam, source=c.source_of(fam, 1), k_gated_better=p["k"],
                         paired_t_p=p["t_p"], wilcoxon_p=p["w_p"], mean_diff_gated_minus_raw=p["mean_diff"],
                         **{f"raw_r{r+1}": p["raw"][r] for r in range(3)}, **{f"gated_r{r+1}": p["gated"][r] for r in range(3)}))
    span = yhi - ylo
    ax.set_ylim(ylo - 0.08 * span, yhi + 0.30 * span)
    top = yhi + 0.06 * span
    for i, fam in enumerate(FAM):
        r_ = [x for x in rows if x["metric"] == met and x["family"] == fam][0]
        star = "*" if r_["paired_t_p"] < 0.05 else ""
        ax.text(i, top, f"{r_['k_gated_better']}/3\np={r_['paired_t_p']:.2f}{star}", ha="center", va="bottom", fontsize=6.6,
                color=(S.C["black"] if not star else S.C["vermilion"]), fontweight=("bold" if star else "normal"))
    ax.set_ylabel(ylab); S.panel(ax, let, dx=-0.09); ax.grid(axis="x", visible=False)
    if met == "rpe_t_norm":
        ax.axhline(1.0, color=S.C["grey"], lw=0.6, ls=":")

axes[-1].set_xticks(range(len(FAM)))
axes[-1].set_xticklabels([f.replace("_L2", "").replace("_L3", "") + ("†" if f in phase2a else "") + f"\n({byp[f]:.0f}%)" for f in FAM])
axes[-1].set_xlabel("Motion family   (in brackets: % of frames on which the gate bypasses derotation)")
h = [plt.Line2D([], [], marker="o", ls="", color=S.RAW_C, label="RAW (each dot = one run)"),
     plt.Line2D([], [], marker="o", ls="", color=S.GATED_C, label="EIS-GATED (each dot = one run)"),
     plt.Line2D([], [], color=S.C["grey"], lw=0.8, label="same run, paired")]
axes[0].legend(handles=h, loc="upper left", ncol=3, fontsize=7, bbox_to_anchor=(0.0, 1.32))
fig.subplots_adjust(hspace=0.16, top=0.94)
S.save(fig, "fig_06_core_matrix")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_06_core_matrix.csv", index=False)
sig = out[out.paired_t_p < 0.05]
print("phase2a families (†):", sorted(phase2a))
print("comparisons with paired-t p<0.05 (of", len(out), "):"); print(sig[["metric","family","k_gated_better","paired_t_p","mean_diff_gated_minus_raw"]].round(4).to_string(index=False))
print("F9 ATE:", out[(out.metric=="ate_rmse")&(out.family=="F9_L2")][["k_gated_better","paired_t_p"]].round(4).values.tolist())

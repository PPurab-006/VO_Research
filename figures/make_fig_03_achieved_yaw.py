"""Fig 3 - Achieved yaw for the rotation families (F6, F9, F10), all three runs each.
Heading = atan2 of the GT body +X axis (unwrapped), duplicates in timestamp dropped.
Runs whose GT heading has >10 deg jumps between consecutive samples (data discontinuities) are drawn dashed and flagged.
Bottom row: |yaw rate| on a 50 Hz grid, 99th percentile printed (max is dominated by the discontinuities in flagged runs).
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

FAMS = ["F6_L2", "F9_L2", "F10_L3"]
COLS = [S.C["vermilion"], S.C["blue"], S.C["green"]]
fig, axes = plt.subplots(2, 3, figsize=(S.COL2 + 0.4, 4.6), sharex=False)
rows = []
for j, fam in enumerate(FAMS):
    for r in c.RUNS:
        d = c.run_dir(fam, r)
        gt = pd.read_csv(d / "dataset_gt.csv"); t, hd, a = c.heading_series(gt)
        n_bad, mj, _ = c.glitch_report(t, hd)
        tg, hg, yr, _, _ = c.yaw_rate_grid(t, hd)
        flagged = n_bad > 0
        ls = "--" if flagged else "-"
        lab = f"R{r}" + (f" (GT discontinuity: {n_bad} jumps)" if flagged else "")
        axes[0, j].plot(t - t[0], hd, color=COLS[r - 1], ls=ls, lw=1.1, label=lab)
        axes[1, j].plot(tg, np.abs(np.gradient(np.interp(tg + t[0], t, hd), 0.02)), color=COLS[r - 1], ls=ls, lw=0.7, alpha=0.9)
        rows.append(dict(family=fam, run=r, source_dir=d.name, n_gt_rows=len(a), n_heading_jumps_gt10deg=n_bad,
                         max_jump_deg=mj, mean_heading_offset_deg=float(hd.mean() - hd[0]), peak_to_peak_deg=float(hd.max() - hd.min()),
                         peak_rate_degps=float(np.abs(np.gradient(np.interp(tg + t[0], t, hd), 0.02)).max()),
                         p99_rate_degps=float(np.percentile(np.abs(np.gradient(np.interp(tg + t[0], t, hd), 0.02)), 99)),
                         median_rate_degps=float(np.median(np.abs(np.gradient(np.interp(tg + t[0], t, hd), 0.02))))))
    axes[0, j].set_ylim(-8, 138); axes[1, j].set_ylim(0, 320)
    axes[1, j].axhline(15, color=S.C["black"], lw=0.7, ls=":")
    axes[0, j].legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), fontsize=6.3, handlelength=1.6, ncol=1, borderaxespad=0.2)
    axes[0, j].set_xlabel("")
    axes[1, j].set_xlabel("Time in active window (s)")
    axes[0, j].set_xticklabels([]); axes[0, j].tick_params(axis="x", length=0)
    axes[0, j].set_title(fam.replace("_L2", "").replace("_L3", ""), fontsize=9, fontweight="bold")
    S.panel(axes[0, j], "abc"[j], dx=-0.2)
axes[0, 0].set_ylabel("Heading (deg, unwrapped)")
axes[1, 0].set_ylabel("|Yaw rate| (deg/s)")
axes[1, 0].text(0.98, 0.94, "dotted: 15 deg/s gate\naxis clipped at 320", transform=axes[1, 0].transAxes, ha="right", va="top", fontsize=6.5)
fig.subplots_adjust(wspace=0.32, hspace=0.42)
S.save(fig, "fig_03_achieved_yaw")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_03_achieved_yaw.csv", index=False)
print(out.round(1).to_string(index=False))

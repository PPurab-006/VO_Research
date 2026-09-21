"""Fig 2 - What each core motion family actually is (ground truth, one representative CLEAN run each).
Top: heading relative to start. Bottom: ground-truth speed. Rate quantities on a 50 Hz grid, after dropping duplicate timestamps.
Runs with GT heading discontinuities are never used as the representative run (see fig_03 / gt_heading_audit).
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

FAM = c.CORE
PAL = [S.C["black"], S.C["orange"], S.C["sky"], S.C["green"], S.C["vermilion"], S.C["blue"], S.C["purple"], S.C["yellow"]]
fig, axes = plt.subplots(2, 4, figsize=(S.COL2 + 0.6, 4.1), sharex=True)
rows = []
for i, fam in enumerate(FAM):
    # representative run: first run with no heading discontinuity
    rep = None
    for r in c.RUNS:
        d = c.run_dir(fam, r); gt = pd.read_csv(d / "dataset_gt.csv"); t, hd, a = c.heading_series(gt)
        if c.glitch_report(t, hd)[0] == 0:
            rep = (r, d, t, hd, a); break
    r, d, t, hd, a = rep
    tg = np.arange(t[0], t[-1], 0.02)
    px, py, pz = (np.interp(tg, t, a[k].values) for k in ("pos_x", "pos_y", "pos_z"))
    # GT timestamps are jittered (15% of consecutive rows < 5 ms apart), so instantaneous speed is meaningless.
    # Use a 0.5 s finite-difference baseline on the 50 Hz interpolated position instead.
    kb = int(0.5 / 0.02)
    P = np.vstack([px, py, pz]).T
    sp = np.full(len(tg), np.nan)
    sp[kb // 2: len(tg) - kb // 2 - (kb % 2 == 1)] = np.linalg.norm(P[kb:] - P[:-kb], axis=1)[: len(tg) - kb] / (kb * 0.02) if False else np.nan
    sp = np.zeros(len(tg)); sp[:] = np.nan
    vb = np.linalg.norm(P[kb:] - P[:-kb], axis=1) / (kb * 0.02)
    sp[kb // 2: kb // 2 + len(vb)] = vb
    sp_s = sp
    hg = np.interp(tg, t, hd)
    zg = np.interp(tg, t, a["pos_z"].values)
    # terminal transition: after t = 60% of the window, first time altitude leaves the cruise band by > 0.15 m
    tt = tg - tg[0]; late = tt > 0.6 * tt[-1]
    zc = np.median(zg[(tt > 0.25 * tt[-1]) & (tt < 0.6 * tt[-1])])
    out_band = np.where(late & (np.abs(zg - zc) > 0.15))[0]
    t_desc = float(tt[out_band[0]]) if len(out_band) else None
    ax0 = axes[0, i % 4] if i < 4 else axes[1, i % 4]
    axr, axc = (i // 4), (i % 4)
    ax = axes[axr, axc]
    ax2 = ax.twinx()
    ax.plot(tg - tg[0], hg - hg[0], color=PAL[i], lw=1.3)
    ax2.plot(tg - tg[0], sp_s, color=S.C["grey"], lw=1.0, ls="--")
    if t_desc is not None:
        ax.axvspan(t_desc, tg[-1] - tg[0], color=S.C["sky"], alpha=0.16, lw=0)
    ax.set_title(fam.replace("_L2", "").replace("_L3", "") + ("†" if c.source_of(fam, 1) == "phase2a" else "") + f"  (R{r})", fontsize=8.5, fontweight="bold")
    ax.set_ylim(-5, 130); ax2.set_ylim(0, 2.4)
    ax.spines["right"].set_visible(False); ax2.spines["right"].set_visible(True); ax2.spines["top"].set_visible(False)
    if axc != 0: ax.set_yticklabels([])
    else: ax.set_ylabel("Heading change (deg)")
    if axc != 3: ax2.set_yticklabels([])
    elif axr == 0: ax2.set_ylabel("Speed (m/s, dashed)", labelpad=2)
    else: ax2.set_ylabel("Speed (m/s, dashed)", labelpad=2)
    if axr == 1: ax.set_xlabel("Time (s)")
    yr = np.abs(np.gradient(hg, 0.02))
    rows.append(dict(family=fam, descent_starts_s=t_desc, representative_run=r, source_dir=d.name, duration_s=float(tg[-1] - tg[0]),
                     heading_p2p_deg=float(hg.max() - hg.min()), mean_speed_mps=float(np.nanmean(sp)),
                     p95_speed_mps=float(np.nanpercentile(sp, 95)),
                     path_length_m=float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))), p99_yaw_rate_dps=float(np.percentile(yr, 99))))
fig.text(0.5, 0.005, "Shaded: terminal transition (altitude leaves the cruise band). Speed differenced over a 0.5 s baseline (GT timestamps are jittered).", ha="center", fontsize=7)
fig.subplots_adjust(wspace=0.14, hspace=0.32, right=0.93, bottom=0.12)
S.save(fig, "fig_02_motion_families")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_02_motion_families.csv", index=False)
print(out.round(2).to_string(index=False))

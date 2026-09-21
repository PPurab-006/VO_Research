"""Fig 8 - Delayed triangulation: an apparent RPE improvement that is a tracking-failure artifact.
For F6, F9, F10 (rotation-heavy families), per-run points (n=3) and mean for EIS-GATED vs DELAYED-TRI:
 (a) pose validity, (b) scale-normalised RPE over the FULL window, (c) the same RPE split into valid frames and starved frames
     (starved = num_inliers_pose < 8, where the pose is stale/zero-motion so per-step error is trivially small).
All values from common.per_run_metrics() (repo evaluate_single_run).
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

df = c.per_run_metrics(); df = df[~df.exploratory]
FAM = ["F6_L2", "F9_L2", "F10_L3"]
MEC = [("EIS-GATED", S.GATED_C), ("DELAYED-TRI", S.DT_C)]
fig, ax = plt.subplots(1, 3, figsize=(S.COL2 + 0.4, 2.9))
rows = []
def cell(a, col, fam, mech, i, off, colr, mark="o", fill=True):
    v = df[(df.family == fam) & (df.mechanism == mech)].sort_values("run")[col].values
    a.scatter(i + off + np.array([-0.05, 0, 0.05]), v, s=15, color=colr if fill else "white", edgecolor=colr, lw=1.0, marker=mark, zorder=3)
    a.hlines(v.mean(), i + off - 0.13, i + off + 0.13, color=colr, lw=2, zorder=4)
    return v
for i, fam in enumerate(FAM):
    for k, (mech, colr) in enumerate(MEC):
        off = (-0.2, 0.2)[k]
        v1 = cell(ax[0], "valid_pose_pct", fam, mech, i, off, colr)
        v2 = cell(ax[1], "rpe_t_norm", fam, mech, i, off, colr)
        v3 = cell(ax[2], "rpe_t_norm_valid", fam, mech, i, off - 0.0, colr, mark="o", fill=True)
        v4 = cell(ax[2], "rpe_t_norm_starved", fam, mech, i, off, colr, mark="s", fill=False)
        rows.append(dict(family=fam, mechanism=mech, valid_pct_mean=v1.mean(), rpe_full_mean=v2.mean(), rpe_valid_mean=v3.mean(),
                         rpe_starved_mean=v4.mean(), **{f"valid_r{r+1}": v1[r] for r in range(3)}, **{f"rpe_full_r{r+1}": v2[r] for r in range(3)}))
for a, yl, let in zip(ax, ("Pose validity (%)", "Scale-normalised RPE, full window", "Scale-normalised RPE, by frame type"), "abc"):
    a.set_xticks(range(3)); a.set_xticklabels([f.replace("_L2", "").replace("_L3", "") for f in FAM]); a.set_ylabel(yl, fontsize=8)
    a.grid(axis="x", visible=False); S.panel(a, let, dx=-0.22)
ax[0].set_ylim(30, 100)
ax[1].set_ylim(0, 1.35); ax[2].set_ylim(0, 1.6)
ax[1].axhline(1.0, color=S.C["grey"], lw=0.6, ls=":"); ax[2].axhline(1.0, color=S.C["grey"], lw=0.6, ls=":")
h = [plt.Line2D([], [], marker="o", ls="", color=S.GATED_C, label="EIS-GATED"), plt.Line2D([], [], marker="o", ls="", color=S.DT_C, label="DELAYED-TRI")]
ax[0].legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=7, ncol=1)
h2 = [plt.Line2D([], [], marker="o", ls="", color=S.C["black"], label="valid frames (filled)"),
      plt.Line2D([], [], marker="s", ls="", markerfacecolor="white", color=S.C["black"], label="starved frames (open)")]
ax[2].legend(handles=h2, loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=6.6, ncol=1)
fig.subplots_adjust(wspace=0.42)
S.save(fig, "fig_08_dt_artifact")
out = pd.DataFrame(rows); out.to_csv(S.DATA / "fig_08_dt_artifact.csv", index=False)
print(out[["family", "mechanism", "valid_pct_mean", "rpe_full_mean", "rpe_valid_mean", "rpe_starved_mean"]].round(3).to_string(index=False))

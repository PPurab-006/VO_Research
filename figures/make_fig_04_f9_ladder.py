"""Fig 4 - EIS effect ladder on F9 (yaw + translation). Per-run points (n=3) + mean.
Validity is computed from the per-mechanism VO CSVs in phase2a_F9_L2_R1..R3 (active window, inliers_pose>=8).
ATE is computed with the repo's evo pipeline (Sim(3)/Umeyama) on the same files.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import io, contextlib
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c
import evaluate_phase3_evo as ev

MECH = [("RAW", "raw_vo.csv", S.RAW_C), ("FIXED", "eis_vo.csv", S.C["orange"]),
        ("INCREMENTAL", "eis_inc_vo.csv", S.C["sky"]), ("NULL", "eis_null_vo.csv", S.C["purple"]),
        ("GATED", "eis_gated_vo.csv", S.GATED_C)]
rows = []
for r in c.RUNS:
    d = c.run_dir("F9_L2", r); gtp = d / "dataset_gt.csv"
    gt = pd.read_csv(gtp); t0, t1 = c.active_window(gt)
    for name, fn, _ in MECH:
        vo = pd.read_csv(d / fn)
        with contextlib.redirect_stdout(io.StringIO()):
            m = ev.evaluate_single_run(str(d / fn), str(gtp))
        rows.append(dict(run=r, mechanism=name, source_dir=d.name, valid_pct=c.valid_pct(c.in_window(vo, t0, t1)),
                         ate_rmse=m["ate_rmse"], rpe_t_norm=m["rpe_t_norm"]))
df = pd.DataFrame(rows)

fig, ax = plt.subplots(1, 2, figsize=(S.COL2, 2.9))
xs = np.arange(len(MECH))
rng = np.random.RandomState(0)
for j, (col, ylab) in enumerate((("valid_pct", "Pose validity (%)"), ("ate_rmse", "ATE RMSE (m)"))):
    a = ax[j]
    for i, (name, _, colr) in enumerate(MECH):
        v = df[df.mechanism == name].sort_values("run")[col].values
        jit = np.array([-0.09, 0.0, 0.09])
        a.scatter(i + jit, v, s=22, color=colr, zorder=3, edgecolor="white", lw=0.4)
        a.hlines(v.mean(), i - 0.26, i + 0.26, color=colr, lw=2.2, zorder=4)
    a.set_xticks(xs); a.set_xticklabels([m[0].replace("INCREMENTAL", "INCR.") for m in MECH], fontsize=8)
    a.set_ylabel(ylab); a.set_xlabel("EIS variant"); a.grid(axis="x", visible=False)
    S.panel(a, "ab"[j], dx=-0.2)
raw_v = df[df.mechanism == "RAW"].valid_pct.mean()
ax[0].axhline(raw_v, color=S.RAW_C, lw=0.8, ls=":", zorder=1)
ax[0].set_ylim(58, 97)
ax[1].axhline(df[df.mechanism == "RAW"].ate_rmse.mean(), color=S.RAW_C, lw=0.8, ls=":", zorder=1)
ax[1].set_ylim(2.3, 4.15)
fig.subplots_adjust(wspace=0.32)
S.save(fig, "fig_04_f9_ladder")
df["mechanism"] = df["mechanism"].replace({"NULL": "EIS-NULL"})   # the bare string "NULL" is parsed as a missing value by pandas/most CSV readers
df.to_csv(S.DATA / "fig_04_f9_ladder.csv", index=False)
print(df.groupby("mechanism", sort=False)[["valid_pct", "ate_rmse"]].agg(["mean", "std"]).round(3))

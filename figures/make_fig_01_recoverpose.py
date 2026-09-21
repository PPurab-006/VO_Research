"""Fig 1 - recoverPose distanceThresh failure and repair.

(a) The repo's own synthetic scene (same seed, intrinsics, points as tests/test_essential_matrix_bookkeeping.py)
    run live: findEssentialMat vs recoverPose(default 50) vs recoverPose(1000), for 3 cases.
(b) Baseline sweep on the same scene: fraction of points accepted vs translation baseline, both thresholds.
    Explains WHY it fails: recoverPose normalises |t|=1, so depth in VO units = Z / baseline.
(c) Real flight: distribution of num_inliers_pose in F9 R1..R3 under the repaired threshold ONLY.
    No old-threshold real-flight measurement exists in committed CSVs (checked: roll_validation_vo.csv
    'num_inliers' equals 'num_inliers_E' on 100% of frames), so none is plotted.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd, cv2
import matplotlib.pyplot as plt
import style as S, common as c

fx, fy, cx, cy = 539.9363, 539.9364, 640.0, 480.0
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
rng = np.random.RandomState(42)   # identical draw order to the test's np.random.seed(42)
np.random.seed(42)
N = 500
X = np.random.uniform(-3.0, 3.0, N); Y = np.random.uniform(-3.0, 3.0, N); Z = np.random.uniform(2.0, 10.0, N)
P3 = np.vstack([X, Y, Z]).T
h1 = (K @ P3.T).T
PTS1 = (h1[:, :2] / h1[:, 2:]).reshape(-1, 1, 2).astype(np.float32)


def run_case(roll_deg, t_vec):
    rad = np.radians(roll_deg)
    Rg = np.array([[np.cos(rad), -np.sin(rad), 0], [np.sin(rad), np.cos(rad), 0], [0, 0, 1]])
    tg = np.array(t_vec, float).reshape(3, 1)
    p2 = (Rg @ P3.T + tg).T
    h2 = (K @ p2.T).T
    PTS2 = (h2[:, :2] / h2[:, 2:]).reshape(-1, 1, 2).astype(np.float32)
    E, mE = cv2.findEssentialMat(PTS1, PTS2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    nE = int(np.sum(mE == 1)) if mE is not None else 0
    out = {"E": nE}
    for lab, kw in (("default50", {}), ("repaired1000", {"distanceThresh": 1000.0})):
        if E is not None and E.shape == (3, 3):
            r = cv2.recoverPose(E, PTS1, PTS2, K, mask=mE.copy(), **kw)
            out[lab] = int(r[0])
        else:
            out[lab] = 0
    return out


cases = [("1.5 cm", 0.0, [0.015, 0, 0]), ("1.5 cm + 10° roll", 10.0, [0.015, 0, 0]), ("20 cm", 0.0, [0.20, 0, 0])]
case_res = [(n, run_case(rd, tv)) for n, rd, tv in cases]

# baseline sweep (0 roll), accepted fraction relative to N
bases = np.geomspace(0.005, 0.60, 28)
sweep = []
for b in bases:
    r = run_case(0.0, [b, 0, 0])
    sweep.append((b, r["E"] / N, r["default50"] / N, r["repaired1000"] / N))
sweep = np.array(sweep)

# real flight: F9 pose-inlier counts under the repaired threshold
real_rows = []
fig_counts = {}
for r in c.RUNS:
    d = c.run_dir("F9_L2", r)
    gt = pd.read_csv(d / "dataset_gt.csv"); t0, t1 = c.active_window(gt)
    vo = c.in_window(pd.read_csv(d / "raw_vo.csv"), t0, t1)
    fig_counts[r] = vo["num_inliers_pose"].values
    real_rows.append(dict(run=r, n_frames=len(vo), valid_pct=c.valid_pct(vo),
                          mean_inliers_pose=vo.num_inliers_pose.mean(), median_inliers_pose=vo.num_inliers_pose.median(),
                          frac_below_8=(vo.num_inliers_pose < 8).mean() * 100))

fig, ax = plt.subplots(1, 3, figsize=(S.COL2 + 0.6, 3.0), gridspec_kw=dict(width_ratios=[1.0, 1.15, 1.0]))

# (a)
a = ax[0]
labels = ["1.5 cm", "1.5 cm\n+10° roll", "20 cm"]
x = np.arange(len(labels)); w = 0.26
vals = {k: [r[k] for _, r in case_res] for k in ("E", "default50", "repaired1000")}
b1 = a.bar(x - w, vals["E"], w, color=S.C["grey"], label="findEssentialMat")
b2 = a.bar(x, vals["default50"], w, color=S.RAW_C, label="recoverPose, distanceThresh=50")
b3 = a.bar(x + w, vals["repaired1000"], w, color=S.GATED_C, label="recoverPose, distanceThresh=1000")
for bars in (b1, b2, b3):
    for r_ in bars:
        a.text(r_.get_x() + r_.get_width() / 2, r_.get_height() + 8, f"{int(r_.get_height())}", ha="center", fontsize=6.5, rotation=90)
a.set_xticks(x); a.set_xticklabels(labels, fontsize=8)
a.set_ylabel("Points accepted (of 500)"); a.set_ylim(0, 700)
a.legend(loc="upper left", bbox_to_anchor=(0.0, 1.0), ncol=1, fontsize=6.5, handlelength=1.2)
a.grid(axis="x", visible=False); S.panel(a, "a")

# (b)
b = ax[1]
b.plot(bases * 100, sweep[:, 1] * 100, color=S.C["grey"], lw=1.2, label="findEssentialMat")
b.plot(bases * 100, sweep[:, 2] * 100, color=S.RAW_C, lw=1.6, label="distanceThresh=50")
b.plot(bases * 100, sweep[:, 3] * 100, color=S.GATED_C, lw=1.6, ls="--", label="distanceThresh=1000")
b.set_xscale("log"); b.set_xlabel("Translation baseline (cm)"); b.set_ylabel("Points accepted (%)")
b.set_ylim(-4, 108)
# theory: depth in VO units = Z / baseline; cutoff crosses at baseline = Z / 50
zmin, zmax = Z.min(), Z.max()
b.axvspan(zmin / 50 * 100, zmax / 50 * 100, color=S.C["sky"], alpha=0.18, lw=0)
b.text(8.2, 8, "Z/baseline = 50\nfor Z = 2-10 m", fontsize=6.5, color=S.C["blue"], va="bottom", ha="center")
b.legend(loc="center left", bbox_to_anchor=(0.02, 0.32), fontsize=6.5); S.panel(b, "b")

# (c)
cc = ax[2]
allv = np.concatenate([fig_counts[r] for r in c.RUNS])
bins = np.arange(0, 2050, 50)
cc.hist(allv, bins=bins, color=S.GATED_C, alpha=0.85, edgecolor="white", lw=0.3)
cc.axvline(8, color=S.RAW_C, lw=1.4)
cc.text(30, cc.get_ylim()[1]*0.96, "orange line: validity threshold (8 inliers)", color=S.RAW_C, fontsize=6.5, va="top")
cc.set_xlabel("Pose inliers per frame"); cc.set_ylabel("Frames")
cc.set_xlim(-20, 2050); S.panel(cc, "c")

fig.subplots_adjust(wspace=0.38, bottom=0.2)
S.save(fig, "fig_01_recoverpose")

# sidecar
rows = []
for n, r in case_res:
    rows.append(dict(panel="a", case=n, findEssentialMat=r["E"], recoverPose_default50=r["default50"], recoverPose_repaired1000=r["repaired1000"]))
for row in sweep:
    rows.append(dict(panel="b", baseline_m=row[0], frac_findEssentialMat=row[1], frac_default50=row[2], frac_repaired1000=row[3]))
for r_ in real_rows:
    rows.append(dict(panel="c", **r_))
pd.DataFrame(rows).to_csv(S.DATA / "fig_01_recoverpose.csv", index=False)
for n, r in case_res: print(n, r)
print(pd.DataFrame(real_rows).round(2).to_string(index=False))

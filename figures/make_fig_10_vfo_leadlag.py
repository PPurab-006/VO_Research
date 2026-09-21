"""Fig 10 - Visual-field observatory (VFO) lead-lag: does any image-space variable lead pose failure?
F9 R1-R3 pooled, active window. Pearson r between predictor at frame t and target at frame t+k (predictor LEADS by k).
(a) continuous target: RAW num_inliers_pose.  (b) binary target: RAW num_inliers_pose < 8 (a failure frame).
Target saturation matters: only ~7% of F9 frames are failures, which limits what a null result can mean.
Bootstrap (frame-block, 1000 resamples, seed 0) gives a noise floor: cells whose 95% CI includes 0 are hatched.
"""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import style as S, common as c

PRED = ["feature_count", "feature_vel_mean", "texture_density", "mean_lk_err", "flow_coherence", "spatial_distribution_score",
        "flow_direction_entropy", "rotational_flow_ratio", "feature_survival_rate", "border_loss_pct",
        "estimated_rotational_flow_magnitude", "estimated_translational_flow_magnitude"]
NICE = {"feature_count": "Feature count", "feature_vel_mean": "Feature speed (mean)", "texture_density": "Texture density", "mean_lk_err": "Mean LK error",
        "flow_coherence": "Flow coherence", "spatial_distribution_score": "Spatial distribution", "flow_direction_entropy": "Flow-direction entropy",
        "rotational_flow_ratio": "Rotational-flow ratio", "feature_survival_rate": "Feature survival rate", "border_loss_pct": "Border loss (%)",
        "estimated_rotational_flow_magnitude": "Est. rotational flow", "estimated_translational_flow_magnitude": "Est. translational flow"}
LAGS = [0, 1, 2, 3, 5, 10]

runs = []
for r in c.RUNS:
    d = c.run_dir("F9_L2", r)
    gt = pd.read_csv(d / "dataset_gt.csv"); t0, t1 = c.active_window(gt)
    v = c.in_window(pd.read_csv(d / "vfo_timeseries.csv"), t0, t1)
    raw = c.in_window(pd.read_csv(d / "raw_vo.csv"), t0, t1)
    n = min(len(v), len(raw)); runs.append((v.iloc[:n], raw.iloc[:n]))

def pooled(p, k, binary):
    xs, ys = [], []
    for v, raw in runs:
        x = v[p].values.astype(float); y = raw["num_inliers_pose"].values.astype(float)
        if binary: y = (y < c.VALID_MIN_INLIERS).astype(float)
        xs.append(x[:len(x) - k] if k else x); ys.append(y[k:] if k else y)
    return np.concatenate(xs), np.concatenate(ys)

# variance guard: no silent zeros
for p in PRED:
    allx = np.concatenate([v[p].values for v, _ in runs])
    assert np.nanstd(allx) > 0, f"predictor {p} has zero variance"

rng = np.random.RandomState(0)
def corr_ci(p, k, binary, B=1000):
    x, y = pooled(p, k, binary); r = np.corrcoef(x, y)[0, 1]
    n = len(x); idx0 = np.arange(n); blk = 30
    bs = []
    for _ in range(B):
        starts = rng.randint(0, n - blk, size=int(np.ceil(n / blk)))
        ii = np.concatenate([np.arange(s, s + blk) for s in starts])[:n]
        bs.append(np.corrcoef(x[ii], y[ii])[0, 1])
    return r, np.percentile(bs, 2.5), np.percentile(bs, 97.5)

rows = []
mats = {}
for binary, key in ((False, "cont"), (True, "bin")):
    M = np.zeros((len(PRED), len(LAGS))); H = np.zeros_like(M, dtype=bool)
    for i, p in enumerate(PRED):
        for j, k in enumerate(LAGS):
            r, lo, hi = corr_ci(p, k, binary)
            M[i, j] = r; H[i, j] = (lo <= 0 <= hi)
            rows.append(dict(target="binary_failure" if binary else "continuous_inliers_pose", predictor=p, lag=k, r=r, ci_lo=lo, ci_hi=hi, ci_includes_zero=bool(H[i, j])))
    mats[key] = (M, H)

n_frames = sum(len(v) for v, _ in runs)
sat = np.mean(np.concatenate([(raw["num_inliers_pose"].values >= c.VALID_MIN_INLIERS) for _, raw in runs])) * 100
fig, ax = plt.subplots(1, 2, figsize=(S.COL2 + 0.6, 4.2), sharey=True)
order = np.argsort([-np.abs(mats["cont"][0][i]).max() for i in range(len(PRED))])
for a, key, title, let in zip(ax, ("cont", "bin"), ("Target: pose inliers (continuous)", "Target: failure frame (inliers < 8)"), "ab"):
    M, H = mats[key]; M = M[order]; H = H[order]
    im = a.imshow(M, cmap="RdBu_r", vmin=-0.25, vmax=0.25, aspect="auto")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            a.text(j, i, f"{M[i, j]:+.2f}", ha="center", va="center", fontsize=6.4, color=("white" if abs(M[i, j]) > 0.14 else "black"))
            if H[i, j]:
                a.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, hatch="////", edgecolor=(0, 0, 0, 0.22), lw=0))
    a.set_xticks(range(len(LAGS))); a.set_xticklabels(LAGS); a.set_xlabel("Lead k (frames)")
    a.set_title(title, fontsize=8.5); S.panel(a, let, dx=-0.02 if key == "cont" else -0.02, dy=1.06)
    a.grid(False)
ax[0].set_yticks(range(len(PRED))); ax[0].set_yticklabels([NICE[PRED[i]] for i in order], fontsize=7.4)
cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02); cb.set_label("Pearson r")
fig.text(0.5, -0.02, f"Hatched: 95% block-bootstrap CI includes 0.   N = {n_frames} frames (F9 R1-R3); {sat:.1f}% of frames are valid, so only {100-sat:.1f}% are failures.", ha="center", fontsize=7)
S.save(fig, "fig_10_vfo_leadlag")
out = pd.DataFrame(rows); out["n_frames"] = n_frames; out["valid_frame_pct"] = sat; out.to_csv(S.DATA / "fig_10_vfo_leadlag.csv", index=False)
top = out.reindex(out.r.abs().sort_values(ascending=False).index).head(6)
print("N =", n_frames, " valid % =", round(sat, 2)); print(top.round(4).to_string(index=False))
print("share of cells whose CI excludes 0:", round((~out.ci_includes_zero).mean() * 100, 1), "%  (", int((~out.ci_includes_zero).sum()), "of", len(out), ")")
print("max |r| continuous:", round(out[out.target.str.startswith('cont')].r.abs().max(), 4), " binary:", round(out[out.target.str.startswith('bin')].r.abs().max(), 4))

# ---- controls that decide whether "leading" is the right word --------------------------------------------------------------
def corr_dir(p, k, binary, direction):
    xs, ys = [], []
    for v, raw in runs:
        x = v[p].values.astype(float); y = raw["num_inliers_pose"].values.astype(float)
        if binary: y = (y < c.VALID_MIN_INLIERS).astype(float)
        if k == 0: xs.append(x); ys.append(y)
        elif direction == "predictor_leads": xs.append(x[:-k]); ys.append(y[k:])
        else: xs.append(x[k:]); ys.append(y[:-k])           # target leads predictor (reversed-time control)
    return float(np.corrcoef(np.concatenate(xs), np.concatenate(ys))[0, 1])
ctrl = []
for binary, key in ((False, "cont"), (True, "bin")):
    M_, _ = mats[key]
    for i in np.argsort(-np.abs(M_).max(axis=1))[:3]:
        p = PRED[i]
        for k in (3, 10):
            ctrl.append(dict(target="binary_failure" if binary else "continuous_inliers_pose", predictor=p, lag=k, r_lag0=corr_dir(p, 0, binary, "predictor_leads"),
                             r_predictor_leads=corr_dir(p, k, binary, "predictor_leads"), r_target_leads=corr_dir(p, k, binary, "target_leads")))
def episodes(m):
    out, k = [], 0
    for x in m:
        if x: k += 1
        elif k: out.append(k); k = 0
    if k: out.append(k)
    return out
ep = [l for _, raw in runs for l in episodes((raw["num_inliers_pose"].values < c.VALID_MIN_INLIERS))]
ctrl_df = pd.DataFrame(ctrl); ctrl_df["failure_frames"] = int(sum(ep)); ctrl_df["failure_episodes"] = len(ep)
ctrl_df["single_frame_episodes"] = int(sum(1 for x in ep if x == 1)); ctrl_df["max_episode_len"] = int(max(ep)); ctrl_df.to_csv(S.DATA / "fig_10_vfo_controls.csv", index=False)
print(ctrl_df.round(3).to_string(index=False))

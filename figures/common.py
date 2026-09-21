"""Shared data layer for every paper figure.

Every number plotted anywhere in figures/ comes from this module, which reads ONLY from
results/datasets/ and reuses the repo's own evaluation code (evaluate_dataset_mechanisms,
get_canonical_active_window). No figure recomputes a statistic on its own.
"""
from __future__ import annotations
import io, os, sys, contextlib, warnings
from pathlib import Path
import numpy as np
import pandas as pd
from scipy import stats
from scipy.spatial.transform import Rotation as R

warnings.filterwarnings("ignore")

REPO = Path(__file__).resolve().parents[1]
DATASETS = REPO / "results" / "datasets"
sys.path.insert(0, str(REPO / "src")); sys.path.insert(0, str(REPO / "src" / "core"))
sys.path.insert(0, str(REPO / "src" / "pipelines"))

# Core matrix: 8 families x 3 runs. Directory is p3_ if it exists, else phase2a_ (repo convention).
CORE = ["F1_L2", "F2_L2", "F4_L2", "F5_L2", "F6_L2", "F9_L2", "F10_L3", "F11_L2"]
EXPLORATORY = ["HOVER_L0", "F3_L2", "F7_L2", "F8_L2"]
RUNS = (1, 2, 3)
ACTIVE_Z = 2.0            # active window = GT rows with pos_z >= 2.0 (repo definition)
VALID_MIN_INLIERS = 8     # pose validity threshold (repo definition)
GLITCH_DEG = 10.0         # heading jump between consecutive GT rows treated as a data glitch


def run_dir(fam: str, r: int, exploratory: bool = False) -> Path:
    if exploratory:
        return DATASETS / f"p3x_{fam}_R{r}"
    p3 = DATASETS / f"p3_{fam}_R{r}"
    return p3 if p3.exists() else DATASETS / f"phase2a_{fam}_R{r}"


def source_of(fam: str, r: int) -> str:
    return run_dir(fam, r).name.split("_")[0]


def active_window(gt: pd.DataFrame):
    t = gt["timestamp_total_sec"].values.astype(float)
    idx = np.where(gt["pos_z"].values.astype(float) >= ACTIVE_Z)[0]
    return t[idx[0]], t[idx[-1]]


def in_window(df: pd.DataFrame, t0: float, t1: float) -> pd.DataFrame:
    t = df["timestamp_total_sec"].values
    return df[(t >= t0) & (t <= t1)].reset_index(drop=True)


def load_run(d: Path):
    gt = pd.read_csv(d / "dataset_gt.csv")
    return gt


def valid_pct(df_active: pd.DataFrame) -> float:
    return float((df_active["num_inliers_pose"] >= VALID_MIN_INLIERS).mean() * 100.0)


# --------------------------------------------------------------------------------------
# Heading / yaw-rate from ground truth, with data-glitch masking
# --------------------------------------------------------------------------------------
def heading_series(gt: pd.DataFrame):
    """Return (t, heading_deg) over the active window; duplicate timestamps dropped."""
    g = gt.drop_duplicates(subset="timestamp_total_sec").reset_index(drop=True)
    t0, t1 = active_window(g)
    a = g[(g.timestamp_total_sec >= t0) & (g.timestamp_total_sec <= t1)].reset_index(drop=True)
    v = R.from_quat(a[["rot_x", "rot_y", "rot_z", "rot_w"]].values).apply(np.array([1.0, 0.0, 0.0]))
    hd = np.degrees(np.unwrap(np.arctan2(v[:, 1], v[:, 0])))
    return a["timestamp_total_sec"].values.astype(float), hd, a


def glitch_report(t, hd):
    jumps = np.abs(np.diff(hd))
    bad = np.where(jumps > GLITCH_DEG)[0]
    return int(len(bad)), float(jumps.max()) if len(jumps) else 0.0, bad


def masked_heading(t, hd):
    """Linearly interpolate across samples adjacent to a >GLITCH_DEG heading jump."""
    n_bad, mj, bad = glitch_report(t, hd)
    h = hd.copy()
    if n_bad == 0:
        return h, n_bad, mj
    mask = np.zeros(len(h), bool)
    for i in bad:                       # jump between i and i+1: distrust both neighbours' rate
        mask[i:i + 2] = True
    keep = ~mask
    h[mask] = np.interp(t[mask], t[keep], h[keep])
    return h, n_bad, mj


def yaw_rate_grid(t, hd, dt=0.02):
    """|d heading/dt| on a uniform 50 Hz grid, after de-glitching."""
    h, n_bad, mj = masked_heading(t, hd)
    tg = np.arange(t[0], t[-1], dt)
    hg = np.interp(tg, t, h)
    return tg - tg[0], hg, np.abs(np.gradient(hg, dt)), n_bad, mj


# --------------------------------------------------------------------------------------
# Per-run metrics: reuse the repo's evaluation code verbatim
# --------------------------------------------------------------------------------------
_CACHE = REPO / "figures" / "data" / "_per_run_metrics.csv"


def per_run_metrics(force: bool = False) -> pd.DataFrame:
    if _CACHE.exists() and not force:
        return pd.read_csv(_CACHE)
    import run_phase3_full_eval as rf
    rows = []
    def add(fam, r, expl):
        d = run_dir(fam, r, expl)
        with contextlib.redirect_stdout(io.StringIO()):
            res = rf.evaluate_dataset_mechanisms(str(d), str(d / "dataset_gt.csv"))
        for mech, m in res.items():
            if m is None:
                continue
            rows.append(dict(family=fam, run=r, mechanism=mech, source_dir=d.name,
                             exploratory=expl, ate_rmse=m["ate_rmse"], rpe_t_m=m["rpe_t_mean"],
                             rpe_t_norm=m["rpe_t_norm"],
                             rpe_t_norm_valid=m.get("rpe_t_norm_valid", np.nan),
                             rpe_t_norm_starved=m.get("rpe_t_norm_starved", np.nan),
                             valid_pose_pct=m["valid_pose_pct"],
                             tracking_loss_pct=m["tracking_loss_pct"],
                             sim3_scale=m["sim3_scale"], n_active_frames=m["n_frames"],
                             drift_per_meter=m.get("drift_per_meter", np.nan)))
    for fam in CORE:
        for r in RUNS:
            add(fam, r, False)
    for fam in EXPLORATORY:
        for r in RUNS:
            add(fam, r, True)
    df = pd.DataFrame(rows)
    _CACHE.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(_CACHE, index=False)
    return df


def paired(df: pd.DataFrame, fam: str, metric: str, a="RAW", b="EIS-GATED", exploratory: bool = False):
    """Paired comparison b-a for one family/metric. Returns dict with k (b better), p values."""
    d = df[(df.family == fam) & (df.exploratory == exploratory)]
    x = d[d.mechanism == a].sort_values("run")[metric].values
    y = d[d.mechanism == b].sort_values("run")[metric].values
    diff = y - x
    better = int((diff > 0).sum() if metric == "valid_pose_pct" else (diff < 0).sum())
    t_p = float(stats.ttest_rel(y, x).pvalue) if len(x) > 1 else np.nan
    try:
        w_p = float(stats.wilcoxon(y, x).pvalue)
    except Exception:
        w_p = np.nan
    return dict(raw=x, gated=y, diff=diff, k=better, t_p=t_p, w_p=w_p, mean_diff=float(diff.mean()))


def gate_bypass_stats(fam: str, exploratory: bool = False):
    """(frames, bypassed) over the active window, pooled over the 3 runs, from the stored eis_gate_scale column.
    In the hard-gate mode the column is exactly 1 (derotate: |yaw rate| <= threshold) or 0 (bypass: identity warp)."""
    tot = byp = 0
    for r in RUNS:
        d = run_dir(fam, r, exploratory); gt = pd.read_csv(d / "dataset_gt.csv"); t0, t1 = active_window(gt)
        g = in_window(pd.read_csv(d / "eis_gated_vo.csv"), t0, t1)
        assert set(np.unique(np.round(g["eis_gate_scale"].values, 6))) <= {0.0, 1.0}, f"{d.name}: eis_gate_scale is not binary (soft-gate mode?)"
        tot += len(g); byp += int((g["eis_gate_scale"].values < 0.5).sum())
    return tot, byp

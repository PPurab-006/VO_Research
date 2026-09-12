#!/usr/bin/env python3
"""
Unit Test: VFO Lead-Lag Cross-Correlation Verification (Phase 2C Step 3 Falsification Check)

Verifies that the cross-correlation lead-lag analysis engine correctly identifies a known,
synthetic lead relationship (X leads Y by exactly 3 frames, i.e. lag=3).

Falsification Criterion:
If the analysis engine fails to identify lag=3 as the peak correlation on a controlled
synthetic signal, the cross-correlation logic is flawed and cannot be trusted on real flight data.
"""

import sys
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr


def compute_lead_lag_cross_correlations(x, y, max_lag=10):
    """
    Computes cross-correlation between predictor series X and outcome series Y for lags k in [0..max_lag].
    Lag k means X is measured k frames BEFORE Y (X leads Y by k frames).
    """
    results = {}
    N = len(y)
    for k in range(max_lag + 1):
        if k == 0:
            x_slice = x
            y_slice = y
        else:
            # X leads Y by k frames: x_slice is X[0 : N-k], y_slice is Y[k : N]
            x_slice = x[:-k]
            y_slice = y[k:]

        if len(x_slice) > 10:
            r, _ = pearsonr(x_slice, y_slice)
            rho, _ = spearmanr(x_slice, y_slice)
            results[k] = {'pearson_r': r, 'spearman_rho': rho}
    return results


def test_synthetic_lead_3():
    np.random.seed(42)
    N = 500
    
    # 1. Construct outcome series Y(t) with synthetic VO degradation events (drops in pose inliers)
    y = np.ones(N, dtype=np.float64) * 30.0 # Normal 30 pose inliers
    y[150:180] = 3.0                        # Drop event 1 (below 8 threshold)
    y[300:330] = 4.0                        # Drop event 2

    # Add mild baseline noise
    y += np.random.normal(0, 0.5, N)

    # 2. Construct predictor series X(t) that LEADS Y(t) by exactly 3 frames
    # When Y drops at t=150, X drops at t=147 (X(t) = Y(t+3))
    true_lead_frames = 3
    x = np.zeros(N, dtype=np.float64)
    x[:-true_lead_frames] = y[true_lead_frames:]
    x[-true_lead_frames:] = y[-1]
    
    # Add measurement noise to X
    x += np.random.normal(0, 1.0, N)

    # 3. Compute cross-correlations
    corrs = compute_lead_lag_cross_correlations(x, y, max_lag=10)

    # 4. Find peak lag
    best_lag = max(corrs.keys(), key=lambda k: abs(corrs[k]['pearson_r']))
    best_r = corrs[best_lag]['pearson_r']

    print(f"[TEST SYNTHETIC LEAD-LAG] True Lead: {true_lead_frames} frames")
    print(f"Lag Correlations:")
    for k, metrics in corrs.items():
        print(f"  Lag {k:2d} frames: Pearson r = {metrics['pearson_r']:+.4f}, Spearman rho = {metrics['spearman_rho']:+.4f}")

    print(f"\n[TEST RESULT] Identified Peak Lag: {best_lag} frames (Pearson r = {best_r:+.4f})")

    # Assertions
    assert best_lag == true_lead_frames, f"FALSIFIED: Expected peak lag={true_lead_frames}, but recovered lag={best_lag}"
    assert best_r > 0.90, f"FALSIFIED: Expected peak correlation > 0.90, but got {best_r}"
    print("✓ VERIFICATION PASSED: Lead-lag correlation engine correctly recovered exact synthetic lead lag=3.")


if __name__ == '__main__':
    test_synthetic_lead_3()

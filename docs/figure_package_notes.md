# Phase 2 figures — integration note for the main repo

## What this package is
Twelve paper figures (PNG 300 dpi + PDF), one sidecar CSV and one draft caption per figure, and the scripts that regenerate all of it.
Every plotted number comes from `results/datasets/` through `figures/common.py`, which reuses the repo's own evaluation code
(`evaluate_dataset_mechanisms`, `get_canonical_active_window`). No figure script computes a statistic of its own; Figs 7 and 9 assert that the
alignment they plot reproduces the reported ATE. Captions are generated from the sidecars (`make_captions.py`), so no caption number is typed by hand.

## How to merge
1. Copy `figures/` into the repo root (alongside `results/`, `src/`).
2. `results/analysis/threshold_sweep.csv` in this package is **transcribed by hand** from the sweep table printed during Gate 1 (the packaging author had no camera frames).
   **Keep the repo's own file and discard this one.** The repo's file must have columns `gate_thresh_deg, repeat, valid_pose_pct, ate_rmse`
   (Fig 5 stops with a clear message otherwise). Its 15 deg/s rows were checked against an independent recomputation and match; the other five thresholds were not.
3. Run `python3 figures/make_all.py` (about 1.5 minutes) then `python3 figures/make_captions.py`.
   Environment used here: Python 3.12, numpy 2.4.4, scipy 1.17.1, pandas 3.0.2, matplotlib 3.10.8, OpenCV 4.13, evo 1.37.0. Fig 1 recomputes the synthetic
   recoverPose test with whatever OpenCV is installed; it gave 0 / 500 / 500 here.
4. The repo's own `figures/` folder from the earlier agent run should be replaced, not merged.

## Repo statements that these results contradict (please correct, do not delete history — append dated correction blocks)
1. **Gate direction.** In `src/core/eis_preprocessor.py` mode `gated` bypasses derotation (H = I) when |yaw rate| > threshold and derotates
   (incremental) when |yaw rate| <= threshold. Any document, README or memo saying EIS becomes *more* active as yaw grows has it backwards.
   Consequence: a higher threshold means more derotated frames; the sweep (Fig 5) moves from RAW toward INCREMENTAL, as it should.
2. **15 deg/s is a chosen operating point.** RAW pose-loss is flat (4.6-8.6%) across yaw-rate bins with overlapping intervals; no knee (Fig 5a).
   Entry 34's "empirically derived" and "generalizes" wording and the "78-87%" claim are unsupported.
3. **failure_log Entry 47** (F6 58.2 +/- 1.1 deg, F10 78.4 +/- 1.8 deg peak to peak, heading bias 4.76 deg) is not supported by ground truth:
   mean heading offset is 77.8-90.7 deg and peak to peak is 116.8-129.8 deg (`figures/data/fig_03_achieved_yaw.csv`).
4. **GT heading discontinuities.** F6 R1, F6 R3 and F10 R2 have 4, 4 and 11 heading jumps > 10 deg between consecutive ground-truth samples (max 28.7 deg).
   Their peak yaw rates are artefacts. Not corrected or removed here. Also: across all 41 run directories, 11-47% (median 24%) of consecutive ground-truth rows are < 5 ms apart (exact duplicates included), so instantaneous GT speed and yaw rate are unreliable; figures use a 0.5 s baseline or a 50 Hz resampled grid.
   Consider re-recording those three runs; add them to KNOWN_LIMITATIONS.md either way.
5. **F6 vs F9** have nearly identical yaw profiles (~119 deg peak to peak); they differ in translation (path 7.6 m vs 29.2 m).
6. **Sim(3) scale test.** The report's 3.4275 m ATE comes from rescaling after alignment ("Method 1A"). Pre-scaling then rigid re-alignment gives 3.1923 m; letting the scale re-fit returns 3.0963 m.
   The test concerns RPE only and is F9 run 1 only (`fig_09_scale_artifact.csv`).
7. **recoverPose real-flight claim.** The 18.6% -> 90.43% figure exists only as report text. In `roll_validation_vo.csv`, `num_inliers` equals `num_inliers_E` on every frame, so no old-threshold
   real-flight measurement is reproducible from committed data. The synthetic result is reproducible. The repaired threshold also loses points below ~1 cm baseline (Fig 1b).
8. **VFO.** Correlations are contemporaneous, not predictive (reversed-time control in `fig_10_vfo_controls.csv`). Only 7.0% of F9 frames are failures, 145 of 146 episodes a single frame.
9. **Attitude source.** EIS uses simulator ground-truth attitude (`load_attitude_telemetry(gt_csv)`), not an estimated IMU. State this in the paper's limitations.
10. **Multiplicity.** 36 RAW-vs-GATED comparisons across Figs 6 and 11. Exactly one core comparison has p < 0.05 (F10 validity, GATED lower in 3/3 runs, p = 0.033).
    HOVER validity is the strongest single result (GATED higher 3/3, +5.9 pp, p = 0.009, exploratory, unadjusted). Neither survives correction for 36 tests.
11. **Report Section A vs Key Synthesis.** F10 numbers in the synthesis text contradict the report's own table (table is right; regenerate the text).
12. Pandas reads the bare string `NULL` as a missing value; the F9 ladder sidecar uses `EIS-NULL` for that reason. Avoid `NULL` as a category label in any CSV.

## Not verified by this package
- Sweep values other than the 15 deg/s row (see 2 above). Offline VO could not be re-run (no camera frames available to the author of this package).
- Figures were inspected as rendered images, not at journal print size.
- Clean-room check done: package extracted into an empty copy of the repo (datasets and src linked), `make_all.py` and `make_captions.py` run, all 12 figures built and all 28 sidecar/caption files were byte-identical to the shipped ones.
- Fig 10's bootstrap uses 30-frame blocks and 1000 resamples (seed 0); the block length is a judgement, not tuned.

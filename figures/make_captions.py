#!/usr/bin/env python3
"""Write figures/captions/fig_XX.md. Every number is read from a sidecar CSV in figures/data/ (rule: no number typed by hand)."""
import sys; sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import numpy as np, pandas as pd
import style as S
D = S.DATA; C_ = S.CAPTIONS
def rd(n): return pd.read_csv(D / n)
def w(n, txt): (C_ / f"fig_{n:02d}.md").write_text(txt.strip() + "\n")
f2 = lambda x: f"{x:.2f}"; f1 = lambda x: f"{x:.1f}"; f3 = lambda x: f"{x:.3f}"; f4 = lambda x: f"{x:.4f}"

# ---- Fig 1
d = rd("fig_01_recoverpose.csv"); a = d[d.panel == "a"]; b = d[d.panel == "b"].astype({"baseline_m": float}); cc = d[d.panel == "c"]
case = {r.case: r for r in a.itertuples()}
b = b.sort_values("baseline_m")
cross50 = b[b.frac_default50 >= 0.99].baseline_m.iloc[0] * 100
min_b = b.baseline_m.iloc[0] * 100; rep_min = b.frac_repaired1000.iloc[0] * 100
rep_full = b[b.frac_repaired1000 >= 0.99].baseline_m.iloc[0] * 100
n_all = int(cc.n_frames.sum()); v_all = float((cc.valid_pct * cc.n_frames).sum() / n_all)
w(1, f"""**Figure 1. A default depth cutoff in `cv2.recoverPose` silently discards valid points at small baselines.**
(a) Synthetic scene from `tests/test_essential_matrix_bookkeeping.py` (500 points, Z = 2-10 m, fixed seed). `findEssentialMat` accepts {int(case['1.5 cm'].findEssentialMat)}/500 points in every case. `recoverPose` with its default `distanceThresh` = 50 accepts {int(case['1.5 cm'].recoverPose_default50)}/500 at a 1.5 cm baseline (with or without 10° roll) and {int(case['20 cm'].recoverPose_default50)}/500 at 20 cm; with `distanceThresh` = 1000 it accepts {int(case['1.5 cm'].recoverPose_repaired1000)}/500 in all three cases.
(b) Fraction of points accepted against translation baseline. `recoverPose` normalises |t| to 1, so depth in VO units is Z / baseline and the 50-unit cutoff removes points until the baseline reaches about {f1(cross50)} cm (shaded: Z/baseline = 50 for Z = 2-10 m). The repaired threshold is not universal: at the smallest baseline tested ({f1(min_b)} cm) it accepts {f1(rep_min)}% of points, reaching 99% or more from {f1(rep_full)} cm.
(c) Pose-inlier counts per frame in real simulated flights (F9, three runs pooled, {n_all} frames, {f1(v_all)}% of frames with 8 or more inliers) under the repaired threshold. No old-threshold real-flight measurement exists in the committed data, so none is shown.""")

# ---- Fig 2
d = rd("fig_02_motion_families.csv").set_index("family")
w(2, f"""**Figure 2. The eight core motion families, from ground truth (one representative run each).**
Solid: heading change; dashed: speed over a 0.5 s baseline (ground-truth timestamps are jittered, so instantaneous speed is not meaningful). Shaded: the terminal transition, where altitude leaves the cruise band; it is part of the analysed window. F1, F2, F4 and F5 change heading by only {f1(d.loc[['F1_L2','F2_L2','F4_L2','F5_L2'],'heading_p2p_deg'].min())}-{f1(d.loc[['F1_L2','F2_L2','F4_L2','F5_L2'],'heading_p2p_deg'].max())}° peak to peak. F6 and F9 have nearly the same yaw profile ({f1(d.loc['F6_L2','heading_p2p_deg'])}° and {f1(d.loc['F9_L2','heading_p2p_deg'])}° peak to peak) and differ in translation: path length {f1(d.loc['F6_L2','path_length_m'])} m against {f1(d.loc['F9_L2','path_length_m'])} m. F6 uses run 2 because runs 1 and 3 contain ground-truth heading discontinuities (Fig. 3). † marks families recorded in the earlier `phase2a_` generation.""")

# ---- Fig 3
d = rd("fig_03_achieved_yaw.csv"); fl = d[d.n_heading_jumps_gt10deg > 0]
w(3, f"""**Figure 3. Achieved yaw for the rotation families (all three runs each).**
Top: unwrapped heading from the ground-truth body x-axis. Bottom: absolute yaw rate on a 50 Hz grid (axis clipped at 320 deg/s; dotted line: the 15 deg/s gate). Heading oscillates about a mean offset of {f1(d.mean_heading_offset_deg.min())}-{f1(d.mean_heading_offset_deg.max())}° from the start heading, with {f1(d.peak_to_peak_deg.min())}-{f1(d.peak_to_peak_deg.max())}° peak to peak. {len(fl)} of {len(d)} runs ({', '.join(f"{r.family[:-3]} R{r.run}" for r in fl.itertuples())}) contain ground-truth heading jumps above 10° between consecutive samples (largest {f1(fl.max_jump_deg.max())}°); they are drawn dashed and their peak rates ({f1(fl.peak_rate_degps.min())}-{f1(fl.peak_rate_degps.max())} deg/s) reflect these discontinuities, not vehicle motion. Median yaw rate on all runs is {f1(d.median_rate_degps.min())}-{f1(d.median_rate_degps.max())} deg/s.""")

# ---- Fig 4
d = rd("fig_04_f9_ladder.csv"); g = d.groupby("mechanism", sort=False).agg(v=("valid_pct", "mean"), a=("ate_rmse", "mean"), asd=("ate_rmse", "std"))
w(4, f"""**Figure 4. Effect of the EIS variant on F9 (yaw plus translation), three runs.**
Dots: individual runs; horizontal marks: means; dotted line: RAW mean. (a) Pose validity. FIXED (reference = first frame) falls to {f1(g.loc['FIXED','v'])}% and INCREMENTAL to {f1(g.loc['INCREMENTAL','v'])}%, against {f1(g.loc['RAW','v'])}% for RAW. NULL, which sends frames through the same resampling with an identity warp, gives {f1(g.loc['EIS-NULL','v'])}%, so resampling alone has no measurable cost. GATED gives {f1(g.loc['GATED','v'])}%. (b) ATE (Sim(3)-aligned). Means: RAW {f2(g.loc['RAW','a'])} m, FIXED {f2(g.loc['FIXED','a'])}, INCREMENTAL {f2(g.loc['INCREMENTAL','a'])}, NULL {f2(g.loc['EIS-NULL','a'])}, GATED {f2(g.loc['GATED','a'])}; the between-run standard deviation within a variant ({f2(g.asd.min())}-{f2(g.asd.max())} m) is comparable to or larger than the differences between variants.""")

# ---- Fig 5
bins = rd("fig_05_threshold_bins.csv"); sw = rd("fig_05_threshold_sweep.csv"); on = rd("fig_05_gate_on_fraction.csv")
ms = sw.groupby("gate_thresh_deg").agg(v=("valid_pose_pct", "mean"), a=("ate_rmse", "mean"), asd=("ate_rmse", "std"))
w(5, f"""**Figure 5. The 15 deg/s gate threshold is a chosen operating point, not an empirically derived optimum.**
(a) RAW pose-loss rate (frames with fewer than 8 pose inliers) against absolute yaw rate on F9 (three runs pooled; 95% bootstrap intervals; frame counts below). The loss rate is {f1(bins.loss_pct.min())}-{f1(bins.loss_pct.max())}% in every bin and the intervals overlap; there is no knee at 15 deg/s. (b, c) Offline replay of the gate on F9 at six thresholds (dots: runs; horizontal marks: means; dashed: RAW mean). In this implementation derotation is applied when the yaw rate is at or below the threshold and bypassed above it, so a higher threshold means more frames are derotated ({f1(on.frac_frames_eis_on.iloc[0]*100)}% of frames at {int(on.gate_thresh_deg.iloc[0])} deg/s, {f1(on.frac_frames_eis_on.iloc[-1]*100)}% at {int(on.gate_thresh_deg.iloc[-1])} deg/s). Validity falls from {f1(ms.v.iloc[0])}% to {f1(ms.v.iloc[-1])}% over that range, toward the always-on INCREMENTAL value ({f1(on.incremental_valid_pct.iloc[0])}%), by less than a uniform-harm interpolation would predict. Mean ATE ({f2(ms.a.min())}-{f2(ms.a.max())} m) does not separate the thresholds given a between-run standard deviation of {f2(ms.asd.min())}-{f2(ms.asd.max())} m. No threshold is marked as best.""")

# ---- Fig 6
ga = rd("fig_06_gate_activity.csv").set_index("family")
d = rd("fig_06_core_matrix.csv"); sig = d[d.paired_t_p < 0.05]
f9a = d[(d.family == "F9_L2") & (d.metric == "ate_rmse")].iloc[0]; f9n = d[(d.family == "F9_L2") & (d.metric == "rpe_t_norm")].iloc[0]
f10v = d[(d.family == "F10_L3") & (d.metric == "valid_pose_pct")].iloc[0]
w(6, f"""**Figure 6. RAW against EIS-GATED on the eight core families (three runs each).**
Dots: individual runs; lines join the same run under the two mechanisms; bars: family means. Labels give k, the number of runs (of 3) in which EIS-GATED is better, and the paired-t p-value (unadjusted). † marks {', '.join(sorted(x.replace('_L2','') for x in d[d.source=='phase2a'].family.unique()))}, recorded in the earlier `phase2a_` generation. Of {len(d)} comparisons, {len(sig)} has p < 0.05: F10 pose validity, where EIS-GATED is lower in all three runs (mean difference {f2(f10v.mean_diff_gated_minus_raw)} percentage points, p = {f3(f10v.paired_t_p)}); with {len(d)} tests this is about what chance alone produces. F9 (yaw plus translation): ATE k = {int(f9a.k_gated_better)}/3, p = {f3(f9a.paired_t_p)}; normalised RPE k = {int(f9n.k_gated_better)}/3, p = {f3(f9n.paired_t_p)}. The gate bypasses derotation on {f1(ga.loc['F6_L2','pct_bypassed'])}%, {f1(ga.loc['F9_L2','pct_bypassed'])}%, {f1(ga.loc['F10_L3','pct_bypassed'])}% and {f1(ga.loc['F11_L2','pct_bypassed'])}% of analysed frames in F6, F9, F10 and F11 and on none in F1, F2, F4 and F5, where EIS-GATED derotates every frame (equivalent to INCREMENTAL): the comparison in those four families is RAW against always-on incremental EIS. EIS-GATED did not significantly improve accuracy over RAW in this study (n = 3).""")

# ---- Fig 7
d = rd("fig_07_f9_trajectories.csv")
w(7, f"""**Figure 7. F9 trajectories, ground truth against RAW and EIS-GATED (Sim(3)-aligned, all three runs).**
Plan view; ATE is printed in the legend. EIS-GATED has lower ATE in runs {', '.join(str(int(r)) for r in d[d.gated_better].run)} ({', '.join(f"{f2(r.ate_gated)} against {f2(r.ate_raw)} m" for r in d[d.gated_better].itertuples())}) and higher ATE in run {', '.join(str(int(r)) for r in d[~d.gated_better].run)} ({', '.join(f"{f2(r.ate_gated)} against {f2(r.ate_raw)} m" for r in d[~d.gated_better].itertuples())}). The direction of the difference is not consistent across runs.""")

# ---- Fig 8
d = rd("fig_08_dt_artifact.csv")
def row(f, m): return d[(d.family == f) & (d.mechanism == m)].iloc[0]
w(8, f"""**Figure 8. Much of delayed triangulation's lower RPE comes from frames on which tracking has failed.**
F6, F9, F10; dots: runs, horizontal marks: means. (a) Pose validity: DELAYED-TRI {f1(row('F6_L2','DELAYED-TRI').valid_pct_mean)}%, {f1(row('F9_L2','DELAYED-TRI').valid_pct_mean)}% and {f1(row('F10_L3','DELAYED-TRI').valid_pct_mean)}% against {f1(row('F6_L2','EIS-GATED').valid_pct_mean)}%, {f1(row('F9_L2','EIS-GATED').valid_pct_mean)}% and {f1(row('F10_L3','EIS-GATED').valid_pct_mean)}% for EIS-GATED. (b) Scale-normalised RPE over the full window is lower for DELAYED-TRI ({f2(row('F6_L2','DELAYED-TRI').rpe_full_mean)}, {f2(row('F9_L2','DELAYED-TRI').rpe_full_mean)}, {f2(row('F10_L3','DELAYED-TRI').rpe_full_mean)} against {f2(row('F6_L2','EIS-GATED').rpe_full_mean)}, {f2(row('F9_L2','EIS-GATED').rpe_full_mean)}, {f2(row('F10_L3','EIS-GATED').rpe_full_mean)}). (c) Splitting frames by validity: on starved frames (fewer than 8 pose inliers) DELAYED-TRI's RPE is {f2(row('F6_L2','DELAYED-TRI').rpe_starved_mean)}, {f2(row('F9_L2','DELAYED-TRI').rpe_starved_mean)} and {f2(row('F10_L3','DELAYED-TRI').rpe_starved_mean)}, because a stale pose yields near-zero step error; on valid frames it is {f2(row('F6_L2','DELAYED-TRI').rpe_valid_mean)}, {f2(row('F9_L2','DELAYED-TRI').rpe_valid_mean)} and {f2(row('F10_L3','DELAYED-TRI').rpe_valid_mean)}, below EIS-GATED's ({f2(row('F6_L2','EIS-GATED').rpe_valid_mean)}, {f2(row('F9_L2','EIS-GATED').rpe_valid_mean)}, {f2(row('F10_L3','EIS-GATED').rpe_valid_mean)}) but computed on a much smaller and possibly easier set of frames. RPE for DELAYED-TRI must always be reported together with validity.""")

# ---- Fig 9
d = rd("fig_09_scale_artifact.csv").set_index("item").value
sc = {k: v for k, v in d.items() if k.startswith("sim3_scale")}
w(9, f"""**Figure 9. Meter-valued RPE is not comparable across runs because the Sim(3) scale differs.**
(a, b) F9 run 1 only. RAW's Sim(3) scale is {f4(d['s_RAW'])} and EIS-GATED's is {f4(d['s_GATED'])}. Meter RPE is {f4(d['meter_RPE_RAW'])} m/step for RAW and {f4(d['meter_RPE_GATED'])} for EIS-GATED; rescaling RAW by EIS-GATED's scale before a rigid alignment gives {f4(d['meter_RPE_RAW_prescaled_by_sGATED'])}, so the gap is a scale effect. Scale-normalised RPE (meter RPE divided by the scale, scale-invariant by construction) is {f4(d['normRPE_RAW'])} for RAW and {f4(d['normRPE_GATED'])} for EIS-GATED. (c) Sim(3) scale of every F9 run: values span {f3(min(sc.values()))}-{f3(max(sc.values()))}, and EIS-GATED's is larger than RAW's in all three runs. This test concerns RPE only. Forcing EIS-GATED's scale on RAW changes RAW's ATE from {f2(d['ATE_RAW'])} m to {f2(d['ATE_RAW_prescaled_by_sGATED_rigid_realign_(method_1B)'])} m (rigid re-alignment) or {f2(d['ATE_RAW_rescaled_after_alignment_(method_1A_archived_test)'])} m (rescale after alignment); this does not establish whether EIS-GATED's ATE difference is real (Fig. 6).""")

# ---- Fig 10
d = rd("fig_10_vfo_leadlag.csv"); ct = rd("fig_10_vfo_controls.csv")
cont = d[d.target.str.startswith("cont")]; bn = d[d.target.str.startswith("bin")]
w(10, f"""**Figure 10. No visual-field variable shows a leading relationship with pose failure in these data.**
Pearson r between each variable at frame t and (a) RAW pose inliers or (b) a failure indicator (fewer than 8 inliers) at frame t + k, F9 runs 1-3 pooled ({int(d.n_frames.iloc[0])} frames). The largest |r| is {f3(cont.r.abs().max())} for the continuous target and {f3(bn.r.abs().max())} for the failure indicator; {int((~d.ci_includes_zero).sum())} of {len(d)} cells have a 95% block-bootstrap interval excluding zero (unhatched). These correlations are not predictive: for the strongest cells the correlation with the target leading the variable by the same lag is of similar size (for example {f3(ct.iloc[0].r_predictor_leads)} against {f3(ct.iloc[0].r_target_leads)} at lag {int(ct.iloc[0].lag)}), i.e. they reflect the contemporaneous state of the scene. Only {f1(100 - d.valid_frame_pct.iloc[0])}% of frames are failures ({int(ct.failure_frames.iloc[0])} frames in {int(ct.failure_episodes.iloc[0])} episodes, {int(ct.single_frame_episodes.iloc[0])} of them a single frame), which limits what a null result can show. It does not show that no precursor exists.""")

# ---- Fig 11
d = rd("fig_11_exploratory.csv"); h = d[(d.family == "HOVER_L0") & (d.metric == "valid_pose_pct")].iloc[0]
ha = d[(d.family == "HOVER_L0") & (d.metric == "ate_rmse")].iloc[0]; hr = d[(d.family == "HOVER_L0") & (d.metric == "rpe_t_norm")].iloc[0]; hy = rd("fig_11_hover_yaw.csv")
w(11, f"""**Figure 11. Exploratory families (HOVER, F3, F7, F8): not aggregated with the core matrix.**
Same layout as Fig. 6; ATE on a log axis. p-values are unadjusted paired-t values with n = 3, and 36 comparisons are reported across Figs. 6 and 11. F3, F7 and F8 show no consistent difference. HOVER (a near-stationary condition) is the exception on pose validity: EIS-GATED is higher in {int(h.k_gated_better)}/3 runs (mean difference +{f1(h.mean_diff_gated_minus_raw)} percentage points, p = {f3(h.paired_t_p)}), while ATE (p = {f3(ha.paired_t_p)}) and normalised RPE (p = {f3(hr.paired_t_p)}) show no significant difference. In all four exploratory families the gate bypasses derotation on {f1(ga.loc[['HOVER_L0','F3_L2','F7_L2','F8_L2'],'pct_bypassed'].max())}% of frames, so EIS-GATED there is always-on incremental EIS. The result is not corrected for multiplicity, comes from a low-parallax scene with three runs, and has no tested mechanism.""")

# ---- Fig 12
d = rd("fig_12_system_diagram.csv").set_index("item").value
w(12, f"""**Figure 12. Processing pipeline for RAW and EIS-GATED.**
Frames from the simulated camera ({int(d.image_width)}×{int(d.image_height)} px, {f1(d.camera_rate_hz_measured)} Hz measured from frame timestamps) pass through an optional EIS stage, then KLT tracking, five-point essential-matrix RANSAC and `recoverPose` (`distanceThresh` = {int(d.recoverPose_distanceThresh_vo_units)} in VO units; a frame is valid with at least {int(d.validity_min_inliers)} pose inliers). RAW omits the EIS stage. In EIS-GATED the yaw rate is computed from consecutive attitude samples and the frame is derotated by the incremental relative rotation only when the yaw rate is at or below the threshold τ ({f1(d.gate_threshold_deg_per_s)} deg/s, a chosen operating point); above it the warp is the identity. Attitude is the simulator's ground-truth quaternion interpolated to frame time, not an estimated IMU attitude.""")
print("captions written:", sorted(p.name for p in C_.glob("fig_*.md")))

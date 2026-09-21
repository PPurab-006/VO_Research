# Patch to the figure package: gate activity (Fig 6 and captions 6, 11)
Replaces three files in figures/ (common.py, make_fig_06_core_matrix.py, make_captions.py). Then run:
    python3 figures/make_all.py && python3 figures/make_captions.py
New output: figures/data/fig_06_gate_activity.csv (pooled active-window frames on which the gate bypasses derotation, per family).
Fig 6 x-axis labels now show that percentage; captions 6 and 11 state it.
Reason: in the stored EIS-GATED results the gate never bypasses derotation in F1, F2, F4, F5 or any exploratory family
(eis_gate_scale is 1 on every frame), so EIS-GATED there is always-on incremental EIS. It bypasses 67-70% of frames in F6, F9, F10 and 6% in F11.
common.gate_bypass_stats() asserts eis_gate_scale is binary (hard-gate mode) and fails loudly otherwise.

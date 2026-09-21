# Robust Monocular Visual Odometry (VO) Under Agile UAV Motion

A ROS 2, Gazebo Sim, and PX4 SITL research framework for quantifying monocular Visual Odometry (VO) performance degradation and evaluating rotation-mitigation mechanisms during dynamic, agile multicopter flight.

---

## Executive Summary & Findings

Monocular Visual Odometry is standard for GPS-denied drone navigation but degrades under high rotational velocities ($\omega_z > 15^\circ/\text{s}$), aggressive maneuvers, and low-parallax flight regimes. 

This repository provides a complete, reproducible controlled-experiment benchmark suite evaluating three VO mechanisms across 36 flight datasets ($n=3$ repeats per cell):
1. **`RAW`**: Un-fused 5-point Essential Matrix RANSAC + Pyramidal KLT feature tracking.
2. **`EIS-GATED`**: Reactive attitude-rate-gated Electronic Image Stabilization (homography derotation active when $|\omega_z| > 15.0^\circ/\text{s}$).
3. **`DELAYED-TRI`**: Rotation-decoupled feature buffering delaying 3D landmark triangulation until rotation subsides.

### Key Conclusions:
- **`EIS-GATED` is the superior mechanism**: It matches `RAW` baseline accuracy on translation-dominant flights (F1, F2, F4: ATE $0.68 - 1.12\text{m}$, valid pose rate $>98\%$) while recovering $78-87\%$ of lost validity on coupled and rotational flights (F5, F6, F9, F10: validity $92.3\% - 94.1\%$).
- **`DELAYED-TRI` suffers structural feature starvation**: Without inertial fusion or long-term temporal buffers, sustained yaw bursts continuously purge pending feature tracks before triangulation criteria can be met (validity collapses to $42.9\% - 55.8\%$).

---

## Project Phase Breakdown

- **Phase 0 — Infrastructure & Calibration (COMPLETE)**:
  - Pinhole camera verification ($30.4\text{ Hz}$, $1280 \times 960$), Ground Truth pose sync ($\le 20\text{ ms}$), and Sim(3) Umeyama scale alignment.
- **Phase 1 — Baseline Flight Matrix (COMPLETE)**:
  - Standardized benchmark sweep across 8 motion families (F1–F11) in Gazebo `agriculture.world`.
- **Phase 2 — Fault Injection & Mechanism Design (COMPLETE)**:
  - Developed `EIS-GATED` ($\omega_{\text{thresh}} = 15.0^\circ/\text{s}$), scale-normalized RPE ($RPE_{\text{norm}} = RPE/s$), and Visual Field Observatory (VFO) diagnostic engine.
- **Phase 3 — Head-to-Head Benchmark Evaluation (COMPLETE)**:
  - Full matrix evaluation across 8 core families + 4 exploratory families ($N=36$ flights). Definitive scientific report published in `results/reports/phase3/phase3_controlled_experiment_report.md`.

---

## Directory Architecture

```
ROS/
├── configs/                       # World models, ROS 2 bridge configs & intrinsics
├── docs/                          # Methodology, research questions & technical logs
│   ├── SSRF_Research_Plan.docx
│   ├── research_question.md
│   ├── methodology.md
│   ├── environment.md
│   └── failure_log.md             # Master Technical Journal (Entries 1-49)
├── plots/                         # Output visualization charts
├── results/                       # Experimental data outputs & reports
│   ├── datasets/                  # Recorded flight datasets (CSV logs per run)
│   ├── data_tables/               # Tabular audit summaries & JSON manifests
│   └── reports/                   # Markdown scientific research reports
│       ├── phase1/
│       ├── phase2/
│       └── phase3/                # Phase 3 Controlled Experiment Report
├── src/                           # Main Python library and production pipelines
│   ├── core/                      # VO algorithms & preprocessors
│   ├── flight/                    # MAVLink SITL flight controllers
│   ├── pipelines/                 # Batch evaluation entrypoints
│   └── archive_phase0/            # Prototype archival scripts
├── tests/                         # Automated test suite
└── README.md
```

---

## Environment Specifications

- **OS**: Ubuntu 22.04 / 26.04 LTS
- **ROS Distribution**: ROS 2 `humble` / `lyrical`
- **Simulator**: Gazebo Sim / Tools `10.4.0` (Ogre2 engine, NVIDIA GPU offload)
- **Autopilot**: PX4 SITL `v1.18.0-beta1-209-g8aba32c862`
- **Python**: Python 3.10+ / 3.14 (`opencv-python` 4.10, `evo` 1.37.0, `scipy` 1.18.0, `numpy`)

---

## Primary Document References

- **Phase 3 Controlled Experiment Report**: [results/reports/phase3/phase3_controlled_experiment_report.md](results/reports/phase3/phase3_controlled_experiment_report.md)
- **Master Failure Log & Technical Journal**: [docs/failure_log.md](docs/failure_log.md)
- **Research Methodology**: [docs/methodology.md](docs/methodology.md)
- **Environment Setup**: [docs/environment.md](docs/environment.md)

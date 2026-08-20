# Failure Log & Technical Journal

## Template
| Date | Problem / Goal | Hypothesis | What I Changed | Result | What I Learned |
| :--- | :--- | :--- | :--- | :--- | :--- |

---

## Log Entries

### Entry 1: 2026-08-20 — Workspace & Environment Scaffolding
- **Date**: 2026-08-20
- **Problem / Goal**: Set up Phase 0 infrastructure for Robust Monocular VO project. Detect and freeze environment specs, create directory layout, install missing dependencies (`evo`, `scipy`), and establish baseline repository.
- **Hypothesis**: Documenting exact environment specifications (Ubuntu 26.04, ROS2 Lyrical, Gazebo 10.4.0, PX4 SITL `g8aba32c862`) and scaffolding directory before writing research code prevents configuration drift and non-reproducible bugs.
- **What I Changed**: Created initial project structure (`research_question.md`, `methodology.md`, `environment.md`, `failure_log.md`, and directory placeholders for `src/`, `configs/`, `experiments/`, `results/`, `plots/`, `report/`). Installed and verified python dependencies (`evo` 1.37.0, `scipy` 1.18.0, `colcon-common-extensions`).
- **Result**: Phase 0 scaffolding established, dependencies installed, and environment frozen.
- **What I Learned**: Environment versions pinned: Ubuntu 26.04 LTS, ROS2 Lyrical, Gazebo 10.4.0, PX4 SITL `v1.18.0-beta1-209-g8aba32c862`, Python 3.14.4, `evo` 1.37.0, `scipy` 1.18.0.

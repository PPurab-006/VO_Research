# Known Limitations & Pipeline Fixes Log

## Pipeline Source Repairs

### `src/pipelines/run_offline_vo.py` (Script Import Fixes)
- **Modification**: Added explicit missing top-level package imports (`os`, `sys`, `cv2`, `numpy as np`, `pandas as pd`, `from scipy.spatial.transform import Rotation as R_scipy`).
- **Impact Assessment**: **Zero numerical change**. The script's algorithmic logic, tracking parameters, RANSAC thresholds, and coordinate transformations remain 100% identical to the original offline VO pipeline.

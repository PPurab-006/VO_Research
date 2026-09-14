#!/usr/bin/env python3
"""
VFO Batch Extraction Script (Phase 2C Step 2)

Extracts VFO variable time-series across all 7 recorded flight datasets:
  - phase2a_F5_L2_R1
  - phase2a_F5_L2_R2
  - phase2a_F5_L2_R3
  - phase2a_F6_L2
  - phase2a_F9_L2_R1
  - phase2a_F9_L2_R2
  - phase2a_F9_L2_R3

Processes raw (unwarped) grayscale camera frames and outputs per-frame
vfo_timeseries.csv for each dataset directory.
"""

import os
import sys
import csv
import cv2
import numpy as np
import pandas as pd

from eis_derotation import EISDerotator
from vfo_observatory import VisualFieldObservatory


def process_dataset(run_id, base_dir="results/datasets"):
    dataset_dir = os.path.join(base_dir, run_id)
    cam_csv_path = os.path.join(dataset_dir, 'camera_frames.csv')
    gt_csv_path = os.path.join(dataset_dir, 'dataset_gt.csv')
    output_csv_path = os.path.join(dataset_dir, 'vfo_timeseries.csv')

    if not os.path.exists(cam_csv_path) or not os.path.exists(gt_csv_path):
        print(f"[VFO EXTRACTION ERROR] Missing dataset files in '{dataset_dir}'")
        return False

    print(f"\n============================================================")
    print(f"[VFO EXTRACTION] Processing dataset: {run_id}")
    print(f"============================================================")

    # Initialize EISDerotator to provide attitude telemetry interpolation for optical flow decomposition
    eis_derotator = EISDerotator(reference_mode='incremental')
    eis_derotator.load_attitude_telemetry(gt_csv_path)

    vfo = VisualFieldObservatory(eis_derotator=eis_derotator)
    df_cam = pd.read_csv(cam_csv_path)

    records = []
    for idx, row in df_cam.iterrows():
        img_filename = row['filename']
        img_path = os.path.join(dataset_dir, 'images', img_filename)
        cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

        total_sec = float(row['timestamp_total_sec'])
        rec = vfo.process_frame(cv_img, total_sec)
        records.append(rec)

    # Save to vfo_timeseries.csv
    df_out = pd.DataFrame(records)
    df_out.to_csv(output_csv_path, index=False)
    print(f"[VFO COMPLETE] Saved {len(records)} VFO time-series frames to '{output_csv_path}'")
    return True


def main():
    datasets = [
        'phase2a_F5_L2_R1',
        'phase2a_F5_L2_R2',
        'phase2a_F5_L2_R3',
        'phase2a_F6_L2',
        'phase2a_F9_L2_R1',
        'phase2a_F9_L2_R2',
        'phase2a_F9_L2_R3'
    ]

    success_count = 0
    for run_id in datasets:
        if process_dataset(run_id):
            success_count += 1

    print(f"\n[VFO BATCH COMPLETE] Successfully processed {success_count}/{len(datasets)} datasets.")


if __name__ == '__main__':
    main()

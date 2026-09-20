"""
Shared matplotlib/seaborn styling configuration for publication figures.
Enforces colorblind-safe palettes, >= 9pt font sizes, 3.4in / 7.0in widths,
PNG + PDF output generation, and clean aesthetic standards.
"""

from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
FIG_OUT_DIR = REPO_ROOT / "figures" / "out"
FIG_DATA_DIR = REPO_ROOT / "figures" / "data"
FIG_CAPTION_DIR = REPO_ROOT / "figures" / "captions"
FIG_PREVIEW_DIR = REPO_ROOT / "figures" / "preview"

FIG_OUT_DIR.mkdir(parents=True, exist_ok=True)
FIG_DATA_DIR.mkdir(parents=True, exist_ok=True)
FIG_CAPTION_DIR.mkdir(parents=True, exist_ok=True)
FIG_PREVIEW_DIR.mkdir(parents=True, exist_ok=True)

# Colorblind-safe palette (Okabe-Ito inspired)
COLOR_RAW = "#D55E00"         # Vermillion / Red-Orange
COLOR_GATED = "#009E73"       # Bluish Green
COLOR_FIXED = "#E69F00"       # Orange
COLOR_INCREMENTAL = "#56B4E9" # Sky Blue
COLOR_NULL = "#CC79A7"        # Reddish Purple
COLOR_ALT = "#0072B2"         # Blue
COLOR_GRAY = "#777777"        # Neutral Gray

COLOR_PALETTE = [COLOR_RAW, COLOR_GATED, COLOR_ALT, COLOR_FIXED, COLOR_INCREMENTAL, COLOR_NULL]

def setup_style():
    """Apply global matplotlib style rules."""
    sns.set_theme(style="ticks", palette="colorblind")
    
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Inter", "Helvetica", "Arial", "DejaVu Sans"],
        "font.size": 9.5,
        "axes.labelsize": 10.0,
        "axes.titlesize": 10.5,
        "xtick.labelsize": 9.0,
        "ytick.labelsize": 9.0,
        "legend.fontsize": 8.5,
        "figure.titlesize": 11.0,
        "lines.linewidth": 1.5,
        "lines.markersize": 5.0,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linestyle": "--",
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.05,
    })

def create_figure(width_type="double", aspect_ratio=0.6, nrows=1, ncols=1, sharex=False, sharey=False):
    """
    Create a figure with exact width constraints.
    width_type: 'single' (3.4 in) or 'double' (7.0 in)
    """
    setup_style()
    width = 3.4 if width_type == "single" else 7.0
    height = width * aspect_ratio
    
    fig, axes = plt.subplots(nrows, ncols, figsize=(width, height), sharex=sharex, sharey=sharey)
    
    if nrows * ncols == 1:
        sns.despine(ax=axes)
    else:
        for ax in np.array(axes).flat:
            sns.despine(ax=ax)
            
    return fig, axes

def save_fig_and_sidecar(fig, base_filename, df_sidecar, caption_md):
    """
    Saves figure to figures/out/<base_filename>.png AND .pdf (300 dpi),
    saves preview to figures/preview/<base_filename>.png (150 dpi),
    saves sidecar CSV to figures/data/<base_filename>.csv,
    and writes caption to figures/captions/<fig_id>.md.
    """
    png_path = FIG_OUT_DIR / f"{base_filename}.png"
    pdf_path = FIG_OUT_DIR / f"{base_filename}.pdf"
    preview_path = FIG_PREVIEW_DIR / f"{base_filename}.png"
    csv_path = FIG_DATA_DIR / f"{base_filename}.csv"
    
    # Extract figure ID (e.g. fig_01 from fig_01_essential_matrix_repair)
    fig_id = "_".join(base_filename.split("_")[:2])
    caption_path = FIG_CAPTION_DIR / f"{fig_id}.md"

    fig.savefig(png_path, dpi=300, bbox_inches="tight", pad_inches=0.05)
    fig.savefig(pdf_path, dpi=300, bbox_inches="tight", pad_inches=0.05)
    fig.savefig(preview_path, dpi=150, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)

    df_sidecar.to_csv(csv_path, index=False)
    caption_path.write_text(caption_md.strip() + "\n")

    print(f"Saved 300dpi PNG: {png_path.relative_to(REPO_ROOT)}")
    print(f"Saved 150dpi PNG: {preview_path.relative_to(REPO_ROOT)}")
    print(f"Saved PDF        : {pdf_path.relative_to(REPO_ROOT)}")
    print(f"Saved Sidecar    : {csv_path.relative_to(REPO_ROOT)}")
    print(f"Saved Caption    : {caption_path.relative_to(REPO_ROOT)}")


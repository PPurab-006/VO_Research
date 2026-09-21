"""Shared figure style. Colorblind-safe (Okabe-Ito), >=9 pt, journal column widths, no in-figure titles."""
from pathlib import Path
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "figures" / "out"
DATA = REPO / "figures" / "data"
CAPTIONS = REPO / "figures" / "captions"
for _p in (OUT, DATA, CAPTIONS):
    _p.mkdir(parents=True, exist_ok=True)

# Okabe-Ito
C = dict(black="#000000", orange="#E69F00", sky="#56B4E9", green="#009E73", yellow="#F0E442",
         blue="#0072B2", vermilion="#D55E00", purple="#CC79A7", grey="#7F7F7F")
RAW_C, GATED_C, DT_C = C["vermilion"], C["green"], C["blue"]
COL1, COL2 = 3.4, 7.0   # inches

mpl.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8.5, "legend.fontsize": 8, "legend.frameon": False,
    "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": 0.8,
    "lines.linewidth": 1.4, "savefig.dpi": 300, "figure.dpi": 150, "pdf.fonttype": 42,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5, "axes.axisbelow": True,
})


def panel(ax, letter, dx=-0.16, dy=1.04):
    ax.text(dx, dy, f"({letter})", transform=ax.transAxes, fontsize=10, fontweight="bold", va="bottom")


def save(fig, name):
    fig.savefig(OUT / f"{name}.png", bbox_inches="tight")
    fig.savefig(OUT / f"{name}.pdf", bbox_inches="tight", metadata={"CreationDate": None, "Creator": "matplotlib"})
    plt.close(fig)


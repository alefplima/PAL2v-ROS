"""Evidence panels of the article: close-up Gazebo captures (Mean, PAL2v, Kalman χ²) + time series of the same run.

  python3 evidence_panels.py            # E4 and E5, scenarios S4_dropout and S6_duplo
Input:  results/evidencias/e{4,5}_gz_<method>_<scenario>_{tNN,amplo}.png and .csv (run_prints.sh)
        (evidencias = evidence; amplo = wide view)
Output: results/evidencias/panel_e4_<scenario>.png, panel_e5_<scenario>.png
"""
import csv
import glob
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EV = os.path.join(HERE, "results", "evidencias")
NAMES = {"mean": "Mean", "pal2v": "PAL2vFusion", "kf_chi2": "Kalman χ²"}
COLORS = {"mean": "#c0392b", "pal2v": "#1b7f5c", "kf_chi2": "#a0522d"}
METHODS = ["mean", "kf_chi2", "pal2v"]
CFG = {
    "e4": {"win": (15, 45), "ylab": "frontal distance [m]", "ylim": (-0.1, 3.7), "what": "distance",
           "title": "E4 — TurtleBot3 in Gazebo (reactive avoidance): fused frontal distance fed back to the controller",
           "fault": {"S4_dropout": "LiDAR dropout", "S6_duplo": "LiDAR dropout + stuck IR"},
           "legend": "in the scene: spheres = true trajectory (yellow during the fault); colored ray = fused distance "
                     "(used by the controller); gray ray = true distance to the obstacle"},
    "e5": {"win": (8, 18), "ylab": "height [m]", "ylim": (-0.2, 6.5), "what": "height",
           "title": "E5 — PX4 SITL (x500) in Gazebo: fused altitude fed back to the controller",
           "fault": {"S4_dropout": "LiDAR dropout", "S6_duplo": "LiDAR dropout + GPS jump"},
           "legend": "in the scene: spheres = true trajectory (yellow during the fault); wall = height × time, yellow "
                     "band = fault window"},
}


def crop(img):
    """Gazebo 3D area: below the toolbar and left of the side panel."""
    a = np.asarray(img.convert("RGB")).astype(int)
    h, w = a.shape[:2]
    orange = (abs(a[..., 0] - 255) < 25) & (abs(a[..., 1] - 87) < 35) & (a[..., 2] < 80)
    rows = np.nonzero(orange.sum(axis=1) > 400)[0]
    if len(rows):                                   # Jetty: orange bar (window not maximized)
        xs = np.nonzero(orange[rows[0]])[0]
        left, right, top = xs.min(), xs.max(), rows.max() + 100
        return img.crop((left + 5, top, left + int((right - left) * 0.645), min(top + 590, h - 30)))
    return img.crop((int(w * 0.04), int(h * 0.22), int(w * 0.69), int(h * 0.90)))   # Harmonic, maximized


def shot_times():
    """Actual instant of each capture, read from the run_prints.sh log (may lag behind the requested one)."""
    import re
    out = {}
    p = os.path.join(EV, "run_prints.log")
    for line in open(p) if os.path.exists(p) else []:
        m = re.search(r"(?:captura|capture): .*/([^/]+)\.png \(t_(?:sim|voo|flight)=([0-9.]+)", line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


TIMES = shot_times()


def load(path):
    r = list(csv.DictReader(open(path)))
    return {k: np.array([float(x[k]) for x in r]) for k in r[0] if k != "rejected"}


def panel(exp, scen):
    c = CFG[exp]
    meth = [m for m in METHODS if os.path.exists(os.path.join(EV, f"{exp}_gz_{m}_{scen}.csv"))]
    if not meth:
        return None
    shots = sorted({os.path.basename(p).rsplit("_", 1)[1][:-4]
                    for m in meth for p in glob.glob(os.path.join(EV, f"{exp}_gz_{m}_{scen}_*.png"))},
                   key=lambda s: (s == "amplo", s))
    fig = plt.figure(figsize=(4.1 * len(shots), 3.1 * len(meth) + 3.3))
    gs = fig.add_gridspec(len(meth) + 1, len(shots), height_ratios=[1] * len(meth) + [1.05],
                          hspace=0.32, wspace=0.03,
                          top=1 - 0.95 / (3.1 * len(meth) + 3.3), bottom=0.06)
    data = {}
    for r, m in enumerate(meth):
        d = data[m] = load(os.path.join(EV, f"{exp}_gz_{m}_{scen}.csv"))
        for col, s in enumerate(shots):
            ax = fig.add_subplot(gs[r, col])
            p = os.path.join(EV, f"{exp}_gz_{m}_{scen}_{s}.png")
            if os.path.exists(p):
                ax.imshow(crop(Image.open(p)))
            ax.set_xticks([]), ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_edgecolor(COLORS[m]), sp.set_linewidth(2.5)
            if s == "amplo":
                ax.set_title(f"{NAMES[m]} · wide view (end)", fontsize=9, color="0.15")
            else:
                t = TIMES.get(f"{exp}_gz_{m}_{scen}_{s}", float(s[1:]))
                k = int(np.argmin(np.abs(d["t"] - t)))
                tag = " · FAULT" if c["win"][0] <= t < c["win"][1] else ""
                ax.set_title(f"{NAMES[m]} · t = {t:.1f} s{tag}\ntrue {c['what']} {max(d['gt'][k], 0):.2f} m · "
                             f"fused {max(d['fused'][k], 0):.2f} m", fontsize=9, color="0.15")
    ax = fig.add_subplot(gs[-1, :])
    w0, w1 = c["win"]
    ax.axvspan(w0, w1, color="#f9e79f", alpha=0.6, lw=0)
    ax.text((w0 + w1) / 2, c["ylim"][1] * 0.93, c["fault"][scen], ha="center", fontsize=9, color="0.3")
    for m in meth:
        d = data[m]
        ax.plot(d["t"], d["gt"], color=COLORS[m], lw=2, label=f"{NAMES[m]} — true")
        ax.plot(d["t"], d["fused"], color=COLORS[m], lw=1, ls=":", label=f"{NAMES[m]} — fused")
        if "contact" in d:
            hit = d["contact"] > 0
            ax.plot(d["t"][hit], np.full(hit.sum(), c["ylim"][0] + 0.05), "|", color=COLORS[m], ms=10)
    if "ref" in data[meth[0]]:
        ax.plot(data[meth[0]]["t"], data[meth[0]]["ref"], color="0.45", lw=1, ls="--", label="reference")
    ax.set_xlim(0, max(d["t"].max() for d in data.values()))
    ax.set_ylim(*c["ylim"])
    ax.set_xlabel("time [s]" + ("   (ticks at the bottom = contact/collision)" if exp == "e4" else ""))
    ax.set_ylabel(c["ylab"])
    ax.grid(axis="y", alpha=0.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(fontsize=8, ncol=len(meth) + 1, loc="upper left", frameon=False, bbox_to_anchor=(0, -0.2))
    fig.suptitle(f"{c['title']} — {scen} ({c['fault'][scen]})\n{c['legend']}", fontsize=11,
                 y=1.0)
    out = os.path.join(EV, f"panel_{exp}_{scen}.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out


if __name__ == "__main__":
    for e in ("e4", "e5"):
        for s in ("S4_dropout", "S6_duplo"):
            print(panel(e, s))

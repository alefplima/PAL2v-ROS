"""Utilities shared by the experiments: paths, reference fusion methods, statistics."""
from __future__ import annotations

import json
import os
import platform
import sys
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "lpa2v_core"))
RESULTS = os.path.join(HERE, "results")
os.makedirs(RESULTS, exist_ok=True)

import lpa2v  # noqa: E402

TEST_SEEDS = list(range(30))          # evaluation seeds (Monte Carlo)
TUNE_SEEDS = list(range(100, 105))    # seeds used ONLY for tuning (separate from evaluation)


def env_info() -> dict:
    import scipy
    return {"date": datetime.now().isoformat(timespec="seconds"),
            "python": sys.version.split()[0], "numpy": np.__version__, "scipy": scipy.__version__,
            "lpa2v": lpa2v.__version__, "platform": platform.platform(),
            "cpu": platform.processor()}


def save_json(name: str, obj) -> str:
    p = os.path.join(RESULTS, name)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False, default=float)
    return p


# --- reference fusion methods (stateless) -----------------------------------------------
def fuse_mean(x, sigma):
    return float(np.mean(x))


def fuse_median(x, sigma):
    return float(np.median(x))


def fuse_ivw(x, sigma):
    """Inverse-variance weighted mean (optimal for Gaussian noise, no fault detection)."""
    w = 1.0 / np.asarray(sigma) ** 2
    return float(np.sum(w * np.asarray(x)) / np.sum(w))


def summarize(values) -> dict:
    v = np.asarray(values, dtype=float)
    return {"mean": float(v.mean()), "std": float(v.std(ddof=1)), "median": float(np.median(v)),
            "min": float(v.min()), "max": float(v.max()), "n": int(v.size)}


def wilcoxon(a, b) -> float:
    """p-value of the Wilcoxon signed-rank test (samples paired by seed)."""
    from scipy.stats import wilcoxon as w
    a, b = np.asarray(a), np.asarray(b)
    if np.allclose(a, b):
        return 1.0
    return float(w(a, b).pvalue)

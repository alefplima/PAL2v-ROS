"""E5 - analysis of the PX4 SITL batch (results/e5_runs) -> results/e5_px4.json, e5_table.md and figures.

Metrics per flight (closed loop: the fused altitude is the drone's only height feedback):
  fusion RMSE in the fault window [m] (fused vs simulator ground truth),
  max. trajectory deviation [m]: |true height - mean nominal trajectory (S0, all methods)| in the fault
      window + 5 s (measures the effect of the fault on the flight, discounting the normal takeoff transient),
  ground touch: true height < 0.15 m after takeoff (t > 8 s),
  detection / false alarm (PAL2v and Kalman χ²): % of samples with the faulty sensor(s) rejected
      inside the window; % with any rejection outside it.
Open loop (paired): the 6 methods applied to the SAME sensor data of the PAL2v flights.
Statistics: Wilcoxon paired by seed against the PAL2vFusion (RMSE in the fault).
JSON keys: resumo = summary, malha_aberta = open loop, por_voo = per flight, janelas = windows,
toque_chao = ground touch, desvio_max = max deviation, alt_min_falha = min altitude in fault, amostras = samples,
deteccao = detection, falso_alarme = false alarm.
"""
import csv
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import env_info, fuse_ivw, fuse_mean, fuse_median, wilcoxon  # noqa: E402
from lpa2v import PAL2vFusion, para_extractor  # noqa: E402
from lpa2v.baselines import KalmanChi2  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
RUNS = os.path.join(RES, "e5_runs")
METHODS = ["pal2v_g", "pal2v", "mean", "median", "ivw", "paraextrctr", "kf_chi2"]
REF = "pal2v_g"                       # main method (graded, no exclusion); "pal2v" = step, v1.0.0
NAMES = {"pal2v_g": "PAL2vFusion (graded)", "pal2v": "PAL2vFusion step (v1.0)", "mean": "Mean", "median": "Median", "ivw": "IVW", "paraextrctr": "ParaExtrctr",
         "kf_chi2": "Kalman χ²"}
COLORS = {"pal2v_g": "#1b7f5c", "pal2v": "#e6ab02", "mean": "#c0392b", "median": "#7d3c98", "ivw": "#d68910", "paraextrctr": "#2e86c1",
          "kf_chi2": "#a0522d"}
SCEN = ["S0_nominal", "S1_travado", "S3_deriva", "S4_dropout", "S5_salto_gps", "S6_duplo"]
# fault window [s] and indices of the faulty sensors (0 baro, 1 LiDAR, 2 GPS) - same as e5.launch.py
WIN = {"S0_nominal": ((8.0, 18.0), []), "S1_travado": ((8.0, 18.0), [1]), "S3_deriva": ((8.0, 18.0), [1]),
       "S4_dropout": ((8.0, 18.0), [1]), "S5_salto_gps": ((25.0, 35.0), [2]), "S6_duplo": ((8.0, 18.0), [1, 2])}
SIGMA, VMAX, MAX_RATE, FS_CSV = [0.15, 0.02, 0.05], 10.0, 4.0, 20.0
KF_Q = 30.0                                   # same as e5.launch.py


def load(path):
    r = list(csv.DictReader(open(path)))
    d = {k: np.array([float(x[k]) for x in r]) for k in ("t", "ref", "gt", "baro", "lidar", "gps", "fused", "ekf")}
    d["rej"] = [set(int(v) for v in x["rejected"].split(";") if v != "") for x in r]
    return d


def nominal_traj():
    """Mean true trajectory of the nominal flights (all methods), on a 0.05 s grid."""
    grid = np.arange(0.0, 60.0, 0.05)
    tr = [np.interp(grid, d["t"], d["gt"]) for d in (load(p) for p in glob.glob(os.path.join(RUNS, "S0_nominal", "*.csv")))]
    return grid, (np.mean(tr, axis=0) if tr else None)


def metrics(d, scen, method, nom):
    (a, b), bad = WIN[scen]
    t = d["t"]
    f = (t >= a) & (t < b)
    f5 = (t >= a) & (t < b + 5.0)
    m = {"rmse_falha": float(np.sqrt(np.mean((d["fused"][f] - d["gt"][f]) ** 2))),
         "toque_chao": bool(np.any(d["gt"][t > 8.0] < 0.15)),
         "alt_min_falha": float(d["gt"][f].min()), "amostras": int(len(t))}
    if nom is not None:
        m["desvio_max"] = float(np.max(np.abs(d["gt"][f5] - np.interp(t[f5], *nom))))
    if method in ("pal2v", "pal2v_g", "kf_chi2"):
        if bad:
            m["deteccao"] = float(np.mean([all(i in r for i in bad) for r, k in zip(d["rej"], f) if k]))
        m["falso_alarme"] = float(np.mean([len(r) > 0 for r, k in zip(d["rej"], f) if not k]))
    return m


def open_loop(d, scen):
    x = np.c_[d["baro"], d["lidar"], d["gps"]]
    fus_g = PAL2vFusion.from_sigma(SIGMA, 0.0, VMAX, max_rate=MAX_RATE, dt=1 / FS_CSV, mode="graded")
    fus = PAL2vFusion.from_sigma(SIGMA, 0.0, VMAX, max_rate=MAX_RATE, dt=1 / FS_CSV, mode="step")
    kf = KalmanChi2(SIGMA, q=KF_Q, dt=1 / FS_CSV)
    out = {k: np.empty(len(x)) for k in METHODS}
    for k in range(len(x)):
        out["mean"][k] = fuse_mean(x[k], SIGMA)
        out["median"][k] = fuse_median(x[k], SIGMA)
        out["ivw"][k] = fuse_ivw(x[k], SIGMA)
        out["paraextrctr"][k] = para_extractor(np.clip(x[k], 0.0, VMAX), 0.0, VMAX)
        out["pal2v"][k] = fus.update(x[k]).value
        out["pal2v_g"][k] = fus_g.update(x[k]).value
        out["kf_chi2"][k] = kf.update(x[k])
    (a, b), _ = WIN[scen]
    f = (d["t"] >= a) & (d["t"] < b)
    return {k: float(np.sqrt(np.mean((v[f] - d["gt"][f]) ** 2))) for k, v in out.items()}


def fmt(v):
    v = np.asarray(v, float)
    return f"{v.mean():.2f} ± {v.std(ddof=1) if len(v) > 1 else 0:.2f}"


def main():
    nom = nominal_traj()
    nom = nom if nom[1] is not None else None
    per = {s: {m: {} for m in METHODS} for s in SCEN}
    ol = {s: {} for s in SCEN}
    for s in SCEN:
        for m in METHODS:
            for p in sorted(glob.glob(os.path.join(RUNS, s, f"{m}_s*.csv"))):
                seed = int(os.path.basename(p).split("_s")[-1][:-4])
                d = load(p)
                per[s][m][seed] = metrics(d, s, m, nom)
                if m == "pal2v":
                    ol[s][seed] = open_loop(d, s)
    L = ["# E5 — PX4 SITL (x500), altitude by baro + LiDAR + GPS fusion in closed loop\n",
         "Mean ± std over seeds. p = Wilcoxon paired by seed vs PAL2vFusion graded (no exclusion) (RMSE in the fault; n ≥ 5).\n"]
    summ = {}
    for s in SCEN:
        (a, b), bad = WIN[s]
        L.append(f"\n**{s}** (window {a:.0f}–{b:.0f} s)\n")
        L.append("| Method | n | Fusion RMSE in fault [m] | Max. trajectory deviation [m] | Min. alt. in fault [m] | % flights w/ ground touch | p |")
        L.append("|---|---|---|---|---|---|---|")
        summ[s] = {}
        ref = per[s][REF] or per[s]["pal2v"]
        refm = REF if per[s][REF] else "pal2v"
        for m in METHODS:
            r = per[s][m]
            if not r:
                continue
            seeds = sorted(r)
            col = lambda k: [r[i][k] for i in seeds if k in r[i]]  # noqa: E731
            common = [i for i in seeds if i in ref]
            p = float("nan")
            if m != refm and len(common) >= 5:
                p = wilcoxon([r[i]["rmse_falha"] for i in common], [ref[i]["rmse_falha"] for i in common])
            pct = 100 * np.mean(col("toque_chao"))
            summ[s][m] = {"n": len(seeds), "pct_toque_chao": pct, "p_vs_pal2v": p,
                          **{k: float(np.mean(col(k))) for k in ("rmse_falha", "desvio_max", "alt_min_falha",
                                                                   "deteccao", "falso_alarme") if col(k)}}
            name = f"**{NAMES[m]}**" if m == refm else NAMES[m]
            dv = fmt(col("desvio_max")) if col("desvio_max") else "n/a"
            L.append(f"| {name} | {len(seeds)} | {fmt(col('rmse_falha'))} | {dv} | {fmt(col('alt_min_falha'))} | "
                     f"{pct:.0f} % | {'—' if m == refm else (f'{p:.1e}' if p == p else 'n/a')} |")
        for m in ("pal2v_g", "pal2v", "kf_chi2"):
            if per[s][m] and "falso_alarme" in summ[s][m]:
                det = f"faulty sensor(s) out of tolerance (rejected in step; attenuated in graded) in {100 * summ[s][m]['deteccao']:.0f} % of the fault; " if bad else ""
                L.append(f"\n{NAMES[m]}: {det}false alarm outside the fault {100 * summ[s][m]['falso_alarme']:.1f} %.")
    L.append("\n## Open loop (same sensor data, PAL2v flights): RMSE in the fault window [m]\n")
    L.append("| Scenario | " + " | ".join(NAMES[m] for m in METHODS) + " |")
    L.append("|---|" + "---|" * len(METHODS))
    ol_s = {}
    for s in SCEN:
        if ol[s]:
            ol_s[s] = {m: fmt([ol[s][i][m] for i in ol[s]]) for m in METHODS}
            L.append(f"| {s} | " + " | ".join(ol_s[s][m] for m in METHODS) + " |")
    md = "\n".join(L) + "\n"
    open(os.path.join(RES, "e5_table.md"), "w").write(md)
    json.dump({"env": env_info(), "config": {"sigma": SIGMA, "vmax": VMAX, "max_rate": MAX_RATE, "kf_q": KF_Q,
                                             "janelas": {k: v[0] for k, v in WIN.items()}},
               "resumo": summ, "malha_aberta": ol_s, "por_voo": per},
              open(os.path.join(RES, "e5_px4.json"), "w"), indent=2, ensure_ascii=False, default=float)
    figures()
    print(md)


def figures():
    """True height (no measurement fault: simulator ground truth) of seed 0, per scenario."""
    show = ["mean", "median", "kf_chi2", "pal2v", "pal2v_g"]
    sc = [s for s in SCEN if s != "S0_nominal"]
    fig, ax = plt.subplots(len(sc), 1, figsize=(10, 2.3 * len(sc)), sharex=True)
    for a, s in zip(ax, sc):
        (w0, w1), _ = WIN[s]
        a.axvspan(w0, w1, color="0.93", lw=0)
        ref = None
        for m in show:
            p = os.path.join(RUNS, s, f"{m}_s0.csv")
            if not os.path.exists(p):
                continue
            d = load(p)
            ref = d
            a.plot(d["t"], d["gt"], color=COLORS[m], lw=2 if m == REF else 1.3, label=NAMES[m])
        if ref is not None:
            a.plot(ref["t"], ref["ref"], color="0.45", lw=1, ls="--", label="reference")
        a.set_ylim(-0.2, 6.5)
        a.set_ylabel("height [m]")
        a.set_title(f"{s}  (gray band = fault)", loc="left", fontsize=9)
        a.grid(axis="y", alpha=0.25)
        for side in ("top", "right"):
            a.spines[side].set_visible(False)
    ax[0].legend(fontsize=8, ncol=5, loc="upper left", frameon=False)
    ax[-1].set_xlabel("flight time [s]")
    fig.suptitle("E5 — true drone height (PX4 SITL) with each fusion method in the feedback loop", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "e5_series.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()

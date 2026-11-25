"""E4 - batch analysis (results/e4_runs) -> results/e4_gazebo.json, e4_table.md and figures.

Metrics per run (closed loop: the given method is the one that brakes the robot):
  collisions (episodes of the contact sensor), time in contact [s], minimum true distance [m],
  traveled distance [m] (Gazebo true pose), fusion RMSE in the fault window [m],
  LiDAR detection/rejection (PAL2v and Kalman χ²): % of fault samples with the LiDAR rejected; false alarm outside.
Open loop (paired): all methods applied to the SAME sensor data of the PAL2v runs.
Statistics: Wilcoxon paired by seed against the PAL2vFusion.
JSON keys: resumo = summary, malha_aberta = open loop, por_rodada = per run, colisoes = collisions,
t_contato = time in contact, dist_min = min distance, percorrido = traveled, rmse_falha/rmse_fora = RMSE in/out
of the fault, amostras = samples, deteccao = detection, falso_alarme = false alarm, pct_colisao = % runs with collision.
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
RUNS = os.path.join(RES, "e4_runs")
METHODS = ["pal2v_g", "pal2v", "mean", "median", "ivw", "paraextrctr", "kf_chi2"]
REF = "pal2v_g"                       # main method (graded, no exclusion); "pal2v" = step, v1.0.0
NAMES = {"pal2v_g": "PAL2vFusion (graded)", "pal2v": "PAL2vFusion step (v1.0)", "mean": "Mean", "median": "Median", "ivw": "IVW", "paraextrctr": "ParaExtrctr",
         "kf_chi2": "Kalman χ²"}
COLORS = {"pal2v_g": "#1b7f5c", "pal2v": "#e6ab02", "mean": "#c0392b", "median": "#7d3c98", "ivw": "#d68910", "paraextrctr": "#2e86c1",
          "kf_chi2": "#a0522d"}
SCEN = ["S0_nominal", "S1_travado", "S2_picos", "S3_deriva", "S4_dropout", "S6_duplo"]
KF_Q = 100.0                          # same as e4.launch.py
SIGMA, RMAX, FS, MAX_RATE = [0.05, 0.02, 0.08], 3.5, 20.0, 1.0
FAULT = (30.0, 75.0)


def load(path):
    r = list(csv.DictReader(open(path)))
    d = {k: np.array([float(x[k]) for x in r]) for k in ("t", "gt", "sonar", "lidar", "ir", "fused", "contact", "x", "y")}
    d["rej_lidar"] = np.array(["1" in x["rejected"].split(";") for x in r])
    return d


def episodes(c, gap=10):
    """Contact episodes (rising edges), merging interruptions shorter than `gap` samples."""
    idx = np.nonzero(c > 0)[0]
    if len(idx) == 0:
        return 0
    return int(1 + np.sum(np.diff(idx) > gap))


def metrics(d, method):
    f = (d["t"] >= FAULT[0]) & (d["t"] < FAULT[1])
    e = d["fused"] - d["gt"]
    step = np.hypot(np.diff(d["x"]), np.diff(d["y"]))
    m = {"colisoes": episodes(d["contact"]), "t_contato": float(d["contact"].sum() / FS),
         "dist_min": float(d["gt"].min()), "percorrido": float(np.nansum(step[step < 0.5])),
         "rmse_falha": float(np.sqrt(np.mean(e[f] ** 2))), "rmse_fora": float(np.sqrt(np.mean(e[~f] ** 2))),
         "amostras": int(len(d["t"]))}
    if method in ("pal2v", "pal2v_g", "kf_chi2"):
        m["deteccao"] = float(d["rej_lidar"][f].mean())
        m["falso_alarme"] = float(d["rej_lidar"][~f].mean())
    return m


def open_loop(d):
    x = np.c_[d["sonar"], d["lidar"], d["ir"]]
    fus_g = PAL2vFusion.from_sigma(SIGMA, 0.0, RMAX, max_rate=MAX_RATE, dt=1 / FS, mode="graded")
    fus = PAL2vFusion.from_sigma(SIGMA, 0.0, RMAX, max_rate=MAX_RATE, dt=1 / FS, mode="step")
    out = {k: np.empty(len(x)) for k in METHODS}
    kf = KalmanChi2(SIGMA, q=KF_Q, dt=1 / FS)
    for k in range(len(x)):
        out["mean"][k] = fuse_mean(x[k], SIGMA)
        out["median"][k] = fuse_median(x[k], SIGMA)
        out["ivw"][k] = fuse_ivw(x[k], SIGMA)
        out["paraextrctr"][k] = para_extractor(x[k], 0.0, RMAX)
        out["pal2v"][k] = fus.update(x[k]).value
        out["pal2v_g"][k] = fus_g.update(x[k]).value
        out["kf_chi2"][k] = kf.update(x[k])
    f = (d["t"] >= FAULT[0]) & (d["t"] < FAULT[1])
    return {k: {"rmse_falha": float(np.sqrt(np.mean((v[f] - d["gt"][f]) ** 2))),
                "max_falha": float(np.max(np.abs(v[f] - d["gt"][f])))} for k, v in out.items()}


def fmt(v):
    v = np.asarray(v, float)
    return f"{v.mean():.2f} ± {v.std(ddof=1) if len(v) > 1 else 0:.2f}"


def main():
    per = {s: {m: {} for m in METHODS} for s in SCEN}
    ol = {s: {} for s in SCEN}
    for s in SCEN:
        for m in METHODS:
            for p in sorted(glob.glob(os.path.join(RUNS, s, f"{m}_s*.csv"))):
                seed = int(os.path.basename(p).split("_s")[-1][:-4])
                d = load(p)
                per[s][m][seed] = metrics(d, m)
                if m == "pal2v":
                    ol[s][seed] = open_loop(d)
    keys = ["colisoes", "t_contato", "dist_min", "percorrido", "rmse_falha"]
    summ, lines = {}, []
    lines.append("# E4 — TurtleBot3 in Gazebo, reactive avoidance with sonar + LiDAR + IR fusion (LiDAR fault at 30–75 s)\n")
    lines.append("Closed loop: the fusion method decides when to brake/turn. Mean ± std over seeds; "
                 "**% runs w/ collision**; p = Wilcoxon paired by seed vs PAL2vFusion graded (no exclusion) (time in contact).\n")
    for s in SCEN:
        lines.append(f"\n**{s}**\n")
        lines.append("| Method | n | % runs w/ collision | Collisions | Time in contact [s] | Min. true dist. [m] | Traveled [m] | RMSE in fault [m] | p |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        summ[s] = {}
        ref = per[s][REF] or per[s]["pal2v"]
        refm = REF if per[s][REF] else "pal2v"
        for m in METHODS:
            r = per[s][m]
            if not r:
                continue
            seeds = sorted(r)
            v = {k: [r[i][k] for i in seeds] for k in keys}
            pct = 100 * np.mean([r[i]["colisoes"] > 0 for i in seeds])
            common = [i for i in seeds if i in ref]
            p = float("nan")
            if m != refm and len(common) >= 5:
                a = [r[i]["t_contato"] for i in common]
                b = [ref[i]["t_contato"] for i in common]
                p = wilcoxon(a, b) if any(x != y for x, y in zip(a, b)) else 1.0
            summ[s][m] = {"n": len(seeds), "pct_colisao": pct, "p_vs_pal2v": p,
                          **{k: {"mean": float(np.mean(v[k])), "std": float(np.std(v[k], ddof=1)) if len(seeds) > 1 else 0.0}
                             for k in keys}}
            if m in ("pal2v", "pal2v_g", "kf_chi2"):
                summ[s][m]["deteccao"] = float(np.mean([r[i]["deteccao"] for i in seeds]))
                summ[s][m]["falso_alarme"] = float(np.mean([r[i]["falso_alarme"] for i in seeds]))
            name = f"**{NAMES[m]}**" if m == refm else NAMES[m]
            lines.append(f"| {name} | {len(seeds)} | {pct:.0f} % | {fmt(v['colisoes'])} | {fmt(v['t_contato'])} | "
                         f"{fmt(v['dist_min'])} | {fmt(v['percorrido'])} | {fmt(v['rmse_falha'])} | "
                         f"{'—' if m == refm else (f'{p:.1e}' if p == p else 'n/a')} |")
        for mm in ("pal2v_g", "pal2v", "kf_chi2"):
            if per[s][mm]:
                lines.append(f"\n{NAMES[mm]}: LiDAR out of tolerance (rejected in step; attenuated in graded) in {100 * summ[s][mm]['deteccao']:.0f} % of the fault "
                             f"samples; false alarm outside the fault {100 * summ[s][mm]['falso_alarme']:.1f} %.")
    # open loop
    lines.append("\n## Open loop (same sensor data, PAL2v runs): RMSE in the fault window [m]\n")
    lines.append("| Scenario | " + " | ".join(NAMES[m] for m in METHODS) + " |")
    lines.append("|---|" + "---|" * len(METHODS))
    ol_s = {}
    for s in SCEN:
        if not ol[s]:
            continue
        ol_s[s] = {m: {"rmse_falha": fmt([ol[s][i][m]["rmse_falha"] for i in ol[s]]),
                       "max_falha": fmt([ol[s][i][m]["max_falha"] for i in ol[s]])} for m in METHODS}
        lines.append(f"| {s} | " + " | ".join(ol_s[s][m]["rmse_falha"] for m in METHODS) + " |")
    md = "\n".join(lines) + "\n"
    open(os.path.join(RES, "e4_table.md"), "w").write(md)
    json.dump({"env": env_info(), "config": {"sigma": SIGMA, "rmax": RMAX, "fs": FS, "max_rate": MAX_RATE,
                                             "fault": FAULT},
               "resumo": summ, "malha_aberta": ol_s, "por_rodada": per},
              open(os.path.join(RES, "e4_gazebo.json"), "w"), indent=2, ensure_ascii=False, default=float)
    figures(per)
    print(md)


def figures(per):
    # 1) time in contact and % of runs with collision
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
    w = 0.115
    for j, m in enumerate(METHODS):
        xs = np.arange(len(SCEN)) + (j - 3) * w
        tc = [np.mean([v["t_contato"] for v in per[s][m].values()]) if per[s][m] else 0 for s in SCEN]
        pc = [100 * np.mean([v["colisoes"] > 0 for v in per[s][m].values()]) if per[s][m] else 0 for s in SCEN]
        ax[0].bar(xs, tc, w, color=COLORS[m], label=NAMES[m])
        ax[1].bar(xs, pc, w, color=COLORS[m], label=NAMES[m])
    for a, t in zip(ax, ("Mean time in contact with an obstacle [s]", "Runs with ≥ 1 collision [%]")):
        a.set_xticks(range(len(SCEN)))
        a.set_xticklabels([s.replace("_", "\n") for s in SCEN])
        a.set_title(t, fontsize=10)
        a.grid(axis="y", alpha=0.3)
    ax[0].legend(fontsize=8)
    fig.suptitle("E4 — closed loop in Gazebo (LiDAR fault at 30–75 s)")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "e4_collisions.png"), dpi=130)
    plt.close(fig)
    # 2) time series of seed 0, per scenario: PAL2v x Mean
    fig, ax = plt.subplots(len(SCEN), 1, figsize=(11, 2.3 * len(SCEN)), sharex=True)
    for a, s in zip(ax, SCEN):
        a.axvspan(*FAULT, color="0.92")
        for m in ("mean", "pal2v_g"):
            p = os.path.join(RUNS, s, f"{m}_s0.csv")
            if not os.path.exists(p):
                continue
            d = load(p)
            a.plot(d["t"], d["gt"], color=COLORS[m], lw=0.8, ls=":")
            a.plot(d["t"], d["fused"], color=COLORS[m], lw=1.1, label=f"{NAMES[m]}: fused (—) and true (···)")
            c = d["t"][d["contact"] > 0]
            if len(c):
                a.plot(c, np.full(len(c), 0.05), "|", color=COLORS[m], ms=8)
        a.set_ylim(0, 3.6)
        a.set_ylabel("dist. [m]")
        a.set_title(s, loc="left", fontsize=9)
        a.legend(fontsize=7, loc="upper right")
    ax[-1].set_xlabel("simulation time [s]   (ticks at the bottom = collision)")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "e4_series.png"), dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    main()

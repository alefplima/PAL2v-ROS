"""E2 - Fusion of 3 redundant distance sensors under faults (Monte Carlo, 30 seeds).

True signal: smoothly varying distance (robot approaching/moving away from a wall).
Sensors: ultrasound (σ=0.05 m), 1D LiDAR (σ=0.02 m), infrared (σ=0.08 m), 50 Hz, 20 s.
Scenarios (fault always on the LiDAR, the most accurate sensor, so as not to favor the weighted mean):
  S0 nominal | S1_travado (stuck) | S2_picos (impulsive spikes) | S3_deriva (growing bias/drift) | S4 dropout (= max range)
Methods: mean, median, inverse-variance weighted mean (IVW), ParaExtrctr, PAL2vFusion.
Metrics: RMSE and maximum absolute error with respect to the true value.
JSON keys (data format of the published results): media = mean, mediana = median, _deteccao = detection,
rejeicao_na_falha_% = rejection in the fault window, falso_alarme_fora_% = false alarm outside it.
"""
import numpy as np

from common import (TEST_SEEDS, env_info, fuse_ivw, fuse_mean, fuse_median, save_json, summarize,
                    wilcoxon)
from lpa2v import PAL2vFusion, para_extractor

FS, T_END = 50.0, 20.0
SIGMA = np.array([0.05, 0.02, 0.08])
RMAX = 4.0
SCEN = ["S0_nominal", "S1_travado", "S2_picos", "S3_deriva", "S4_dropout"]
FAULT = (8.0, 14.0)   # fault window [s]
METHODS = ["media", "mediana", "IVW", "ParaExtrctr", "PAL2v", "PAL2v_G"]


def truth(t):
    return 1.5 + 0.8 * np.sin(2 * np.pi * t / 10.0) + 0.2 * np.sin(2 * np.pi * t / 3.3)


def sensors(t, z, scen, rng):
    x = z[:, None] + rng.normal(0, SIGMA, (len(t), 3))
    f = (t >= FAULT[0]) & (t < FAULT[1])
    i = 1  # LiDAR
    if scen == "S1_travado":
        idx = np.argmax(f)
        x[f, i] = x[idx, i]
    elif scen == "S2_picos":
        spk = f & (rng.random(len(t)) < 0.15)
        x[spk, i] += rng.choice([-1, 1], spk.sum()) * rng.uniform(0.5, 1.5, spk.sum())
    elif scen == "S3_deriva":
        x[f, i] += 0.25 * (t[f] - FAULT[0])          # 0.25 m/s drift
    elif scen == "S4_dropout":
        x[f, i] = RMAX
    return np.clip(x, 0.0, RMAX)


def run(seed, scen):
    rng = np.random.default_rng(seed)
    t = np.arange(0, T_END, 1 / FS)
    z = truth(t)
    x = sensors(t, z, scen, rng)
    # NAP + ParaExtrctr network in the UD [0, RMAX]; contradiction limits by the default rule (k = 4)
    # "PAL2v" = mode step (v1.0.0, limit case); "PAL2v_G" = mode graded (no exclusion, v1.1)
    fus = PAL2vFusion.from_sigma(SIGMA, 0.0, RMAX, max_rate=0.6, dt=1 / FS, mode="step")
    fus_g = PAL2vFusion.from_sigma(SIGMA, 0.0, RMAX, max_rate=0.6, dt=1 / FS, mode="graded")
    out = {m: np.empty(len(t)) for m in METHODS}
    rej = np.zeros(len(t), bool)          # LiDAR rejected by the PAL2vFusion?
    rej_g = np.zeros(len(t), bool)        # LiDAR outside the tolerance (attenuated) in the graded mode?
    for k in range(len(t)):
        out["media"][k] = fuse_mean(x[k], SIGMA)
        out["mediana"][k] = fuse_median(x[k], SIGMA)
        out["IVW"][k] = fuse_ivw(x[k], SIGMA)
        out["ParaExtrctr"][k] = para_extractor(x[k], 0.0, RMAX)
        o = fus.update(x[k])
        out["PAL2v"][k] = o.value
        rej[k] = 1 in o.rejected
        og = fus_g.update(x[k])
        out["PAL2v_G"][k] = og.value
        rej_g[k] = 1 in og.rejected
    met = {m: {"rmse": float(np.sqrt(np.mean((v - z) ** 2))),
               "maxerr": float(np.max(np.abs(v - z)))} for m, v in out.items()}
    f = (t >= FAULT[0]) & (t < FAULT[1])
    det = {"rejeicao_na_falha_%": float(100 * rej[f].mean()),
           "falso_alarme_fora_%": float(100 * rej[~f].mean()),
           "G_rejeicao_na_falha_%": float(100 * rej_g[f].mean()),
           "G_falso_alarme_fora_%": float(100 * rej_g[~f].mean())}
    return met, (t, z, x, out), det


def main():
    res = {"env": env_info(), "seeds": TEST_SEEDS, "sigma": SIGMA.tolist(), "fault": FAULT,
           "scenarios": {}}
    examples = {}
    for scen in SCEN:
        per, dets = {}, []
        for s in TEST_SEEDS:
            met, data, det = run(s, scen)
            dets.append(det)
            for m, v in met.items():
                per.setdefault(m, {"rmse": [], "maxerr": []})
                per[m]["rmse"].append(v["rmse"])
                per[m]["maxerr"].append(v["maxerr"])
            if s == 0:
                examples[scen] = data
        res["scenarios"][scen] = {
            m: {"rmse": summarize(v["rmse"]), "maxerr": summarize(v["maxerr"]),
                "p_wilcoxon_rmse_vs_PAL2v": wilcoxon(v["rmse"], per["PAL2v"]["rmse"]),
                "p_wilcoxon_rmse_vs_PAL2v_G": wilcoxon(v["rmse"], per["PAL2v_G"]["rmse"])}
            for m, v in per.items()}
        res["scenarios"][scen]["_deteccao"] = {
            k: summarize([d[k] for d in dets]) for k in dets[0]}
    save_json("e2_fusion.json", res)

    # table
    lines = ["| Scenario | Mean | Median | IVW | ParaExtrctr | PAL2v step (v1.0) | **PAL2v graded** "
             "| Out of tolerance in fault / false alarm (step; graded) |",
             "|---|---|---|---|---|---|---|---|"]
    for scen in SCEN:
        r = res["scenarios"][scen]
        cells = [f"{r[m]['rmse']['mean'] * 100:.1f} ± {r[m]['rmse']['std'] * 100:.1f}"
                 for m in METHODS]
        d = r["_deteccao"]
        cells.append(f"{d['rejeicao_na_falha_%']['mean']:.0f}% / {d['falso_alarme_fora_%']['mean']:.1f}%; "
                     f"{d['G_rejeicao_na_falha_%']['mean']:.0f}% / {d['G_falso_alarme_fora_%']['mean']:.1f}%")
        lines.append(f"| {scen} | " + " | ".join(cells) + " |")
    table = "RMSE [cm], mean ± std over 30 seeds\n\n" + "\n".join(lines)
    print(table)
    with open(f"{__import__('common').RESULTS}/e2_table.md", "w", encoding="utf-8") as fh:
        fh.write(table + "\n")

    # figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(SCEN), 1, figsize=(9, 2.2 * len(SCEN)), sharex=True)
    for ax, scen in zip(axs, SCEN):
        t, z, x, out = examples[scen]
        ax.plot(t, x[:, 1], color="0.75", lw=0.8, label="LiDAR (faulty sensor)")
        ax.plot(t, z, "k", lw=1.5, label="true")
        ax.plot(t, out["IVW"], color="#d95f02", lw=1, label="IVW")
        ax.plot(t, out["mediana"], color="#7570b3", lw=1, label="median")
        ax.plot(t, out["PAL2v"], color="#e6ab02", lw=1.0, label="PAL2v step")
        ax.plot(t, out["PAL2v_G"], color="#1b9e77", lw=1.3, label="PAL2v graded")
        ax.axvspan(*FAULT, color="#e7298a", alpha=0.08)
        ax.set_ylabel("dist. [m]")
        ax.set_title(scen, fontsize=9, loc="left")
        ax.set_ylim(0, RMAX + 0.2)
    axs[0].legend(ncol=6, fontsize=7, loc="upper right")
    axs[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(f"{__import__('common').RESULTS}/e2_fusion.png", dpi=150)


if __name__ == "__main__":
    main()

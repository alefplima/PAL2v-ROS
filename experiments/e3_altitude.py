"""E3 - Altitude control of a quadcopter (1-DOF vertical dynamics) with redundant sensors.

Plant:   m z'' = T - m g - b z' + w(t) ;  T follows the command with a 1st-order lag (τ_m = 0.05 s),
         m = 1.5 kg, b = 0.4 N·s/m, T ∈ [0, 30] N, dt = 0.01 s (100 Hz), 25 s.
Disturbance w(t): Ornstein-Uhlenbeck process (σ = 0.6 N, τ = 1 s) + a +4 N gust at t ∈ [17, 18] s.
Sensors: barometer (σ = 0.30 m, slow drift), LiDAR/rangefinder (σ = 0.03 m), GPS (σ = 0.50 m, 10 Hz).
Reference: 0 → 2 m (t=1 s) → 5 m (t=8 s) → 3 m (t=14 s).
Scenarios: N nominal | F faults: LiDAR dropout in [10,12] s (reads 8 m) + +3 m GPS jump in [20,21] s.

Compared arrangements (same plant, same seeds):
  A  PID      + IVW mean          ("naive" reference)
  B  PID      + median            (robust reference)
  C  PID      + PAL2vFusion       (contribution in perception)
  D  PAL2v-PI + PAL2vFusion       (contribution in perception + control)
  E  paraconsistent hybrid PI (Coelho, Da Silva Filho et al., CEP 2019) + PAL2vFusion
Tuning: log-uniform random search minimizing the IAE in the NOMINAL scenario with seeds 100-104 (TUNE_SEEDS),
same budget for every controller; evaluation on 30 seeds 0-29 (TEST_SEEDS).
Metrics (on the TRUE altitude): IAE, overshoot at the 2→5 m step, settling time (2 %),
maximum error in the fault windows, control effort (standard deviation of dT/dt).
"""


import numpy as np

from common import (RESULTS, TEST_SEEDS, TUNE_SEEDS, env_info, fuse_ivw, fuse_median, save_json,
                    summarize, wilcoxon)
from lpa2v import PID, PAL2vController, PAL2vFusion, ParaconsistentHybridPI

M, G, B, TAU_M = 1.5, 9.81, 0.4, 0.05
U_MIN, U_MAX = 0.0, 30.0
DT, T_END = 0.01, 25.0
HOVER = M * G
SIGMA = np.array([0.30, 0.03, 0.50])
FAULTS = [(10.0, 12.0), (20.0, 21.0)]
UD = (0.0, 10.0)      # altitude Universe of Discourse [m] for the evidence extractor


def reference(t):
    return np.where(t < 1, 0.0, np.where(t < 8, 2.0, np.where(t < 14, 5.0, 3.0)))


def make_fusion():
    # same rule as E2 (library); maximum plausible vertical rate of 4 m/s.
    # Note: the 10 Hz GPS with zero-order hold jumps every 10 samples; the 4σ√2 term covers that jump.
    # altitude UD: [0, 10] m
    return PAL2vFusion.from_sigma(SIGMA, UD[0], UD[1], max_rate=4.0, dt=DT)


def simulate(seed, scen, controller, fusion_mode, log=False):
    rng = np.random.default_rng(seed)
    t = np.arange(0, T_END, DT)
    n = len(t)
    r = reference(t)
    # OU disturbance + gust
    w = np.zeros(n)
    for k in range(1, n):
        w[k] = w[k - 1] - DT * w[k - 1] / 1.0 + 0.6 * np.sqrt(2 * DT / 1.0) * rng.normal()
    w[(t >= 17) & (t < 18)] += 4.0
    baro_bias = np.cumsum(rng.normal(0, 0.02 * np.sqrt(DT), n))
    noise = rng.normal(0, 1, (n, 3)) * SIGMA
    z = zd = 0.0
    thrust = HOVER
    fus = make_fusion()
    controller.reset()
    Z, Y, U = np.empty(n), np.empty(n), np.empty(n)
    gps_hold = 0.0
    for k in range(n):
        tk = t[k]
        baro = z + baro_bias[k] + noise[k, 0]
        lidar = min(max(z + noise[k, 1], 0.0), 8.0)
        if k % 10 == 0:
            gps_hold = z + noise[k, 2]
        gps = gps_hold
        if scen == "F":
            if 10.0 <= tk < 12.0:
                lidar = 8.0
            if 20.0 <= tk < 21.0:
                gps += 3.0
        x = [baro, lidar, gps]
        if fusion_mode == "ivw":
            y = fuse_ivw(x, SIGMA)
        elif fusion_mode == "median":
            y = fuse_median(x, SIGMA)
        else:
            y = fus.update(x).value
        u = controller.update(r[k], y, DT)
        # plant
        thrust += DT / TAU_M * (u - thrust)
        zdd = (thrust - M * G - B * zd + w[k]) / M
        zd += DT * zdd
        z += DT * zd
        if z < 0:
            z, zd = 0.0, max(zd, 0.0)
        Z[k], Y[k], U[k] = z, y, u
    return t, r, Z, Y, U


def metrics(t, r, Z, U):
    e = r - Z
    iae = float(np.sum(np.abs(e[t >= 1.0])) * DT)
    s = (t >= 8) & (t < 14)
    over = float(max(0.0, (Z[s].max() - 5.0) / 3.0 * 100))
    band = np.abs(Z[s] - 5.0) > 0.02 * 3.0
    ts = float(t[s][np.where(band)[0][-1]] - 8.0) if band.any() else 0.0
    fmask = np.zeros_like(t, bool)
    for a, b in FAULTS:
        fmask |= (t >= a) & (t < b + 1.0)
    errf = float(np.max(np.abs(e[fmask])))
    effort = float(np.std(np.diff(U) / DT))
    return {"IAE": iae, "overshoot_%": over, "settling_s": ts, "maxerr_fault_m": errf,
            "effort_dUdt": effort}


def make_pid(kp, ki, kd):
    return PID(kp, ki, kd, U_MIN, U_MAX, u_ff=HOVER)


def make_hpi(kp, ki):
    return ParaconsistentHybridPI(UD[0], UD[1], kp, ki, U_MIN, U_MAX, u_ff=HOVER)


def make_pal(kc, ki, td, e_max):
    return PAL2vController(e_max, td, kc, ki, U_MIN, U_MAX, u_ff=HOVER)


# Tuning: log-uniform random search with the SAME budget for every controller.
TUNE_BUDGET = 200
PID_BOUNDS = [(1.0, 60.0), (0.1, 15.0), (0.3, 30.0)]               # kp, ki, kd
PAL_BOUNDS = [(1.0, 120.0), (0.1, 15.0), (0.05, 4.0), (0.3, 6.0)]  # kc, ki, td, e_max
HPI_BOUNDS = [(0.5, 300.0), (0.01, 100.0)]                         # kp, ki (UD units)


def tune(factory, bounds, fusion_mode, seed=12345):
    rng = np.random.default_rng(seed)
    lo = np.log([b[0] for b in bounds])
    hi = np.log([b[1] for b in bounds])
    best, best_p, hist = np.inf, None, []
    for _ in range(TUNE_BUDGET):
        p = tuple(float(v) for v in np.exp(rng.uniform(lo, hi)))
        cost = np.mean([metrics(*_strip(simulate(s, "N", factory(*p), fusion_mode)))["IAE"]
                        for s in TUNE_SEEDS])
        hist.append((p, float(cost)))
        if cost < best:
            best, best_p = cost, p
    return best_p, float(best), hist


def _strip(sim):
    t, r, Z, Y, U = sim
    return t, r, Z, U


def main():
    print(f"tuning PID (random search, {TUNE_BUDGET} evaluations)...")
    pid_p, pid_c, pid_h = tune(make_pid, PID_BOUNDS, "pal2v")
    print("  PID kp,ki,kd =", np.round(pid_p, 3), "IAE", round(pid_c, 3), flush=True)
    print(f"tuning PAL2v-PI (random search, {TUNE_BUDGET} evaluations)...")
    pal_p, pal_c, pal_h = tune(make_pal, PAL_BOUNDS, "pal2v")
    print("  PAL2v-PI kc,ki,td,E =", np.round(pal_p, 3), "IAE", round(pal_c, 3), flush=True)
    print(f"tuning hybrid PI CEP 2019 (random search, {TUNE_BUDGET} evaluations)...")
    hpi_p, hpi_c, hpi_h = tune(make_hpi, HPI_BOUNDS, "pal2v")
    print("  HybridPI kp,ki =", np.round(hpi_p, 3), "IAE", round(hpi_c, 3), flush=True)
    pid_grid, pal_grid = PID_BOUNDS, PAL_BOUNDS

    configs = {
        "A_PID+IVW": (lambda: make_pid(*pid_p), "ivw"),
        "B_PID+median": (lambda: make_pid(*pid_p), "median"),
        "C_PID+PAL2vFusion": (lambda: make_pid(*pid_p), "pal2v"),
        "D_PAL2vPI+PAL2vFusion": (lambda: make_pal(*pal_p), "pal2v"),
        "E_HybridPI_CEP2019+PAL2vFusion": (lambda: make_hpi(*hpi_p), "pal2v"),
    }
    res = {"env": env_info(),
           "tuning": {"method": f"log-uniform random search, {TUNE_BUDGET} evaluations, "
                                f"nominal IAE criterion, seeds {TUNE_SEEDS}",
                      "pid": {"kp_ki_kd": pid_p, "IAE": pid_c, "bounds": pid_grid, "history": pid_h},
                      "pal2v_pi": {"kc_ki_td_emax": pal_p, "IAE": pal_c, "bounds": pal_grid,
                                   "history": pal_h},
                      "hybrid_pi_cep2019": {"kp_ki": hpi_p, "IAE": hpi_c, "bounds": HPI_BOUNDS,
                                            "history": hpi_h}},
           "results": {}}
    raw, examples = {}, {}
    for scen in ["N", "F"]:
        for name, (fac, fm) in configs.items():
            ms = []
            for s in TEST_SEEDS:
                sim = simulate(s, scen, fac(), fm)
                ms.append(metrics(*_strip(sim)))
                if s == 0:
                    examples[(scen, name)] = sim
            raw[(scen, name)] = ms
            res["results"].setdefault(scen, {})[name] = {
                k: summarize([m[k] for m in ms]) for k in ms[0]}
        ref = raw[(scen, "D_PAL2vPI+PAL2vFusion")]
        for name in configs:
            res["results"][scen][name]["p_wilcoxon_IAE_vs_D"] = wilcoxon(
                [m["IAE"] for m in raw[(scen, name)]], [m["IAE"] for m in ref])
    save_json("e3_altitude.json", res)

    keys = [("IAE", "IAE [m·s]", 1), ("overshoot_%", "Overshoot [%]", 1),
            ("settling_s", "Settling [s]", 2), ("maxerr_fault_m", "Max. error in faults [m]", 2),
            ("effort_dUdt", "σ(dT/dt) [N/s]", 0)]
    out = []
    for scen, title in [("N", "Nominal scenario"), ("F", "Scenario with sensor faults")]:
        out.append(f"\n**{title}** (mean ± std, 30 seeds)\n")
        out.append("| Arrangement | " + " | ".join(k[1] for k in keys) + " | p (IAE vs D) |")
        out.append("|---|" + "---|" * (len(keys) + 1))
        for name in configs:
            r = res["results"][scen][name]
            out.append(f"| {name} | " + " | ".join(
                f"{r[k]['mean']:.{d}f} ± {r[k]['std']:.{d}f}" for k, _, d in keys)
                + f" | {r['p_wilcoxon_IAE_vs_D']:.1e} |")
    table = "\n".join(out)
    print(table)
    with open(f"{RESULTS}/e3_table.md", "w", encoding="utf-8") as fh:
        fh.write(f"Tuning (random search, {TUNE_BUDGET} eval.): PID (kp,ki,kd)={np.round(pid_p, 3)}; "
                 f"PAL2v-PI (kc,ki,td,E)={np.round(pal_p, 3)}; HybridPI CEP2019 (kp,ki)={np.round(hpi_p, 3)}\n" + table + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"A_PID+IVW": "#d95f02", "B_PID+median": "#7570b3", "C_PID+PAL2vFusion": "#e6ab02",
              "D_PAL2vPI+PAL2vFusion": "#1b9e77", "E_HybridPI_CEP2019+PAL2vFusion": "#66a61e"}
    fig, axs = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for ax, scen in zip(axs, ["N", "F"]):
        t, r, *_ = examples[(scen, "A_PID+IVW")]
        ax.plot(t, r, "k--", lw=1, label="reference")
        for name in configs:
            _, _, Z, _, _ = examples[(scen, name)]
            ax.plot(t, Z, color=colors[name], lw=1.2, label=name)
        if scen == "F":
            for a, b in FAULTS:
                ax.axvspan(a, b, color="#e7298a", alpha=0.1)
        ax.axvspan(17, 18, color="0.5", alpha=0.1)
        ax.set_ylabel("true altitude [m]")
        ax.set_title("nominal" if scen == "N" else "faults: LiDAR dropout (10–12 s), GPS jump (20–21 s)",
                     fontsize=9, loc="left")
    axs[0].legend(fontsize=7, ncol=3)
    axs[-1].set_xlabel("time [s]  (gray band: wind gust)")
    fig.tight_layout()
    fig.savefig(f"{RESULTS}/e3_altitude.png", dpi=150)


if __name__ == "__main__":
    main()

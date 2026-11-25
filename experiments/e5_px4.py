"""E5 - batch of PX4 SITL flights (x500 + downward LiDAR) with altitude by PAL2v fusion.

Usage (SYSTEM python3, with /opt/ros/<distro>, ~/px4_ros_ws/install and ros2_ws/install sourced):
  python3 e5_px4.py --methods pal2v --scenarios S0_nominal --seeds 0      # pilot flight
  python3 e5_px4.py                                                         # batch (resumes)

Each flight: ros2 launch lpa2v_px4 e5.launch.py -> results/e5_runs/<scenario>/<method>_s<seed>.csv
Only one flight at a time (PX4 SITL uses agent port 8888 and DDS domain 0).
progresso.log (progress) lines: "ok N samples", "INVALID (simulator artifact: ...)" (flight redone, up to 3
attempts), "FAILED (...)". The published v1.0.0 logs carry the same tokens in Portuguese: "ok N amostras",
"INVALIDO (artefato do simulador: ...)", "FALHOU (não decolou | sem CSV)".
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "results", "e5_runs")
METHODS = ["pal2v", "pal2v_g", "mean", "median", "ivw", "paraextrctr", "kf_chi2"]
SCENARIOS = ["S0_nominal", "S1_travado", "S3_deriva", "S4_dropout", "S5_salto_gps", "S6_duplo"]


def _kill_leftovers():
    """Ensures no PX4/Gazebo/agent is left from a previous flight (port 8888 and domain 0 are unique)."""
    pats = ["px4_sitl_default/bin/px4 -d", "MicroXRCEAgent udp4 -p 8888",
            "gz sim --verbose=1 -r -s " + os.path.expanduser("~/PX4-Autopilot"), "lpa2v_px4/lib/lpa2v_px4",
            "__node:=e5_fusion"]
    for p in pats:
        subprocess.run(["pkill", "-9", "-f", p], capture_output=True)


SIM_ARTIFACTS = ("Accel #0 fail", "Gyro #0 fail", "Baro #0 fail", "Mag #0 fail")


def _sim_artifacts(logdir):
    """SIMULATED sensor timeouts (IMU/baro/mag do not reach PX4 for lack of CPU) invalidate the flight: they are
    not part of the experiment. Failsafes caused by the faults injected in the PAL2v layer remain valid."""
    found = {}
    for root, _, files in os.walk(logdir):
        for f in files:
            if f.endswith(".log"):
                for line in open(os.path.join(root, f), errors="ignore"):
                    for k in SIM_ARTIFACTS:
                        if k in line:
                            found[k] = found.get(k, 0) + 1
    return ", ".join(f"{k} x{v}" for k, v in found.items())


def run_one(method, scen, seed, a, log, tries=3):
    for i in range(tries):
        msg = _run_one(method, scen, seed, a, log)
        if not msg.startswith("INVALID"):
            return msg
    return msg


def _run_one(method, scen, seed, a, log):
    out = os.path.join(RUNS, scen, f"{method}_s{seed}.csv")
    if os.path.exists(out):
        return "pulada"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    tmp = out + ".part"
    for old in (tmp, tmp + ".falhou", tmp + ".tmp"):
        if os.path.exists(old):
            os.remove(old)
    _kill_leftovers()
    time.sleep(2)
    logdir = os.path.join(RUNS, "logs", f"{scen}_{method}_s{seed}")
    if os.path.exists(logdir):                       # previous (invalid) attempt: kept as evidence
        os.rename(logdir, f"{logdir}.anterior_{int(time.time())}")
    env = dict(os.environ, ROS_DOMAIN_ID="0", ROS_LOG_DIR=logdir)   # PX4 log of each flight (evidence)
    cmd = ["ros2", "launch", "lpa2v_px4", "e5.launch.py", f"method:={method}", f"scenario:={scen}",
           f"seed:={seed}", f"duration:={a.duration}", f"out:={tmp}", f"gui:={'true' if a.gui else 'false'}",
           f"sigma:={a.sigma}", "auto_shutdown:=false", f"partition:=e5_{os.getpid()}"]
    t0 = time.time()
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    limit = a.duration + 240
    while time.time() - t0 < limit and p.poll() is None:
        if os.path.exists(tmp) or os.path.exists(tmp + ".falhou"):
            break
        time.sleep(1)
    time.sleep(1)
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    p.wait()
    _kill_leftovers()
    bad = _sim_artifacts(logdir)
    if os.path.exists(tmp) and bad:
        os.replace(tmp, out + ".invalido")
        msg = f"INVALID (simulator artifact: {bad})"
    elif os.path.exists(tmp):
        n = sum(1 for _ in open(tmp)) - 1
        os.replace(tmp, out)
        msg = f"ok {n} samples"
    else:
        msg = "FAILED (" + ("did not take off" if os.path.exists(tmp + ".falhou") else "no CSV") + ")"
    log.write(f"{datetime.now():%H:%M:%S} {scen:13s} {method:12s} s{seed:<3d} {msg} ({time.time() - t0:.0f} s)\n")
    log.flush()
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(5)))
    ap.add_argument("--methods", nargs="+", default=METHODS)
    ap.add_argument("--scenarios", nargs="+", default=SCENARIOS)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--sigma", default="0.15,0.02,0.05")   # measured in the nominal flight (I-E5)
    ap.add_argument("--gui", action="store_true")
    a = ap.parse_args()
    os.makedirs(RUNS, exist_ok=True)
    total, k = len(a.seeds) * len(a.scenarios) * len(a.methods), 0
    with open(os.path.join(RUNS, "progresso.log"), "a") as log:
        log.write(f"# {datetime.now().isoformat(timespec='seconds')} lote: {vars(a)}\n")
        for seed in a.seeds:
            for scen in a.scenarios:
                for m in a.methods:
                    k += 1
                    print(f"[{k}/{total}] {scen} {m} s{seed}: {run_one(m, scen, seed, a, log)}", flush=True)
    print("end", file=sys.stderr)


if __name__ == "__main__":
    main()

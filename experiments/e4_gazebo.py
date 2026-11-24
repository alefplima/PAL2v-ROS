"""E4 - batch of Gazebo runs (TurtleBot3, reactive avoidance, fusion of sonar + LiDAR + IR with LiDAR faults).

Usage (SYSTEM python3, with /opt/ros/<distro> and ros2_ws/install sourced, no micromamba in PATH):
  python3 e4_gazebo.py                 # full batch (resumes: skips runs whose CSV already exists)
  python3 e4_gazebo.py --seeds 0 1     # subset

Each run: ros2 launch lpa2v_sim e4.launch.py (headless, fixed rtf) -> results/e4_runs/<scenario>/<method>_s<seed>.csv
The launch is ended with SIGKILL on the process group as soon as the CSV appears (the ros_gz
parameter_bridge segfaults on SIGINT and Ubuntu would open an apport warning per run).
Analysis: e4_analysis.py
Log progresso.log (progress): "ok N samples" or "FAILED (...)" (retried in the next pass). The published v1.0.0
log carries the same tokens in Portuguese ("ok N amostras", "FALHOU").
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "results", "e4_runs")
METHODS = ["pal2v", "mean", "median", "ivw", "paraextrctr", "kf_chi2"]
SCENARIOS = ["S0_nominal", "S1_travado", "S2_picos", "S3_deriva", "S4_dropout", "S6_duplo"]


def _lock(path):
    """Atomic lock per run (several workers in parallel). A lock of a dead process is taken over."""
    for _ in range(2):
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, str(os.getpid()).encode())
            os.close(fd)
            return True
        except FileExistsError:
            try:
                pid = int(open(path).read().strip() or 0)
                os.kill(pid, 0)
                return False                  # another live worker holds this run
            except (ValueError, ProcessLookupError, FileNotFoundError):
                try:
                    os.remove(path)           # orphan lock
                except FileNotFoundError:
                    pass
    return False


def _robot_moved(path):
    """Some sample with commanded linear speed != 0 (otherwise the avoid_controller never published)."""
    import csv
    return any(float(r["v"]) != 0.0 for r in csv.DictReader(open(path)))


def run_one(method, scen, seed, a, log):
    out = os.path.join(RUNS, scen, f"{method}_s{seed}.csv")
    if os.path.exists(out):
        return "pulada"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    lock = out + ".lock"
    if not _lock(lock):
        return "held by another worker"
    try:
        if os.path.exists(out):
            return "pulada"
        return _run(method, scen, seed, a, log, out)
    finally:
        os.remove(lock)


def _run(method, scen, seed, a, log, out):
    tmp = out + ".part"
    for old in (tmp, tmp + ".tmp"):
        if os.path.exists(old):
            os.remove(old)                    # leftover of an interrupted run
    env = dict(os.environ, GZ_PARTITION=f"e4batch_{os.getpid()}", ROS_DOMAIN_ID=str(a.domain))
    cmd = ["ros2", "launch", "lpa2v_sim", "e4.launch.py", f"method:={method}", f"scenario:={scen}",
           f"seed:={seed}", f"duration:={a.duration}", f"fault_start:={a.fault[0]}",
           f"fault_end:={a.fault[1]}", f"rtf:={a.rtf}", f"out:={tmp}", "auto_shutdown:=false"]
    t0 = time.time()
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    limit = a.duration / a.rtf * 6 + 90       # margin for a loaded machine (slow run > lost run)
    while not os.path.exists(tmp) and time.time() - t0 < limit and p.poll() is None:
        time.sleep(0.5)
    time.sleep(0.5)                       # ensures the CSV write has finished
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    p.wait()
    if not os.path.exists(tmp):
        msg = f"FAILED (no CSV in {time.time() - t0:.0f} s)"
    elif not _robot_moved(tmp):
        os.replace(tmp, out + ".invalido_robo_parado")   # controller did not start: infrastructure failure
        msg = "FAILED (robot stopped: v = 0 for the whole run)"
    else:
        n = sum(1 for _ in open(tmp)) - 1
        os.replace(tmp, out)
        msg = f"ok {n} samples"
    log.write(f"{datetime.now():%H:%M:%S} w{a.domain} {scen:11s} {method:12s} s{seed:<3d} {msg} "
              f"({time.time() - t0:.0f} s)\n")
    log.flush()
    time.sleep(1.0)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    ap.add_argument("--methods", nargs="+", default=METHODS)
    ap.add_argument("--scenarios", nargs="+", default=SCENARIOS)
    ap.add_argument("--duration", type=float, default=90.0)
    ap.add_argument("--fault", type=float, nargs=2, default=(30.0, 75.0))
    ap.add_argument("--rtf", type=float, default=2.5)
    ap.add_argument("--domain", type=int, default=81)
    a = ap.parse_args()
    os.makedirs(RUNS, exist_ok=True)
    total = len(a.seeds) * len(a.scenarios) * len(a.methods)
    k = 0
    with open(os.path.join(RUNS, "progresso.log"), "a") as log:
        log.write(f"# {datetime.now().isoformat(timespec='seconds')} batch: {vars(a)}\n")
        for seed in a.seeds:                  # seed in the outer loop: partial results stay balanced
            for scen in a.scenarios:
                for m in a.methods:
                    k += 1
                    r = run_one(m, scen, seed, a, log)
                    print(f"[{k}/{total}] {scen} {m} s{seed}: {r}", flush=True)
    print("end of batch", file=sys.stderr)


if __name__ == "__main__":
    main()

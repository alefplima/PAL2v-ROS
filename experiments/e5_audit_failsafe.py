"""E5 - audit of the PX4 failsafe warnings in the valid flights (results/e5_runs/logs).

The batch integrity criterion (e5_px4.py, SIM_ARTIFACTS) marks as INVALID only the simulated-sensor
timeouts (Accel/Gyro/Baro/Mag #0 fail). After the batch, a second simulator artifact showed up,
"Preflight Fail: Battery unhealthy" -> "Failsafe activated". The batch criterion was NOT changed, so that the
code remains the one that generated the published data; this script audits, flight by flight, whether the
warning changed the flight.

For each valid flight (directory without ".anterior" = previous attempt) with the warning, it reports:
  instant of the warning after arming [s], failsafe mode changes, landings and disarms.
Output: results/e5_failsafe_audit.md
Usage: python3 e5_audit_failsafe.py      (needs the batch logs, published in the Zenodo data package)
"""
import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "results", "e5_runs", "logs")
OUT = os.path.join(HERE, "results", "e5_failsafe_audit.md")
PAT = re.compile(r"Battery unhealthy|Failsafe activated|Attitude failure")


def stamp(line):
    try:
        return float(line.split()[0])
    except (IndexError, ValueError):
        return None


def audit(run_dir):
    logs = sorted(glob.glob(os.path.join(run_dir, "**", "launch.log"), recursive=True), key=os.path.getmtime)
    if not logs:
        return None
    lines = open(logs[-1], errors="replace").read().splitlines()
    armed = next((stamp(l) for l in lines if "Armed by external" in l), None)
    hits = [l for l in lines if PAT.search(l)]
    if not hits:
        return None
    t0 = stamp(hits[0])
    reason = re.sub(r"^.*(WARN|INFO)\s+\[[^\]]+\]\s*", "", hits[0]).strip()
    modes = sorted({m.strip() for l in lines for m in re.findall(r"Failsafe: ([^\t]+)", l)})
    return {"after_arm": None if armed is None or t0 is None else t0 - armed, "reason": reason,
            "modes": modes, "landings": sum("Landing detected" in l for l in lines),
            "disarms": sum("Disarmed" in l for l in lines)}


def main():
    rows = []
    for d in sorted(os.listdir(LOGS)):
        if ".anterior" in d or not os.path.isdir(os.path.join(LOGS, d)):
            continue
        r = audit(os.path.join(LOGS, d))
        if r:
            rows.append((d, r))
    with open(OUT, "w") as f:
        f.write("# E5 - failsafe audit of the valid flights\n\n")
        f.write(f"Valid flights with a warning: {len(rows)}\n\n")
        f.write("| flight | warning after arming [s] | 1st warning | failsafe modes | landings | disarms |\n")
        f.write("| --- | --- | --- | --- | --- | --- |\n")
        for d, r in rows:
            t = "?" if r["after_arm"] is None else f"{r['after_arm']:.0f}"
            f.write(f"| {d} | {t} | {r['reason']} | {', '.join(r['modes']) or '—'} | {r['landings']} | "
                    f"{r['disarms']} |\n")
        f.write("\nNo mode change, landing or disarm = the warning did not change the flight. With an effect, check in the "
                "height series (flight CSV) whether the ground touch came before the warning.\n")
    print(OUT)
    for d, r in rows:
        print(d, r)


if __name__ == "__main__":
    main()

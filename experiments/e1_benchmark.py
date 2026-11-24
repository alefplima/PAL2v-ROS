"""E1 - Computational cost of the PAL2v algorithms (latency per call).

Rationale: the "low complexity / edge computing" argument (de Carvalho Jr. et al., EAAI 2024)
needs numbers. Measures the mean time per call in pure Python (worst case; C++/MicroPython aside).
"""
import random
import time

from common import env_info, save_json
from lpa2v import LearningCell, PAL2vController, PAL2vFusion, analyze, para_extractor

N = 200_000
rng = random.Random(0)


def bench(fn, n):
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1e6   # µs/call


def main():
    mus = [(rng.random(), rng.random()) for _ in range(1024)]
    k = [0]

    def pan_call():
        m, l = mus[k[0] & 1023]
        k[0] += 1
        analyze(m, l)

    cell = LearningCell(lf=0.8)
    fus3 = PAL2vFusion.from_sigma([0.3, 0.03, 0.5], 0.0, 10.0, max_rate=4.0, dt=0.01)
    ctl = PAL2vController(2.0, 0.6, 12.0, 3.0, 0.0, 30.0, 14.7)
    data10 = [rng.random() * 10 for _ in range(10)]
    res = {
        "env": env_info(),
        "n_calls": N,
        "us_per_call": {
            "PAN (analyze, 12 states)": bench(pan_call, N),
            "lPANCell step": bench(lambda: cell.step(0.7), N),
            "PAL2vFusion 3 sensors": bench(lambda: fus3.update([2.0, 2.01, 2.2]), N // 4),
            "PAL2vController update": bench(lambda: ctl.update(3.0, 2.5, 0.01), N // 4),
            "ParaExtrctr 10 samples": bench(lambda: para_extractor(data10, 0, 10), N // 10),
        },
    }
    for name, us in res["us_per_call"].items():
        print(f"{name:32s} {us:8.2f} µs   ({1e6 / us:,.0f} calls/s)")
    save_json("e1_benchmark.json", res)


if __name__ == "__main__":
    main()

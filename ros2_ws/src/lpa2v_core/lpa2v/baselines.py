"""Classical (non-paraconsistent) comparison methods used in the experiments.

KalmanChi2: 1D Kalman filter (position + velocity, white acceleration) with per-sensor fault detection and
exclusion by the χ² test of the normalized innovation (NIS), the classical FDI approach in redundant fusion.

  - At each step: prediction; NIS_i = ν_i² / S_i of each sensor against the predicted state; sensors with
    NIS_i > gate (χ²₁; 6.63 = 99 %) are rejected; the accepted ones enter sequential updates.
  - If ALL are rejected for `n_reset` consecutive steps (the state itself got lost, e.g. at a real step
    of the quantity), the filter is reinitialized at the median of the readings with a large covariance.
    Meanwhile it only predicts.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple


class KalmanChi2:
    def __init__(self, sigma: Sequence[float], q: float, dt: float, gate: float = 6.635,
                 n_reset: int = 3, p0_vel: float = 1.0):
        self.r = [float(s) ** 2 for s in sigma]
        self.q, self.dt, self.gate, self.n_reset, self.p0_vel = float(q), float(dt), float(gate), int(n_reset), p0_vel
        self.x = None              # [position, velocity]
        self.P = None              # [[p00, p01], [p01, p11]]
        self.all_rej = 0
        self.rejected: List[int] = []

    def _init(self, z: Sequence[float]) -> None:
        s = sorted(z)
        n = len(s)
        m = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
        self.x = [m, 0.0]
        self.P = [[max(self.r), 0.0], [0.0, self.p0_vel]]
        self.all_rej = 0

    def _predict(self) -> None:
        dt, q = self.dt, self.q
        x0, x1 = self.x
        self.x = [x0 + dt * x1, x1]
        (a, b), (_, d) = self.P
        a2 = a + 2 * dt * b + dt * dt * d + q * dt ** 4 / 4
        b2 = b + dt * d + q * dt ** 3 / 2
        d2 = d + q * dt * dt
        self.P = [[a2, b2], [b2, d2]]

    def _nis(self, z: float, r: float) -> float:
        return (z - self.x[0]) ** 2 / (self.P[0][0] + r)

    def _update(self, z: float, r: float) -> None:
        (a, b), (_, d) = self.P
        s = a + r
        k0, k1 = a / s, b / s
        nu = z - self.x[0]
        self.x = [self.x[0] + k0 * nu, self.x[1] + k1 * nu]
        self.P = [[(1 - k0) * a, (1 - k0) * b], [(1 - k0) * b, d - k1 * b]]

    def update(self, z: Sequence[float]) -> float:
        z = [float(v) for v in z]
        if self.x is None or any(math.isnan(v) for v in self.x):
            self._init(z)
            self.rejected = []
            return self.x[0]
        self._predict()
        nis = [self._nis(zi, ri) for zi, ri in zip(z, self.r)]
        acc = [i for i, v in enumerate(nis) if v <= self.gate]
        self.rejected = [i for i in range(len(z)) if i not in acc]
        if not acc:
            self.all_rej += 1
            if self.all_rej >= self.n_reset:
                self._init(z)
                self.rejected = []
            return self.x[0]
        self.all_rej = 0
        for i in sorted(acc, key=lambda i: nis[i]):
            self._update(z[i], self.r[i])
        return self.x[0]

    def step(self, z: Sequence[float]) -> Tuple[float, List[int]]:
        v = self.update(z)
        return v, list(self.rejected)

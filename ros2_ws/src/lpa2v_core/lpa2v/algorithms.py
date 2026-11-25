"""Higher-level PAL2v algorithms, built on the PAN (core.analyze).

- para_extractor      : ParaExtrctr (Da Silva Filho et al., 2011; Paraconsistent-Lib 2025)
- LearningCell        : paraconsistent artificial neural cell of learning (lPANCell), also used
                        as the "PAL2v filter" (Robotics 2023, Eqs. 8 and 9)
- PAL2vFusion         : NAP + ParaExtrctr network for redundant-sensor fusion (proposed in this work)
- ParaconsistentHybridPI : paraconsistent hybrid PI of Coelho, Da Silva Filho et al. (CEP 2019)
- PAL2vController     : PAL2v-PI with evidence of the predicted error (proposed in this work)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

from .core import analyze, clamp01, denormalize, normalize


# --------------------------------------------------------------------------- ParaExtrctr
def para_extractor(values: Sequence[float], vmin: float, vmax: float) -> float:
    """Paraconsistent algorithm extractor of contradiction effects (ParaExtrctr), Da Silva Filho 2009.

    1. normalize the set to [0,1];  2. mu = max, lam = 1 - min;  3. compute muER;
    4. remove max and min and insert muER;  5. repeat until one value is left;  6. denormalize.
    """
    vals = [float(v) for v in values]
    if not vals:
        raise ValueError("empty set")
    lo, hi = float(vmin), float(vmax)      # limits of the Universe of Discourse (UD)
    g = [normalize(v, lo, hi) for v in vals]
    while len(g) > 1:
        g.sort()
        mu_min, mu_max = g.pop(0), g.pop(-1)
        g.append(analyze(mu_max, 1.0 - mu_min).mu_er)
    return denormalize(g[0], lo, hi)


# --------------------------------------------------------------------------- lPANCell / filter
@dataclass
class LearningCell:
    """lPANCell: muE(k+1) = ((mu1 - (1 - muE(k))) * lF + 1) / 2.

    With lF = 1 it converges geometrically (factor 1/2 per iteration) to the pattern mu1;
    lF < 1 slows the response (low-pass). muctr is the normalized contradiction degree.
    """
    lf: float = 1.0
    mu_e: float = 0.5
    mu_ctr: float = 0.5

    def step(self, mu1: float) -> float:
        mu1 = clamp01(mu1)
        comp = 1.0 - self.mu_e
        self.mu_ctr = clamp01((mu1 + comp * self.lf) / 2.0)
        self.mu_e = clamp01(((mu1 - comp) * self.lf + 1.0) / 2.0)
        return self.mu_e

    def reset(self, mu_e: float = 0.5) -> None:
        self.mu_e, self.mu_ctr = mu_e, 0.5


class PAL2vFilter:
    """PAL2v filter for a physical quantity: normalize, apply the lPANCell, denormalize."""

    def __init__(self, vmin: float, vmax: float, lf: float = 1.0, iterations: int = 1):
        self.vmin, self.vmax, self.iterations = vmin, vmax, max(1, int(iterations))
        self.cell = LearningCell(lf=lf)
        self._init = False

    def update(self, x: float) -> float:
        mu = normalize(x, self.vmin, self.vmax)
        if not self._init:
            self.cell.reset(mu)
            self._init = True
        for _ in range(self.iterations):
            self.cell.step(mu)
        return denormalize(self.cell.mu_e, self.vmin, self.vmax)


# --------------------------------------------------------------------------- Fusion
@dataclass
class FusionOutput:
    value: float                 # fused value (physical unit)
    mu: List[float]              # evidence degree of each sensor in the UD
    gct: List[float]             # contradiction degree of each sensor against the group (final NAP)
    phi_e: List[float]           # evidence interval phiE = 1 - |Gct| of each sensor
    labels: List[str]            # Para-Analyzer on Gct: "t" consistent, "⊤" / "⊥" contradiction
    certainty: float             # AND connective (minimum) of the phiE of the accepted sensors; 0 = indefiniteness
    contradiction: float         # OR connective (maximum) of |Gct| over all sensors
    rejected: List[int] = field(default_factory=list)
    indefinite: bool = False     # no decision possible: output held (retention)
    mu_group: float = 0.5        # muE of the accepted group (ParaExtrctr output)
    weights: List[float] = field(default_factory=list)  # tolerance interval of each sensor (1 = full evidence)


class PAL2vFusion:
    """Fusion of N redundant sensors using ONLY PAL2v algorithms (no mean, no median).

    Structure (paraconsistent analysis network):
      1. Evidence extractor: mu_i = (x_i - vmin)/(vmax - vmin) in the Universe of Discourse (UD).
      2. Temporal analysis of each sensor by a learning cell (CNAPap), one iteration = one sample:
         (mu_i(k), 1 - p_i), where p_i is the learned pattern (the previous reading, cell run to convergence).
         Gct_i^t = mu_i(k) - p_i; its tolerance is rate_lim_i (equivalently Gc = 1 - |Gct| >= 1 - rate_lim_i).
      3. ParaExtrctr over the sensors -> muE of the group (contradiction effects extracted).
      4. Spatial NAP of each sensor against the ParaExtrctr of the OTHERS: (mu_i, 1 - muE_o) -> Gct_i.

    Two modes for what is done with a contradictory sensor:
      mode="graded" (no exclusion; recommended): NO source leaves the network. Each sensor has a tolerance
        interval phi_i = AND(phi_t, phi_s) (minimum connective) with phi(r) = 1 while the normalized contradiction
        r = |Gct|/lim <= 1 (tolerance zone, as Ftct of the CNAPp) and falling linearly to 0 at r = band.
        Its evidence enters the ParaExtrctr as  mu~_i = muE_o + phi_i (mu_i - muE_o): a fully contradictory sensor
        contributes the evidence of the others (Gct = 0 against them), a consistent one contributes its own.
        Sensors are processed from the most to the least contradictory (Gauss-Seidel order), so a faulty sensor
        does not contaminate the reference of the healthy ones.
      mode="step": the limit case band -> 1 (phi is 1 or 0), i.e. the sensor above the limit is retained
        (v1.0 behaviour). Kept for reproducibility of v1.0.0.
    With 2 sources contradicting each other neither can be blamed: the network's logical output is
    INDEFINITENESS (certainty = 0, as item 7 of the NAP). The published value is then the last defined output
    (hold_on_indefinite=True): this retention is a decision of the application layer that consumes the
    indefiniteness signal (like the H signal of the CNAPd), not an exclusion made by the logic.
    """

    def __init__(self, vmin: float, vmax: float, ct_lim: Sequence[float],
                 rate_lim: Sequence[float], mode: str = "graded", band: float = 1.5,
                 hold_on_indefinite: bool = True):
        if vmax <= vmin:
            raise ValueError("vmax must be greater than vmin")
        self.n = len(ct_lim)
        if self.n < 2:
            raise ValueError("fusion requires at least 2 sensors")
        if len(rate_lim) != self.n:
            raise ValueError("ct_lim and rate_lim must have the same length")
        self.vmin, self.vmax = float(vmin), float(vmax)
        self.ct_lim = [float(v) for v in ct_lim]
        self.rate_lim = [float(v) for v in rate_lim]
        if mode not in ("step", "graded"):
            raise ValueError("mode must be 'step' or 'graded'")
        if band <= 1.0:
            raise ValueError("band must be > 1")
        self.mode, self.band, self.hold = mode, float(band), bool(hold_on_indefinite)
        self._prev: Optional[List[float]] = None
        self._last: Optional[float] = None

    @classmethod
    def from_sigma(cls, sigma: Sequence[float], vmin: float, vmax: float, max_rate: float,
                   dt: float, k: float = 4.0, **kw) -> "PAL2vFusion":
        """Contradiction control limits from the sensor noise (in UD units).

        ct_lim_i   = k * sqrt(sigma_i^2 + max_{j!=i} sigma_j^2) / (vmax - vmin): the comparison is made
                     against the group, whose uncertainty is bounded by the worst of the other sensors.
        rate_lim_i = (max_rate * dt + k * sigma_i * sqrt(2)) / (vmax - vmin): maximum physical change per
                     sample plus the noise of the difference between two samples.
        """
        s = [float(v) for v in sigma]
        span = float(vmax) - float(vmin)
        ct, rt = [], []
        for i, si in enumerate(s):
            sc = max(sj for j, sj in enumerate(s) if j != i)
            ct.append(k * math.sqrt(si ** 2 + sc ** 2) / span)
            rt.append((max_rate * dt + k * si * math.sqrt(2.0)) / span)
        return cls(vmin, vmax, ct, rt, **kw)

    def reset(self) -> None:
        self._prev, self._last = None, None

    @staticmethod
    def _extract(mus: List[float]) -> float:
        g = list(mus)
        while len(g) > 1:
            g.sort()
            mu_min, mu_max = g.pop(0), g.pop(-1)
            g.append(analyze(mu_max, 1.0 - mu_min).mu_er)
        return g[0]

    def _phi(self, r: float) -> float:
        """Tolerance interval: 1 inside the tolerance (r <= 1), linear to 0 at r = band."""
        return clamp01((self.band - r) / (self.band - 1.0))

    def update(self, x: Sequence[float]) -> FusionOutput:
        if self.mode == "graded":
            return self._update_graded(x)
        return self._update_step(x)

    def _update_graded(self, x: Sequence[float]) -> FusionOutput:
        mu = [normalize(float(v), self.vmin, self.vmax) for v in x]
        if len(mu) != self.n:
            raise ValueError(f"expected {self.n} sensors, got {len(mu)}")
        prev = self._prev if self._prev is not None else mu
        self._prev = mu
        n = self.n
        # 2. temporal CNAPap
        phi_t = [self._phi(abs(analyze(mu[i], 1.0 - prev[i]).dct) / self.rate_lim[i]) for i in range(n)]
        # 3-4. spatial NAP, most contradictory first; nobody is removed, evidence is attenuated
        mt, w, done = list(mu), [1.0] * n, set()
        indefinite = all(p == 0.0 for p in phi_t)
        pair_phi = 1.0
        while len(done) < n and not indefinite:
            best = None
            for i in range(n):
                if i in done:
                    continue
                others = [j for j in range(n) if j != i and (j not in done or w[j] > 0.0)]
                mo = self._extract([mt[j] for j in others]) if others else mu[i]
                r = abs(analyze(mu[i], 1.0 - mo).dct) / self.ct_lim[i]
                if best is None or r > best[0]:
                    best = (r, i, mo, others)
            r, i, mo, others = best
            if len(others) <= 1:
                # only two effective sources: contradiction cannot be attributed to either one
                pair_phi = self._phi(r) if others else 1.0
                for j in [k for k in range(n) if k not in done]:
                    ref = mo if j == i else mu[i]       # each one is referred to the other source
                    w[j] = phi_t[j]                     # only the temporal analysis attenuates
                    mt[j] = ref + w[j] * (mu[j] - ref)
                    done.add(j)
                indefinite = pair_phi == 0.0
                break
            w[i] = min(phi_t[i], self._phi(r))
            mt[i] = mo + w[i] * (mu[i] - mo)
            done.add(i)
        mu_g = self._extract(mt)
        gct = [analyze(m, 1.0 - mu_g).dct for m in mu]
        phi_e = [1.0 - abs(g) for g in gct]
        eff = [i for i in range(n) if w[i] > 0.0]
        labels = ["t" if (w[i] >= 1.0 and not indefinite) else ("⊤" if gct[i] >= 0 else "⊥")
                  for i in range(n)]
        cert = 0.0 if indefinite or not eff else min(min(phi_e[i] for i in eff), pair_phi)
        if indefinite and self.hold and self._last is not None:
            value = self._last
        else:
            value = denormalize(mu_g, self.vmin, self.vmax)
            if not indefinite:
                self._last = value
        attenuated = [i for i in range(n) if w[i] < 1.0]   # outside the tolerance (not removed)
        return FusionOutput(value, mu, gct, phi_e, labels, cert, max(abs(g) for g in gct),
                            attenuated, indefinite, mu_g, w)

    def _update_step(self, x: Sequence[float]) -> FusionOutput:
        mu = [normalize(float(v), self.vmin, self.vmax) for v in x]
        if len(mu) != self.n:
            raise ValueError(f"expected {self.n} sensors, got {len(mu)}")
        prev = self._prev if self._prev is not None else mu
        self._prev = mu
        # 2. temporal NAP
        cand = [i for i in range(self.n)
                if abs(analyze(mu[i], 1.0 - prev[i]).dct) <= self.rate_lim[i]]
        # 3-4. ParaExtrctr + spatial NAP with iterative removal of the most contradictory sensor
        indefinite = len(cand) == 0
        mu_g = 0.5
        while cand:
            mu_g = self._extract([mu[i] for i in cand])
            if len(cand) == 1:
                break
            # each sensor is confronted with the ParaExtrctr of the OTHERS (the sensor does not "pull" the group)
            ratio = {i: abs(analyze(mu[i], 1.0 - self._extract([mu[j] for j in cand if j != i])).dct)
                     / self.ct_lim[i] for i in cand}
            worst = max(ratio, key=ratio.get)
            if ratio[worst] <= 1.0:
                break
            if len(cand) <= 2:
                indefinite = True     # two sensors in contradiction: no decision
                break
            cand.remove(worst)
        gct = [analyze(m, 1.0 - mu_g).dct for m in mu]
        phi_e = [1.0 - abs(g) for g in gct]
        labels = []
        for i, g in enumerate(gct):
            if i in cand and not indefinite:
                labels.append("t")
            else:
                labels.append("⊤" if g >= 0 else "⊥")
        if indefinite:
            rejected = list(range(self.n)) if not cand else [i for i in range(self.n)
                                                             if i not in cand]
            value = self._last if self._last is not None else denormalize(mu_g, self.vmin, self.vmax)
            cert = 0.0
        else:
            rejected = [i for i in range(self.n) if i not in cand]
            value = denormalize(mu_g, self.vmin, self.vmax)
            self._last = value
            cert = min(phi_e[i] for i in cand)
        return FusionOutput(value, mu, gct, phi_e, labels, cert, max(abs(g) for g in gct),
                            rejected, indefinite, mu_g,
                            [0.0 if i in rejected else 1.0 for i in range(self.n)])


# --------------------------------------------------------------------------- Controller
class PAL2vController:
    """Paraconsistent hybrid PI controller (PAL2v-PI).

    Proposition: P = "the control action must increase".
      mu  = 0.5 + e / (2 E)               evidence from the current error
      mu2 = 0.5 + (e + Td * de/dt) / (2 E) evidence from the error predicted Td seconds ahead
      lam = 1 - mu2
    If both sources agree, DcR -> ±1 (strong action). If they disagree (the error is still positive but
    the output is approaching fast), Dct grows, DcR -> 0 and the action is attenuated, which damps the
    overshoot without an explicit derivative term.
      u = u_ff + Kc * DcR + Ki * ∫ e dt     (anti-windup by conditional integration)
    """

    def __init__(self, e_max: float, td: float, kc: float, ki: float,
                 u_min: float, u_max: float, u_ff: float = 0.0, d_alpha: float = 0.2):
        self.e_max, self.td, self.kc, self.ki = e_max, td, kc, ki
        self.u_min, self.u_max, self.u_ff = u_min, u_max, u_ff
        self.d_alpha = d_alpha
        self.integ = 0.0
        self._y_prev: Optional[float] = None
        self._dy = 0.0
        self.last = None

    def reset(self) -> None:
        self.integ, self._y_prev, self._dy, self.last = 0.0, None, 0.0, None

    def update(self, r: float, y: float, dt: float) -> float:
        e = r - y
        if self._y_prev is None:
            self._y_prev = y
        dy_raw = (y - self._y_prev) / dt
        self._y_prev = y
        self._dy += self.d_alpha * (dy_raw - self._dy)       # filtered derivative (1st order)
        de = -self._dy                                       # derivative on measurement (no "kick")
        mu = clamp01(0.5 + e / (2.0 * self.e_max))
        mu2 = clamp01(0.5 + (e + self.td * de) / (2.0 * self.e_max))
        res = analyze(mu, 1.0 - mu2)
        self.last = res
        u_unsat = self.u_ff + self.kc * res.dcr + self.ki * self.integ
        u = min(max(u_unsat, self.u_min), self.u_max)
        if u == u_unsat or (u_unsat > self.u_max and e < 0) or (u_unsat < self.u_min and e > 0):
            self.integ += e * dt
        return u


class ParaconsistentHybridPI:
    """Paraconsistent Hybrid PI controller of Coelho, Da Silva Filho et al. (Control Eng. Practice, 2019).

    Network of 2 NAPs (PANnet). NAP 1 (complete) handles the redundant measurements and gives the PV as muER.
    Here it is replaced by the PAL2vFusion, which generalizes NAP 1 to N sensors. NAP 2 (partial) receives:
      mu  = PV normalized in the UD ; lam = 1 - normalized SP
      Gct   = mu + lam - 1        (0 when PV = SP; > 0 when PV > SP)  -> input of the INTEGRAL action
      muctr = (mu + lam) / 2      (0.5 when PV = SP)                   -> input of the PROPORTIONAL action
    Reverse action (increase the output when PV < SP):
      MV = u_ff + (u_max - u_min) * [ Kp * (0.5 - muctr) + Ki * ∫ (-Gct) dt ]
    Anti-windup by conditional integration (same as the reference PID).
    """

    def __init__(self, vmin: float, vmax: float, kp: float, ki: float,
                 u_min: float, u_max: float, u_ff: float = 0.0):
        self.vmin, self.vmax, self.kp, self.ki = vmin, vmax, kp, ki
        self.u_min, self.u_max, self.u_ff = u_min, u_max, u_ff
        self.integ = 0.0
        self.last = None

    def reset(self) -> None:
        self.integ, self.last = 0.0, None

    def update(self, r: float, y: float, dt: float) -> float:
        res = analyze(normalize(y, self.vmin, self.vmax), 1.0 - normalize(r, self.vmin, self.vmax))
        self.last = res
        span = self.u_max - self.u_min
        e_i = -res.dct
        u_unsat = self.u_ff + span * (self.kp * (0.5 - res.mu_ect) + self.ki * self.integ)
        u = min(max(u_unsat, self.u_min), self.u_max)
        if u == u_unsat or (u_unsat > self.u_max and e_i < 0) or (u_unsat < self.u_min and e_i > 0):
            self.integ += e_i * dt
        return u


class PID:
    """Classical reference PID (derivative on the filtered measurement, conditional anti-windup)."""

    def __init__(self, kp: float, ki: float, kd: float, u_min: float, u_max: float,
                 u_ff: float = 0.0, d_alpha: float = 0.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_min, self.u_max, self.u_ff, self.d_alpha = u_min, u_max, u_ff, d_alpha
        self.reset()

    def reset(self) -> None:
        self.integ, self._y_prev, self._dy = 0.0, None, 0.0

    def update(self, r: float, y: float, dt: float) -> float:
        e = r - y
        if self._y_prev is None:
            self._y_prev = y
        self._dy += self.d_alpha * ((y - self._y_prev) / dt - self._dy)
        self._y_prev = y
        u_unsat = self.u_ff + self.kp * e + self.ki * self.integ - self.kd * self._dy
        u = min(max(u_unsat, self.u_min), self.u_max)
        if u == u_unsat or (u_unsat > self.u_max and e < 0) or (u_unsat < self.u_min and e > 0):
            self.integ += e * dt
        return u

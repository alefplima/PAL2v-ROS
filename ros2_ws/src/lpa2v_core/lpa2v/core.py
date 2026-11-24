"""Core of the Paraconsistent Annotated Logic with two-valued annotation (PAL2v / LPA2v).

Paraconsistent Analysis Node (PAN, "NAP" in Portuguese) equations, following Prof. J. I. Da Silva Filho's
course material (NAP algorithm; Da Silva Filho, Lambert-Torres and Abe, 2009) and JILSA 2012:

    1. mu, lambda                            (lambda = 1 - mu2 for a second source)
    2. Gc   = mu - lambda                    (certainty degree)
    3. Gct  = mu + lambda - 1                (contradiction degree)
    4. d    = sqrt((1 - |Gc|)^2 + Gct^2)
    5. muctr = (mu + lambda)/2               (normalized contradiction degree)
    6. phiE = 1 - |2 muctr - 1|              (resulting evidence interval)
    7. if phiE <= 0.25 or d > 1:  S1 = 0.5, S2 = phiE(±)   -> INDEFINITENESS
    8. GcR  = 1 - d (Gc > 0) ;  d - 1 (Gc < 0)
    9. muER = (GcR + 1)/2
   10. S1 = muER, S2 = phiE(±)

No statistical operation (mean, median) is used: only the equations above.

No external dependencies: runs on a microcontroller with MicroPython, in a ROS 2 node or in a notebook.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict

# Labels of the 12 logical states of the Para-Analyzer (order: 4 extreme + 8 non-extreme)
T = "t"            # True
F = "F"            # False
TOP = "⊤"          # Inconsistent
BOT = "⊥"          # Paracomplete / indeterminate
QT_TOP = "Qt→⊤"    # Quasi-true tending to inconsistent
QT_BOT = "Qt→⊥"    # Quasi-true tending to paracomplete
QF_TOP = "QF→⊤"    # Quasi-false tending to inconsistent
QF_BOT = "QF→⊥"    # Quasi-false tending to paracomplete
QTOP_T = "Q⊤→t"    # Quasi-inconsistent tending to true
QTOP_F = "Q⊤→F"    # Quasi-inconsistent tending to false
QBOT_T = "Q⊥→t"    # Quasi-paracomplete tending to true
QBOT_F = "Q⊥→F"    # Quasi-paracomplete tending to false
UNDEF = "I"        # Undefined (exact center of the lattice)

STATES = (T, F, TOP, BOT, QT_TOP, QT_BOT, QF_TOP, QF_BOT, QTOP_T, QTOP_F, QBOT_T, QBOT_F)


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else float(x)


def normalize(value: float, vmin: float, vmax: float, inverse: bool = False) -> float:
    """Evidence-degree extractor (JILSA 2012, Alg. 3.2): linear variation, direct or inverse."""
    if vmax == vmin:
        raise ValueError("vmax must differ from vmin")
    mu = clamp01((value - vmin) / (vmax - vmin))
    return 1.0 - mu if inverse else mu


def denormalize(mu: float, vmin: float, vmax: float) -> float:
    return vmin + clamp01(mu) * (vmax - vmin)


@dataclass(frozen=True)
class PANResult:
    """Outputs of a Paraconsistent Analysis Node (PAN)."""
    mu: float
    lam: float
    dc: float
    dct: float
    phi: float
    d: float
    D: float
    dcr: float
    mu_e: float
    mu_ect: float
    mu_er: float
    phi_e: float
    phi_signed: float          # signed certainty interval φ(±): + if Dct>0, - if Dct<0
    label: str                 # logical state (12 regions)
    decision: float            # PANCd: 1.0 (T), 0.0 (F) or 0.5 (undefined)
    indefinite: bool = False   # item 7 of the NAP algorithm (phiE <= 0.25 or d > 1)
    extras: Dict[str, float] = field(default_factory=dict)


def classify(dc: float, dct: float, c_lim: float = 0.5, ct_lim: float = 0.5) -> str:
    """12-state Para-Analyzer.

    c_lim  = certainty control limit (Vscc = +c_lim, Vicc = -c_lim)
    ct_lim = contradiction control limit (Vscct = +ct_lim, Vicct = -ct_lim)
    """
    if dc >= c_lim:
        return T
    if dc <= -c_lim:
        return F
    if dct >= ct_lim:
        return TOP
    if dct <= -ct_lim:
        return BOT
    if dc == 0.0 and dct == 0.0:
        return UNDEF
    adc, adct = abs(dc), abs(dct)
    if dc >= 0 and dct >= 0:
        return QT_TOP if adc >= adct else QTOP_T
    if dc >= 0 and dct < 0:
        return QT_BOT if adc >= adct else QBOT_T
    if dc < 0 and dct >= 0:
        return QF_TOP if adc >= adct else QTOP_F
    return QF_BOT if adc >= adct else QBOT_F


PHI_E_MIN = 0.25   # evidence interval below which the NAP declares indefiniteness


def analyze(mu: float, lam: float, ftc: float = 0.5,
            c_lim: float | None = None, ct_lim: float | None = None,
            phi_e_min: float = PHI_E_MIN) -> PANResult:
    """Full Paraconsistent Analysis Node (PAN) + Para-Analyzer + decision (PANCd).

    mu   : favorable evidence degree   [0,1]
    lam  : unfavorable evidence degree [0,1]  (use lam = 1 - mu2 for a second source)
    ftc  : certainty tolerance factor (Paraconsistent-Lib). By default it sets
           c_lim = ftc and ct_lim = 1 - ftc, and it is the PANCd decision threshold on muER.
    phi_e_min : indefiniteness limit of item 7 of the NAP algorithm (0.25 in the course material).
    """
    mu, lam = clamp01(mu), clamp01(lam)
    if c_lim is None:
        c_lim = ftc
    if ct_lim is None:
        ct_lim = 1.0 - ftc
    dc = mu - lam
    dct = mu + lam - 1.0
    phi = 1.0 - abs(dct)
    d = math.hypot(1.0 - abs(dc), dct)
    D = min(d, 1.0)
    mu_ect = (mu + lam) / 2.0                     # muctr (item 5)
    phi_e = 1.0 - abs(2.0 * mu_ect - 1.0)         # item 6
    indefinite = phi_e <= phi_e_min or d > 1.0    # item 7
    if indefinite or dc == 0:
        dcr = 0.0                                 # S1 = 0.5
    elif dc > 0:
        dcr = 1.0 - d                             # item 8
    else:
        dcr = d - 1.0
    mu_e = (dc + 1.0) / 2.0
    mu_er = (dcr + 1.0) / 2.0                     # item 9
    phi_signed = phi if dct > 0 else (-phi if dct < 0 else 0.0)
    if indefinite:
        decision = 0.5
    elif mu_er > ftc:
        decision = 1.0
    elif mu_er < ftc:
        decision = 0.0
    else:
        decision = 0.5
    return PANResult(mu, lam, dc, dct, phi, d, D, dcr, mu_e, mu_ect, mu_er, phi_e,
                     phi_signed, classify(dc, dct, c_lim, ct_lim), decision, indefinite)


def pan(mu1: float, mu2: float, **kw) -> PANResult:
    """Shortcut for two favorable evidence sources: lam = 1 - mu2."""
    return analyze(mu1, 1.0 - clamp01(mu2), **kw)

"""Validation tests against published numerical examples (run: python -m unittest discover -s tests)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lpa2v import (ParaconsistentHybridPI, PAL2vController, PAL2vFusion, LearningCell, analyze, classify,  # noqa: E402
                   normalize, pan, para_extractor)
from lpa2v.core import BOT, F, QT_TOP, QTOP_T, T, TOP, UNDEF  # noqa: E402


class TestPANJILSA2012(unittest.TestCase):
    """Numerical example of Da Silva Filho (JILSA 2012, Section 4): mu1 = 0.87, mu2 = 0.68."""

    def test_example(self):
        r = pan(0.87, 0.68)
        self.assertAlmostEqual(r.lam, 0.32, places=9)
        self.assertAlmostEqual(r.dc, 0.55, places=9)
        self.assertAlmostEqual(r.dct, 0.19, places=9)
        self.assertAlmostEqual(r.d, 0.488466989, places=8)
        self.assertAlmostEqual(r.dcr, 0.51153301, places=7)
        self.assertAlmostEqual(r.mu_er, 0.755766505, places=8)
        self.assertAlmostEqual(r.phi, 0.81, places=9)
        self.assertAlmostEqual(r.phi_signed, 0.81, places=9)   # Dct > 0 -> φ(+)

    def test_normalization_example(self):
        # Temperature 38 °C in the universe [25, 42] -> 0.764706 ; voltage 413 V, inverse in [400, 480] -> 0.8375
        self.assertAlmostEqual(normalize(38, 25, 42), 0.764706, places=6)
        self.assertAlmostEqual(normalize(413, 400, 480, inverse=True), 0.8375, places=9)


class TestLatticeVertices(unittest.TestCase):
    def test_vertices(self):
        self.assertEqual(analyze(1, 0).label, T)
        self.assertEqual(analyze(0, 1).label, F)
        self.assertEqual(analyze(1, 1).label, TOP)
        self.assertEqual(analyze(0, 0).label, BOT)
        self.assertEqual(analyze(0.5, 0.5).label, UNDEF)
        self.assertAlmostEqual(analyze(1, 0).mu_er, 1.0)
        self.assertAlmostEqual(analyze(0, 1).mu_er, 0.0)
        self.assertAlmostEqual(analyze(1, 1).mu_er, 0.5)   # total contradiction -> no certainty

    def test_d_greater_than_one(self):
        r = analyze(0.95, 0.9)   # small Dc>0, large Dct -> d>1 -> DcR = 0
        self.assertGreater(r.d, 0.0)
        r2 = analyze(0.1, 0.0)  # d = sqrt(0.81+0.81) > 1
        self.assertGreater(r2.d, 1.0)
        self.assertEqual(r2.dcr, 0.0)

    def test_consistent_evidence_identity(self):
        # no contradiction (lam = 1 - mu): muER = mu
        for mu in (0.0, 0.1, 0.37, 0.5, 0.8, 1.0):
            self.assertAlmostEqual(analyze(mu, 1 - mu).mu_er, mu, places=12)

    def test_non_extreme_regions(self):
        self.assertEqual(classify(0.3, 0.1), QT_TOP)
        self.assertEqual(classify(0.1, 0.3), QTOP_T)


class TestLearningCellRobotics2023(unittest.TestCase):
    """Table 2 of Da Silva Filho et al. (Robotics 2023): lPANCell with lF = 1."""

    def test_table2_sequence(self):
        c = LearningCell(lf=1.0, mu_e=0.0009765)
        expected = {1.0: [0.50048828, 0.75024414, 0.87512207, 0.93756104, 0.96878052],
                    }
        for mu1, seq in expected.items():
            for v in seq:
                self.assertAlmostEqual(c.step(mu1), v, places=6)

    def test_converges_to_pattern(self):
        c = LearningCell(lf=1.0)
        for _ in range(60):
            c.step(0.4)
        self.assertAlmostEqual(c.mu_e, 0.4, places=9)

    def test_lower_lf_is_slower(self):
        fast, slow = LearningCell(lf=1.0), LearningCell(lf=0.4)
        for _ in range(5):
            fast.step(1.0)
            slow.step(1.0)
        self.assertGreater(fast.mu_e, slow.mu_e)


class TestNAPIndefinition(unittest.TestCase):
    """Item 7 of the NAP algorithm (course material): phiE <= 0.25 or d > 1 -> S1 = 0.5."""

    def test_low_evidence_interval(self):
        r = analyze(0.9, 0.9)                 # Gct = 0.8 -> phiE = 0.2
        self.assertAlmostEqual(r.phi_e, 0.2, places=12)
        self.assertTrue(r.indefinite)
        self.assertEqual(r.mu_er, 0.5)

    def test_d_above_one(self):
        r = analyze(0.6, 0.6)                 # Gc = 0, Gct = 0.2 -> d > 1
        self.assertTrue(r.indefinite)
        self.assertEqual(r.mu_er, 0.5)

    def test_definite(self):
        self.assertFalse(pan(0.87, 0.68).indefinite)


class TestParaExtractor(unittest.TestCase):
    """Solution of the course exercise (Exercicioa632831.xlsx, ParaExtrctr with 5 measurements)."""

    def test_professor_exercise_fuel_A(self):
        v = para_extractor([0.015748, 0.011811, 0.007874, 0.017717, 0.019685], 0.0, 1.0)
        self.assertAlmostEqual(v, 0.015216, places=6)

    def test_professor_exercise_fuel_B(self):
        v = para_extractor([0.055118, 0.070866, 0.064961, 0.000000, 0.059055], 0.0, 1.0)
        self.assertAlmostEqual(v, 0.057568, places=6)

    def test_professor_exercise_fuel_C_8(self):
        v = para_extractor([0.850394, 0.938976, 0.921260, 0.850394, 1.000000], 0.0, 1.0)
        self.assertAlmostEqual(v, 0.897977, places=6)

    def test_identical_values(self):
        self.assertAlmostEqual(para_extractor([3.0, 3.0, 3.0], 0, 10), 3.0, places=9)

    def test_within_range(self):
        v = para_extractor([2.0, 2.2, 1.9, 2.1, 6.0], 0, 10)
        self.assertTrue(0 <= v <= 10)


class TestFusion(unittest.TestCase):
    """PAL2vFusion: NAP + ParaExtrctr network, no mean and no median."""

    @staticmethod
    def make(rate=0.02):
        return PAL2vFusion(0.0, 10.0, ct_lim=[0.05, 0.05, 0.05], rate_lim=[rate] * 3)

    def test_consistent_sensors(self):
        out = self.make().update([2.0, 2.0, 2.0])
        self.assertEqual(out.rejected, [])
        self.assertAlmostEqual(out.value, 2.0, places=9)
        self.assertEqual(out.labels, ["t", "t", "t"])

    def test_rejects_stuck_sensor(self):
        fus = self.make()
        for _ in range(3):
            out = fus.update([2.0, 2.05, 8.0])   # sensor 2 stuck at 8 m
        self.assertEqual(out.rejected, [2])
        self.assertEqual(out.labels[2], "⊤")
        self.assertTrue(2.0 <= out.value <= 2.05)   # ParaExtrctr of the two accepted sensors

    def test_sensor_does_not_pull_its_own_reference(self):
        # I5: 3 m jump in a noisy sensor with a wide limit (2.33 m). Against the group that includes it, the
        # contradiction stays below the limit; against the ParaExtrctr of the others, it is rejected.
        fus = PAL2vFusion(0.0, 10.0, ct_lim=[0.233, 0.200, 0.233], rate_lim=[1.0] * 3)
        out = fus.update([2.0, 2.0, 5.0])
        self.assertEqual(out.rejected, [2])
        self.assertAlmostEqual(out.value, 2.0, places=9)

    def test_rejects_spike(self):
        fus = self.make()
        fus.update([2.0, 2.0, 2.0])
        out = fus.update([2.0, 2.0, 2.6])
        self.assertIn(2, out.rejected)
        self.assertAlmostEqual(out.value, 2.0, places=9)

    def test_two_sensors_in_contradiction_is_indefinite(self):
        fus = PAL2vFusion(0.0, 10.0, ct_lim=[0.05, 0.05], rate_lim=[1.0, 1.0])
        first = fus.update([2.0, 2.0])
        out = fus.update([2.0, 6.0])
        self.assertTrue(out.indefinite)
        self.assertEqual(out.certainty, 0.0)
        self.assertAlmostEqual(out.value, first.value)   # retention of the last defined output

    def test_no_mean_no_median(self):
        # ParaExtrctr of 2 values is not the arithmetic mean: (2, 3) in the UD [0, 10]
        out = PAL2vFusion(0.0, 10.0, [1, 1], [1, 1]).update([2.0, 3.0])
        self.assertNotAlmostEqual(out.value, 2.5, places=3)


class TestHybridPICEP2019(unittest.TestCase):
    """NAP 2 of the hybrid PI (Coelho et al., CEP 2019): Gct = 0 and muctr = 0.5 when PV = SP."""

    def test_nap2_equilibrium_and_signs(self):
        c = ParaconsistentHybridPI(0, 100, kp=1, ki=0, u_min=0, u_max=1)
        c.update(40.0, 40.0, 0.1)
        self.assertAlmostEqual(c.last.dct, 0.0)
        self.assertAlmostEqual(c.last.mu_ect, 0.5)
        c.update(40.0, 50.0, 0.1)          # PV > SP -> Gct > 0
        self.assertGreater(c.last.dct, 0.0)

    def test_first_order_plant_tracks_setpoint(self):
        c = ParaconsistentHybridPI(0, 2, kp=4.0, ki=4.0, u_min=-10, u_max=10)
        y, dt = 0.0, 0.01
        for _ in range(3000):
            y += dt * (-y + c.update(1.0, y, dt))
        self.assertAlmostEqual(y, 1.0, places=2)


class TestController(unittest.TestCase):
    def test_first_order_plant_tracks_setpoint(self):
        ctl = PAL2vController(e_max=2.0, td=0.5, kc=4.0, ki=1.0, u_min=-10, u_max=10)
        y, dt = 0.0, 0.01
        for _ in range(3000):
            u = ctl.update(1.0, y, dt)
            y += dt * (-y + u)
        self.assertAlmostEqual(y, 1.0, places=2)


if __name__ == "__main__":
    unittest.main()

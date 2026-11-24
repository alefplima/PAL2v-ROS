"""Nodes of experiment E4 (TurtleBot3 in Gazebo, reactive avoidance with 3 frontal distance sensors).

  range_sensors     scan_raw + front_raw (Gazebo, noiseless) -> /e4/{sonar,lidar,ir,gt} (Float64)
                    Gaussian noise and faults (on the LiDAR) are injected here, with a fixed seed, so that the
                    runs are reproducible and equal to those of E2 (S0 nominal, S1 stuck [travado], S2 spikes
                    [picos], S3 drift [deriva], S4 dropout).
  baseline_fusion   mean | median | IVW | ParaExtrctr | kf_chi2  (the PAL2v arrangement uses the lpa2v_ros/pal2v_fusion node)
  avoid_controller  fused distance -> cmd_vel (moves forward / brakes / turns left)
  e4_logger         records the run to CSV and ends the launch after `duration` (simulation time)

The three sensors measure the same quantity: the shortest free distance in the ±15° frontal sector,
measured from the center of the LDS. The sonar and the IR are emulated on top of the auxiliary `front_ranger`.
"""
from __future__ import annotations

import csv
import math
import os
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from lpa2v import para_extractor
from lpa2v.baselines import KalmanChi2
from lpa2v_msgs.msg import FusionState

RMAX = 3.5
SECTOR = math.radians(15.0)
SCENARIOS = ("S0_nominal", "S1_travado", "S2_picos", "S3_deriva", "S4_dropout", "S6_duplo")
# S6_duplo (double): two simultaneous faults in the same window = S4 on the LiDAR (dropout) + S1 on the IR (stuck)
IDX = {"sonar": 0, "lidar": 1, "ir": 2}


def sector_min(scan: LaserScan, half: float = SECTOR) -> float:
    """Shortest range in the sector [-half, half]. Below the minimum (-inf) = 0; no return (+inf) = RMAX."""
    best = RMAX
    for k, r in enumerate(scan.ranges):
        a = scan.angle_min + k * scan.angle_increment
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) > half + 1e-6 or math.isnan(r):
            continue
        if math.isinf(r):
            r = 0.0 if r < 0 else RMAX
        best = min(best, r)
    return best


def clip(v: float) -> float:
    return min(max(v, 0.0), RMAX)


def _sim_s(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


class RangeSensors(Node):
    def __init__(self):
        super().__init__("range_sensors")
        p = self.declare_parameters("", [
            ("seed", 0), ("scenario", "S0_nominal"),
            ("fault_start", 40.0), ("fault_end", 100.0),
            ("sigma", [0.05, 0.02, 0.08]),          # sonar, LiDAR, IR [m] (same as E2)
            ("fast_rate_hz", 20.0),                  # sonar and IR; the LiDAR follows the LDS (5 Hz)
            ("spike_prob", 0.15), ("drift_rate", 0.25)])
        g = {x.name: x.value for x in p}
        if g["scenario"] not in SCENARIOS:
            raise ValueError(f"invalid scenario: {g['scenario']}")
        self.g = g
        self.rng = random.Random(int(g["seed"]))
        self.gt = None
        self.stuck = None
        self.stuck_ir = None
        self.pub = {k: self.create_publisher(Float64, f"/e4/{k}", 10) for k in ("sonar", "lidar", "ir", "gt")}
        self.create_subscription(LaserScan, "/front_raw", self._front, 10)
        self.create_subscription(LaserScan, "/scan_raw", self._lidar, 10)
        self.create_timer(1.0 / g["fast_rate_hz"], self._fast)

    def _front(self, m: LaserScan):
        self.gt = sector_min(m)
        self.pub["gt"].publish(Float64(data=self.gt))

    def _fast(self):
        if self.gt is None:
            return
        s = self.g["sigma"]
        self.pub["sonar"].publish(Float64(data=clip(self.gt + self.rng.gauss(0, s[0]))))
        ir = self.gt + self.rng.gauss(0, s[2])
        if self.g["scenario"] == "S6_duplo" and self.g["fault_start"] <= _sim_s(self) < self.g["fault_end"]:
            self.stuck_ir = ir if self.stuck_ir is None else self.stuck_ir
            ir = self.stuck_ir
        self.pub["ir"].publish(Float64(data=clip(ir)))

    def _lidar(self, m: LaserScan):
        x = sector_min(m) + self.rng.gauss(0, self.g["sigma"][1])
        t = _sim_s(self)
        sc, t0 = self.g["scenario"], self.g["fault_start"]
        if t0 <= t < self.g["fault_end"]:
            if sc == "S1_travado":
                self.stuck = x if self.stuck is None else self.stuck
                x = self.stuck
            elif sc == "S2_picos" and self.rng.random() < self.g["spike_prob"]:
                x += self.rng.choice((-1, 1)) * self.rng.uniform(0.5, 1.5)
            elif sc == "S3_deriva":
                x += self.g["drift_rate"] * (t - t0)
            elif sc in ("S4_dropout", "S6_duplo"):
                x = RMAX
        self.pub["lidar"].publish(Float64(data=clip(x)))


class BaselineFusion(Node):
    """Comparison methods, with the same interface as pal2v_fusion (Float64 topics -> `fused`).
    kf_chi2 also publishes `fusion_state` (value + rejected only) to measure detection as for PAL2v."""

    def __init__(self):
        super().__init__("baseline_fusion")
        p = self.declare_parameters("", [
            ("method", "median"), ("input_topics", ["/e4/sonar", "/e4/lidar", "/e4/ir"]),
            ("sigma", [0.05, 0.02, 0.08]), ("rate_hz", 20.0), ("vmin", 0.0), ("vmax", RMAX),
            ("kf_q", 1.0), ("kf_gate", 6.635), ("kf_n_reset", 3)])
        g = {x.name: x.value for x in p}
        self.method = g["method"]
        vmin, vmax = g["vmin"], g["vmax"]
        w = [1.0 / s ** 2 for s in g["sigma"]]
        self.kf = None
        self.pub_state = None
        if self.method == "kf_chi2":
            self.kf = KalmanChi2(g["sigma"], q=g["kf_q"], dt=1.0 / g["rate_hz"], gate=g["kf_gate"],
                                 n_reset=g["kf_n_reset"])
            self.pub_state = self.create_publisher(FusionState, "fusion_state", 10)
        self.f = {
            "kf_chi2": lambda x: self.kf.update(x),
            "mean": lambda x: sum(x) / len(x),
            "median": lambda x: sorted(x)[len(x) // 2] if len(x) % 2 else
            0.5 * (sorted(x)[len(x) // 2 - 1] + sorted(x)[len(x) // 2]),
            "ivw": lambda x: sum(wi * xi for wi, xi in zip(w, x)) / sum(w),
            "paraextrctr": lambda x: para_extractor(x, vmin, vmax),
        }[self.method]
        self.latest = [None] * len(g["input_topics"])
        for i, t in enumerate(g["input_topics"]):
            self.create_subscription(Float64, t, lambda m, i=i: self.latest.__setitem__(i, m.data), 10)
        self.pub = self.create_publisher(Float64, "fused", 10)
        self.create_timer(1.0 / g["rate_hz"], self._tick)

    def _tick(self):
        if None not in self.latest:
            v = float(self.f(self.latest))
            self.pub.publish(Float64(data=v))
            if self.pub_state is not None:
                self.pub_state.publish(FusionState(value=v, rejected=self.kf.rejected))


class AvoidController(Node):
    """Reactive avoidance: moves forward with speed proportional to the clearance; below d_turn it turns left in
    place until the front is clear (d > d_clear). Uses only the fused distance."""

    def __init__(self):
        super().__init__("avoid_controller")
        p = self.declare_parameters("", [
            ("v_max", 0.20), ("w_turn", 0.8), ("d_stop", 0.30), ("d_slow", 0.80),
            ("d_turn", 0.40), ("d_clear", 0.80), ("rate_hz", 20.0)])
        self.g = {x.name: x.value for x in p}
        self.d = None
        self.turning = False
        self.create_subscription(Float64, "/e4/fused", lambda m: setattr(self, "d", m.data), 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / self.g["rate_hz"], self._tick)

    def _tick(self):
        g, cmd = self.g, Twist()
        if self.d is not None:
            if self.turning and self.d > g["d_clear"]:
                self.turning = False
            elif not self.turning and self.d < g["d_turn"]:
                self.turning = True
            if self.turning:
                cmd.angular.z = g["w_turn"]
            else:
                s = (self.d - g["d_stop"]) / (g["d_slow"] - g["d_stop"])
                cmd.linear.x = g["v_max"] * min(max(s, 0.0), 1.0)
        self.pub.publish(cmd)


class E4Logger(Node):
    COLS = ("t", "gt", "sonar", "lidar", "ir", "fused", "rejected", "contact", "x", "y", "v", "w")

    def __init__(self):
        super().__init__("e4_logger")
        p = self.declare_parameters("", [("out", "/tmp/e4_run.csv"), ("duration", 120.0),
                                         ("rate_hz", 20.0)])
        g = {x.name: x.value for x in p}
        self.out, self.duration = g["out"], g["duration"]
        self.v = {k: None for k in ("gt", "sonar", "lidar", "ir", "fused")}
        self.rej = ""
        self.contact = False
        self.odom = (math.nan, math.nan, 0.0, 0.0)
        self.rows = []
        self.done = False
        for k in self.v:
            self.create_subscription(Float64, f"/e4/{k}", lambda m, k=k: self.v.__setitem__(k, m.data), 10)
        self.create_subscription(FusionState, "/e4/fusion_state",
                                 lambda m: setattr(self, "rej", ";".join(map(str, m.rejected))), 10)
        self.create_subscription(Contacts, "/bumper", self._bump, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(TFMessage, "/true_pose", self._pose, 10)
        self.create_timer(1.0 / g["rate_hz"], self._tick)

    def _bump(self, m: Contacts):
        for c in m.contacts:
            names = c.collision1.name + c.collision2.name
            if "ground" not in names:
                self.contact = True

    def _odom(self, m: Odometry):
        # only the commanded/measured speed comes from odometry; the position comes from the Gazebo true pose
        # (wheel odometry keeps integrating when the robot pushes an obstacle and slips)
        tw = m.twist.twist
        self.odom = (self.odom[0], self.odom[1], tw.linear.x, tw.angular.z)

    def _pose(self, m: TFMessage):
        # The ros_gz_bridge (Rolling) delivers Pose_V -> TFMessage with empty names; the model always comes
        # before its links, so, without a name, the 1st transform is the robot's (the only dynamic model).
        tfs = [tf for tf in m.transforms if tf.child_frame_id == "tb3"] or m.transforms[:1]
        for tf in tfs:
            p = tf.transform.translation
            self.odom = (p.x, p.y, self.odom[2], self.odom[3])

    def _tick(self):
        if self.done or None in self.v.values():
            return
        t = _sim_s(self)
        self.rows.append((round(t, 3), *(self.v[k] for k in ("gt", "sonar", "lidar", "ir", "fused")),
                          self.rej, int(self.contact), *self.odom))
        self.contact = False
        if t >= self.duration:
            self.done = True
            os.makedirs(os.path.dirname(os.path.abspath(self.out)), exist_ok=True)
            tmp = self.out + ".tmp"             # atomic write: whoever waits for the file never sees it half-written
            with open(tmp, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(self.COLS)
                w.writerows(self.rows)
            os.replace(tmp, self.out)
            self.get_logger().info(f"{len(self.rows)} amostras -> {self.out}")
            raise SystemExit(0)


def _run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def range_sensors_main():
    _run(RangeSensors)


def baseline_fusion_main():
    _run(BaselineFusion)


def avoid_controller_main():
    _run(AvoidController)


def logger_main():
    _run(E4Logger)

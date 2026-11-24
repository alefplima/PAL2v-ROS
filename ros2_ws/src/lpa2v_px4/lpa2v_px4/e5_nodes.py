"""Nodes of experiment E5: PX4 SITL (x500 with downward LiDAR) in Gazebo, altitude by PAL2v fusion.

  alt_sensors   PX4 (uXRCE-DDS) -> /e5/{baro,lidar,gps,gt,ekf} (Float64, height relative to takeoff)
                Calibrates the zero of each sensor on the ground and injects the faults (fixed seeds/windows).
  offboard_alt  outer altitude loop: reference 0 -> 2 -> 5 -> 3 m, vertical velocity command
                to the PX4 (offboard mode) computed ONLY from the fused altitude /e5/fused; x, y held at 0.
  e5_logger     records the flight to CSV and ends after `duration` (s after the start of the flight).

The fusion is the lpa2v_ros/pal2v_fusion node (PAL2v) or lpa2v_sim/baseline_fusion (mean, median, IVW,
ParaExtrctr, kf_chi2), on the topics /e5/{baro,lidar,gps}. The PX4 EKF2 keeps using the sensors WITHOUT the
injected faults (the `failure` command is not implemented in the PX4 Gazebo); it appears only as a reference.
"""
from __future__ import annotations

import csv
import math
import os
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool, Float64, String

from lpa2v_msgs.msg import FusionState
from px4_msgs.msg import (DistanceSensor, OffboardControlMode, SensorGps, TrajectorySetpoint,
                          VehicleAirData, VehicleAttitude, VehicleCommand, VehicleLocalPosition,
                          VehicleStatus)

QOS_PX4_IN = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
SCENARIOS = ("S0_nominal", "S1_travado", "S3_deriva", "S4_dropout", "S5_salto_gps", "S6_duplo")
# S6_duplo (double): two simultaneous faults in the LiDAR window = S4 (LiDAR dropout) + S5 (GPS jump)
NAN = float("nan")
LIDAR_DROPOUT = 8.0          # LiDAR dropout reading (same as E3)


def _status_topics(node, cb):
    # PX4 >= 1.16 publishes versioned messages with suffix _vN
    for t in ("/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"):
        node.create_subscription(VehicleStatus, t, cb, qos_profile_sensor_data)


class AltSensors(Node):
    """Height relative to takeoff from baro, LiDAR (tilt-corrected) and GPS."""

    def __init__(self):
        super().__init__("alt_sensors")
        p = self.declare_parameters("", [
            ("seed", 0), ("scenario", "S0_nominal"), ("calib_s", 3.0),
            ("rate_hz", 50.0), ("drift_rate", 0.25), ("gps_jump", 3.0),
            ("fault_lidar", [8.0, 18.0]), ("fault_gps", [25.0, 35.0])])
        self.g = {x.name: x.value for x in p}
        if self.g["scenario"] not in SCENARIOS:
            raise ValueError(f"invalid scenario: {self.g['scenario']}")
        self.rng = random.Random(int(self.g["seed"]))
        self.raw = {"baro": None, "lidar": None, "gps": None, "gt": None, "ekf": None}
        self.tilt_cos = 1.0
        self.calib = {k: [] for k in self.raw}
        self.zero = None
        self.t_flight = None            # (monotonic) instant of the start of the flight, from offboard_alt
        self.stuck = None
        self.pub = {k: self.create_publisher(Float64, f"/e5/{k}", 10) for k in self.raw}
        self.pub_ready = self.create_publisher(Bool, "/e5/calibrated", 10)
        q = qos_profile_sensor_data
        self.create_subscription(VehicleAirData, "/fmu/out/vehicle_air_data",
                                 lambda m: self._set("baro", m.baro_alt_meter), q)
        self.create_subscription(DistanceSensor, "/fmu/out/distance_sensor",
                                 lambda m: self._set("lidar", m.current_distance), q)
        self.create_subscription(SensorGps, "/fmu/out/vehicle_gps_position",
                                 lambda m: self._set("gps", m.altitude_msl_m), q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_groundtruth",
                                 lambda m: self._set("gt", -m.z), q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 lambda m: self._set("ekf", -m.z), q)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._att, q)
        self.create_subscription(Float64, "/e5/flight_t0", self._t0, 10)
        self.t_start = time.monotonic()
        self.create_timer(1.0 / self.g["rate_hz"], self._tick)

    def _set(self, k, v):
        self.raw[k] = float(v)

    def _att(self, m: VehicleAttitude):
        w, x, y, z = m.q
        # cos(angle between the body z axis and the vertical) = 1 - 2(x² + y²)
        self.tilt_cos = max(0.5, 1.0 - 2.0 * (x * x + y * y))

    def _t0(self, m: Float64):
        if self.t_flight is None:
            self.t_flight = m.data

    def _tick(self):
        if None in self.raw.values():
            return
        if self.zero is None:
            for k, v in self.raw.items():
                self.calib[k].append(v * (self.tilt_cos if k == "lidar" else 1.0))
            if time.monotonic() - self.t_start >= self.g["calib_s"] and len(self.calib["baro"]) > 20:
                self.zero = {k: sum(v) / len(v) for k, v in self.calib.items()}
                self.get_logger().info(f"zeros: {self.zero}")
            return
        self.pub_ready.publish(Bool(data=True))
        h = {k: self.raw[k] * (self.tilt_cos if k == "lidar" else 1.0) - self.zero[k] for k in self.raw}
        t = time.monotonic() - self.t_flight if self.t_flight is not None else -1.0
        sc = self.g["scenario"]
        a, b = self.g["fault_lidar"]
        if a <= t < b:
            if sc == "S1_travado":
                self.stuck = h["lidar"] if self.stuck is None else self.stuck
                h["lidar"] = self.stuck
            elif sc == "S3_deriva":
                h["lidar"] += self.g["drift_rate"] * (t - a)
            elif sc in ("S4_dropout", "S6_duplo"):
                h["lidar"] = LIDAR_DROPOUT
            if sc == "S6_duplo":
                h["gps"] += self.g["gps_jump"]
        a, b = self.g["fault_gps"]
        if sc == "S5_salto_gps" and a <= t < b:
            h["gps"] += self.g["gps_jump"]
        for k, v in h.items():
            self.pub[k].publish(Float64(data=v))


class OffboardAlt(Node):
    """Arms, enters offboard and follows the altitude reference with a P loop on the fused altitude."""

    def __init__(self):
        super().__init__("offboard_alt")
        p = self.declare_parameters("", [
            ("kp", 0.8), ("vz_up_max", 1.5), ("vz_down_max", 1.0), ("rate_hz", 20.0),
            ("profile_t", [0.0, 20.0, 40.0]), ("profile_z", [2.0, 5.0, 3.0])])
        self.g = {x.name: x.value for x in p}
        self.z = None
        self.calibrated = False
        self.status = None
        self.t0 = None
        self.k = 0
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PX4_IN)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_PX4_IN)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QOS_PX4_IN)
        self.pub_ref = self.create_publisher(Float64, "/e5/ref", 10)
        self.pub_t0 = self.create_publisher(Float64, "/e5/flight_t0", 10)
        self.pub_phase = self.create_publisher(String, "/e5/phase", 10)
        self.create_subscription(Float64, "/e5/fused", lambda m: setattr(self, "z", m.data), 10)
        self.create_subscription(Bool, "/e5/calibrated", lambda m: setattr(self, "calibrated", m.data), 10)
        _status_topics(self, lambda m: setattr(self, "status", m))
        self.create_timer(1.0 / self.g["rate_hz"], self._tick)

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self._now_us()
        m.command, m.param1, m.param2 = command, float(p1), float(p2)
        m.target_system, m.target_component, m.source_system, m.source_component = 1, 1, 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _ref(self, t):
        z = 0.0
        for ti, zi in zip(self.g["profile_t"], self.g["profile_z"]):
            if t >= ti:
                z = zi
        return z

    def _tick(self):
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.position, mode.velocity = True, True
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [0.0, 0.0, NAN]
        sp.velocity = [NAN, NAN, 0.0]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = NAN
        st = self.status
        flying = st is not None and st.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
            and st.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.calibrated and self.z is not None:
            if not flying:
                self.k += 1
                if self.k % 20 == 0:              # 1 s of setpoints before; repeats every 1 s
                    self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.pub_phase.publish(String(data="armando"))
            else:
                if self.t0 is None:
                    self.t0 = time.monotonic()
                    self.get_logger().info("offboard + armed: flight start")
                self.pub_t0.publish(Float64(data=self.t0))
                t = time.monotonic() - self.t0
                r = self._ref(t)
                vz_up = min(max(self.g["kp"] * (r - self.z), -self.g["vz_down_max"]), self.g["vz_up_max"])
                sp.velocity = [NAN, NAN, -vz_up]           # NED: z down
                self.pub_ref.publish(Float64(data=r))
                self.pub_phase.publish(String(data="voo"))
        self.pub_sp.publish(sp)


class E5Logger(Node):
    COLS = ("t", "ref", "gt", "baro", "lidar", "gps", "fused", "ekf", "rejected", "armed", "nav_state")

    def __init__(self):
        super().__init__("e5_logger")
        p = self.declare_parameters("", [("out", "/tmp/e5_run.csv"), ("duration", 60.0),
                                         ("rate_hz", 20.0), ("timeout", 120.0)])
        g = {x.name: x.value for x in p}
        self.out, self.duration, self.timeout = g["out"], g["duration"], g["timeout"]
        self.v = {k: None for k in ("ref", "gt", "baro", "lidar", "gps", "fused", "ekf")}
        self.rej = ""
        self.t0 = None
        self.status = None
        self.rows = []
        self.done = False
        self.t_boot = time.monotonic()
        for k in self.v:
            self.create_subscription(Float64, f"/e5/{k}", lambda m, k=k: self.v.__setitem__(k, m.data), 10)
        self.create_subscription(FusionState, "/e5/fusion_state",
                                 lambda m: setattr(self, "rej", ";".join(map(str, m.rejected))), 10)
        self.create_subscription(Float64, "/e5/flight_t0", lambda m: setattr(self, "t0", m.data), 10)
        _status_topics(self, lambda m: setattr(self, "status", m))
        self.create_timer(1.0 / g["rate_hz"], self._tick)

    def _write(self, ok):
        os.makedirs(os.path.dirname(os.path.abspath(self.out)), exist_ok=True)
        tmp = self.out + ".tmp"
        with open(tmp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            w.writerows(self.rows)
        os.replace(tmp, self.out if ok else self.out + ".falhou")
        self.get_logger().info(f"{len(self.rows)} amostras -> {self.out}{'' if ok else '.falhou'}")
        raise SystemExit(0)

    def _tick(self):
        if self.done:
            return
        if self.t0 is None:
            if time.monotonic() - self.t_boot > self.timeout:
                self.done = True
                self._write(False)
            return
        if None in self.v.values():
            return
        t = time.monotonic() - self.t0
        st = self.status
        self.rows.append((round(t, 3), *(self.v[k] for k in ("ref", "gt", "baro", "lidar", "gps", "fused", "ekf")),
                          self.rej, int(st is not None and st.arming_state == VehicleStatus.ARMING_STATE_ARMED),
                          st.nav_state if st is not None else -1))
        if t >= self.duration:
            self.done = True
            self._write(True)


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


def alt_sensors_main():
    _run(AltSensors)


def offboard_alt_main():
    _run(OffboardAlt)


def logger_main():
    _run(E5Logger)

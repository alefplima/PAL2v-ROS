"""1-DOF altitude simulator (same plant and sensors as experiment E3) to test the nodes without Gazebo/PX4.

Publishes /baro_alt, /lidar_alt, /gps_alt (Float64), /alt_true and /alt_setpoint; subscribes to /thrust_cmd.
Parameter `fault` (bool): injects the LiDAR dropout (reads 8 m) in [10, 12] s and the +3 m GPS jump in
[20, 21] s, as in scenario F of E3. Reference: 0 -> 2 m (1 s) -> 5 m (8 s) -> 3 m (14 s).
"""
from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

M, G, B, TAU_M = 1.5, 9.81, 0.4, 0.05
SIGMA = (0.30, 0.03, 0.50)


class AltitudeSim(Node):
    def __init__(self):
        super().__init__("altitude_sim")
        p = self.declare_parameters("", [("rate_hz", 100.0), ("fault", False), ("seed", 0)])
        g = {x.name: x.value for x in p}
        self.dt = 1.0 / g["rate_hz"]
        self.fault = g["fault"]
        self.rng = random.Random(g["seed"])
        self.t = self.z = self.zd = self.w = self.baro_bias = 0.0
        self.thrust = self.u = M * G
        self.k = 0
        self.gps_hold = 0.0
        self.create_subscription(Float64, "/thrust_cmd", lambda m: setattr(self, "u", m.data), 10)
        self.pubs = {n: self.create_publisher(Float64, n, 10)
                     for n in ("/baro_alt", "/lidar_alt", "/gps_alt", "/alt_true", "/alt_setpoint")}
        self.create_timer(self.dt, self.tick)

    def ref(self):
        t = self.t
        return 0.0 if t < 1 else 2.0 if t < 8 else 5.0 if t < 14 else 3.0

    def tick(self):
        dt, n = self.dt, self.rng.gauss
        # OU disturbance + +4 N gust in [17, 18] s
        self.w += -dt * self.w + 0.6 * math.sqrt(2 * dt) * n(0, 1)
        gust = 4.0 if 17.0 <= self.t < 18.0 else 0.0
        self.thrust += dt / TAU_M * (min(max(self.u, 0.0), 30.0) - self.thrust)
        zdd = (self.thrust - M * G - B * self.zd + self.w + gust) / M
        self.zd += dt * zdd
        self.z += dt * self.zd
        if self.z < 0:
            self.z, self.zd = 0.0, max(self.zd, 0.0)
        self.baro_bias += n(0, 0.02 * math.sqrt(dt))
        baro = self.z + self.baro_bias + n(0, SIGMA[0])
        lidar = min(max(self.z + n(0, SIGMA[1]), 0.0), 8.0)
        if self.k % 10 == 0:
            self.gps_hold = self.z + n(0, SIGMA[2])
        gps = self.gps_hold
        if self.fault and 10.0 <= self.t < 12.0:
            lidar = 8.0
        if self.fault and 20.0 <= self.t < 21.0:
            gps += 3.0
        for name, v in (("/baro_alt", baro), ("/lidar_alt", lidar), ("/gps_alt", gps),
                        ("/alt_true", self.z), ("/alt_setpoint", self.ref())):
            self.pubs[name].publish(Float64(data=float(v)))
        self.t += dt
        self.k += 1


def main():
    rclpy.init()
    node = AltitudeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

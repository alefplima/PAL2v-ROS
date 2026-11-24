"""E5 - Gazebo screenshots (PX4 SITL, x500) with the trajectory drawn in the scene (visual evidence).

Usage (system python3, ROS 2 + ~/px4_ros_ws/install + ros2_ws/install sourced):
  python3 e5_screenshots.py --method mean --scenario S4_dropout --times 6 13 25 --wide 38

During the flight, draws in Gazebo itself (visual markers, they do not affect the physics):
  - true 3D trail of the drone (simulator ground truth): spheres in the method color, yellow during the fault;
  - "wall" behind the drone with the height x time curve of the flight, the reference (dashed gray) and the
    fault-window band, so that each capture also shows the history.
Camera: follows the drone closely (side view, floor always in frame) at the --times instants; at the --wide
instant it takes a wide shot with the whole trajectory and the curve.
Instants counted from /e5/flight_t0 (the same clock as the fault injection).
Output: results/evidencias/e5_gz_<method>_<scenario>_t<tt>.png (+ _amplo = wide), the CSV and the PX4 log of the flight.
The log line "capture: <png> (t_flight=...)" is parsed by evidence_panels.py; "simulator artifact: no" by
run_prints_e5_retry.sh.
"""
import argparse
import glob
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from PIL import ImageGrab
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "evidencias")
PART = "e5shot"
os.environ["GZ_PARTITION"] = PART            # before importing/creating the gz-transport node

_stderr = os.dup(2)
os.dup2(os.open(os.devnull, os.O_WRONLY), 2)   # silences protobuf warnings (gz-msgs10 x 11) on import
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.empty_pb2 import Empty  # noqa: E402
from gz.msgs10.gui_camera_pb2 import GUICamera  # noqa: E402
from gz.msgs10.marker_pb2 import Marker  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
os.dup2(_stderr, 2)

COLOR = {"pal2v": (0.11, 0.50, 0.36), "mean": (0.75, 0.22, 0.17), "median": (0.49, 0.24, 0.60),
         "ivw": (0.84, 0.54, 0.06), "paraextrctr": (0.18, 0.53, 0.76), "kf_chi2": (0.63, 0.32, 0.18)}
FAULT_RGB = (0.95, 0.77, 0.06)
WIN = {"S0_nominal": None, "S1_travado": (8, 18), "S3_deriva": (8, 18), "S4_dropout": (8, 18),
       "S5_salto_gps": (25, 35), "S6_duplo": (8, 18)}
FAULT_TXT = {"S1_travado": "fault: stuck LiDAR", "S3_deriva": "fault: LiDAR drift",
             "S4_dropout": "fault: LiDAR dropout", "S5_salto_gps": "fault: GPS jump",
             "S6_duplo": "fault: LiDAR + GPS"}
PROFILE = [(0, 2.0), (20, 5.0), (40, 3.0)]      # height reference (same as offboard_alt)
CX, CY0, KT = 1.2, -0.8, -0.11                  # wall: fixed x, y = CY0 + KT * t (camera looks along +x: -y is on the right)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Scene:
    def __init__(self, method, scenario, duration):
        self.gz = GzNode()
        self.rgb = COLOR.get(method, (0.2, 0.2, 0.2))
        self.win = WIN[scenario]
        self.scenario = scenario
        self.duration = duration
        self.n = 0
        self.curve = []

    def _send(self, m):
        self.gz.request("/marker", m, Marker, Empty, 50)   # the MarkerManager does not reply: do not wait
        return True

    def _marker(self, ns, mid, typ, rgb, alpha=1.0, scale=(1, 1, 1), pos=(0, 0, 0), pts=(), text=""):
        m = Marker()
        m.action, m.ns, m.id, m.type = Marker.ADD_MODIFY, ns, mid, typ
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.scale.x, m.scale.y, m.scale.z = scale
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = (*rgb, alpha)
        for p in pts:
            q = m.point.add()
            q.x, q.y, q.z = p
        if text:
            m.text = text
        return self._send(m)

    def static(self):
        """Wall: axes, reference and fault band; drawn once at the start."""
        ok = self._marker("eixo", 1, Marker.LINE_STRIP, (0.25, 0.25, 0.25),
                          pts=[(CX, CY0, 6.0), (CX, CY0, 0.02), (CX, CY0 + KT * self.duration, 0.02)])
        ref = []
        for (t1, z1), (t2, _) in zip(PROFILE, PROFILE[1:] + [(self.duration, None)]):
            ref += [(CX, CY0 + KT * t1, z1), (CX, CY0 + KT * t2, z1)]
        self._marker("referencia", 1, Marker.LINE_STRIP, (0.45, 0.45, 0.45), pts=ref)
        for z in (1, 2, 3, 4, 5):            # 1 m ticks on the vertical axis
            self._marker("marca", z, Marker.LINE_LIST, (0.25, 0.25, 0.25),
                         pts=[(CX, CY0 - 0.08, z), (CX, CY0 + 0.08, z)])
        if self.win:
            a, b = self.win
            self._marker("falha", 1, Marker.BOX, FAULT_RGB, alpha=0.35,
                         scale=(0.02, abs(KT) * (b - a), 6.0), pos=(CX + 0.02, CY0 + KT * (a + b) / 2, 3.0))
        return ok

    def add(self, t, x, y, z):
        in_fault = self.win is not None and self.win[0] <= t < self.win[1]
        rgb = FAULT_RGB if in_fault else self.rgb
        self.n += 1
        self._marker("rastro", self.n, Marker.SPHERE, rgb, scale=(0.07, 0.07, 0.07), pos=(x, y, z))
        self.curve.append((CX - 0.01, CY0 + KT * t, max(z, 0.02)))
        if len(self.curve) >= 2:
            self._marker("curva", 1, Marker.LINE_STRIP, self.rgb, pts=self.curve)
        self._marker("agora", 1, Marker.SPHERE, rgb, scale=(0.12, 0.12, 0.12), pos=self.curve[-1])

    def camera(self, pos, pitch, yaw=0.0):
        c = GUICamera()
        c.pose.position.x, c.pose.position.y, c.pose.position.z = pos
        qx, qy, qz, qw = quat(0.0, pitch, yaw)
        c.pose.orientation.x, c.pose.orientation.y, c.pose.orientation.z, c.pose.orientation.w = qx, qy, qz, qw
        ok, _ = self.gz.request("/gui/move_to/pose", c, GUICamera, Boolean, 2000)
        return ok

    def follow(self, h):
        """Close side view: camera 1.9 m to the side, looking slightly below the drone (floor in frame)."""
        d, up, look = 1.9, 0.45, -0.05
        return self.camera((-d, 0.6, h + up), math.atan2(up + look, d), yaw=-0.18)

    def wide(self):
        mid = (CY0 + KT * self.duration) / 2           # between the drone (y = 0) and the end of the curve
        return self.camera((-8.5, mid, 3.0), 0.06)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="mean")
    ap.add_argument("--scenario", default="S4_dropout")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--times", type=float, nargs="+", default=(7.0, 13.0, 25.0))
    ap.add_argument("--wide", type=float, default=38.0)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    tag = f"{a.method}_{a.scenario}"
    csv = os.path.join(OUT, f"e5_gz_{tag}.csv")
    for f in glob.glob(csv) + glob.glob(os.path.join(OUT, f"e5_gz_{tag}_t*.png")):
        os.remove(f)                            # everything in this capture must come from the same flight
    logdir = os.path.join(OUT, f"e5_gz_{tag}_log")
    if os.path.isdir(logdir):                    # log of this flight only (the artifact check depends on it)
        import shutil
        shutil.rmtree(logdir)
    env = dict(os.environ, ROS_DOMAIN_ID="0", ROS_LOG_DIR=logdir)
    cmd = ["ros2", "launch", "lpa2v_px4", "e5.launch.py", f"method:={a.method}", f"scenario:={a.scenario}",
           f"seed:={a.seed}", f"duration:={a.duration}", f"out:={csv}", "gui:=true",
           "auto_shutdown:=false", f"partition:={PART}"]
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    os.environ["ROS_DOMAIN_ID"] = "0"
    rclpy.init()
    node = rclpy.create_node("e5_screenshots")
    st = {"t0": None, "pos": None}
    node.create_subscription(Float64, "/e5/flight_t0", lambda m: st.__setitem__("t0", st["t0"] or m.data), 10)
    # simulator ground truth (PX4 NED) -> Gazebo world (ENU): x = east = y_ned, y = north = x_ned, z = -z_ned
    node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_groundtruth",
                             lambda m: st.__setitem__("pos", (m.y, m.x, -m.z)), qos_profile_sensor_data)
    scene = Scene(a.method, a.scenario, a.duration)
    shots = sorted([(t, f"t{int(t):02d}") for t in a.times] + [(a.wide, "amplo")])
    t_start, drawn, last_pt, last_cam = time.time(), False, -1.0, -1.0
    deadline = t_start + a.duration + 240
    try:
        while shots and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if not drawn and time.time() - t_start > 25:              # the Gazebo window is already open
                drawn = scene.follow(0.0)              # the camera responds: the GUI is ready
                if drawn:
                    time.sleep(1.0)
                    scene.static()
                print("scene drawn" if drawn else "waiting for Gazebo", flush=True)
                if not drawn:
                    t_start += 5
            if st["t0"] is None or st["pos"] is None or not drawn:
                continue
            t = time.monotonic() - st["t0"]
            if t - last_pt >= 0.25:
                scene.add(t, *st["pos"])
                last_pt = t
            nxt_t, name = shots[0]
            if name != "amplo" and t - last_cam >= 0.5 and t >= nxt_t - 4:   # follows the drone before the capture
                scene.follow(st["pos"][2])
                last_cam = t
            if name == "amplo" and t >= nxt_t - 1.5 and last_cam < nxt_t - 1.5:
                scene.wide()
                last_cam = t
            if t >= nxt_t:
                time.sleep(0.4)                                        # camera settled
                p = os.path.join(OUT, f"e5_gz_{tag}_{name}.png")
                ImageGrab.grab().save(p)
                shots.pop(0)
                print("capture:", p, f"(t_flight={t:.1f} s, height {st['pos'][2]:.2f} m)", flush=True)
        while not os.path.exists(csv) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if st["t0"] is not None and st["pos"] is not None and drawn:
                t = time.monotonic() - st["t0"]
                if t - last_pt >= 0.25:
                    scene.add(t, *st["pos"])
                    last_pt = t
    finally:
        os.killpg(proc.pid, signal.SIGKILL)
        for pat in ("px4_sitl_default/bin/px4 -d", "MicroXRCEAgent udp4 -p 8888", "gz sim --verbose=1",
                    "lpa2v_px4/lib/lpa2v_px4", "__node:=e5_fusion"):
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        for pid in subprocess.run(["pgrep", "-fx", "gz sim -g"], capture_output=True, text=True).stdout.split():
            try:                                  # GUI of this partition (the pkill above does not catch it)
                if f"GZ_PARTITION={PART}" in open(f"/proc/{pid}/environ").read().split("\0"):
                    os.kill(int(pid), signal.SIGKILL)
            except (OSError, ValueError):
                pass
        node.destroy_node()
        rclpy.try_shutdown()
    pat = "Accel #0 fail|Gyro #0 fail|Baro #0 fail|Mag #0 fail|Battery unhealthy"
    bad = subprocess.run(["grep", "-rhoE", pat, logdir], capture_output=True, text=True).stdout
    art = bool(bad.strip())
    if art:
        print("artifacts:", sorted(set(bad.split("\n")) - {""}))
    print("CSV:", csv if os.path.exists(csv) else "not generated", "| simulator artifact:", "YES" if art else "no")
    sys.exit(0)


if __name__ == "__main__":
    main()

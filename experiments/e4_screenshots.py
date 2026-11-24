"""E4 - Gazebo screenshots (TurtleBot3) with a close camera and the trajectory drawn in the scene.

Usage (system python3, ROS 2 + ros2_ws/install sourced, no micromamba):
  python3 e4_screenshots.py --method mean --scenario S4_dropout --times 12 26 40 --wide 48

Draws in Gazebo itself (visual markers, no effect on the physics):
  - robot trail on the floor (simulator true position): spheres in the method color, yellow during the fault;
  - "ray" in front of the robot with the length of the FUSED distance (the one the controller uses), in the
    method color, and a thin gray ray with the TRUE distance to the obstacle: when the two diverge, the fusion
    is misleading the controller.
Camera: behind and above the robot, at ~0.8 m (identification of the robot and of the obstacle ahead) at the
--times instants; at --wide, top view of the arena with the full trajectory. Time = simulation time (/clock).
Output: results/evidencias/e4_gz_<method>_<scenario>_t<tt>.png (+ _amplo = wide) and the CSV of the same run.
The log line "capture: <png> (t_sim=...)" is parsed by evidence_panels.py.
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
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "evidencias")
PART = "e4shot"
DOMAIN = "82"
os.environ["GZ_PARTITION"] = PART

_stderr = os.dup(2)
os.dup2(os.open(os.devnull, os.O_WRONLY), 2)
from gz.msgs.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs.empty_pb2 import Empty  # noqa: E402
from gz.msgs.gui_camera_pb2 import GUICamera  # noqa: E402
from gz.msgs.marker_pb2 import Marker  # noqa: E402
from gz.transport import Node as GzNode  # noqa: E402
os.dup2(_stderr, 2)

COLOR = {"pal2v": (0.11, 0.50, 0.36), "mean": (0.75, 0.22, 0.17), "median": (0.49, 0.24, 0.60),
         "ivw": (0.84, 0.54, 0.06), "paraextrctr": (0.18, 0.53, 0.76), "kf_chi2": (0.63, 0.32, 0.18)}
FAULT_RGB = (0.95, 0.77, 0.06)


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Scene:
    def __init__(self, method, fault):
        self.gz = GzNode()
        self.rgb = COLOR.get(method, (0.2, 0.2, 0.2))
        self.fault = fault
        self.n = 0

    def _marker(self, ns, mid, typ, rgb, alpha=1.0, scale=(1, 1, 1), pos=(0, 0, 0), pts=()):
        m = Marker()
        m.action, m.ns, m.id, m.type = Marker.ADD_MODIFY, ns, mid, typ
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.scale.x, m.scale.y, m.scale.z = scale
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = (*rgb, alpha)
        for p in pts:
            q = m.point.add()
            q.x, q.y, q.z = p
        self.gz.request("/marker", m, Marker, Empty, 50)     # the MarkerManager does not reply: do not wait

    def add(self, t, x, y, yaw, fused, real):
        in_fault = self.fault[0] <= t < self.fault[1]
        rgb = FAULT_RGB if in_fault else self.rgb
        self.n += 1
        self._marker("rastro", self.n, Marker.SPHERE, rgb, scale=(0.035, 0.035, 0.035), pos=(x, y, 0.02))
        # rays leaving the front of the robot (TB3 burger radius ~0.1 m), at LiDAR height
        x0, y0, z = x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw), 0.18
        if fused is not None:
            f = max(0.0, min(fused, 3.5))
            self._marker("raio_fundido", 1, Marker.LINE_LIST, self.rgb,
                         pts=[(x0, y0, z), (x0 + f * math.cos(yaw), y0 + f * math.sin(yaw), z)])
            self._marker("fim_fundido", 1, Marker.SPHERE, self.rgb, scale=(0.09, 0.09, 0.09),
                         pos=(x0 + f * math.cos(yaw), y0 + f * math.sin(yaw), z))
        if real is not None:
            r = max(0.0, min(real, 3.5))
            self._marker("raio_real", 1, Marker.LINE_LIST, (0.35, 0.35, 0.35),
                         pts=[(x0, y0, z + 0.04), (x0 + r * math.cos(yaw), y0 + r * math.sin(yaw), z + 0.04)])

    def camera(self, pos, pitch, yaw):
        c = GUICamera()
        c.pose.position.x, c.pose.position.y, c.pose.position.z = pos
        qx, qy, qz, qw = quat(0.0, pitch, yaw)
        c.pose.orientation.x, c.pose.orientation.y, c.pose.orientation.z, c.pose.orientation.w = qx, qy, qz, qw
        ok, _ = self.gz.request("/gui/move_to/pose", c, GUICamera, Boolean, 2000)
        return ok

    def follow(self, x, y, yaw):
        """Behind and slightly to the side of the robot, at ~1.2 m, looking forward and down (sees the obstacle)."""
        back, side, h = 0.70, 0.30, 0.50
        cx = x - back * math.cos(yaw) + side * math.sin(yaw)
        cy = y - back * math.sin(yaw) - side * math.cos(yaw)
        look = math.atan2(y + 0.6 * math.sin(yaw) - cy, x + 0.6 * math.cos(yaw) - cx)
        return self.camera((cx, cy, h), 0.50, look)

    def wide(self):
        return self.camera((0.0, -2.6, 4.0), 1.02, math.pi / 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="mean")
    ap.add_argument("--scenario", default="S4_dropout")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=50.0)
    ap.add_argument("--fault", type=float, nargs=2, default=(15.0, 45.0))
    ap.add_argument("--times", type=float, nargs="+", default=(12.0, 26.0, 40.0))
    ap.add_argument("--wide", type=float, default=48.0)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    tag = f"{a.method}_{a.scenario}"
    csv = os.path.join(OUT, f"e4_gz_{tag}.csv")
    for f in glob.glob(csv) + glob.glob(os.path.join(OUT, f"e4_gz_{tag}_t*.png")) + \
            glob.glob(os.path.join(OUT, f"e4_gz_{tag}_amplo.png")):
        os.remove(f)
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, GZ_PARTITION=PART)
    cmd = ["ros2", "launch", "lpa2v_sim", "e4.launch.py", f"method:={a.method}", f"scenario:={a.scenario}",
           f"seed:={a.seed}", f"duration:={a.duration}", f"fault_start:={a.fault[0]}",
           f"fault_end:={a.fault[1]}", f"out:={csv}", "gui:=true", "auto_shutdown:=false"]
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    rclpy.init()
    node = rclpy.create_node("e4_screenshots")
    st = {"t": -1.0, "pose": None, "fused": None, "gt": None}
    node.create_subscription(Clock, "/clock", lambda m: st.__setitem__(
        "t", m.clock.sec + m.clock.nanosec * 1e-9), 10)

    def pose(m):
        tfs = [tf for tf in m.transforms if tf.child_frame_id == "tb3"] or m.transforms[:1]
        for tf in tfs:
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            st["pose"] = (p.x, p.y, yaw)
    node.create_subscription(TFMessage, "/true_pose", pose, 10)
    node.create_subscription(Float64, "/e4/fused", lambda m: st.__setitem__("fused", m.data), 10)
    node.create_subscription(Float64, "/e4/gt", lambda m: st.__setitem__("gt", m.data), 10)
    scene = Scene(a.method, a.fault)
    shots = sorted([(t, f"t{int(t):02d}") for t in a.times] + [(a.wide, "amplo")])
    ready, last_pt, last_cam = False, -1.0, -1.0
    t_start = time.time()
    deadline = t_start + a.duration * 3 + 120
    try:
        while shots and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            t = st["t"]
            if not ready and t > 0.3 and st["pose"] is not None and time.time() - t_start > 8:
                ready = scene.follow(*st["pose"])        # the camera responds: the GUI is ready
                print("GUI ready" if ready else "waiting for the GUI", flush=True)
                if not ready:
                    time.sleep(2)
                continue
            if not ready or st["pose"] is None:
                continue
            if t - last_pt >= 0.25:
                scene.add(t, *st["pose"], st["fused"], st["gt"])
                last_pt = t
            nxt, name = shots[0]
            if name != "amplo" and t - last_cam >= 0.2:
                scene.follow(*st["pose"])
                last_cam = t
            if name == "amplo" and t >= nxt - 1.5 and last_cam < nxt - 1.5:
                scene.wide()
                last_cam = t
            if t >= nxt:
                time.sleep(0.4)
                p = os.path.join(OUT, f"e4_gz_{tag}_{name}.png")
                ImageGrab.grab().save(p)
                shots.pop(0)
                print("capture:", p, f"(t_sim={t:.1f} s, fused {st['fused']:.2f} m, true {st['gt']:.2f} m)",
                      flush=True)
        while not os.path.exists(csv) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        os.killpg(proc.pid, signal.SIGKILL)
        for pid in subprocess.run(["pgrep", "-f", "gz sim"], capture_output=True, text=True).stdout.split():
            try:                                  # only the Gazebo of this partition (not the batch one)
                if f"GZ_PARTITION={PART}" in open(f"/proc/{pid}/environ").read().split("\0"):
                    os.kill(int(pid), signal.SIGKILL)
            except (OSError, ValueError):
                pass
        node.destroy_node()
        rclpy.try_shutdown()
    print("CSV:", csv if os.path.exists(csv) else "not generated")
    sys.exit(0)


if __name__ == "__main__":
    main()

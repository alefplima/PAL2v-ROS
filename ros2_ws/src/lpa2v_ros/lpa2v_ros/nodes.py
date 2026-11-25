"""ROS 2 nodes of the LPA2v library.

Each node is a thin wrapper around the `lpa2v` core (pure Python), so all the logic is
tested outside ROS (lpa2v_core/tests) and the node only handles topics, parameters and time.

Nodes:
  para_analyzer   Evidence  -> PANState
  pal2v_fusion    N x Float64 (or sensor_msgs/Range) -> Float64 + FusionState + DiagnosticArray
  pal2v_filter    Float64 -> Float64 (lPANCell)
  pal2v_controller setpoint + measurement (Float64) -> command (Float64)
"""
from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

from lpa2v import PAL2vController, PAL2vFilter, PAL2vFusion, analyze
from lpa2v_msgs.msg import Evidence, FusionState, PANState


def _fill_pan(msg: PANState, r, stamp) -> PANState:
    msg.header.stamp = stamp
    msg.mu, msg.lam, msg.dc, msg.dct = r.mu, r.lam, r.dc, r.dct
    msg.dcr, msg.mu_er, msg.phi, msg.phi_signed = r.dcr, r.mu_er, r.phi, r.phi_signed
    msg.label, msg.decision = r.label, r.decision
    return msg


class ParaAnalyzerNode(Node):
    def __init__(self):
        super().__init__("para_analyzer")
        self.declare_parameter("ftc", 0.5)
        self.pub = self.create_publisher(PANState, "pan_state", 10)
        self.create_subscription(Evidence, "evidence", self.cb, 10)

    def cb(self, ev: Evidence):
        r = analyze(ev.mu, ev.lam, ftc=self.get_parameter("ftc").value)
        out = _fill_pan(PANState(), r, self.get_clock().now().to_msg())
        out.header.frame_id = ev.proposition
        self.pub.publish(out)


class FusionNode(Node):
    """Fuses N redundant sensors with the NAP + ParaExtrctr network (no mean, no median). Parameters:
    input_topics (str[]), input_type ('float64'|'range'), vmin/vmax (UD), ct_lim (float[], contradiction
    limits in the UD), rate_lim (float[], temporal contradiction limits in the UD), rate_hz (float),
    mode ("graded" default, no exclusion | "step" v1.0.0 limit case), band (graded: normalized contradiction where phi reaches 0)."""

    def __init__(self):
        super().__init__("pal2v_fusion")
        p = self.declare_parameters("", [
            ("input_topics", ["sensor_0", "sensor_1", "sensor_2"]),
            ("input_type", "float64"),
            ("vmin", 0.0),
            ("vmax", 10.0),
            ("ct_lim", [0.05, 0.05, 0.05]),
            ("rate_lim", [0.1, 0.1, 0.1]),
            ("rate_hz", 50.0),
            ("mode", "graded"),
            ("band", 1.5),
        ])
        g = {x.name: x.value for x in p}
        topics = list(g["input_topics"])
        self.fusion = PAL2vFusion(g["vmin"], g["vmax"], g["ct_lim"], g["rate_lim"], mode=g["mode"],
                                  band=g["band"])
        self.latest = [None] * len(topics)
        msg_t = Range if g["input_type"] == "range" else Float64
        for i, t in enumerate(topics):
            self.create_subscription(msg_t, t, lambda m, i=i: self._store(i, m), 10)
        self.pub_val = self.create_publisher(Float64, "fused", 10)
        self.pub_state = self.create_publisher(FusionState, "fusion_state", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / g["rate_hz"], self.tick)

    def _store(self, i, m):
        self.latest[i] = m.range if isinstance(m, Range) else m.data

    def tick(self):
        if any(v is None for v in self.latest):
            return
        t0 = time.perf_counter()
        out = self.fusion.update(self.latest)
        dt_us = (time.perf_counter() - t0) * 1e6
        stamp = self.get_clock().now().to_msg()
        self.pub_val.publish(Float64(data=out.value))
        s = FusionState()
        s.header.stamp = stamp
        s.value, s.certainty, s.contradiction = out.value, out.certainty, out.contradiction
        s.mu, s.gct, s.phi_e, s.labels = out.mu, out.gct, out.phi_e, out.labels
        s.mu_group, s.indefinite = out.mu_group, out.indefinite
        s.rejected = list(out.rejected)
        self.pub_state.publish(s)
        d = DiagnosticArray()
        d.header.stamp = stamp
        st = DiagnosticStatus(name="lpa2v/fusion", hardware_id=self.get_name())
        st.level = DiagnosticStatus.OK if not out.rejected else DiagnosticStatus.WARN
        if out.indefinite:
            st.level = DiagnosticStatus.ERROR     # indefiniteness: output held
        st.message = "ok" if not out.rejected else (
            f"rejected sensors: {out.rejected}" if self.fusion.mode == "step"
            else f"sensors outside the tolerance (attenuated): {out.rejected}")
        st.values = [KeyValue(key="certainty", value=f"{out.certainty:.3f}"),
                     KeyValue(key="contradiction", value=f"{out.contradiction:.3f}"),
                     KeyValue(key="compute_us", value=f"{dt_us:.1f}")]
        d.status = [st]
        self.pub_diag.publish(d)


class FilterNode(Node):
    def __init__(self):
        super().__init__("pal2v_filter")
        p = self.declare_parameters("", [("vmin", 0.0), ("vmax", 10.0), ("lf", 1.0),
                                         ("iterations", 1)])
        g = {x.name: x.value for x in p}
        self.f = PAL2vFilter(g["vmin"], g["vmax"], g["lf"], g["iterations"])
        self.pub = self.create_publisher(Float64, "filtered", 10)
        self.create_subscription(Float64, "raw", lambda m: self.pub.publish(
            Float64(data=self.f.update(m.data))), 10)


class ControllerNode(Node):
    """PAL2v-PI controller. Publishes `command` at rate_hz with the latest setpoint/measurement."""

    def __init__(self):
        super().__init__("pal2v_controller")
        p = self.declare_parameters("", [
            ("e_max", 2.0), ("td", 0.6), ("kc", 12.0), ("ki", 3.0),
            ("u_min", 0.0), ("u_max", 30.0), ("u_ff", 0.0), ("rate_hz", 100.0)])
        g = {x.name: x.value for x in p}
        self.dt = 1.0 / g["rate_hz"]
        self.ctl = PAL2vController(g["e_max"], g["td"], g["kc"], g["ki"],
                                   g["u_min"], g["u_max"], g["u_ff"])
        self.r = None
        self.y = None
        self.create_subscription(Float64, "setpoint", lambda m: setattr(self, "r", m.data), 10)
        self.create_subscription(Float64, "measurement", lambda m: setattr(self, "y", m.data), 10)
        self.pub = self.create_publisher(Float64, "command", 10)
        self.pub_pan = self.create_publisher(PANState, "controller_pan", 10)
        self.create_timer(self.dt, self.tick)

    def tick(self):
        if self.r is None or self.y is None:
            return
        u = self.ctl.update(self.r, self.y, self.dt)
        self.pub.publish(Float64(data=u))
        if self.ctl.last is not None:
            self.pub_pan.publish(_fill_pan(PANState(), self.ctl.last,
                                           self.get_clock().now().to_msg()))


def _run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def para_analyzer_main():
    _run(ParaAnalyzerNode)


def fusion_main():
    _run(FusionNode)


def filter_main():
    _run(FilterNode)


def controller_main():
    _run(ControllerNode)

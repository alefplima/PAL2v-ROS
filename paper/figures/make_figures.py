"""Conceptual figures of the article: PAL2v lattice/unit square (QUPC) and ROS 2 architecture."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

OUT = os.path.dirname(os.path.abspath(__file__))

# 1) Unit square / lattice: (Gc, Gct) plane with the extreme regions and a PAN example
fig, ax = plt.subplots(figsize=(5.6, 5.4))
ax.add_patch(Polygon([(1, 0), (0, 1), (-1, 0), (0, -1)], closed=True, fc="#eaf4ef", ec="0.3", lw=1.2))
for (x, y, t, ha, va) in [(1, 0, "t  true\n(Gc = +1)", "left", "center"), (-1, 0, "F  false\n(Gc = −1)", "right", "center"),
                          (0, 1, "⊤  inconsistent\n(Gct = +1)", "center", "bottom"), (0, -1, "⊥  paracomplete\n(Gct = −1)", "center", "top")]:
    ax.plot(x, y, "o", color="#1b7f5c", ms=7)
    ax.text(x + (0.08 if ha == "left" else -0.1 if ha == "right" else 0), y + (0.07 if va == "bottom" else -0.06 if va == "top" else 0),
            t, ha=ha, va=va, fontsize=9)
lim = 0.5
ax.add_patch(Polygon([(-lim, lim * 0 + 0.5), (lim, 0.5), (lim, -0.5), (-lim, -0.5)], closed=True, fc="none", ec="#2e86c1", ls="--", lw=1))
ax.text(0.02, 0.53, "control limits\n(Vscct, Vicct, Vscc, Vicc)", fontsize=7.5, color="#2e86c1")
mu, la = 0.87, 1 - 0.68
gc, gct = mu - la, mu + la - 1
ax.plot(gc, gct, "s", color="#c0392b", ms=7)
ax.annotate("PAN: μ = 0.87; λ = 0.32\nGc = 0.55; Gct = 0.19", (gc, gct), (0.35, -0.55), fontsize=8,
            arrowprops=dict(arrowstyle="->", color="0.4"), color="#c0392b")
ax.axhline(0, color="0.6", lw=0.6); ax.axvline(0, color="0.6", lw=0.6)
ax.set_xlim(-1.45, 1.45); ax.set_ylim(-1.35, 1.35); ax.set_aspect("equal")
ax.set_xlabel("certainty degree  Gc = μ − λ"); ax.set_ylabel("contradiction degree  Gct = μ + λ − 1")
for s in ("top", "right"): ax.spines[s].set_visible(False)
ax.set_title("PAL2v lattice in the Gc × Gct plane (transformed unit square)", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_lattice.png"), dpi=200); plt.close(fig)

# 2) Architecture: sensors -> fusion (PAL2v or baseline) -> control -> simulator
fig, ax = plt.subplots(figsize=(10, 4.2)); ax.axis("off"); ax.set_xlim(0, 10); ax.set_ylim(0, 4.6)
def box(x, y, w, h, t, fc, ec="0.3", fs=8.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.05,rounding_size=0.08", fc=fc, ec=ec, lw=1.1))
    ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=fs)
def arrow(x1, y1, x2, y2, t=""):
    ax.annotate("", (x2, y2), (x1, y1), arrowprops=dict(arrowstyle="->", lw=1.2, color="0.25"))
    if t: ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 0.12, t, ha="center", fontsize=7.5, color="0.25")
box(0.1, 1.2, 1.9, 1.9, "Gazebo\n(Jetty: TurtleBot3 /\nHarmonic: PX4 x500)\n\nsimulator ground truth", "#eef2f7")
box(2.6, 2.75, 2.0, 0.6, "sensor 1 (LiDAR)", "#fdf2e9"); box(2.6, 1.95, 2.0, 0.6, "sensor 2 (sonar / baro)", "#fdf2e9")
box(2.6, 1.15, 2.0, 0.6, "sensor 3 (IR / GPS)", "#fdf2e9")
box(2.6, 0.2, 2.0, 0.6, "fault injection\nS1 S2 S3 S4 S5 S6", "#f9e79f", fs=7.5)
box(5.2, 1.2, 2.0, 1.9, "fusion node\nPAL2vFusion\n(PAN + ParaExtrctr)\nor baseline:\nMean, Median, IVW,\nParaExtrctr, Kalman χ²", "#e8f6ef", ec="#1b7f5c")
box(7.8, 2.05, 2.0, 1.05, "control\n(reactive avoidance /\nPX4 offboard altitude)", "#eef2f7")
box(7.8, 0.6, 2.0, 0.95, "CSV log + /diagnostics\n(rejected sensor)", "#f4f6f6", fs=7.5)
for y in (3.05, 2.25, 1.45): arrow(2.0, 2.15, 2.6, y)
arrow(3.6, 0.8, 3.6, 1.15)
for y in (3.05, 2.25, 1.45): arrow(4.6, y, 5.2, 2.15)
arrow(7.2, 2.4, 7.8, 2.6, "fused value"); arrow(7.2, 1.7, 7.8, 1.1, "state")
ax.annotate("", (1.05, 3.1), (8.8, 3.1), arrowprops=dict(arrowstyle="->", lw=1.2, color="0.25", connectionstyle="arc3,rad=0.25"))
ax.text(5.0, 4.4, "command (cmd_vel / vertical velocity setpoint) — closed loop", ha="center", fontsize=8, color="0.25")
fig.savefig(os.path.join(OUT, "fig_architecture.png"), dpi=200, bbox_inches="tight"); plt.close(fig)
print("ok")

"""
robot_arm_sim.py
================
Real-time 3D Simulation GUI สำหรับแขนหุ่นยนต์ 4DOF
รัน standalone หรือควบคู่กับ robot_arm_gui.py ก็ได้

ตรรกะ IK เหมือนกับที่ Arduino (robot_arm_4dof.ino) ใช้ทุกประการ:
  - Base     : หมุนรอบแกน Z
  - Shoulder : ยกขึ้น-ลงในระนาบ vertical
  - Elbow    : พับข้อศอก
  - Wrist    : IK ชดเชยให้ตั้งฉากพื้น + Offset ก้มเงย (จำลอง pin 13)
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import serial
import queue

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
import numpy as np

# ============================================================
#  พารามิเตอร์หุ่นยนต์ (ต้องตรงกับ .ino)
# ============================================================
L1 = 95.0    # height of base (mm)
L2 = 120.0   # upper arm
L3 = 120.0   # forearm
L4 = 130.0   # end-effector / wrist
STEP_SIZE  = 1.0    # mm per interpolation step
STEP_DELAY = 0.015  # seconds per step

# ============================================================
#  Serial (optional)
# ============================================================
SERIAL_PORT = "COM6"
BAUD_RATE   = 115200

# ============================================================
#  IK Solver  (คืน 4 มุม – wrist คือมุม servo ที่ตั้งฉากพื้น)
# ============================================================
def solve_ik(x: float, y: float, z: float):
    """(x,y,z) = gripper tip.  คืน (theta1_deg, theta2_deg, theta3_deg, wrist_servo_deg)
    หรือ None ถ้านอก reach.  wrist_servo_deg คือมุมที่ให้ L4 ชี้ลงตรง (offset ยัง = 0)."""
    wx, wy, wz = x, y, z + L4
    r = math.sqrt(wx*wx + wy*wy)
    if r < 1e-6:
        return None
    t1_math = math.atan2(wy, wx)
    z_adj  = wz - L1
    d      = math.sqrt(r*r + z_adj*z_adj)
    if d > (L2 + L3):
        return None
    cos3 = (d*d - L2*L2 - L3*L3) / (2.0 * L2 * L3)
    cos3 = max(-1.0, min(1.0, cos3))
    theta3 = math.acos(cos3)
    alpha  = math.atan2(z_adj, r)
    beta   = math.atan2(L3*math.sin(theta3), L2 + L3*math.cos(theta3))
    theta2 = alpha + beta
    wrist_comp  = (theta2 - theta3)
    wrist_servo = max(0.0, min(180.0, 90.0 + math.degrees(wrist_comp)))
    theta1_deg = max(0.0, min(180.0, 90.0 + math.degrees(t1_math)))
    return (theta1_deg, math.degrees(theta2),
            math.degrees(theta3), wrist_servo)


# ============================================================
#  Forward Kinematics → ตำแหน่ง joint ทุกตัว
#
#  wrist_offset_deg (เทียบเท่า pin 13 บน Arduino):
#    0  → L4 ชี้ลงแนวดิ่ง (ตั้งฉากพื้น)
#   +X  → เงย (L4 เอียงไปข้างหน้า)
#   -X  → ก้ม (L4 เอียงกลับ)
#
#  ทิศทาง L4 ในระนาบ vertical ของแขน (plane ที่มี t1):
#    l4_dir = [ cos(t1)*sin(wo),  sin(t1)*sin(wo),  -cos(wo) ]
#  wo = 0 → ตรงลงพอดี (0, 0, -1) ✓
# ============================================================
def fk_joints(x: float, y: float, z: float, wrist_offset_deg: float = 0.0):
    """คืน 5 จุด: origin → base_top → shoulder → wrist → gripper_tip"""
    angles = solve_ik(x, y, z)
    if angles is None:
        return None
    t1_deg, t2_deg, t3_deg, _ = angles
    t1 = math.radians(t1_deg - 90.0)
    t2 = math.radians(t2_deg)
    t3 = math.radians(t3_deg)
    wo = math.radians(wrist_offset_deg)   # wrist tilt offset

    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, L1])
    p2 = p1 + np.array([math.cos(t1)*L2*math.cos(t2),
                         math.sin(t1)*L2*math.cos(t2),
                         L2*math.sin(t2)])
    t2e    = t2 - t3
    l3_dir = np.array([math.cos(t1)*math.cos(t2e),
                        math.sin(t1)*math.cos(t2e),
                        math.sin(t2e)])
    p3 = p2 + L3 * l3_dir   # wrist joint

    # L4 ชี้ลงตั้งฉาก + offset จาก pin 13
    l4_dir = np.array([math.cos(t1)*math.sin(wo),
                        math.sin(t1)*math.sin(wo),
                        -math.cos(wo)])
    p4 = p3 + L4 * l4_dir   # gripper tip

    return [p0, p1, p2, p3, p4]


def fk_from_angles(base_deg, shoulder_deg, elbow_deg, wrist_deg=90.0):
    t1 = math.radians(base_deg)
    t2 = math.radians(shoulder_deg - 90.0)
    t3 = math.radians(elbow_deg)
    t4 = math.radians(wrist_deg - 90.0)
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, L1])
    p2 = p1 + np.array([math.cos(t1)*L2*math.cos(t2),
                         math.sin(t1)*L2*math.cos(t2),
                         L2*math.sin(t2)])
    t2e    = t2 - t3
    l3_dir = np.array([math.cos(t1)*math.cos(t2e),
                        math.sin(t1)*math.cos(t2e),
                        math.sin(t2e)])
    p3 = p2 + L3 * l3_dir
    t2w    = t2e - t4
    l4_dir = np.array([math.cos(t1)*math.cos(t2w),
                        math.sin(t1)*math.cos(t2w),
                        math.sin(t2w)])
    p4 = p3 + L4 * l4_dir
    return [p0, p1, p2, p3, p4]


# ============================================================
#  Main Application
# ============================================================
class RobotSimApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("🤖 Robot Arm – Real-time 3D Simulation")
        self.root.configure(bg="#1a1a2e")

        self.current = np.array([120.0, 0.0, 85.0])
        self.target  = np.array([120.0, 0.0, 85.0])
        # wrist_tilt_offset: จำลองค่า pin 13  (−60 ถึง +60°)
        self.wrist_tilt_offset = 0.0
        self.lock = threading.Lock()

        self.ser = None
        self._connect_serial()

        self.cmd_queue = queue.Queue(maxsize=1)
        self._build_ui()

        self.running = True
        threading.Thread(target=self._interpolation_loop, daemon=True).start()
        self._animate()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------------------------------------
    def _connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.serial_ok = True
            print(f"✅ Serial connected: {SERIAL_PORT}")
        except Exception:
            self.ser = None
            self.serial_ok = False
            print(f"⚠️  Serial {SERIAL_PORT} not found – simulation only mode")

    # ----------------------------------------------------------
    def _build_ui(self):
        BG   = "#1a1a2e"
        CARD = "#16213e"
        ACC  = "#0f3460"
        BLUE = "#e94560"

        left = tk.Frame(self.root, bg=CARD, width=340, padx=18, pady=18)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        tk.Label(left, text="🤖 Robot Arm Controller",
                 font=("Segoe UI", 14, "bold"), bg=CARD, fg="white").pack(pady=(0, 8))

        status_txt = (f"🔴 Serial: {SERIAL_PORT} (offline)" if not self.serial_ok
                      else f"🟢 Serial: {SERIAL_PORT}")
        self.lbl_serial = tk.Label(left, text=status_txt,
                                   font=("Segoe UI", 9), bg=CARD, fg="#aaaaaa")
        self.lbl_serial.pack(pady=(0, 8))

        # ── Notebook ─────────────────────────────────────────
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook",      background=CARD, borderwidth=0)
        style.configure("TNotebook.Tab",  background=ACC,  foreground="#aaaacc",
                        font=("Segoe UI", 10, "bold"), padding=[12, 4])
        style.map("TNotebook.Tab",
                  background=[("selected", BLUE)],
                  foreground=[("selected", "white")])
        style.configure("TFrame", background=CARD)

        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)

        # ── Tab 1 : Slider ────────────────────────────────────
        tab_slider = ttk.Frame(nb)
        nb.add(tab_slider, text="🎚  Slider")

        slider_cfg = dict(orient="horizontal", length=280, bg=CARD,
                          fg="white", troughcolor=ACC,
                          highlightthickness=0, activebackground=BLUE)

        def make_slider(parent, label, from_, to, init):
            tk.Label(parent, text=label, font=("Segoe UI", 10, "bold"),
                     bg=CARD, fg="#aaaacc").pack(anchor="w", pady=(10, 0))
            sv = tk.Scale(parent, from_=from_, to=to, **slider_cfg)
            sv.set(init)
            sv.pack()
            return sv

        self.sl_x = make_slider(tab_slider, "X  –  หน้า / หลัง  (mm)", 0, 230,   120)
        self.sl_y = make_slider(tab_slider, "Y  –  ซ้าย / ขวา   (mm)", -175, 175, 0)
        self.sl_z = make_slider(tab_slider, "Z  –  ขึ้น / ลง    (mm)",    0, 200,  85)

        # ── Wrist: ล็อกตั้งฉากพื้นเสมอ ──────────────────────
        tk.Label(tab_slider,
                 text="📌 Wrist  –  ตั้งฉากพื้นเสมอ  (ล็อก)",
                 font=("Segoe UI", 10, "bold"), bg=CARD, fg="#aaffaa",
                 justify="left").pack(anchor="w", pady=(10, 0))
        self.sl_w = tk.Scale(tab_slider, from_=-60, to=60, **slider_cfg,
                             state="disabled")
        self.sl_w.set(0)
        self.sl_w.pack()
        self.lbl_tilt_state = tk.Label(tab_slider, text="✅  ตั้งฉากพื้น (ล็อก)",
                                       font=("Segoe UI", 9), bg=CARD, fg="#aaffaa")
        self.lbl_tilt_state.pack()

        self.sl_x.configure(command=self._on_slider)
        self.sl_y.configure(command=self._on_slider)
        self.sl_z.configure(command=self._on_slider)
        # sl_w ไม่ต้อง bind command เพราะล็อกไว้

        # ── Tab 2 : IK ────────────────────────────────────────
        tab_ik = ttk.Frame(nb)
        nb.add(tab_ik, text="🎯  IK")

        tk.Label(tab_ik, text="ป้อนพิกัดปลายแขน (mm)",
                 font=("Segoe UI", 11, "bold"), bg=CARD, fg="white").pack(pady=(14, 6))

        entry_cfg = dict(font=("Segoe UI", 13), width=8,
                         bg="#0f3460", fg="white", insertbackground="white",
                         relief="flat", justify="center")

        def make_entry_row(parent, label_text, default_val):
            row = tk.Frame(parent, bg=CARD)
            row.pack(pady=6)
            tk.Label(row, text=label_text, font=("Segoe UI", 11, "bold"),
                     bg=CARD, fg="#aaaacc", width=5, anchor="e").pack(side="left", padx=(0, 6))
            ent = tk.Entry(row, **entry_cfg)
            ent.insert(0, str(default_val))
            ent.pack(side="left")
            tk.Label(row, text="mm", font=("Segoe UI", 10),
                     bg=CARD, fg="#888888").pack(side="left", padx=(4, 0))
            return ent

        self.ent_x = make_entry_row(tab_ik, "X:",  120)
        self.ent_y = make_entry_row(tab_ik, "Y:",  0)
        self.ent_z = make_entry_row(tab_ik, "Z:",  85)

        self.lbl_ik_result = tk.Label(tab_ik, text="",
                                      font=("Segoe UI", 9), bg=CARD, fg="#aaffaa",
                                      justify="center", wraplength=280)
        self.lbl_ik_result.pack(pady=6)
        self.lbl_ik_warn = tk.Label(tab_ik, text="",
                                    font=("Segoe UI", 9, "bold"), bg=CARD, fg="#ff6666")
        self.lbl_ik_warn.pack(pady=2)

        btn_move = tk.Button(tab_ik, text="▶  Move to XYZ",
                             font=("Segoe UI", 12, "bold"),
                             bg="#0f3460", fg="#66ccff", relief="flat",
                             activebackground="#1a5a9f", activeforeground="white",
                             padx=12, pady=8,
                             command=self._on_ik_move)
        btn_move.pack(pady=(8, 4), fill="x", padx=20)

        for ent in (self.ent_x, self.ent_y, self.ent_z):
            ent.bind("<Return>", lambda e: self._on_ik_move())

        # ── Labels ───────────────────────────────────────────
        self.lbl_coords = tk.Label(left, text="",
                                   font=("Segoe UI", 10), bg=CARD, fg="#66ccff")
        self.lbl_coords.pack(pady=(6, 0))

        self.lbl_angles = tk.Label(left, text="",
                                   font=("Segoe UI", 9), bg=CARD, fg="#aaffaa",
                                   justify="left")
        self.lbl_angles.pack(pady=2)

        self.lbl_warn = tk.Label(left, text="", font=("Segoe UI", 9, "bold"),
                                 bg=CARD, fg="#ff6666")
        self.lbl_warn.pack(pady=2)

        tk.Button(left, text="🏠  กลับ Home (90°)",
                  font=("Segoe UI", 11, "bold"),
                  bg="#e94560", fg="white", relief="flat",
                  padx=12, pady=8,
                  command=self._set_home).pack(pady=(10, 4), fill="x")

        # ── 3D Plot ──────────────────────────────────────────
        right = tk.Frame(self.root, bg=BG)
        right.pack(side="right", fill="both", expand=True)

        self.fig = plt.Figure(figsize=(6, 6), facecolor=BG)
        self.ax  = self.fig.add_subplot(111, projection="3d")
        self._setup_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self._update_labels(120, 0, 85)

    # ----------------------------------------------------------
    def _setup_axes(self):
        ax = self.ax
        ax.set_facecolor("#0d0d1a")
        ax.set_xlabel("X (mm)", color="#888888")
        ax.set_ylabel("Y (mm)", color="#888888")
        ax.set_zlabel("Z (mm)", color="#888888")
        ax.set_xlim(-300, 300)
        ax.set_ylim(-300, 300)
        ax.set_zlim(0, 400)
        ax.tick_params(colors="#555555")
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#222244")
        ax.yaxis.pane.set_edgecolor("#222244")
        ax.zaxis.pane.set_edgecolor("#222244")
        ax.grid(True, color="#222244", linewidth=0.5)
        ax.set_title("3D Arm Simulation", color="white", pad=12, fontsize=12)

    # ----------------------------------------------------------
    def _on_slider(self, _val=None):
        if not all(hasattr(self, a) for a in ('sl_x','sl_y','sl_z')):
            return
        x = float(self.sl_x.get())
        y = float(self.sl_y.get())
        z = float(self.sl_z.get())

        # Wrist ล็อก = 0 ตลอดเวลา
        self.wrist_tilt_offset = 0.0
        with self.lock:
            self.target = np.array([x, y, z])
        self._send_serial(x, y, z)
        self._update_labels(x, y, z)

    # ----------------------------------------------------------
    def _on_ik_move(self):
        try:
            x = float(self.ent_x.get())
            y = float(self.ent_y.get())
            z = float(self.ent_z.get())
        except ValueError:
            self.lbl_ik_warn.config(text="⚠️  กรุณากรอกตัวเลขให้ถูกต้อง")
            self.lbl_ik_result.config(text="")
            return

        angles = solve_ik(x, y, z)
        if angles is None:
            self.lbl_ik_warn.config(text="⚠️  พิกัดนี้เกิน reach ของแขนกล!")
            self.lbl_ik_result.config(text="")
            return

        self.lbl_ik_warn.config(text="✅  IK แก้ได้!")
        t1, t2, t3, t4 = angles
        wo = self.wrist_tilt_offset
        self.lbl_ik_result.config(
            text=f"Base={t1:.1f}°  Shoulder={t2:.1f}°\n"
                 f"Elbow={t3:.1f}°  Wrist servo={t4+wo:.1f}°\n"
                 f"(Tilt offset: {wo:+.0f}°  →  {'ตั้งฉาก' if abs(wo)<2 else ('เงย' if wo>0 else 'ก้ม')})")

        with self.lock:
            self.target = np.array([x, y, z])

        self.sl_x.set(int(max(0,   min(230, x))))
        self.sl_y.set(int(max(-175, min(175, y))))
        self.sl_z.set(int(max(0,    min(200, z))))

        self._send_serial(x, y, z)
        self._update_labels(x, y, z)

    # ----------------------------------------------------------
    def _set_home(self):
        self.sl_x.set(120)
        self.sl_y.set(0)
        self.sl_z.set(85)
        self.sl_w.set(0)
        with self.lock:
            self.target = np.array([120.0, 0.0, 85.0])
            self.wrist_tilt_offset = 0.0
        if self.ser:
            try:
                self.ser.write(b"HOME\n")
            except Exception:
                pass

    # ----------------------------------------------------------
    def _send_serial(self, x, y, z):
        """ส่งเฉพาะ X,Y,Z  — wrist offset ถูกควบคุมโดย pin 13 บน Arduino โดยตรง"""
        if self.ser:
            try:
                cmd = f"{int(x)},{int(y)},{int(z)}\n"
                self.ser.write(cmd.encode())
            except Exception:
                pass

    # ----------------------------------------------------------
    def _update_labels(self, x, y, z):
        angles = solve_ik(float(x), float(y), float(z))
        wo     = self.wrist_tilt_offset
        coord_txt = (f"📍  X = {int(x):>5} mm  |  Y = {int(y):>5} mm  |  Z = {int(z):>5} mm")
        self.lbl_coords.config(text=coord_txt)
        if angles:
            t1, t2, t3, t4 = angles
            self.lbl_angles.config(
                text=f"Base={t1:.1f}°  Shoulder={t2:.1f}°\n"
                     f"Elbow={t3:.1f}°  Wrist={t4+wo:.1f}°  (offset {wo:+.0f}°)")
            self.lbl_warn.config(text="")
        else:
            self.lbl_angles.config(text="")
            self.lbl_warn.config(text="⚠️  เกิน reach ของแขนกล!")

    # ----------------------------------------------------------
    def _interpolation_loop(self):
        while self.running:
            with self.lock:
                cur = self.current.copy()
                tgt = self.target.copy()

            delta    = tgt - cur
            distance = np.linalg.norm(delta)

            if distance > STEP_SIZE:
                step = (delta / distance) * STEP_SIZE
                with self.lock:
                    self.current += step
                time.sleep(STEP_DELAY)
            elif distance > 0.001:
                with self.lock:
                    self.current = tgt.copy()
                time.sleep(STEP_DELAY)
            else:
                time.sleep(0.005)

    # ----------------------------------------------------------
    def _animate(self):
        with self.lock:
            pos = self.current.copy()
            wo  = self.wrist_tilt_offset

        joints = fk_joints(pos[0], pos[1], pos[2], wo)
        self.ax.cla()
        self._setup_axes()

        if joints:
            xs = [p[0] for p in joints]
            ys = [p[1] for p in joints]
            zs = [p[2] for p in joints]

            colors = ["#4488ff", "#44bbff", "#44ffcc", "#ffaa44"]
            widths = [4, 4, 4, 3]
            for i in range(len(joints) - 1):
                self.ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]],
                             color=colors[i], linewidth=widths[i], solid_capstyle="round")

            joint_colors = ["#ffffff", "#ffdd44", "#ff8844", "#00ffcc", "#ff00aa"]
            sizes        = [80, 120, 100, 90, 60]
            for i, (jx, jy, jz) in enumerate(zip(xs, ys, zs)):
                self.ax.scatter(jx, jy, jz, c=joint_colors[i], s=sizes[i],
                                depthshade=False, zorder=5)

            tip = joints[-1]
            self.ax.scatter(tip[0], tip[1], tip[2],
                            c="#ff0055", s=180, marker="*",
                            depthshade=False, zorder=10)

            # แสดงเส้นแนวดิ่งจาก wrist ลงพื้น
            wrist = joints[-2]
            self.ax.plot([wrist[0], wrist[0]], [wrist[1], wrist[1]],
                         [0, wrist[2]], color="#333355", linewidth=1, linestyle="--")

            # แสดงเส้นแนวดิ่งอ้างอิง (สีเขียวจาง) ถ้า gripper ไม่ตั้งฉาก
            if abs(wo) > 2:
                self.ax.plot([wrist[0], wrist[0]], [wrist[1], wrist[1]],
                             [wrist[2], wrist[2] - L4],
                             color="#224422", linewidth=1, linestyle=":")

            # Workspace circle
            theta_arr = np.linspace(0, 2*np.pi, 120)
            reach = L2 + L3
            self.ax.plot(reach * np.cos(theta_arr),
                         reach * np.sin(theta_arr),
                         np.zeros(120),
                         color="#333355", linewidth=1, linestyle=":")

            # Label tilt state บน 3D
            tilt_lbl = "↕ ตั้งฉาก" if abs(wo) < 2 else (f"↗ เงย {wo:.0f}°" if wo > 0 else f"↘ ก้ม {abs(wo):.0f}°")
            self.ax.set_title(f"3D Arm Simulation  |  Wrist: {tilt_lbl}",
                              color="white", pad=12, fontsize=11)
        else:
            self.ax.text(0, 0, 200, "OUT OF REACH", color="red",
                         ha="center", fontsize=14, fontweight="bold")

        self.canvas.draw_idle()

        if self.running:
            self.root.after(30, self._animate)

    # ----------------------------------------------------------
    def _on_close(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.root.destroy()


# ============================================================
#  Entry point
# ============================================================
if __name__ == "__main__":
    root = tk.Tk()
    app  = RobotSimApp(root)
    root.mainloop()

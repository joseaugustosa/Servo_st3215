"""
GUI gráfica — braço a1..a4 + garra.

Separadores:
  Cartesiano (IK) | Articular | Parâmetros | Ligação
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from arm_model import ArmController, JOINT_NAMES, JOINT_LABELS
from kinematics import JointAngles, Pose
from comm import create_backend, create_backend_from_config, list_serial_ports


class ArmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ST3215 — Braço a1..a4 + Garra")
        self.root.geometry("1240x760")
        self.root.minsize(1040, 680)

        self.arm = ArmController()
        self.backend = create_backend_from_config()
        self._updating = False
        try:
            self.backend.connect()
        except Exception:
            pass

        self.workspace = float(self.arm.cfg.get("gui", {}).get("workspace_mm", 350))
        self._build_layout()
        self._load_param_fields()
        self._load_connection_fields()
        self._sync_sliders_from_arm()
        self._redraw()
        self._push_hardware(silent=True)

    def _build_layout(self):
        main = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        left = ttk.Frame(main)
        right = ttk.Frame(main)
        main.add(left, weight=3)
        main.add(right, weight=2)

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        cart = ttk.Frame(nb, padding=8)
        joint = ttk.Frame(nb, padding=8)
        params = ttk.Frame(nb, padding=8)
        conn = ttk.Frame(nb, padding=8)
        nb.add(cart, text="Cartesiano (IK)")
        nb.add(joint, text="Articular a1..a4")
        nb.add(params, text="Parâmetros")
        nb.add(conn, text="Ligação")

        self.vars: dict = {}
        reach = self.arm.geom.L2 + self.arm.geom.L3 + self.arm.geom.L4

        self._add_slider(cart, "x", "X (mm)", -reach, reach, self._on_cartesian)
        self._add_slider(cart, "y", "Y (mm)", -reach, reach, self._on_cartesian)
        self._add_slider(cart, "z", "Z (mm)", 0, self.arm.geom.L1 + reach, self._on_cartesian)
        self._add_slider(cart, "tool_pitch", "Pitch efetor (°)", -90, 90, self._on_cartesian)
        self._add_slider(cart, "gripper", "Garra (°)", 0, 90, self._on_gripper)

        self.elbow_up_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            cart,
            text="Cotovelo para cima (elbow-up)",
            variable=self.elbow_up_var,
            command=self._on_cartesian,
        ).pack(anchor=tk.W, pady=6)

        for key, label, lo, hi in [
            ("j_a1", "a1 Base (°)", -150, 150),
            ("j_a2", "a2 Ombro (°)", -90, 90),
            ("j_a3", "a3 Cotovelo (°)", -140, 140),
            ("j_a4", "a4 Pulso (°)", -120, 120),
            ("j_gripper", "Garra (°)", 0, 90),
        ]:
            self._add_slider(joint, key, label, lo, hi, self._on_joint)

        self._build_params_tab(params)
        self._build_connection_tab(conn)

        status = ttk.LabelFrame(right, text="Estado", padding=8)
        status.pack(fill=tk.X, pady=(8, 0))
        self.status_var = tk.StringVar(value="")
        self.ik_var = tk.StringVar(value="")
        self.pose_var = tk.StringVar(value="")
        ttk.Label(status, textvariable=self.status_var).pack(anchor=tk.W)
        ttk.Label(status, textvariable=self.ik_var, wraplength=340).pack(anchor=tk.W)
        ttk.Label(status, textvariable=self.pose_var, wraplength=340).pack(anchor=tk.W)

        btns = ttk.Frame(right)
        btns.pack(fill=tk.X, pady=8)
        ttk.Button(btns, text="Home", command=self._home).pack(side=tk.LEFT, padx=2)
        ttk.Button(btns, text="Enviar → hardware", command=lambda: self._push_hardware(False)).pack(
            side=tk.LEFT, padx=2
        )

    def _build_params_tab(self, parent):
        ttk.Label(parent, text="Comprimentos dos elos (mm) — editáveis").pack(anchor=tk.W)
        self.param_entries = {}
        for key, label in [
            ("L1", "L1 base→ombro"),
            ("L2", "L2 a2 ombro→cotovelo"),
            ("L3", "L3 a3 cotovelo→pulso"),
            ("L4", "L4 a4 pulso→garra"),
        ]:
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=28).pack(side=tk.LEFT)
            e = ttk.Entry(row, width=10)
            e.pack(side=tk.LEFT)
            self.param_entries[key] = e

        ttk.Separator(parent).pack(fill=tk.X, pady=8)
        ttk.Label(parent, text="IDs dos servos no bus").pack(anchor=tk.W)
        self.id_entries = {}
        for name in JOINT_NAMES:
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=JOINT_LABELS[name], width=28).pack(side=tk.LEFT)
            e = ttk.Entry(row, width=10)
            e.pack(side=tk.LEFT)
            self.id_entries[name] = e

        ttk.Separator(parent).pack(fill=tk.X, pady=8)
        ttk.Label(parent, text="Offset (°) e sinal (+1 / -1)").pack(anchor=tk.W)
        self.offset_entries = {}
        self.sign_entries = {}
        for name in JOINT_NAMES:
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=JOINT_LABELS[name], width=16).pack(side=tk.LEFT)
            ttk.Label(row, text="ofs").pack(side=tk.LEFT)
            eo = ttk.Entry(row, width=8)
            eo.pack(side=tk.LEFT, padx=2)
            ttk.Label(row, text="sign").pack(side=tk.LEFT)
            es = ttk.Entry(row, width=5)
            es.pack(side=tk.LEFT, padx=2)
            self.offset_entries[name] = eo
            self.sign_entries[name] = es

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=10)
        ttk.Button(row, text="Aplicar", command=self._apply_params).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Guardar config.yaml", command=self._save_params).pack(
            side=tk.LEFT, padx=2
        )

    def _build_connection_tab(self, parent):
        ttk.Label(parent, text="Modo de ligação").pack(anchor=tk.W)
        self.mode_var = tk.StringVar(value="simulation")
        for value, text in [
            ("simulation", "Só simulação (sem hardware)"),
            ("serial", "Serial (cabo USB)"),
            ("wifi", "Wi‑Fi (HTTP no ESP32)"),
            ("both", "Serial + Wi‑Fi em simultâneo"),
        ]:
            ttk.Radiobutton(parent, text=text, variable=self.mode_var, value=value).pack(anchor=tk.W)

        ttk.Separator(parent).pack(fill=tk.X, pady=8)
        ttk.Label(parent, text="Serial").pack(anchor=tk.W)
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="Porta", width=12).pack(side=tk.LEFT)
        self.serial_port_var = tk.StringVar(value="COM3")
        self.serial_combo = ttk.Combobox(row, textvariable=self.serial_port_var, width=16)
        self.serial_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Atualizar", command=self._refresh_ports).pack(side=tk.LEFT)

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="Baud", width=12).pack(side=tk.LEFT)
        self.serial_baud_var = tk.StringVar(value="115200")
        ttk.Entry(row, textvariable=self.serial_baud_var, width=12).pack(side=tk.LEFT)

        ttk.Separator(parent).pack(fill=tk.X, pady=8)
        ttk.Label(parent, text="Wi‑Fi / HTTP").pack(anchor=tk.W)
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="URL", width=12).pack(side=tk.LEFT)
        self.wifi_url_var = tk.StringVar(value="http://192.168.4.1")
        ttk.Entry(row, textvariable=self.wifi_url_var, width=28).pack(side=tk.LEFT)

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=10)
        ttk.Button(row, text="Ligar / Religar", command=self._reconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Guardar ligação", command=self._save_connection).pack(
            side=tk.LEFT, padx=2
        )

        tip = ttk.Label(
            parent,
            text="ESP32 em AP: Wi‑Fi ESP32_DEV → http://192.168.4.1/arm\n"
            "(página 3D com arrastar da garra). API: POST /api/arm",
            wraplength=340,
            foreground="#555",
        )
        tip.pack(anchor=tk.W, pady=6)

    def _add_slider(self, parent, key, label, lo, hi, command):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=3)
        ttk.Label(frame, text=label, width=18).pack(side=tk.LEFT)
        var = tk.DoubleVar(value=0.0)
        self.vars[key] = var
        val_lbl = ttk.Label(frame, width=7)

        def on_move(_v=None, c=command, l=val_lbl, vr=var):
            l.config(text=f"{vr.get():.1f}")
            c()

        scale = ttk.Scale(frame, from_=lo, to=hi, variable=var, command=on_move)
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        val_lbl.pack(side=tk.RIGHT)
        val_lbl.config(text=f"{var.get():.1f}")

    # ---- params / connection helpers ----
    def _load_param_fields(self):
        a = self.arm.cfg["arm"]
        for k in ("L1", "L2", "L3", "L4"):
            self.param_entries[k].delete(0, tk.END)
            self.param_entries[k].insert(0, str(a[k]))
        for name in JOINT_NAMES:
            self.id_entries[name].delete(0, tk.END)
            self.id_entries[name].insert(0, str(a["servo_ids"][name]))
            self.offset_entries[name].delete(0, tk.END)
            self.offset_entries[name].insert(0, str(a["joint_offset_deg"][name]))
            self.sign_entries[name].delete(0, tk.END)
            self.sign_entries[name].insert(0, str(a["joint_sign"][name]))

    def _load_connection_fields(self):
        c = self.arm.cfg["connection"]
        mode = c.get("mode", "simulation")
        if mode == "http":
            mode = "wifi"
        self.mode_var.set(mode)
        self.serial_port_var.set(c.get("serial_port", "COM3"))
        self.serial_baud_var.set(str(c.get("serial_baud", 115200)))
        self.wifi_url_var.set(c.get("wifi_url", c.get("http_base_url", "http://192.168.4.1")))
        self._refresh_ports()

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.serial_combo["values"] = ports
        if ports and self.serial_port_var.get() not in ports:
            self.serial_port_var.set(ports[0])

    def _apply_params(self):
        try:
            L1 = float(self.param_entries["L1"].get())
            L2 = float(self.param_entries["L2"].get())
            L3 = float(self.param_entries["L3"].get())
            L4 = float(self.param_entries["L4"].get())
            ids = {n: int(self.id_entries[n].get()) for n in JOINT_NAMES}
            ofs = {n: float(self.offset_entries[n].get()) for n in JOINT_NAMES}
            signs = {n: int(self.sign_entries[n].get()) for n in JOINT_NAMES}
        except ValueError:
            messagebox.showerror("Parâmetros", "Valores inválidos.")
            return
        self.arm.update_geometry(L1, L2, L3, L4)
        self.arm.update_servo_ids(ids)
        self.arm.update_offsets(ofs)
        self.arm.update_signs(signs)
        self.workspace = max(200.0, L1 + L2 + L3 + L4 + 50)
        self._sync_sliders_from_arm()
        self._redraw()
        self.ik_var.set("Parâmetros aplicados")

    def _save_params(self):
        self._apply_params()
        self.arm.save_config()
        messagebox.showinfo("Config", f"Guardado em {self.arm.config_path.name}")

    def _save_connection(self):
        self.arm.cfg["connection"]["mode"] = self.mode_var.get()
        self.arm.cfg["connection"]["serial_port"] = self.serial_port_var.get()
        self.arm.cfg["connection"]["serial_baud"] = int(self.serial_baud_var.get())
        self.arm.cfg["connection"]["wifi_url"] = self.wifi_url_var.get().strip()
        self.arm.save_config()
        messagebox.showinfo("Ligação", "Preferências de ligação guardadas.")

    # ---- motion callbacks ----
    def _sync_sliders_from_arm(self):
        self._updating = True
        try:
            p = self.arm.pose
            self.vars["x"].set(p.x)
            self.vars["y"].set(p.y)
            self.vars["z"].set(p.z)
            self.vars["tool_pitch"].set(math.degrees(p.tool_pitch))
            self.vars["gripper"].set(math.degrees(self.arm.joints.gripper))
            jd = self.arm.joints_deg()
            for name in JOINT_NAMES:
                self.vars[f"j_{name}"].set(jd[name])
        finally:
            self._updating = False

    def _on_cartesian(self, *_):
        if self._updating:
            return
        self.arm.elbow_up = bool(self.elbow_up_var.get())
        result = self.arm.move_to_pose(
            Pose(
                x=self.vars["x"].get(),
                y=self.vars["y"].get(),
                z=self.vars["z"].get(),
                tool_pitch=math.radians(self.vars["tool_pitch"].get()),
            )
        )
        self.arm.set_gripper(math.radians(self.vars["gripper"].get()))
        self._updating = True
        try:
            jd = self.arm.joints_deg()
            for name in JOINT_NAMES:
                self.vars[f"j_{name}"].set(jd[name])
        finally:
            self._updating = False
        self.ik_var.set(self.arm.last_ik_message if result.ok else f"IK falhou: {result.message}")
        self._redraw()
        self._push_hardware(silent=True)

    def _on_joint(self, *_):
        if self._updating:
            return
        q = JointAngles(
            a1=math.radians(self.vars["j_a1"].get()),
            a2=math.radians(self.vars["j_a2"].get()),
            a3=math.radians(self.vars["j_a3"].get()),
            a4=math.radians(self.vars["j_a4"].get()),
            gripper=math.radians(self.vars["j_gripper"].get()),
        )
        self.arm.set_joints(q)
        p = self.arm.pose
        self._updating = True
        try:
            self.vars["x"].set(p.x)
            self.vars["y"].set(p.y)
            self.vars["z"].set(p.z)
            self.vars["tool_pitch"].set(math.degrees(p.tool_pitch))
            self.vars["gripper"].set(math.degrees(q.gripper))
        finally:
            self._updating = False
        self.ik_var.set("Modo articular a1..a4")
        self._redraw()
        self._push_hardware(silent=True)

    def _on_gripper(self, *_):
        if self._updating:
            return
        self.arm.set_gripper(math.radians(self.vars["gripper"].get()))
        self._updating = True
        try:
            self.vars["j_gripper"].set(self.vars["gripper"].get())
        finally:
            self._updating = False
        self._redraw()
        self._push_hardware(silent=True)

    def _home(self):
        self.arm.set_joints(
            JointAngles(
                a1=0.0,
                a2=math.radians(45),
                a3=math.radians(-60),
                a4=math.radians(15),
                gripper=0.0,
            )
        )
        self._sync_sliders_from_arm()
        self.ik_var.set("Home")
        self._redraw()
        self._push_hardware(silent=True)

    def _reconnect(self):
        try:
            try:
                self.backend.disconnect()
            except Exception:
                pass
            self.backend = create_backend(
                mode=self.mode_var.get(),
                wifi_url=self.wifi_url_var.get().strip(),
                serial_port=self.serial_port_var.get(),
                serial_baud=int(self.serial_baud_var.get()),
            )
            self.backend.connect()
            messagebox.showinfo("Ligação", f"Ligado: {self.backend.label}")
            self._update_status()
        except Exception as exc:
            messagebox.showerror("Ligação", str(exc))

    def _push_hardware(self, silent: bool = False):
        cmds = self.arm.servo_command_list()
        try:
            self.backend.send_positions(cmds)
            self._update_status()
        except Exception as exc:
            if not silent:
                messagebox.showerror("Envio", str(exc))
            self.status_var.set(f"Erro envio: {exc}")

    def _update_status(self):
        state = "ligado" if self.backend.connected else "desligado"
        self.status_var.set(f"{self.backend.label} ({state})")
        p = self.arm.pose
        self.pose_var.set(
            f"Pose: X={p.x:.1f} Y={p.y:.1f} Z={p.z:.1f}  pitch={math.degrees(p.tool_pitch):.1f}°"
        )

    def _redraw(self):
        self.ax.clear()
        pts = self.arm.link_points()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]

        self.ax.scatter([0], [0], [0], c="#888", s=40)
        self.ax.plot(xs, ys, zs, "-o", color="#1f6feb", linewidth=3, markersize=7)
        labels = ["base", "ombro", "cotovelo", "pulso", "garra"]
        for i, (x, y, z) in enumerate(pts):
            self.ax.text(x, y, z, f"  {labels[i]}", fontsize=8)

        self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="#e5534b", s=80)
        g = self.arm.joints.gripper
        open_mm = 25.0 * (1.0 - g / (math.pi / 2.0 + 1e-9))
        a1 = self.arm.joints.a1
        dx = -math.sin(a1) * open_mm
        dy = math.cos(a1) * open_mm
        self.ax.plot(
            [xs[-1] - dx, xs[-1] + dx],
            [ys[-1] - dy, ys[-1] + dy],
            [zs[-1], zs[-1]],
            color="#e5534b",
            linewidth=2,
        )

        w = self.workspace
        self.ax.plot([-w, w], [0, 0], [0, 0], color="#ddd", linewidth=1)
        self.ax.plot([0, 0], [-w, w], [0, 0], color="#ddd", linewidth=1)
        self.ax.set_xlim(-w, w)
        self.ax.set_ylim(-w, w)
        self.ax.set_zlim(0, w)
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title("Simulação 3D — a1..a4 + garra")
        try:
            self.ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass
        self.canvas.draw_idle()
        self._update_status()


def main():
    root = tk.Tk()
    style = ttk.Style()
    if "vista" in style.theme_names():
        style.theme_use("vista")
    ArmGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()

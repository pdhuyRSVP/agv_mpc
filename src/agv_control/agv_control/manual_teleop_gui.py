#!/usr/bin/env python3
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String


class ManualTeleopNode(Node):
    def __init__(self):
        super().__init__("agv_manual_teleop_gui")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/wheel_vel",
            self._wheel_vel_callback,
            10,
        )
        self.create_subscription(String, "/wheel_vel_raw", self._wheel_raw_callback, 10)

        self._lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self.real_v = 0.0
        self.real_w = 0.0
        self.raw_text = "waiting..."
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.create_timer(0.05, self._publish_current_cmd)

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def set_command(self, linear_x: float, angular_z: float):
        with self._cmd_lock:
            self.cmd_linear_x = float(linear_x)
            self.cmd_angular_z = float(angular_z)

    def _publish_current_cmd(self):
        with self._cmd_lock:
            linear_x = self.cmd_linear_x
            angular_z = self.cmd_angular_z
        self.publish_cmd(linear_x, angular_z)

    def _wheel_vel_callback(self, msg: TwistWithCovarianceStamped):
        with self._lock:
            self.real_v = msg.twist.twist.linear.x
            self.real_w = msg.twist.twist.angular.z

    def _wheel_raw_callback(self, msg: String):
        with self._lock:
            self.raw_text = msg.data

    def snapshot(self):
        with self._lock:
            return self.real_v, self.real_w, self.raw_text


class ManualTeleopGui:
    def __init__(self, node: ManualTeleopNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("AGV Manual Teleop")
        self.root.geometry("440x360")
        self.root.minsize(380, 320)

        self.active_key = None
        self.linear_speed = tk.DoubleVar(value=0.10)
        self.angular_speed = tk.DoubleVar(value=0.10)
        self.real_v_var = tk.StringVar(value="0.000 m/s")
        self.real_w_var = tk.StringVar(value="0.000 rad/s")
        self.raw_var = tk.StringVar(value="waiting...")
        self.status_var = tk.StringVar(value="Released")

        self._build_ui()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._command_loop)
        self.root.after(200, self._status_loop)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        speed_frame = ttk.LabelFrame(outer, text="Command speed", padding=10)
        speed_frame.pack(fill="x")

        ttk.Label(speed_frame, text="Linear").grid(row=0, column=0, sticky="w")
        ttk.Scale(
            speed_frame,
            from_=0.02,
            to=0.50,
            variable=self.linear_speed,
            orient="horizontal",
        ).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Spinbox(
            speed_frame,
            from_=0.02,
            to=0.50,
            increment=0.01,
            textvariable=self.linear_speed,
            width=6,
            format="%.2f",
        ).grid(row=0, column=2)
        ttk.Label(speed_frame, text="m/s").grid(row=0, column=3, padx=(4, 0))

        ttk.Label(speed_frame, text="Angular").grid(row=1, column=0, sticky="w")
        ttk.Scale(
            speed_frame,
            from_=0.02,
            to=1.00,
            variable=self.angular_speed,
            orient="horizontal",
        ).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Spinbox(
            speed_frame,
            from_=0.02,
            to=1.00,
            increment=0.01,
            textvariable=self.angular_speed,
            width=6,
            format="%.2f",
        ).grid(row=1, column=2)
        ttk.Label(speed_frame, text="rad/s").grid(row=1, column=3, padx=(4, 0))
        speed_frame.columnconfigure(1, weight=1)

        pad = {"padx": 6, "pady": 6, "sticky": "nsew"}
        buttons = ttk.Frame(outer, padding=(0, 12, 0, 4))
        buttons.pack(fill="both", expand=True)
        for idx in range(3):
            buttons.columnconfigure(idx, weight=1)
            buttons.rowconfigure(idx, weight=1)

        self._make_hold_button(buttons, "Tien (i)", "i").grid(row=0, column=1, **pad)
        self._make_hold_button(buttons, "Xoay trai (j)", "j").grid(row=1, column=0, **pad)
        stop = ttk.Button(buttons, text="Dung", command=self.stop)
        stop.grid(row=1, column=1, **pad)
        self._make_hold_button(buttons, "Xoay phai (l)", "l").grid(row=1, column=2, **pad)
        self._make_hold_button(buttons, "Lui (k)", "k").grid(row=2, column=1, **pad)

        actual = ttk.LabelFrame(outer, text="Driver feedback", padding=10)
        actual.pack(fill="x", pady=(8, 0))
        ttk.Label(actual, text="Linear").grid(row=0, column=0, sticky="w")
        ttk.Label(actual, textvariable=self.real_v_var).grid(row=0, column=1, sticky="e")
        ttk.Label(actual, text="Angular").grid(row=1, column=0, sticky="w")
        ttk.Label(actual, textvariable=self.real_w_var).grid(row=1, column=1, sticky="e")
        ttk.Label(actual, text="Raw RPM").grid(row=2, column=0, sticky="w")
        ttk.Label(actual, textvariable=self.raw_var).grid(row=2, column=1, sticky="e")
        actual.columnconfigure(1, weight=1)

        ttk.Label(outer, textvariable=self.status_var).pack(anchor="w", pady=(8, 0))

    def _make_hold_button(self, parent, text, key):
        button = ttk.Button(parent, text=text)
        button.bind("<ButtonPress-1>", lambda _event: self.start(key))
        button.bind("<ButtonRelease-1>", lambda _event: self.stop())
        button.bind("<Leave>", lambda _event: self.stop())
        return button

    def _bind_keys(self):
        for key in ("i", "k", "j", "l"):
            self.root.bind(f"<KeyPress-{key}>", lambda _event, k=key: self.start(k))
            self.root.bind(f"<KeyRelease-{key}>", lambda _event, k=key: self._release_key(k))
        self.root.bind("<space>", lambda _event: self.stop())

    def start(self, key):
        self.active_key = key
        self.node.set_command(*self._current_command())
        self.status_var.set(f"Holding {key}")

    def stop(self):
        self.active_key = None
        self.node.set_command(0.0, 0.0)
        self.node.publish_cmd(0.0, 0.0)
        self.status_var.set("Released")

    def _release_key(self, key):
        if self.active_key == key:
            self.stop()

    def _current_command(self):
        v = self._finite_positive(self.linear_speed.get(), 0.10)
        w = self._finite_positive(self.angular_speed.get(), 0.10)
        if self.active_key == "i":
            return v, 0.0
        if self.active_key == "k":
            return -v, 0.0
        if self.active_key == "j":
            return 0.0, w
        if self.active_key == "l":
            return 0.0, -w
        return 0.0, 0.0

    @staticmethod
    def _finite_positive(value, fallback):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return fallback
        if not math.isfinite(value):
            return fallback
        return max(0.0, value)

    def _command_loop(self):
        v, w = self._current_command()
        self.node.set_command(v, w)
        self.root.after(100, self._command_loop)

    def _status_loop(self):
        real_v, real_w, raw = self.node.snapshot()
        self.real_v_var.set(f"{real_v:.3f} m/s")
        self.real_w_var.set(f"{real_w:.3f} rad/s")
        self.raw_var.set(raw)
        self.root.after(200, self._status_loop)

    def _on_close(self):
        self.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ManualTeleopNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        ManualTeleopGui(node).run()
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

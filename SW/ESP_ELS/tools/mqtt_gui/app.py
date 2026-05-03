import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class MqttGuiApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Dreimaskin MQTT Control")
        self.root.geometry("1200x800")

        self.event_queue = queue.Queue()
        self.client = None
        self.connected = False
        self.connecting = False
        self.status_rows = {}
        self.last_speed_steps_per_s = None

        # Data history for plotting (keep last 300 samples)
        self.max_history = 300
        self.timestamps = deque(maxlen=self.max_history)
        self.speed_history = deque(maxlen=self.max_history)
        self.position_history = deque(maxlen=self.max_history)
        self.target_history = deque(maxlen=self.max_history)
        self.distance_history = deque(maxlen=self.max_history)
        self.sample_time = time.time()

        self._build_ui()
        self._poll_events()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        conn_frame = ttk.LabelFrame(main, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X)

        self.broker_var = tk.StringVar(value="192.168.1.92")
        self.port_var = tk.StringVar(value="1883")
        self.client_id_var = tk.StringVar(value=f"dreimaskin_gui_{int(time.time())}")
        self.base_topic_var = tk.StringVar(value="dreimaskin_els")

        ttk.Label(conn_frame, text="Broker").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(conn_frame, textvariable=self.broker_var, width=22).grid(row=0, column=1, padx=6)

        ttk.Label(conn_frame, text="Port").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(conn_frame, textvariable=self.port_var, width=8).grid(row=0, column=3, padx=6)

        ttk.Label(conn_frame, text="Client ID").grid(row=0, column=4, sticky=tk.W)
        ttk.Entry(conn_frame, textvariable=self.client_id_var, width=24).grid(row=0, column=5, padx=6)

        ttk.Label(conn_frame, text="Base topic").grid(row=1, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Entry(conn_frame, textvariable=self.base_topic_var, width=22).grid(row=1, column=1, padx=6, pady=(8, 0))

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=1, column=4, pady=(8, 0), sticky=tk.EW)

        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.grid(row=1, column=5, pady=(8, 0), sticky=tk.EW)

        control_frame = ttk.LabelFrame(main, text="Control", padding=10)
        control_frame.pack(fill=tk.X, pady=(10, 0))

        self.target_var = tk.StringVar(value="0")
        ttk.Label(control_frame, text="Target").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(control_frame, textvariable=self.target_var, width=12).grid(row=0, column=1, padx=6)
        self.send_target_btn = ttk.Button(control_frame, text="Send Target", command=self.send_target, state=tk.DISABLED)
        self.send_target_btn.grid(row=0, column=2)

        self.mode_var = tk.StringVar(value="position")
        ttk.Label(control_frame, text="Mode").grid(row=0, column=3, sticky=tk.W, padx=(20, 0))
        mode_box = ttk.Combobox(control_frame, textvariable=self.mode_var, values=["position", "follow"], width=12, state="readonly")
        mode_box.grid(row=0, column=4, padx=6)
        self.send_mode_btn = ttk.Button(control_frame, text="Send Mode", command=self.send_mode, state=tk.DISABLED)
        self.send_mode_btn.grid(row=0, column=5)

        self.steps_per_rev_var = tk.StringVar(value="1000")
        self.steps_per_rev_var.trace_add("write", self._on_steps_per_rev_changed)
        ttk.Label(control_frame, text="Steps/Rev").grid(row=1, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Entry(control_frame, textvariable=self.steps_per_rev_var, width=12).grid(row=1, column=1, padx=6, pady=(8, 0))

        self.mm_per_rev_var = tk.StringVar(value="1.00000")
        ttk.Label(control_frame, text="mm/rev").grid(row=2, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Entry(control_frame, textvariable=self.mm_per_rev_var, width=12).grid(row=2, column=1, padx=6, pady=(8, 0))
        self.send_pitch_btn = ttk.Button(control_frame, text="Send Pitch", command=self.send_pitch, state=tk.DISABLED)
        self.send_pitch_btn.grid(row=2, column=2, pady=(8, 0))

        self.stepper_enable_btn = ttk.Button(
            control_frame,
            text="Enable Stepper",
            command=lambda: self.send_stepper_enable(True),
            state=tk.DISABLED,
        )
        self.stepper_enable_btn.grid(row=1, column=2, pady=(8, 0))

        self.stepper_disable_btn = ttk.Button(
            control_frame,
            text="Disable Stepper",
            command=lambda: self.send_stepper_enable(False),
            state=tk.DISABLED,
        )
        self.stepper_disable_btn.grid(row=1, column=3, pady=(8, 0))

        status_frame = ttk.LabelFrame(main, text="Live Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(10, 0))

        self.position_var = tk.StringVar(value="-")
        self.speed_var = tk.StringVar(value="-")
        self.target_status_var = tk.StringVar(value="-")
        self.distance_to_target_var = tk.StringVar(value="-")
        self.mode_status_var = tk.StringVar(value="-")
        self.rpm_var = tk.StringVar(value="-")
        self.alarm_var = tk.StringVar(value="-")
        self.stepper_status_var = tk.StringVar(value="Unknown")
        self.loop_time_var = tk.StringVar(value="-")
        self.batch_time_var = tk.StringVar(value="-")
        self.last_update_var = tk.StringVar(value="-")
        self.alive_var = tk.StringVar(value="-")
        self.conn_state_var = tk.StringVar(value="Disconnected")

        highlights_frame = ttk.Frame(status_frame)
        highlights_frame.grid(row=0, column=0, columnspan=2, sticky=tk.EW, pady=(0, 10))
        for column in range(3):
            highlights_frame.columnconfigure(column, weight=1)

        self._highlight_card(highlights_frame, 0, "Position", self.position_var)
        self._highlight_card(highlights_frame, 1, "Target", self.target_status_var)
        self._highlight_card(highlights_frame, 2, "Distance to Target", self.distance_to_target_var)

        self._status_row(status_frame, 1, "Connection", self.conn_state_var)
        self._status_row(status_frame, 2, "Speed", self.speed_var)
        self._status_row(status_frame, 3, "Mode", self.mode_status_var)
        self._status_row(status_frame, 4, "RPM", self.rpm_var)
        self._status_row(status_frame, 5, "Alarm", self.alarm_var)
        self._status_row(status_frame, 6, "Stepper", self.stepper_status_var)
        self._status_row(status_frame, 7, "Loop Time (us)", self.loop_time_var)
        self._status_row(status_frame, 8, "Batch Time (us)", self.batch_time_var)
        self._status_row(status_frame, 9, "Alive", self.alive_var)
        self._status_row(status_frame, 10, "Last Update", self.last_update_var)

        all_status_frame = ttk.LabelFrame(main, text="All MQTT Status Topics", padding=10)
        all_status_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.status_tree = ttk.Treeview(all_status_frame, columns=("topic", "value"), show="headings", height=8)
        self.status_tree.heading("topic", text="Topic Suffix")
        self.status_tree.heading("value", text="Value")
        self.status_tree.column("topic", width=260, anchor=tk.W)
        self.status_tree.column("value", width=260, anchor=tk.W)
        self.status_tree.pack(fill=tk.BOTH, expand=True)

        # Create plot frames
        plots_frame = ttk.LabelFrame(main, text="Graphs", padding=10)
        plots_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        plots_frame.columnconfigure(0, weight=1)
        plots_frame.columnconfigure(1, weight=1)

        # Speed plot
        self.fig_speed = Figure(figsize=(4.5, 3), dpi=100)
        self.ax_speed = self.fig_speed.add_subplot(111)
        self.ax_speed.set_title("Speed")
        self.ax_speed.set_xlabel("Time (samples)")
        self.ax_speed.set_ylabel("Speed")
        self.fig_speed.tight_layout()
        self.canvas_speed = FigureCanvasTkAgg(self.fig_speed, master=plots_frame)
        self.canvas_speed.get_tk_widget().grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 5))

        # Position, Target, Distance plot
        self.fig_motion = Figure(figsize=(4.5, 3), dpi=100)
        self.ax_motion = self.fig_motion.add_subplot(111)
        self.ax_motion.set_title("Position, Target, Distance to Target")
        self.ax_motion.set_xlabel("Time (samples)")
        self.ax_motion.set_ylabel("Units")
        self.fig_motion.tight_layout()
        self.canvas_motion = FigureCanvasTkAgg(self.fig_motion, master=plots_frame)
        self.canvas_motion.get_tk_widget().grid(row=0, column=1, sticky=tk.NSEW, padx=(5, 0))

        log_frame = ttk.LabelFrame(main, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=12, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _status_row(self, frame, row, name, var):
        ttk.Label(frame, text=f"{name}:", width=12).grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(frame, textvariable=var).grid(row=row, column=1, sticky=tk.W, pady=2)

    def _highlight_card(self, frame, column, title, var):
        card = ttk.LabelFrame(frame, text=title, padding=10)
        card.grid(row=0, column=column, padx=4, sticky=tk.EW)
        ttk.Label(card, textvariable=var, anchor=tk.CENTER, font=("TkDefaultFont", 16, "bold")).pack(fill=tk.X)

    def _update_plots(self):
        """Redraw both graphs with current history data."""
        # Update speed plot
        self.ax_speed.clear()
        if self.speed_history:
            self.ax_speed.plot(list(self.speed_history), label="Speed", color="blue", linewidth=1.5)
            self.ax_speed.legend()
        self.ax_speed.set_title("Speed")
        self.ax_speed.set_xlabel("Time (samples)")
        self.ax_speed.set_ylabel("Speed")
        self.ax_speed.grid(True, alpha=0.3)
        self.fig_speed.tight_layout()
        self.canvas_speed.draw_idle()

        # Update motion plot
        self.ax_motion.clear()
        if self.position_history:
            self.ax_motion.plot(list(self.position_history), label="Position", color="green", linewidth=1.5)
        if self.target_history:
            self.ax_motion.plot(list(self.target_history), label="Target", color="red", linewidth=1.5, linestyle="--")
        if self.distance_history:
            self.ax_motion.plot(list(self.distance_history), label="Distance to Target", color="orange", linewidth=1.5, linestyle=":")
        if self.position_history or self.target_history or self.distance_history:
            self.ax_motion.legend()
        self.ax_motion.set_title("Position, Target, Distance to Target")
        self.ax_motion.set_xlabel("Time (samples)")
        self.ax_motion.set_ylabel("Units")
        self.ax_motion.grid(True, alpha=0.3)
        self.fig_motion.tight_layout()
        self.canvas_motion.draw_idle()

    def log(self, msg):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def topics(self):
        base = self.base_topic_var.get().strip().rstrip("/")
        return {
            "cmd_target": f"{base}/command",
            "cmd_mode": f"{base}/command/mode",
            "cmd_pitch": f"{base}/command/pitch",
            "cmd_stepper": f"{base}/command",
            "status_pos": f"{base}/status/position",
            "status_speed": f"{base}/status/speed",
            "status_target": f"{base}/status/target",
            "status_distance": f"{base}/status/distance_to_target",
            "status_mode": f"{base}/status/mode",
            "status_alarm": f"{base}/status/alarm",
            "status_loop_time": f"{base}/status/loop_time_us",
            "status_batch_time": f"{base}/status/batch_time_us",
            "status_alive": f"{base}/status",
            "status_all": f"{base}/status/#",
        }

    def connect(self):
        if self.connected or self.connecting:
            return

        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Port", "Port must be an integer")
            return

        broker = self.broker_var.get().strip()
        client_id = self.client_id_var.get().strip()
        if not broker or not client_id:
            messagebox.showerror("Missing Value", "Broker and Client ID are required")
            return

        self.connecting = True
        self.conn_state_var.set("Connecting...")
        self.log(f"Connecting to {broker}:{port} ...")

        client_kwargs = {"client_id": client_id, "clean_session": True}
        if hasattr(mqtt, "CallbackAPIVersion"):
            client_kwargs["callback_api_version"] = mqtt.CallbackAPIVersion.VERSION2
        self.client = mqtt.Client(**client_kwargs)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        def worker():
            try:
                self.client.connect(broker, port, keepalive=30)
                self.client.loop_start()
            except Exception as exc:
                self.event_queue.put(("connect_error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def disconnect(self):
        if not self.client:
            return
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception as exc:
            self.log(f"Disconnect error: {exc}")

    def send_target(self):
        if not self.connected or not self.client:
            return
        value = self.target_var.get().strip()
        try:
            int(value)
        except ValueError:
            messagebox.showerror("Invalid Target", "Target must be an integer")
            return

        t = self.topics()
        ok = self.client.publish(t["cmd_target"], value, qos=0, retain=False)
        if ok.rc == mqtt.MQTT_ERR_SUCCESS:
            self.target_status_var.set(value)
            self.log(f"Published target: {value}")
        else:
            self.log("Failed to publish target")

    def send_mode(self):
        if not self.connected or not self.client:
            return
        mode = self.mode_var.get().strip()
        if mode not in ("position", "follow"):
            messagebox.showerror("Invalid Mode", "Mode must be position or follow")
            return

        t = self.topics()
        ok = self.client.publish(t["cmd_mode"], mode, qos=0, retain=False)
        if ok.rc == mqtt.MQTT_ERR_SUCCESS:
            self.log(f"Published mode: {mode}")
        else:
            self.log("Failed to publish mode")

    def send_stepper_enable(self, enable):
        if not self.connected or not self.client:
            return

        payload = "enable stepper" if enable else "disable stepper"
        t = self.topics()
        ok = self.client.publish(t["cmd_stepper"], payload, qos=0, retain=False)
        if ok.rc == mqtt.MQTT_ERR_SUCCESS:
            self.stepper_status_var.set("Enabled" if enable else "Disabled")
            self.log(f"Published stepper command: {payload}")
        else:
            self.log("Failed to publish stepper command")

    def send_pitch(self):
        if not self.connected or not self.client:
            return

        pitch_text = self.mm_per_rev_var.get().strip()
        pitch_value = self._extract_number(pitch_text)
        if pitch_value is None or pitch_value <= 0:
            messagebox.showerror("Invalid Pitch", "mm/rev must be a positive number")
            return

        # Keep a stable precision so firmware parsers can safely scale to 1e5 if needed.
        payload = f"{pitch_value:.5f}"
        t = self.topics()
        ok = self.client.publish(t["cmd_pitch"], payload, qos=0, retain=False)
        if ok.rc == mqtt.MQTT_ERR_SUCCESS:
            self.mm_per_rev_var.set(payload)
            self.log(f"Published pitch (mm/rev): {payload}")
        else:
            self.log("Failed to publish pitch command")

    def _update_status_tree(self, suffix, payload):
        item_id = self.status_rows.get(suffix)
        if item_id is None:
            self.status_rows[suffix] = self.status_tree.insert("", tk.END, values=(suffix, payload))
        else:
            self.status_tree.item(item_id, values=(suffix, payload))

    def _update_speed_derived(self, payload):
        speed_steps_per_s = self._extract_number(payload)
        if speed_steps_per_s is None:
            self.rpm_var.set("-")
            return

        self.last_speed_steps_per_s = speed_steps_per_s

        steps_per_rev = self._extract_number(self.steps_per_rev_var.get())
        if steps_per_rev is None or steps_per_rev <= 0:
            self.rpm_var.set("-")
            return

        rpm = (speed_steps_per_s * 60.0) / steps_per_rev
        self.rpm_var.set(f"{rpm:.1f}")

    def _extract_number(self, text):
        try:
            return float(str(text).strip())
        except ValueError:
            match = re.search(r"[-+]?\d*\.?\d+", str(text))
            return float(match.group(0)) if match else None

    def _on_steps_per_rev_changed(self, *_):
        if self.last_speed_steps_per_s is not None:
            self._update_speed_derived(str(self.last_speed_steps_per_s))

    def _update_last_update_time(self):
        self.last_update_var.set(time.strftime("%H:%M:%S"))

    def _reason_code_to_int(self, reason_code):
        try:
            return int(reason_code)
        except (TypeError, ValueError):
            return -1

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        self.event_queue.put(("connected", self._reason_code_to_int(reason_code)))

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties=None):
        self.event_queue.put(("disconnected", self._reason_code_to_int(reason_code)))

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode(errors="replace")
        self.event_queue.put(("message", msg.topic, payload))

    def _poll_events(self):
        while True:
            try:
                evt = self.event_queue.get_nowait()
            except queue.Empty:
                break

            kind = evt[0]
            if kind == "connect_error":
                self.connecting = False
                self.connected = False
                self.conn_state_var.set("Disconnected")
                self.log(f"Connect error: {evt[1]}")
            elif kind == "connected":
                self.connecting = False
                rc = evt[1]
                if rc == 0:
                    self.connected = True
                    self.conn_state_var.set("Connected")
                    self.connect_btn.configure(state=tk.DISABLED)
                    self.disconnect_btn.configure(state=tk.NORMAL)
                    self.send_target_btn.configure(state=tk.NORMAL)
                    self.send_mode_btn.configure(state=tk.NORMAL)
                    self.send_pitch_btn.configure(state=tk.NORMAL)
                    self.stepper_enable_btn.configure(state=tk.NORMAL)
                    self.stepper_disable_btn.configure(state=tk.NORMAL)
                    self.log("Connected")

                    t = self.topics()
                    for topic in (t["status_all"],):
                        self.client.subscribe(topic)
                        self.log(f"Subscribed: {topic}")
                else:
                    self.connected = False
                    self.conn_state_var.set(f"Connect failed (rc={rc})")
                    self.log(f"Connect failed rc={rc}")
            elif kind == "disconnected":
                self.connecting = False
                self.connected = False
                self.conn_state_var.set("Disconnected")
                self.connect_btn.configure(state=tk.NORMAL)
                self.disconnect_btn.configure(state=tk.DISABLED)
                self.send_target_btn.configure(state=tk.DISABLED)
                self.send_mode_btn.configure(state=tk.DISABLED)
                self.send_pitch_btn.configure(state=tk.DISABLED)
                self.stepper_enable_btn.configure(state=tk.DISABLED)
                self.stepper_disable_btn.configure(state=tk.DISABLED)
                self.log("Disconnected")
            elif kind == "message":
                topic = evt[1]
                payload = evt[2]
                t = self.topics()
                plots_updated = False
                self._update_last_update_time()

                base_status = t["status_alive"]
                if topic == base_status:
                    suffix = "alive"
                elif topic.startswith(base_status + "/"):
                    suffix = topic[len(base_status) + 1 :]
                else:
                    suffix = topic

                self._update_status_tree(suffix, payload)

                if suffix == "position":
                    self.position_var.set(payload)
                    try:
                        self.position_history.append(float(payload))
                        plots_updated = True
                    except ValueError:
                        pass
                elif suffix == "speed":
                    self.speed_var.set(payload)
                    self._update_speed_derived(payload)
                    speed_value = self._extract_number(payload)
                    try:
                        if speed_value is None:
                            raise ValueError
                        self.speed_history.append(float(speed_value))
                        plots_updated = True
                    except ValueError:
                        pass
                elif suffix == "target":
                    self.target_status_var.set(payload)
                    try:
                        self.target_history.append(float(payload))
                        plots_updated = True
                    except ValueError:
                        pass
                elif suffix == "distance_to_target":
                    self.distance_to_target_var.set(payload)
                    try:
                        self.distance_history.append(float(payload))
                        plots_updated = True
                    except ValueError:
                        pass
                elif suffix == "mode":
                    self.mode_status_var.set(payload)
                elif suffix == "alarm":
                    self.alarm_var.set(payload)
                elif suffix == "loop_time_us":
                    self.loop_time_var.set(payload)
                elif suffix == "batch_time_us":
                    self.batch_time_var.set(payload)
                elif suffix == "alive":
                    self.alive_var.set(payload)

                if plots_updated:
                    self._update_plots()

        self.root.after(50, self._poll_events)


def main():
    root = tk.Tk()
    app = MqttGuiApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()

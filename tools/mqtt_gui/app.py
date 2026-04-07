import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import paho.mqtt.client as mqtt


class MqttGuiApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Dreimaskin MQTT Control")
        self.root.geometry("820x560")

        self.event_queue = queue.Queue()
        self.client = None
        self.connected = False
        self.connecting = False

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

        status_frame = ttk.LabelFrame(main, text="Live Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(10, 0))

        self.position_var = tk.StringVar(value="-")
        self.speed_var = tk.StringVar(value="-")
        self.target_status_var = tk.StringVar(value="-")
        self.distance_to_target_var = tk.StringVar(value="-")
        self.mode_status_var = tk.StringVar(value="-")
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
        self._status_row(status_frame, 4, "Alive", self.alive_var)

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
            "status_pos": f"{base}/status/position",
            "status_speed": f"{base}/status/speed",
            "status_target": f"{base}/status/target",
            "status_distance": f"{base}/status/distance_to_target",
            "status_mode": f"{base}/status/mode",
            "status_alive": f"{base}/status",
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

        self.client = mqtt.Client(client_id=client_id, clean_session=True)
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

    def _on_connect(self, client, userdata, flags, rc):
        self.event_queue.put(("connected", rc))

    def _on_disconnect(self, client, userdata, rc):
        self.event_queue.put(("disconnected", rc))

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
                    self.log("Connected")

                    t = self.topics()
                    for topic in (
                        t["status_pos"],
                        t["status_speed"],
                        t["status_target"],
                        t["status_distance"],
                        t["status_mode"],
                        t["status_alive"],
                    ):
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
                self.log("Disconnected")
            elif kind == "message":
                topic = evt[1]
                payload = evt[2]
                t = self.topics()

                if topic == t["status_pos"]:
                    self.position_var.set(payload)
                elif topic == t["status_speed"]:
                    self.speed_var.set(payload)
                elif topic == t["status_target"]:
                    self.target_status_var.set(payload)
                elif topic == t["status_distance"]:
                    self.distance_to_target_var.set(payload)
                elif topic == t["status_mode"]:
                    self.mode_status_var.set(payload)
                elif topic == t["status_alive"]:
                    self.alive_var.set(payload)

        self.root.after(50, self._poll_events)


def main():
    root = tk.Tk()
    app = MqttGuiApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()

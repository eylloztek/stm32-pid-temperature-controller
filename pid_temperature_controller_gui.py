import tkinter as tk
from tkinter import ttk, messagebox

import math
import queue
import re
import threading
from collections import deque

import serial
import serial.tools.list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter, MaxNLocator


class PIDTemperatureControllerGUI:
    UART_VALUE_PATTERN = re.compile(
        r"([A-Za-z][A-Za-z0-9_ °C%/-]*)\s*[:=]\s*"
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    )

    UART_MODE_PATTERN = re.compile(
        r"\bMode\s*[:=]\s*([A-Za-z_ -]+)",
        re.IGNORECASE
    )

    PID_AUTOTUNE_RULES = {
        "Ziegler-Nichols": {
            "rule": "ziegler-nichols",
            "kp": 7.66141138481997,
            "ki": 0.4086086071903984,
            "kd": 35.91286586634361
        },
        "Tyreus-Luyben": {
            "rule": "tyreus-luyben",
            "kp": 5.920181524633613,
            "ki": 0.07104217829560336,
            "kd": 35.239175741866745
        },
        "Ciancone-Marlin": {
            "rule": "ciancone-marlin",
            "kp": 3.9467876830890756,
            "ki": 0.46308975481578485,
            "kd": 18.272165199486462
        },
        "Pessen Integral": {
            "rule": "pessen-integral",
            "kp": 9.30314239585282,
            "ki": 0.6202094930568547,
            "kd": 52.46132929992192
        },
        "Some Overshoot": {
            "rule": "some-overshoot",
            "kp": 4.341466451397983,
            "ki": 0.2315448774078924,
            "kd": 54.26833064247478
        },
        "No Overshoot": {
            "rule": "no-overshoot",
            "kp": 2.60487987083879,
            "ki": 0.13892692644473545,
            "kd": 32.56099838548487
        },
        "Brewing": {
            "rule": "brewing",
            "kp": 104.1951948335516,
            "ki": 0.8335615586684127,
            "kd": 205.64841085569392
        }
    }

    def __init__(self, root):
        self.root = root
        self.root.title("STM32 PID Temperature Controller")
        self.root.geometry("1220x900")
        self.root.minsize(1100, 760)
        self.root.resizable(True, True)

        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.plot_enabled = False
        self.rx_queue = queue.Queue()

        self.max_points = 300
        self.time_data = deque(maxlen=self.max_points)
        self.temperature_data = deque(maxlen=self.max_points)
        self.setpoint_data = deque(maxlen=self.max_points)
        self.pid_output_data = deque(maxlen=self.max_points)

        self.sample_index = 0
        self.sample_time_s = 1.0

        self.current_setpoint = 25.0
        self.current_temperature = None
        self.current_humidity = None
        self.current_pressure = None
        self.current_mode = "COOLING"
        self.current_autotune_rule = None

        self.temperature_var = tk.StringVar(value="-- °C")
        self.humidity_var = tk.StringVar(value="-- %")
        self.pressure_var = tk.StringVar(value="-- hPa")
        self.pid_output_var = tk.StringVar(value="-- %")
        self.mode_var = tk.StringVar(value="COOLING")
        self.autotune_info_var = tk.StringVar(
            value="Select a rule to fill Kp, Ki and Kd."
        )

        self.rise_time_var = tk.StringVar(value="--")
        self.settling_time_var = tk.StringVar(value="--")
        self.overshoot_var = tk.StringVar(value="--")
        self.steady_state_error_var = tk.StringVar(value="--")

        self.latched_rise_time = None
        self.latched_settling_time = None
        self.latched_overshoot_percent = None
        self.latched_overshoot_value = None
        self.latest_steady_state_error = None

        self.create_widgets()
        self.refresh_ports()
        self.update_plot()
        self.process_serial_queue()

    def create_widgets(self):
        main_frame = tk.Frame(self.root, bg="#d7e8f6")
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_container = tk.Frame(main_frame, bg="#d7e8f6", width=390)
        left_container.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)
        left_container.pack_propagate(False)

        left_canvas = tk.Canvas(
            left_container,
            bg="#d7e8f6",
            highlightthickness=0,
            width=370
        )
        left_scrollbar = ttk.Scrollbar(
            left_container,
            orient=tk.VERTICAL,
            command=left_canvas.yview
        )
        left_frame = tk.Frame(left_canvas, bg="#d7e8f6")

        left_window = left_canvas.create_window(
            (0, 0),
            window=left_frame,
            anchor="nw"
        )

        def update_left_scroll_region(_event=None):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))

        def resize_left_window(event):
            left_canvas.itemconfigure(left_window, width=event.width)

        left_frame.bind("<Configure>", update_left_scroll_region)
        left_canvas.bind("<Configure>", resize_left_window)
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        right_frame = tk.Frame(main_frame, bg="#d7e8f6")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=15, pady=15)

        connection_frame = tk.LabelFrame(
            left_frame,
            text="Connection",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        connection_frame.pack(fill=tk.X, pady=5)

        tk.Label(connection_frame, text="COM Port:", bg="#d7e8f6").grid(
            row=0,
            column=0,
            sticky="w"
        )

        self.port_combo = ttk.Combobox(
            connection_frame,
            width=16,
            state="readonly"
        )
        self.port_combo.grid(row=0, column=1, padx=5, pady=4)

        self.refresh_button = tk.Button(
            connection_frame,
            text="⟳",
            width=3,
            command=self.refresh_ports
        )
        self.refresh_button.grid(row=0, column=2, padx=5)

        tk.Label(connection_frame, text="Baudrate:", bg="#d7e8f6").grid(
            row=1,
            column=0,
            sticky="w"
        )

        self.baud_combo = ttk.Combobox(
            connection_frame,
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600"
            ],
            width=16,
            state="readonly"
        )
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=1, column=1, padx=5, pady=4)

        tk.Label(connection_frame, text="StopBits:", bg="#d7e8f6").grid(
            row=2,
            column=0,
            sticky="w"
        )

        self.stopbits_combo = ttk.Combobox(
            connection_frame,
            values=["One", "Two"],
            width=16,
            state="readonly"
        )
        self.stopbits_combo.set("One")
        self.stopbits_combo.grid(row=2, column=1, padx=5, pady=4)

        tk.Label(connection_frame, text="Parity:", bg="#d7e8f6").grid(
            row=3,
            column=0,
            sticky="w"
        )

        self.parity_combo = ttk.Combobox(
            connection_frame,
            values=["None", "Even", "Odd"],
            width=16,
            state="readonly"
        )
        self.parity_combo.set("None")
        self.parity_combo.grid(row=3, column=1, padx=5, pady=4)

        self.connect_button = tk.Button(
            connection_frame,
            text="Connect",
            width=12,
            command=self.connect_serial
        )
        self.connect_button.grid(row=4, column=0, pady=10)

        self.disconnect_button = tk.Button(
            connection_frame,
            text="Disconnect",
            width=12,
            command=self.disconnect_serial
        )
        self.disconnect_button.grid(row=5, column=0, pady=4)

        self.status_label = tk.Label(
            connection_frame,
            text="DISCONNECTED",
            bg="#f2c6c6",
            fg="black",
            width=18
        )
        self.status_label.grid(row=4, column=1, padx=5)

        parameter_frame = tk.LabelFrame(
            left_frame,
            text="PID Parameters",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        parameter_frame.pack(fill=tk.X, pady=8)

        self.setpoint_entry = self.create_parameter_row(
            parameter_frame,
            "Set Point (°C):",
            0,
            self.send_setpoint
        )
        self.kp_entry = self.create_parameter_row(parameter_frame, "Kp:", 1, self.send_kp)
        self.ki_entry = self.create_parameter_row(parameter_frame, "Ki:", 2, self.send_ki)
        self.kd_entry = self.create_parameter_row(parameter_frame, "Kd:", 3, self.send_kd)

        self.setpoint_entry.insert(0, "25.00")
        self.kp_entry.insert(0, "8.0")
        self.ki_entry.insert(0, "0.05")
        self.kd_entry.insert(0, "0.0")

        mode_frame = tk.LabelFrame(
            left_frame,
            text="Control Mode",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        mode_frame.pack(fill=tk.X, pady=4)

        tk.Label(mode_frame, text="Mode:", bg="#d7e8f6").grid(
            row=0,
            column=0,
            sticky="w",
            pady=5
        )

        self.mode_combo = ttk.Combobox(
            mode_frame,
            values=["COOLING", "HEATING"],
            width=16,
            state="readonly"
        )
        self.mode_combo.set("COOLING")
        self.mode_combo.grid(row=0, column=1, padx=8, pady=5)

        self.mode_button = tk.Button(
            mode_frame,
            text="SET MODE",
            width=10,
            command=self.send_mode
        )
        self.mode_button.grid(row=0, column=2, padx=5, pady=5)

        autotune_frame = tk.LabelFrame(
            left_frame,
            text="PID Autotune Presets",
            bg="#d7e8f6",
            padx=10,
            pady=8
        )
        autotune_frame.pack(fill=tk.X, pady=5)

        tk.Label(autotune_frame, text="Rule:", bg="#d7e8f6").grid(
            row=0,
            column=0,
            sticky="w",
            pady=4
        )

        self.autotune_combo = ttk.Combobox(
            autotune_frame,
            values=list(self.PID_AUTOTUNE_RULES.keys()),
            width=18,
            state="readonly"
        )
        self.autotune_combo.set("Select rule")
        self.autotune_combo.grid(row=0, column=1, padx=8, pady=4, sticky="ew")
        self.autotune_combo.bind(
            "<<ComboboxSelected>>",
            self.on_autotune_rule_selected
        )

        self.autotune_send_button = tk.Button(
            autotune_frame,
            text="SEND GAINS",
            width=10,
            command=self.send_autotune_gains
        )
        self.autotune_send_button.grid(row=0, column=2, padx=5, pady=4)

        tk.Label(
            autotune_frame,
            textvariable=self.autotune_info_var,
            bg="#d7e8f6",
            anchor="w",
            justify="left",
            wraplength=330
        ).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 0))

        autotune_frame.grid_columnconfigure(1, weight=1)

        control_frame = tk.Frame(left_frame, bg="#d7e8f6")
        control_frame.pack(fill=tk.X, pady=8)

        self.start_button = tk.Button(
            control_frame,
            text="START",
            width=12,
            command=self.start_pid
        )
        self.start_button.grid(row=0, column=0, padx=8)

        self.stop_button = tk.Button(
            control_frame,
            text="STOP",
            width=12,
            command=self.stop_pid
        )
        self.stop_button.grid(row=0, column=1, padx=8)

        self.clear_button = tk.Button(
            control_frame,
            text="CLEAR GRAPH",
            width=26,
            command=self.clear_graph
        )
        self.clear_button.grid(row=1, column=0, columnspan=2, pady=8)

        live_frame = tk.LabelFrame(
            left_frame,
            text="Live Sensor Data",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        live_frame.pack(fill=tk.X, pady=6)

        self.create_value_row(live_frame, "Temperature:", self.temperature_var, 0)
        self.create_value_row(live_frame, "Humidity:", self.humidity_var, 1)
        self.create_value_row(live_frame, "Pressure:", self.pressure_var, 2)
        self.create_value_row(live_frame, "PID Output:", self.pid_output_var, 3)
        self.create_value_row(live_frame, "Mode:", self.mode_var, 4)

        metrics_frame = tk.LabelFrame(
            left_frame,
            text="Temperature Response Metrics",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        metrics_frame.pack(fill=tk.X, pady=6)

        self.create_value_row(metrics_frame, "Rise Time:", self.rise_time_var, 0)
        self.create_value_row(metrics_frame, "Settling Time:", self.settling_time_var, 1)
        self.create_value_row(metrics_frame, "Overshoot:", self.overshoot_var, 2)
        self.create_value_row(metrics_frame, "Steady State Error:", self.steady_state_error_var, 3)

        graph_frame = tk.LabelFrame(
            right_frame,
            text="Graph",
            bg="#d7e8f6",
            padx=10,
            pady=10
        )
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(7.8, 5.2), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.ax.set_title("PID Temperature Response")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Temperature / Set Point (°C)")
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 320)
        self.ax.grid(True)

        self.ax.yaxis.set_major_locator(MaxNLocator(nbins=8))
        self.ax.yaxis.set_major_formatter(FuncFormatter(self.format_y_axis))

        self.ax_pid = self.ax.twinx()
        self.ax_pid.set_ylabel("PID Output / PWM (%)")
        self.ax_pid.set_ylim(0, 100)

        self.temperature_line, = self.ax.plot(
            [],
            [],
            label="Temperature",
            color="#1f77b4",
            linewidth=1.8
        )

        self.setpoint_line, = self.ax.plot(
            [],
            [],
            label="Set Point",
            color="#ff7f0e",
            linewidth=1.8
        )

        self.pid_output_line, = self.ax_pid.plot(
            [],
            [],
            label="PID Output",
            color="#2ca02c",
            linewidth=1.8
        )

        lines1, labels1 = self.ax.get_legend_handles_labels()
        lines2, labels2 = self.ax_pid.get_legend_handles_labels()
        self.ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.figure, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_value_row(self, parent, label_text, text_variable, row):
        parent.grid_columnconfigure(1, weight=1)

        tk.Label(
            parent,
            text=label_text,
            bg="#d7e8f6",
            anchor="w"
        ).grid(
            row=row,
            column=0,
            sticky="w",
            pady=4
        )

        tk.Label(
            parent,
            textvariable=text_variable,
            bg="#d7e8f6",
            anchor="w",
            justify="left",
            width=24
        ).grid(
            row=row,
            column=1,
            sticky="ew",
            padx=8
        )

    def create_parameter_row(self, parent, label_text, row, command):
        tk.Label(parent, text=label_text, bg="#d7e8f6").grid(
            row=row,
            column=0,
            sticky="w",
            pady=7
        )

        entry = tk.Entry(parent, width=12)
        entry.grid(row=row, column=1, padx=8, pady=7)

        button = tk.Button(parent, text="SET", width=8, command=command)
        button.grid(row=row, column=2, padx=5, pady=7)

        return entry

    def refresh_ports(self):
        previous_selection = self.port_combo.get()

        ports = serial.tools.list_ports.comports()
        port_names = [port.device for port in ports]

        self.port_combo["values"] = port_names

        if previous_selection in port_names:
            self.port_combo.set(previous_selection)
        elif port_names:
            self.port_combo.set(port_names[0])
        else:
            self.port_combo.set("")

    def connect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            return

        port = self.port_combo.get()

        if not port:
            messagebox.showerror("Error", "No COM port selected.")
            return

        try:
            baudrate = int(self.baud_combo.get())

            stopbits = serial.STOPBITS_ONE
            if self.stopbits_combo.get() == "Two":
                stopbits = serial.STOPBITS_TWO

            parity = serial.PARITY_NONE
            if self.parity_combo.get() == "Even":
                parity = serial.PARITY_EVEN
            elif self.parity_combo.get() == "Odd":
                parity = serial.PARITY_ODD

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=parity,
                stopbits=stopbits,
                timeout=0.1
            )

            self.running = True
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()

            self.status_label.config(text="CONNECTED", bg="#bfe6bf")

        except (serial.SerialException, ValueError) as e:
            self.serial_port = None
            self.status_label.config(text="FAILED", bg="#f2c6c6")
            messagebox.showerror("Serial Error", str(e))

    def disconnect_serial(self):
        self.running = False
        self.plot_enabled = False

        if self.serial_port:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except serial.SerialException:
                pass

        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=0.3)

        self.serial_thread = None
        self.serial_port = None

        self.status_label.config(text="DISCONNECTED", bg="#f2c6c6")

    def read_serial(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    line = self.serial_port.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if line:
                        self.rx_queue.put(line)

            except serial.SerialException:
                if self.running:
                    self.rx_queue.put("SERIAL_ERROR")
                break

    def process_serial_queue(self):
        while not self.rx_queue.empty():
            line = self.rx_queue.get()

            if line == "SERIAL_ERROR":
                self.disconnect_serial()
                messagebox.showerror("Serial Error", "Serial connection lost.")
                continue

            parsed = self.parse_uart_data(line)

            if parsed is None:
                continue

            temperature, setpoint, pid_output, humidity, pressure, mode = parsed
            self.update_live_values(temperature, humidity, pressure, pid_output, mode)

            if self.plot_enabled:
                self.sample_index += 1

                self.time_data.append(self.sample_index)
                self.temperature_data.append(temperature)
                self.setpoint_data.append(setpoint)

                if pid_output is None:
                    self.pid_output_data.append(math.nan)
                else:
                    self.pid_output_data.append(pid_output)

                self.update_response_metrics()

        self.root.after(20, self.process_serial_queue)

    def parse_uart_data(self, line):
        matches = self.UART_VALUE_PATTERN.findall(line)

        if not matches:
            print("Invalid UART data:", line)
            return None

        data = {}

        for key, value in matches:
            normalized_key = self.normalize_key(key)

            try:
                data[normalized_key] = float(value)
            except ValueError:
                print("Invalid numeric value:", line)
                return None

        setpoint = self.first_not_none(
            data.get("setpoint"),
            data.get("targettemperature"),
            data.get("target")
        )

        temperature = self.first_not_none(
            data.get("temperature"),
            data.get("temp"),
            data.get("measuredtemperature")
        )

        pid_output = self.first_not_none(
            data.get("pidoutput"),
            data.get("pwm"),
            data.get("fanpwm")
        )

        humidity = data.get("humidity")
        pressure = data.get("pressure")

        mode = self.current_mode
        mode_match = self.UART_MODE_PATTERN.search(line)
        if mode_match:
            mode = self.normalize_mode(mode_match.group(1))

        if setpoint is None or temperature is None:
            print("Missing SetPoint or Temperature:", line)
            return None

        self.current_setpoint = setpoint
        self.current_temperature = temperature
        self.current_humidity = humidity
        self.current_pressure = pressure
        self.current_mode = mode

        return temperature, setpoint, pid_output, humidity, pressure, mode

    def update_live_values(self, temperature, humidity, pressure, pid_output, mode):
        self.temperature_var.set(f"{temperature:.2f} °C")

        if humidity is None:
            self.humidity_var.set("-- %")
        else:
            self.humidity_var.set(f"{humidity:.2f} %")

        if pressure is None:
            self.pressure_var.set("-- hPa")
        else:
            self.pressure_var.set(f"{pressure:.2f} hPa")

        if pid_output is None:
            self.pid_output_var.set("-- %")
        else:
            self.pid_output_var.set(f"{pid_output:.2f} %")

        self.mode_var.set(mode)

    @staticmethod
    def first_not_none(*values):
        for value in values:
            if value is not None:
                return value

        return None

    @staticmethod
    def normalize_key(key):
        return (
            key.strip()
            .lower()
            .replace(" ", "")
            .replace("_", "")
            .replace("-", "")
            .replace("/", "")
            .replace("(", "")
            .replace(")", "")
            .replace("°", "")
            .replace("%", "")
        )

    @staticmethod
    def normalize_mode(mode):
        normalized = mode.strip().upper().replace(" ", "").replace("_", "")

        if "HEAT" in normalized:
            return "HEATING"

        if "COOL" in normalized:
            return "COOLING"

        return normalized

    @staticmethod
    def format_y_axis(value, _):
        abs_value = abs(value)

        if abs_value == 0:
            return "0"

        if abs_value < 0.01:
            return f"{value:.4f}".rstrip("0").rstrip(".")

        if abs_value < 1:
            return f"{value:.3f}".rstrip("0").rstrip(".")

        if abs_value < 10:
            return f"{value:.2f}".rstrip("0").rstrip(".")

        if abs_value < 100:
            return f"{value:.1f}".rstrip("0").rstrip(".")

        return f"{value:.0f}"

    def set_dynamic_ylim(self, axis, values, min_span, margin_ratio=0.15):
        y_min = min(values)
        y_max = max(values)

        center = (y_min + y_max) / 2.0
        span = y_max - y_min

        if span < min_span:
            y_min = center - (min_span / 2.0)
            y_max = center + (min_span / 2.0)
        else:
            margin = span * margin_ratio
            y_min -= margin
            y_max += margin

        axis.set_ylim(y_min, y_max)

    def reset_metrics(self):
        self.latched_rise_time = None
        self.latched_settling_time = None
        self.latched_overshoot_percent = None
        self.latched_overshoot_value = None
        self.latest_steady_state_error = None

        self.rise_time_var.set("--")
        self.settling_time_var.set("--")
        self.overshoot_var.set("--")
        self.steady_state_error_var.set("--")

    def refresh_metric_labels(self):
        if self.latched_rise_time is None:
            self.rise_time_var.set("--")
        else:
            self.rise_time_var.set(f"{self.latched_rise_time:.2f} s")

        if self.latched_settling_time is None:
            self.settling_time_var.set("--")
        else:
            self.settling_time_var.set(f"{self.latched_settling_time:.2f} s")

        if self.latched_overshoot_percent is None:
            self.overshoot_var.set("--")
        else:
            self.overshoot_var.set(
                f"{self.latched_overshoot_percent:.2f}% / "
                f"{self.latched_overshoot_value:.3f} °C"
            )

        if self.latest_steady_state_error is None:
            self.steady_state_error_var.set("--")
        else:
            self.steady_state_error_var.set(
                f"{self.latest_steady_state_error:.3f} °C"
            )

    def update_response_metrics(self):
        metrics = self.calculate_response_metrics()

        if metrics is None:
            self.refresh_metric_labels()
            return

        rise_time = metrics["rise_time"]
        settling_time = metrics["settling_time"]
        overshoot_percent = metrics["overshoot_percent"]
        overshoot_value = metrics["overshoot_value"]
        steady_state_error = metrics["steady_state_error"]
        has_overshoot = metrics["has_overshoot"]

        if self.latched_rise_time is None and rise_time is not None:
            self.latched_rise_time = rise_time

        if self.latched_settling_time is None and settling_time is not None:
            self.latched_settling_time = settling_time
            self.latched_overshoot_percent = overshoot_percent
            self.latched_overshoot_value = overshoot_value

        if self.latched_settling_time is None and has_overshoot:
            self.latched_overshoot_percent = overshoot_percent
            self.latched_overshoot_value = overshoot_value

        self.latest_steady_state_error = steady_state_error

        self.refresh_metric_labels()

    def calculate_response_metrics(self):
        temperature_values = list(self.temperature_data)
        setpoint_values = list(self.setpoint_data)

        if len(temperature_values) < 5 or len(setpoint_values) < 5:
            return None

        initial_value = temperature_values[0]
        final_setpoint = setpoint_values[-1]

        step_amplitude = final_setpoint - initial_value
        abs_step = abs(step_amplitude)

        min_valid_step = max(0.02 * abs(final_setpoint), 0.10)

        rise_time = None
        overshoot_percent = None
        overshoot_value = 0.0
        has_overshoot = False

        if abs_step >= min_valid_step:
            direction = 1.0 if step_amplitude > 0.0 else -1.0

            y_10 = initial_value + 0.10 * step_amplitude
            y_90 = initial_value + 0.90 * step_amplitude

            index_10 = None
            index_90 = None

            for i, value in enumerate(temperature_values):
                if index_10 is None:
                    if direction * (value - y_10) >= 0.0:
                        index_10 = i

                if index_10 is not None:
                    if direction * (value - y_90) >= 0.0:
                        index_90 = i
                        break

            if index_10 is not None and index_90 is not None:
                rise_time = (index_90 - index_10) * self.sample_time_s

            if direction > 0.0:
                peak_value = max(temperature_values)
                overshoot_value = max(0.0, peak_value - final_setpoint)
            else:
                peak_value = min(temperature_values)
                overshoot_value = max(0.0, final_setpoint - peak_value)

            overshoot_percent = (overshoot_value / abs_step) * 100.0

            overshoot_threshold = max(0.05, 0.002 * abs(final_setpoint))
            has_overshoot = overshoot_value > overshoot_threshold

        settling_tolerance = max(0.02 * abs(final_setpoint), 0.20)

        settling_index = None
        settling_required_samples = 10

        if len(temperature_values) >= settling_required_samples:
            for i in range(len(temperature_values) - settling_required_samples + 1):
                remaining_values = temperature_values[i:]

                is_settled = all(
                    abs(value - final_setpoint) <= settling_tolerance
                    for value in remaining_values
                )

                if is_settled:
                    settling_index = i
                    break

        if settling_index is None:
            settling_time = None
        else:
            settling_time = settling_index * self.sample_time_s

        steady_state_window = min(20, len(temperature_values))
        steady_state_average = (
            sum(temperature_values[-steady_state_window:]) / steady_state_window
        )

        steady_state_error = abs(final_setpoint - steady_state_average)

        return {
            "rise_time": rise_time,
            "settling_time": settling_time,
            "overshoot_percent": overshoot_percent,
            "overshoot_value": overshoot_value,
            "has_overshoot": has_overshoot,
            "steady_state_error": steady_state_error
        }

    def update_plot(self):
        self.temperature_line.set_data(self.time_data, self.temperature_data)
        self.setpoint_line.set_data(self.time_data, self.setpoint_data)
        self.pid_output_line.set_data(self.time_data, self.pid_output_data)

        if len(self.time_data) >= 1:
            x_min = self.time_data[0]
            x_max = self.time_data[-1]

            if x_min == x_max:
                self.ax.set_xlim(x_min - 1, x_max + 1)
            else:
                self.ax.set_xlim(x_min, x_max)

            self.ax_pid.set_xlim(self.ax.get_xlim())

            left_values = list(self.temperature_data) + list(self.setpoint_data)
            left_values = [
                value for value in left_values
                if math.isfinite(value)
            ]

            right_values = [
                value for value in self.pid_output_data
                if math.isfinite(value)
            ]

            if left_values:
                self.ax.set_ylim(0, 320)

            if right_values:
                self.set_dynamic_ylim(self.ax_pid, right_values, min_span=10.0)
            else:
                self.ax_pid.set_ylim(0, 100)

            self.ax.yaxis.set_major_locator(MaxNLocator(nbins=8))
            self.ax_pid.yaxis.set_major_locator(MaxNLocator(nbins=8))

        self.canvas.draw_idle()
        self.root.after(50, self.update_plot)

    def send_uart_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Warning", "Serial port is not connected.")
            return False

        try:
            self.serial_port.write(command.encode("utf-8"))
            self.serial_port.flush()
            return True

        except serial.SerialException as e:
            messagebox.showerror("Serial Error", str(e))
            return False

    def send_setpoint(self):
        value = self.get_float_from_entry(self.setpoint_entry, "Set Point")

        if value is None:
            return

        if self.send_uart_command(f"SETPOINT:{value:.2f}\r\n"):
            self.current_setpoint = value
            self.clear_graph()

    def send_kp(self):
        value = self.get_float_from_entry(self.kp_entry, "Kp")

        if value is None:
            return

        if self.send_uart_command(f"KP:{value}\r\n"):
            self.clear_graph()

    def send_ki(self):
        value = self.get_float_from_entry(self.ki_entry, "Ki")

        if value is None:
            return

        if self.send_uart_command(f"KI:{value}\r\n"):
            self.clear_graph()

    def send_kd(self):
        value = self.get_float_from_entry(self.kd_entry, "Kd")

        if value is None:
            return

        if self.send_uart_command(f"KD:{value}\r\n"):
            self.clear_graph()

    def send_mode(self):
        mode = self.mode_combo.get().strip().upper()

        if mode not in ("COOLING", "HEATING"):
            messagebox.showerror("Invalid Mode", "Mode must be COOLING or HEATING.")
            return

        if self.send_uart_command(f"MODE:{mode}\r\n"):
            self.current_mode = mode
            self.mode_var.set(mode)
            self.clear_graph()

    def on_autotune_rule_selected(self, _event=None):
        self.apply_autotune_rule(send_to_stm=False, show_message=False)

    def get_selected_autotune_gains(self):
        rule_name = self.autotune_combo.get().strip()

        if rule_name not in self.PID_AUTOTUNE_RULES:
            messagebox.showerror(
                "Invalid Autotune Rule",
                "Please select a valid PID autotune rule."
            )
            return None

        return rule_name, self.PID_AUTOTUNE_RULES[rule_name]

    @staticmethod
    def format_gain(value):
        return f"{value:.12g}"

    @staticmethod
    def set_entry_value(entry, value):
        entry.delete(0, tk.END)
        entry.insert(0, PIDTemperatureControllerGUI.format_gain(value))

    def apply_autotune_rule(self, send_to_stm=False, show_message=True):
        selected = self.get_selected_autotune_gains()

        if selected is None:
            return False

        rule_name, gains = selected

        self.set_entry_value(self.kp_entry, gains["kp"])
        self.set_entry_value(self.ki_entry, gains["ki"])
        self.set_entry_value(self.kd_entry, gains["kd"])

        self.current_autotune_rule = gains["rule"]
        self.autotune_info_var.set(
            f"{rule_name}: Kp={self.format_gain(gains['kp'])}, "
            f"Ki={self.format_gain(gains['ki'])}, "
            f"Kd={self.format_gain(gains['kd'])}"
        )

        if send_to_stm:
            if not self.serial_port or not self.serial_port.is_open:
                messagebox.showwarning(
                    "Serial Port Not Connected",
                    "Autotune gains were applied to the GUI fields, but they "
                    "were not sent to STM32 because the serial port is not connected."
                )
                return False

            commands = [
                f"KP:{self.format_gain(gains['kp'])}\r\n",
                f"KI:{self.format_gain(gains['ki'])}\r\n",
                f"KD:{self.format_gain(gains['kd'])}\r\n"
            ]

            for command in commands:
                if not self.send_uart_command(command):
                    return False

            self.clear_graph()

        if show_message:
            if send_to_stm:
                messagebox.showinfo(
                    "Autotune Gains Sent",
                    f"{rule_name} gains were applied and sent to STM32."
                )
            else:
                messagebox.showinfo(
                    "Autotune Gains Applied",
                    f"{rule_name} gains were applied to the GUI fields."
                )

        return True

    def send_autotune_gains(self):
        self.apply_autotune_rule(send_to_stm=True, show_message=True)

    def get_float_from_entry(self, entry, name):
        try:
            return float(entry.get())

        except ValueError:
            messagebox.showerror("Invalid Value", f"{name} must be a number.")
            return None

    def start_pid(self):
        if not self.send_all_parameters():
            return

        if self.send_uart_command("START\r\n"):
            self.clear_graph()
            self.plot_enabled = True

    def stop_pid(self):
        self.plot_enabled = False
        self.send_uart_command("STOP\r\n")

    def clear_graph(self):
        self.time_data.clear()
        self.temperature_data.clear()
        self.setpoint_data.clear()
        self.pid_output_data.clear()

        self.sample_index = 0

        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 320)
        self.ax_pid.set_ylim(0, 100)

        self.reset_metrics()
        self.canvas.draw_idle()

    def send_all_parameters(self):
        setpoint = self.get_float_from_entry(self.setpoint_entry, "Set Point")
        kp = self.get_float_from_entry(self.kp_entry, "Kp")
        ki = self.get_float_from_entry(self.ki_entry, "Ki")
        kd = self.get_float_from_entry(self.kd_entry, "Kd")
        mode = self.mode_combo.get().strip().upper()

        if setpoint is None or kp is None or ki is None or kd is None:
            return False

        if mode not in ("COOLING", "HEATING"):
            messagebox.showerror("Invalid Mode", "Mode must be COOLING or HEATING.")
            return False

        commands = [
            f"MODE:{mode}\r\n",
            f"SETPOINT:{setpoint:.2f}\r\n",
            f"KP:{kp}\r\n",
            f"KI:{ki}\r\n",
            f"KD:{kd}\r\n"
        ]

        for command in commands:
            if not self.send_uart_command(command):
                return False

        self.current_setpoint = setpoint
        self.current_mode = mode
        self.mode_var.set(mode)

        return True

    def on_close(self):
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTemperatureControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

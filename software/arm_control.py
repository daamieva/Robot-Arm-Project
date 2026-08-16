"""
Purpose: Python host application for the 6-DOF robot arm. Provides an Arm
         class wrapping the firmware's serial command protocol (see the
         protocol comment block at the top of firmware/src/main.cpp), plus
         a minimal Tkinter GUI for manual control and live monitoring.
Author: daamieva
Date: 2026-08-06
Dependencies: pyserial, tkinter (Python standard library)
"""

import csv
import os
import queue
import threading
import time
import tkinter as tk
from datetime import datetime, timezone

import serial

STATE_PREFIX = "STATE:"
NUM_JOINTS = 6

# Joint metadata: (display name, min degrees, max degrees), index-matched to
# JOINTS[] in firmware/include/config.h. Index 3 (gripper) is wired to
# PCA9685 channel 6 on the firmware side, but that's a firmware-internal
# detail — the serial protocol only ever addresses joints by index 0-5, so
# nothing here needs to know about channel numbers.
JOINT_INFO = [
    ("Base Swivel", 0.0, 180.0),
    ("Shoulder", 0.0, 160.0),
    ("Elbow", 0.0, 180.0),
    ("Gripper", 40.0, 115.0),
    ("Wrist Swivel", 0.0, 180.0),
    ("Wrist Rotation", 0.0, 75.0),
]

# Cartesian end-effector sliders (millimeters), matching the workspace the
# firmware's inverse kinematics (firmware/src/kinematics.cpp) can reach given
# the measured arm geometry there.
XYZ_INFO = [
    ("X", -250.0, 300.0),
    ("Y", -250.0, 250.0),
    ("Z", 50.0, 430.0),
]


class Arm:
    """Serial client for the arm firmware's newline-terminated command protocol."""

    def __init__(self, port: str, baud: int = 9600):
        """
        Opens the serial connection and starts a background thread that
        continuously reads STATE broadcasts.

        Parameters:
            port: serial device path, e.g. "/dev/cu.usbmodem11301" or "COM5".
            baud: baud rate; must match SERIAL_BAUD_RATE in firmware
                  include/config.h (9600).

        The Arduino Mega resets whenever a serial connection asserts DTR —
        that's standard bootloader behavior, not something we're working
        around here. The board reboots and firmware setup() runs again;
        anything written before that finishes is lost. The 2 second sleep
        below waits out that reset and is required, not a defensive
        nicety — removing it will cause the first command(s) sent right
        after connecting to silently vanish.
        """
        self._ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Mandatory: wait out the Mega's auto-reset-on-connect.
        self._ser.reset_input_buffer()

        self.current_angles = [0.0] * NUM_JOINTS

        self._state_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._response_queue = queue.Queue()

        self._log_enabled = False
        self._log_filename = None

        # Most recent angle THIS client explicitly asked each joint to move
        # to (as opposed to current_angles, which is what the firmware last
        # reported). None until a joint has been commanded at least once.
        # Used to log commanded-vs-reported state side by side.
        self._last_commanded = [None] * NUM_JOINTS

        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self):
        """
        Background thread body. Continuously reads lines from the serial
        port for as long as the connection is open. STATE:... lines update
        self.current_angles (and are appended to the CSV log if logging is
        enabled); every other line (OK / ERR:... replies) is pushed onto a
        queue for set_joint()/go_home()/set_mode() to pick up.
        """
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue  # read timeout, no data yet
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith(STATE_PREFIX):
                self._handle_state_line(line)
            else:
                self._response_queue.put(line)

    def _handle_state_line(self, line: str):
        """
        Parses one STATE:<a0>,...,<a5> line, updates self.current_angles,
        and logs a CSV row if logging is enabled.

        Parameters:
            line: full line received, e.g. "STATE:90.0,67.0,90.0,40.0,139.0,31.0".
        """
        payload = line[len(STATE_PREFIX):]
        parts = payload.split(",")
        if len(parts) != NUM_JOINTS:
            return
        try:
            angles = [float(p) for p in parts]
        except ValueError:
            return

        with self._state_lock:
            self.current_angles = angles

        if self._log_enabled:
            self._append_csv_row(angles)

    def _send_and_wait(self, cmd: str, timeout: float = 1.0):
        """
        Sends a command line and waits for the next non-STATE reply.

        Parameters:
            cmd: command text without the trailing newline, e.g. "J2:90".
            timeout: seconds to wait for a reply before giving up.

        Returns:
            The reply string (e.g. "OK", "ERR:INVALID_JOINT"), or None if
            no reply arrived within the timeout.
        """
        with self._cmd_lock:
            # Drop any reply left over from a previous call that timed out,
            # so we don't return a stale response for this new command.
            while not self._response_queue.empty():
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break

            self._ser.write((cmd + "\n").encode())

            try:
                return self._response_queue.get(timeout=timeout)
            except queue.Empty:
                return None

    def set_joint(self, joint: int, angle: float) -> bool:
        """
        Sends J<joint>:<angle> to move a single joint.

        Parameters:
            joint: joint index 0-5, matching JOINTS[] order in firmware
                   config.h (0=base, 1=shoulder, 2=elbow, 3=gripper,
                   4=wrist_swivel, 5=wrist_rotation).
            angle: target angle in degrees. The firmware clamps this to
                   that joint's configured safety limits, so out-of-range
                   values are accepted but silently clamped, not rejected.

        Returns:
            True if the firmware replied OK, False on ERR reply or timeout.
        """
        self._last_commanded[joint] = angle
        reply = self._send_and_wait(f"J{joint}:{angle}")
        return reply == "OK"

    def send_joint_nowait(self, joint: int, angle: float):
        """
        Sends J<joint>:<angle> without waiting for a reply.

        set_joint() blocks (up to 1s) for the firmware's OK/ERR reply, which
        is fine for a single deliberate call but is the wrong tool for
        high-frequency updates like a slider being dragged: Tkinter's Scale
        fires its callback on every intermediate value during a drag, and if
        each one blocks the GUI thread, mouse events queue up faster than
        they can be processed — once unblocked, Tk replays that whole
        backlog of stale positions, which looks like the arm oscillating
        wildly well after you've stopped moving the slider. This method
        just writes the command and returns immediately, so the GUI thread
        never blocks and the arm only ever chases your current position.

        Parameters:
            joint: joint index 0-5.
            angle: target angle in degrees.
        """
        self._last_commanded[joint] = angle
        with self._cmd_lock:
            self._ser.write(f"J{joint}:{angle}\n".encode())

    def send_position_nowait(self, x: float, y: float, z: float):
        """
        Sends XYZ:<x>,<y>,<z> without waiting for a reply, for the same
        non-blocking-during-drag reason as send_joint_nowait.

        The firmware solves this via inverse kinematics (see
        firmware/src/kinematics.cpp and the XYZ: entry in docs/protocol.md)
        and moves the arm accordingly, replying OK, or ERR:UNREACHABLE if
        the position is outside the arm's physical workspace.

        Parameters:
            x, y, z: target end-effector position in millimeters, in the
                     firmware's coordinate frame (origin at the base
                     rotation axis; see docs/protocol.md).
        """
        with self._cmd_lock:
            self._ser.write(f"XYZ:{x},{y},{z}\n".encode())

    def go_home(self) -> bool:
        """
        Sends HOME, moving every joint to its configured home position.
        Requires SERIAL mode on the firmware side (see set_mode) — in POTS
        mode the pot loop would immediately overwrite the home position.

        Returns:
            True if the firmware replied OK, False on ERR reply or timeout.
        """
        reply = self._send_and_wait("HOME")
        return reply == "OK"

    def set_mode(self, mode: str) -> bool:
        """
        Switches the firmware's control source.

        Parameters:
            mode: "POTS" (potentiometers drive the joints) or "SERIAL"
                  (only J<n>:<angle>/HOME commands drive the joints).

        Returns:
            True if the firmware replied OK, False on ERR reply or timeout.
        """
        if mode not in ("POTS", "SERIAL"):
            raise ValueError("mode must be 'POTS' or 'SERIAL'")
        reply = self._send_and_wait(mode)
        return reply == "OK"

    def get_state(self) -> list:
        """
        Returns the most recently received joint angles.

        Returns:
            A list of 6 floats in JOINTS[] order (index 3 = gripper).
        """
        with self._state_lock:
            return list(self.current_angles)

    def log_to_csv(self, filename: str):
        """
        Enables CSV logging to `filename`: from this call onward, every
        STATE update received by the background thread is appended as a
        row of (timestamp, commanded_j0..commanded_j5, reported_j0..
        reported_j5). "Commanded" is the last angle this client explicitly
        requested for that joint (blank if never commanded, e.g. still in
        POTS mode); "reported" is what the firmware's STATE broadcast says
        is actually applied. Writes a header row first if the file doesn't
        exist yet or is empty.

        Parameters:
            filename: path to the CSV file to create/append to.
        """
        self._log_filename = filename
        is_new = not os.path.exists(filename) or os.path.getsize(filename) == 0
        if is_new:
            header = (
                ["timestamp"]
                + [f"commanded_j{i}" for i in range(NUM_JOINTS)]
                + [f"reported_j{i}" for i in range(NUM_JOINTS)]
            )
            with open(filename, "w", newline="") as f:
                csv.writer(f).writerow(header)
        self._log_enabled = True

    def stop_csv_logging(self):
        """Disables CSV logging started by log_to_csv(), without closing the file."""
        self._log_enabled = False

    def _append_csv_row(self, angles: list):
        """
        Appends one row to the active CSV log: an ISO-8601 UTC timestamp,
        the last commanded angle for each joint (blank if never commanded),
        then the 6 reported angles from the STATE broadcast that triggered
        this row.

        Parameters:
            angles: list of 6 reported floats from the STATE line just received.
        """
        if not self._log_filename:
            return
        timestamp = datetime.now(timezone.utc).isoformat()
        commanded = ["" if v is None else v for v in self._last_commanded]
        with open(self._log_filename, "a", newline="") as f:
            csv.writer(f).writerow([timestamp] + commanded + angles)

    def close(self):
        """Stops the background reader thread and closes the serial port."""
        self._running = False
        try:
            self._ser.close()
        except serial.SerialException:
            pass


class ArmControllerGUI:
    """Minimal Tkinter GUI for connecting to and driving an Arm."""

    def __init__(self, root: tk.Tk):
        """
        Builds the GUI widgets. Does not connect to any serial port yet —
        that happens when the user clicks Connect.

        Parameters:
            root: the Tk root window to build the interface in.
        """
        self.root = root
        self.root.title("Arm Controller")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.arm = None
        self.mode = "POTS"  # Matches the firmware's default control mode on boot.
        self._updating_from_state = False  # Guards against slider->command feedback loops.

        self._build_connection_row()
        self._build_mode_row()
        self._build_sliders()
        self._build_xyz_sliders()
        self._build_log_checkbox()
        self._build_status_bar()

    def _build_connection_row(self):
        """Builds the port entry field and Connect button."""
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X, padx=8, pady=6)

        tk.Label(frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="/dev/cu.usbmodem11301")
        self.port_entry = tk.Entry(frame, textvariable=self.port_var, width=24)
        self.port_entry.pack(side=tk.LEFT, padx=4)

        self.connect_button = tk.Button(frame, text="Connect", command=self._on_connect)
        self.connect_button.pack(side=tk.LEFT, padx=4)

    def _build_mode_row(self):
        """Builds the POTS/SERIAL mode toggle button."""
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X, padx=8, pady=(0, 6))

        self.mode_button = tk.Button(frame, text="Switch to SERIAL", command=self._on_toggle_mode)
        self.mode_button.pack(side=tk.LEFT)

    def _build_sliders(self):
        """Builds one labeled slider per joint."""
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X, padx=8, pady=4)

        self.sliders = []
        self.slider_labels = []

        for i, (name, min_deg, max_deg) in enumerate(JOINT_INFO):
            row = tk.Frame(frame)
            row.pack(fill=tk.X, pady=2)

            label = tk.Label(row, text=f"{name}: --", width=22, anchor="w")
            label.pack(side=tk.LEFT)

            slider = tk.Scale(
                row,
                from_=min_deg,
                to=max_deg,
                orient=tk.HORIZONTAL,
                resolution=1,
                length=280,
                command=self._make_slider_callback(i),
            )
            slider.pack(side=tk.LEFT, padx=6)

            self.sliders.append(slider)
            self.slider_labels.append(label)

    def _build_xyz_sliders(self):
        """
        Builds the X/Y/Z sliders, wired to the XYZ:<x>,<y>,<z> command. The
        firmware moves the arm to the requested position via inverse
        kinematics, or replies ERR:UNREACHABLE if it's outside the arm's
        workspace.

        Unlike the joint sliders, these are debounced to <ButtonRelease-1>
        rather than sending on every drag tick (Scale's `command`): each
        XYZ command triggers a full IK solve and moves every joint at once,
        which is much heavier than a single joint nudge, so firing it
        continuously during a drag was visibly shaking the arm between
        intermediate solutions. The slider's own built-in numeric display
        still updates live while dragging — only the actual arm movement is
        deferred to release.
        """
        frame = tk.LabelFrame(self.root, text="Cartesian (X/Y/Z)")
        frame.pack(fill=tk.X, padx=8, pady=4)

        self.xyz_sliders = []

        for i, (name, min_val, max_val) in enumerate(XYZ_INFO):
            row = tk.Frame(frame)
            row.pack(fill=tk.X, pady=2)

            tk.Label(row, text=f"{name}:", width=22, anchor="w").pack(side=tk.LEFT)

            slider = tk.Scale(
                row,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                resolution=1,
                length=280,
            )
            slider.bind("<ButtonRelease-1>", self._on_xyz_release)
            slider.pack(side=tk.LEFT, padx=6)
            self.xyz_sliders.append(slider)

    def _build_log_checkbox(self):
        """Builds the 'Log CSV' checkbox."""
        frame = tk.Frame(self.root)
        frame.pack(fill=tk.X, padx=8, pady=4)

        self.log_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            frame, text="Log CSV", variable=self.log_var, command=self._on_toggle_logging
        ).pack(side=tk.LEFT)

    def _build_status_bar(self):
        """Builds the bottom status bar showing connection state and last STATE."""
        self.status_var = tk.StringVar(value="Disconnected")
        status_label = tk.Label(
            self.root, textvariable=self.status_var, anchor="w", relief=tk.SUNKEN
        )
        status_label.pack(fill=tk.X, side=tk.BOTTOM, padx=2, pady=2)

    def _on_connect(self):
        """Connect button handler: opens the Arm connection and starts polling."""
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("Enter a port first")
            return

        try:
            self.arm = Arm(port)
        except serial.SerialException as exc:
            self.status_var.set(f"Connect failed: {exc}")
            return

        self.connect_button.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.DISABLED)
        self.status_var.set(f"Connected to {port} | Mode: {self.mode}")
        self._poll_state()

    def _on_toggle_mode(self):
        """Mode button handler: switches the firmware between POTS and SERIAL."""
        if self.arm is None:
            self.status_var.set("Connect first")
            return

        new_mode = "SERIAL" if self.mode == "POTS" else "POTS"
        if self.arm.set_mode(new_mode):
            self.mode = new_mode
            self.mode_button.config(
                text="Switch to POTS" if self.mode == "SERIAL" else "Switch to SERIAL"
            )
        else:
            self.status_var.set(f"Mode switch to {new_mode} failed")

    def _make_slider_callback(self, joint_index: int):
        """
        Builds the Tkinter `command` callback for one joint's slider.

        Parameters:
            joint_index: which joint (0-5) this slider controls.

        Returns:
            A function suitable for Scale(command=...), which sends
            J<joint_index>:<value> only when the change came from the user
            dragging the slider in SERIAL mode (not from a programmatic
            update mirroring an incoming STATE broadcast). Uses
            send_joint_nowait rather than set_joint so dragging doesn't
            block the GUI thread — see send_joint_nowait's docstring.
        """
        def callback(value):
            if self._updating_from_state:
                return
            if self.arm is None or self.mode != "SERIAL":
                return
            self.arm.send_joint_nowait(joint_index, float(value))

        return callback

    def _on_xyz_release(self, event):
        """
        <ButtonRelease-1> handler shared by all three X/Y/Z sliders. Reads
        all three sliders' current positions and sends one combined
        XYZ:<x>,<y>,<z> command — see _build_xyz_sliders for why this is
        debounced to release rather than firing on every drag tick.

        Parameters:
            event: the Tkinter event object (unused; required by bind()).
        """
        if self.arm is None or self.mode != "SERIAL":
            return
        x = self.xyz_sliders[0].get()
        y = self.xyz_sliders[1].get()
        z = self.xyz_sliders[2].get()
        self.arm.send_position_nowait(x, y, z)

    def _on_toggle_logging(self):
        """Log CSV checkbox handler: enables/disables CSV logging on the Arm."""
        if self.arm is None:
            self.log_var.set(False)
            return

        if self.log_var.get():
            filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_log.csv")
            self.arm.log_to_csv(filename)
            self.status_var.set(f"Logging to {filename}")
        else:
            self.arm.stop_csv_logging()

    def _poll_state(self):
        """
        Periodic UI refresh: pulls the latest angles from the Arm and
        updates every slider and label to match, guarding against the
        update itself triggering a slider->command feedback loop.
        Reschedules itself every 100ms as long as connected.
        """
        if self.arm is not None:
            angles = self.arm.get_state()

            self._updating_from_state = True
            for i, angle in enumerate(angles):
                self.sliders[i].set(angle)
                name = JOINT_INFO[i][0]
                self.slider_labels[i].config(text=f"{name}: {angle:.1f}°")
            self._updating_from_state = False

            state_text = ",".join(f"{a:.1f}" for a in angles)
            self.status_var.set(f"Mode: {self.mode} | STATE: {state_text}")

        self.root.after(100, self._poll_state)

    def _on_close(self):
        """Window close handler: stops the Arm's background thread cleanly."""
        if self.arm is not None:
            self.arm.close()
        self.root.destroy()


def main():
    """Creates the Tk root window, builds the GUI, and runs the event loop."""
    root = tk.Tk()
    ArmControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()

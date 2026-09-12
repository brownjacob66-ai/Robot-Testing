import time
import math
import threading
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import ikpy.chain
import ikpy.link
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# =============================================================================
# 1. HARDWARE RATIOS, KINEMATIC CONSTANTS & UNITS
# =============================================================================
SPD_BASE = 222.0
SPD_SHOULDER = 223

SPD_ELBOW = 408
SPD_WRIST_PITCH = 402
SPD_WRIST_ROLL = 40.0

SHOULDER_HEIGHT = 300.0
UPPER_ARM_LENGTH = 250.0
FOREARM_LENGTH = 160.0
WRIST_LENGTH = 80.0
L_DOP = 50.0
TOTAL_WRIST_PIVOT_OFFSET = WRIST_LENGTH + L_DOP

LOAD_CLEARANCE_MM = 80.0
Z_LAP_SURFACE = 0.0

LAP_CENTER_X = 400.0
LAP_CENTER_Y = 0.0

BASE_SWIVEL_DEG = 4.0
BASE_SWIVEL_FREQUENCY_HZ = 0.4
SPARK_OUT_TIME_SEC = 15.0

TARGET_CUTTING_FORCE_GRAMS = 150.0
FORCE_TOUCH_THRESHOLD_GRAMS = 20.0
FORCE_WARN_THRESHOLD_GRAMS = 250.0
FORCE_ABORT_THRESHOLD_GRAMS = 320.0

# =============================================================================
# 2. DICE PROFILES (unchanged)
# =============================================================================
DICE_PROFILES = {
    "D4": {
        "name": "Tetrahedron (D4)",
        "faces": 4, "hold_type": "face", "default_size": 20.0, "rc_factor": 3.0,
        "stages": [
            {"name": "Stage 1: Base Face", "pitch": 0.0, "rolls": [0.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 2: Side Faces", "pitch": 70.53, "rolls": [0.0, 120.0, 240.0]}
        ]
    },
    "D6": {
        "name": "Cube (D6)",
        "faces": 6, "hold_type": "face", "default_size": 20.0, "rc_factor": 1.732,
        "stages": [
            {"name": "Stage 1: Top Face", "pitch": 0.0, "rolls": [0.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 2: Side Faces", "pitch": 90.0, "rolls": [0.0, 90.0, 180.0, 270.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 3: Bottom Face", "pitch": 0.0, "rolls": [0.0]}
        ]
    },
    "D8": {
        "name": "Octahedron (D8)",
        "faces": 8, "hold_type": "corner", "default_size": 20.0, "rc_factor": 1.414,
        "stages": [
            {"name": "Stage 1: Top Pyramid", "pitch": 35.26, "rolls": [0.0, 90.0, 180.0, 270.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 2: Bottom Pyramid", "pitch": 35.26, "rolls": [45.0, 135.0, 225.0, 315.0]}
        ]
    },
    "D10": {
        "name": "Pentagonal Trapezohedron (D10)",
        "faces": 10, "hold_type": "corner", "default_size": 20.0, "rc_factor": 1.350,
        "stages": [
            {"name": "Stage 1: Top Cap", "pitch": 30.12, "rolls": [0.0, 72.0, 144.0, 216.0, 288.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 2: Bottom Cap", "pitch": 30.12, "rolls": [36.0, 108.0, 180.0, 252.0, 324.0]}
        ]
    },
    "D12": {
        "name": "Dodecahedron (D12)",
        "faces": 12, "hold_type": "face", "default_size": 20.0, "rc_factor": 1.258,
        "stages": [
            {"name": "Stage 1: Top Face", "pitch": 0.0, "rolls": [0.0]},
            {"name": "Stage 2: Upper Ring", "pitch": 63.43, "rolls": [0.0, 72.0, 144.0, 216.0, 288.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 3: Lower Ring", "pitch": 63.43, "rolls": [36.0, 108.0, 180.0, 252.0, 324.0]},
            {"name": "Stage 4: Bottom Face", "pitch": 0.0, "rolls": [0.0]}
        ]
    },
    "D20": {
        "name": "Icosahedron (D20)",
        "faces": 20, "hold_type": "corner", "default_size": 20.0, "rc_factor": 1.218,
        "stages": [
            {"name": "Stage 1: Top Cap", "pitch": 20.91, "rolls": [0.0, 72.0, 144.0, 216.0, 288.0]},
            {"name": "Stage 2: Upper Mid Ring", "pitch": 52.62, "rolls": [36.0, 108.0, 180.0, 252.0, 324.0]},
            {"name": "FLIP_DOP", "pitch": None, "rolls": []},
            {"name": "Stage 3: Lower Mid Ring", "pitch": 52.62, "rolls": [0.0, 72.0, 144.0, 216.0, 288.0]},
            {"name": "Stage 4: Bottom Cap", "pitch": 20.91, "rolls": [36.0, 108.0, 180.0, 252.0, 324.0]}
        ]
    }
}

# =============================================================================
# 3. KINEMATIC CHAINS (unchanged)
# =============================================================================
arm = ikpy.chain.Chain(
    name='lapidary_5axis_arm',
    links=[
        ikpy.link.OriginLink(),
        ikpy.link.URDFLink(name="base_yaw",        origin_translation=[0, 0, SHOULDER_HEIGHT],  origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        ikpy.link.URDFLink(name="shoulder_pitch",  origin_translation=[0, 0, 0],                 origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        ikpy.link.URDFLink(name="elbow_pitch",     origin_translation=[0, 0, UPPER_ARM_LENGTH], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        ikpy.link.URDFLink(name="wrist_pitch",     origin_translation=[0, 0, FOREARM_LENGTH],   origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        ikpy.link.URDFLink(name="dop_tip",         origin_translation=[0, 0, TOTAL_WRIST_PIVOT_OFFSET], origin_orientation=[0, 0, 0], rotation=[0, 0, 0])
    ],
    active_links_mask=[False, False, True, True, True, False]
)

wrist_chain = ikpy.chain.Chain(
    name='wrist_positioning_chain',
    links=[
        ikpy.link.OriginLink(),
        ikpy.link.URDFLink(name="base_yaw",        origin_translation=[0, 0, SHOULDER_HEIGHT],  origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        ikpy.link.URDFLink(name="shoulder_pitch",  origin_translation=[0, 0, 0],                 origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        ikpy.link.URDFLink(name="elbow_pitch",     origin_translation=[0, 0, UPPER_ARM_LENGTH], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        ikpy.link.URDFLink(name="wrist_pivot",     origin_translation=[0, 0, FOREARM_LENGTH],   origin_orientation=[0, 0, 0], rotation=[0, 0, 0])
    ],
    active_links_mask=[False, False, True, True, False]
)

# =============================================================================
# 4. MATH HELPERS (unchanged)
# =============================================================================
def calculate_inradius(size_mm, profile_key):
    prof = DICE_PROFILES[profile_key]
    if prof["hold_type"] == "face":
        return size_mm / 2.0
    else:
        return (size_mm / 2.0) / prof["rc_factor"]

def compute_geometry_target_offset(size_mm, profile_key, stage):
    prof = DICE_PROFILES[profile_key]
    r_i = calculate_inradius(size_mm, profile_key)
    pitch_deg = stage.get("pitch", 0.0) or 0.0
    pitch_rad = math.radians(pitch_deg)
    if prof["hold_type"] == "corner":
        return r_i * math.cos(pitch_rad)
    else:
        return r_i

def compute_wrist_target_coordinates(lap_x, lap_y, z_lap, pitch_deg, roll_deg, size_mm, profile_key, swivel_offset_y=0.0):
    r_i = calculate_inradius(size_mm, profile_key)
    pitch_rad = math.radians(pitch_deg)
    total_effective_length = TOTAL_WRIST_PIVOT_OFFSET + r_i
    delta_z = total_effective_length * math.cos(pitch_rad)
    delta_r = total_effective_length * math.sin(pitch_rad)
    effective_lap_y = lap_y + swivel_offset_y
    base_angle_rad = math.atan2(effective_lap_y, lap_x)
    target_wx = lap_x - delta_r * math.cos(base_angle_rad)
    target_wy = effective_lap_y - delta_r * math.sin(base_angle_rad)
    target_wz = z_lap + delta_z
    return target_wx, target_wy, target_wz

def compute_calibration_wrist_target(lap_x, lap_y, z_lap, pitch_deg=0.0, swivel_offset_y=0.0):
    """
    Calibration target using EMPTY DOP ONLY (no die geometry term).
    This keeps lap probing referenced strictly to dop tip.
    """
    pitch_rad = math.radians(pitch_deg)
    effective_length = TOTAL_WRIST_PIVOT_OFFSET  # no inradius term
    delta_z = effective_length * math.cos(pitch_rad)
    delta_r = effective_length * math.sin(pitch_rad)

    effective_lap_y = lap_y + swivel_offset_y
    base_angle_rad = math.atan2(effective_lap_y, lap_x)

    target_wx = lap_x - delta_r * math.cos(base_angle_rad)
    target_wy = effective_lap_y - delta_r * math.sin(base_angle_rad)
    target_wz = z_lap + delta_z
    return target_wx, target_wy, target_wz

def convert_angles_to_steps(base_deg, shoulder_deg, elbow_deg, pitch_deg, roll_deg):
    s_base = int(round(base_deg * SPD_BASE))
    s_shoulder = int(round(shoulder_deg * SPD_SHOULDER))
    s_elbow = int(round(elbow_deg * SPD_ELBOW))
    s_pitch = int(round(-1*(pitch_deg) * SPD_WRIST_PITCH))
    s_roll = int(round(roll_deg * SPD_WRIST_ROLL))
    return s_base, s_shoulder, s_elbow, s_pitch, s_roll

def convert_steps_to_rads(s_base, s_shoulder, s_elbow, s_pitch, s_roll):
    b_deg = s_base / SPD_BASE
    s_deg = s_shoulder / SPD_SHOULDER
    e_deg = s_elbow / SPD_ELBOW
    p_deg = -1 *(s_pitch / SPD_WRIST_PITCH)
    r_deg = s_roll / SPD_WRIST_ROLL
    return [0.0, math.radians(b_deg), math.radians(s_deg), math.radians(e_deg), math.radians(p_deg), math.radians(r_deg)]

# =============================================================================
# 5. ASYNC INTERFACE – now dual serial (Motion + Force)
# =============================================================================
class AsyncRobotInterface:
    def __init__(self, motion_port='COM3', force_port='COM4', baudrate=115200, force_sim=False):
        self.latest_force_grams = 0.0
        self.current_steps = [0, 0, 0, 0, 0]
        self.actual_joint_rads = [0.0] * 6
        self.running = True
        self.last_event = None
        self.last_probe_result = None
        self.simulation_mode = force_sim
        self.motion_ser = None
        self.force_ser = None
        self._target_sim_force = 0.0

        if not force_sim:
            try:
                self.motion_ser = serial.Serial(motion_port, baudrate, timeout=0.05)
                self.force_ser  = serial.Serial(force_port,  baudrate, timeout=0.05)
            except Exception as e:
                raise ConnectionError(f"Could not open serial ports. Error: {e}")

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _set_current_steps(self, steps):
        self.current_steps = [int(step) for step in steps[:5]]
        self.actual_joint_rads = convert_steps_to_rads(*self.current_steps)

    def _handle_motion_line(self, line):
        if line.startswith("POS:"):
            parts = line.replace("POS:", "").strip().split(",")
            if len(parts) >= 5:
                self._set_current_steps(parts)
        elif line.startswith("PROBE_HIT:"):
            self.last_probe_result = line
            parts = line.replace("PROBE_HIT:", "").strip().split(",")
            if len(parts) >= 5:
                self._set_current_steps(parts)
            self.last_event = "PROBE_HIT"
        elif "PROBE FAILED" in line:
            self.last_probe_result = "FAILED"
            self.last_event = "PROBE_FAILED"
        elif line in ["HOME_COMPLETE", "ABORT_COMPLETE", "MOVE_COMPLETE", "STOP_CLEARED", "STOP_TRIGGERED"]:
            self.last_event = line

    def _read_loop(self):
        while self.running:
            if self.simulation_mode:
                self.latest_force_grams += (self._target_sim_force - self.latest_force_grams) * 0.25
            else:
                # Force board
                if self.force_ser and self.force_ser.in_waiting > 0:
                    try:
                        line = self.force_ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("FORCE:"):
                            self.latest_force_grams = float(line.replace("FORCE:", "").strip())
                    except Exception:
                        pass

                # Motion board
                if self.motion_ser and self.motion_ser.in_waiting > 0:
                    try:
                        line = self.motion_ser.readline().decode('utf-8', errors='ignore').strip()
                        self._handle_motion_line(line)
                    except Exception:
                        pass
            time.sleep(0.008)

    def set_sim_target_force(self, target_val):
        self._target_sim_force = target_val

    def send_move(self, b_step, s_step, e_step, wp_step, wr_step):
        self.last_event = None
        if self.simulation_mode:
            # NEW: update simulated robot state so visualization reflects commanded move
            self.current_steps = [b_step, s_step, e_step, wp_step, wr_step]
            self.actual_joint_rads = convert_steps_to_rads(b_step, s_step, e_step, wp_step, wr_step)
            return
        if self.motion_ser:
            cmd = f"MOVE {b_step} {s_step} {e_step} {wp_step} {wr_step}\n"
            self.motion_ser.write(cmd.encode('utf-8'))

    def send_probe(self, b_step, s_step, e_step, wp_step, wr_step, target_z=-15.0):
        """
        Send probe command to motion board (or simulate plunge in sim mode).
        In simulation, animates descent from current position to target_z.
        """
        self.last_event = None
        self.last_probe_result = None
        if self.simulation_mode:
            # Simulate the plunge trajectory
            steps_in_plunge = [b_step, s_step, e_step, wp_step, wr_step]
            self.current_steps = steps_in_plunge
            self.actual_joint_rads = convert_steps_to_rads(*steps_in_plunge)
            time.sleep(0.8)
            return
        if self.motion_ser:
            cmd = f"PROBE {b_step} {s_step} {e_step} {wp_step} {wr_step}\n"
            self.motion_ser.write(cmd.encode('utf-8'))

    def wait_for_probe(self, timeout=20.0):
        start = time.time()
        while time.time() - start < timeout:
            # REMOVE this old shortcut:
            # if self.simulation_mode:
            #     return "PROBE_HIT:0,0,0,0,0"

            # KEEP this for both real + sim:
            if self.last_probe_result:
                res = self.last_probe_result
                self.last_probe_result = None
                return res
            time.sleep(0.05)
        return None
    
    def wait_for_move_complete(self, timeout=30.0):
        """Block until the Motion Mega reports MOVE_COMPLETE or timeout."""
        if self.simulation_mode:
            time.sleep(2.0)
            return True
        start = time.time()
        while time.time() - start < timeout:
            if self.last_event == "MOVE_COMPLETE":
                self.last_event = None
                return True
            if self.last_event in ("ABORT_COMPLETE", "STOP_TRIGGERED"):
                return False
            time.sleep(0.05)
        return False

    def home(self):
        """Send HOME command and wait for HOME_COMPLETE."""
        if self.simulation_mode:
            time.sleep(1.5)
            return True
        if self.motion_ser:
            self.last_event = None
            self.motion_ser.write(b"HOME\n")
        start = time.time()
        while time.time() - start < 120.0:
            if self.last_event == "HOME_COMPLETE":
                return True
            time.sleep(0.05)
        return False

    def abort(self):
        """Send ABORT command."""
        if self.simulation_mode:
            return
        if self.motion_ser:
            self.motion_ser.write(b"ABORT\n")

    def clear_stop(self):
        """Send CLEAR_STOP command."""
        self.last_event = None
        self.last_probe_result = None
        if self.simulation_mode:
            return
        if self.motion_ser:
            self.motion_ser.write(b"CLEAR_STOP\n")

    def close(self):
        """Shutdown the interface gracefully."""
        self.running = False
        if self.motion_ser: self.motion_ser.close()
        if self.force_ser:  self.force_ser.close()

# =============================================================================
# 6. VISUALIZER (unchanged)
# =============================================================================
class ArmVisualizer:
    def __init__(self, fig):
        self.fig = fig
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.force_text = self.fig.text(
            0.05, 0.90, "Load: 0.0 g",
            fontsize=11, fontweight='bold', color='green',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.85)
        )
        self.ax.view_init(elev=25, azim=45)

    def render(self, target_rads, actual_rads=None, z_surface=0.0, current_facet_name="", contact_detected=False, force_grams=0.0):
        self.ax.cla()
        arm.plot(target_rads, self.ax)
        if actual_rads is not None:
            arm.plot(actual_rads, self.ax)
            for line in self.ax.lines[-5:]:
                line.set_linestyle('--')
                line.set_color('darkorange')

        theta = np.linspace(0, 2 * np.pi, 20)
        disk_x = LAP_CENTER_X + 75.0 * np.cos(theta)
        disk_y = LAP_CENTER_Y + 75.0 * np.sin(theta)
        disk_z = np.full_like(theta, z_surface)
        self.ax.plot(disk_x, disk_y, disk_z, color='red' if contact_detected else 'cyan', linewidth=2)

        fk_matrix = arm.forward_kinematics(target_rads)
        tip_z = fk_matrix[2, 3]
        self.ax.set(xlim=[-150, 450], ylim=[-300, 300], zlim=[0, 550])
        self.ax.set_title(f"Pose: {current_facet_name}\nDop Z: {tip_z:.2f} mm | Lap Z: {z_surface:.2f} mm")

        self.force_text.set_text(f"Load: {force_grams:.1f} g")
        if force_grams >= FORCE_WARN_THRESHOLD_GRAMS:
            self.force_text.set_color('red')
        elif force_grams >= FORCE_TOUCH_THRESHOLD_GRAMS:
            self.force_text.set_color('orange')
        else:
            self.force_text.set_color('green')

# =============================================================================
# 7. HMI
# =============================================================================
class LapidaryHMI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("5-Axis Robotic Lapidary HMI - Dual Arduino")
        self.geometry("1400x850")

        self.robot = None
        self.visualizer = None
        self.worker_thread = None
        self.resume_event = threading.Event()
        self.abort_flag = threading.Event()

        self._build_layout()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_layout(self):
        ctrl_frame = ttk.LabelFrame(self, text=" Machine Execution & Setup ", width=380)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        ctrl_frame.pack_propagate(False)

        ttk.Label(ctrl_frame, text="Motion Port:").pack(anchor="w", padx=10, pady=(10, 0))
        self.motion_port_var = tk.StringVar(value="COM4")
        ttk.Combobox(ctrl_frame, textvariable=self.motion_port_var,
                     values=["SIM", "COM3", "COM4", "COM5", "/dev/ttyUSB0"]).pack(fill=tk.X, padx=10, pady=2)

        ttk.Label(ctrl_frame, text="Force Port:").pack(anchor="w", padx=10, pady=(8, 0))
        self.force_port_var = tk.StringVar(value="COM3")
        ttk.Combobox(ctrl_frame, textvariable=self.force_port_var,
                     values=["SIM", "COM3", "COM4", "COM5", "/dev/ttyUSB1"]).pack(fill=tk.X, padx=10, pady=2)

        ttk.Button(ctrl_frame, text="Initialize Robot Connection", command=self.connect_robot).pack(fill=tk.X, padx=10, pady=5)

        ttk.Separator(ctrl_frame, orient='horizontal').pack(fill=tk.X, pady=10, padx=5)

        ttk.Label(ctrl_frame, text="Dice Geometry Profile:").pack(anchor="w", padx=10)
        self.profile_var = tk.StringVar(value="D20")
        ttk.Combobox(ctrl_frame, textvariable=self.profile_var, values=list(DICE_PROFILES.keys())).pack(fill=tk.X, padx=10, pady=2)

        ttk.Label(ctrl_frame, text="Finished Size (Bounding mm):").pack(anchor="w", padx=10)
        self.size_var = tk.StringVar(value="26.0")
        ttk.Entry(ctrl_frame, textvariable=self.size_var).pack(fill=tk.X, padx=10, pady=2)

        ttk.Separator(ctrl_frame, orient='horizontal').pack(fill=tk.X, pady=10, padx=5)

        self.btn_home = ttk.Button(ctrl_frame, text="🏠 Home All Axes", state=tk.DISABLED,
                                   command=lambda: self.run_worker(self.task_home))
        self.btn_home.pack(fill=tk.X, padx=10, pady=3)

        self.btn_cal = ttk.Button(ctrl_frame, text="📍 Calibrate Z-Lap Surface", state=tk.DISABLED,
                                  command=lambda: self.run_worker(self.task_calibrate))
        self.btn_cal.pack(fill=tk.X, padx=10, pady=3)

        self.btn_cut = ttk.Button(ctrl_frame, text="⚙️ Execute Faceting Routine", state=tk.DISABLED,
                                  command=lambda: self.run_worker(self.task_cut))
        self.btn_cut.pack(fill=tk.X, padx=10, pady=3)

        self.btn_resume = ttk.Button(ctrl_frame, text="▶ RESUME JOB (Dop Flipped)", state=tk.DISABLED,
                                     command=self.resume_job)
        self.btn_resume.pack(fill=tk.X, padx=10, pady=(15, 3))

        self.btn_abort = ttk.Button(ctrl_frame, text="🛑 ABORT", state=tk.DISABLED, command=self.request_abort)
        self.btn_abort.pack(fill=tk.X, padx=10, pady=3)

        ttk.Label(ctrl_frame, text="Telemetry & System Log:").pack(anchor="w", padx=10, pady=(15, 0))
        self.log_text = scrolledtext.ScrolledText(ctrl_frame, height=14, state='disabled')
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        view_frame = ttk.Frame(self)
        view_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = plt.figure(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=view_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        self.log_text.configure(state='normal')
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state='disabled')

    def connect_robot(self):
        motion_port = self.motion_port_var.get()
        force_port  = self.force_port_var.get()
        sim_mode = (motion_port == "SIM" or force_port == "SIM")

        try:
            self.robot = AsyncRobotInterface(motion_port=motion_port, force_port=force_port, force_sim=sim_mode)
        except ConnectionError as e:
            messagebox.showerror("Connection Error", str(e))
            return

        self.visualizer = ArmVisualizer(self.fig)
        self.is_homed = False
        self.is_probed = False
        global Z_LAP_SURFACE
        Z_LAP_SURFACE = 0.0

        self.btn_home.config(state=tk.NORMAL)
        self.btn_cal.config(state=tk.DISABLED)
        self.btn_cut.config(state=tk.DISABLED)
        self.btn_abort.config(state=tk.NORMAL)
        self.log(f"Connected – Motion: {motion_port}  Force: {force_port}")

    def trigger_render(self, target_rads, actual_rads, z_surf, facet_name, contact, force_grams):
        self.after(0, self._sync_render, target_rads, actual_rads, z_surf, facet_name, contact, force_grams)

    def _sync_render(self, target_rads, actual_rads, z_surf, facet_name, contact, force_grams):
        if self.visualizer:
            self.visualizer.render(target_rads, actual_rads, z_surf, facet_name, contact, force_grams)
            self.canvas.draw_idle()

    def run_worker(self, target_function):
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Busy", "A process is currently running.")
            return
        self.abort_flag.clear()
        self.worker_thread = threading.Thread(target=target_function, daemon=True)
        self.worker_thread.start()

    def resume_job(self):
        self.resume_event.set()
        self.btn_resume.config(state=tk.DISABLED)
        self.log("Resume signal received.")

    def request_abort(self):
        self.abort_flag.set()
        if self.robot:
            self.robot.abort()
        self.log("ABORT requested.")

    # =========================================================================
    # WORKER TASKS
    # =========================================================================
    def execute_wrist_motion(self, target_wx, target_wy, target_wz, pitch_deg, roll_deg,
                            profile_key, facet_name="", contact_detected=False,
                            base_swivel_deg=0.0):
        try:
            wrist_rads = wrist_chain.inverse_kinematics(
                target_position=[target_wx, target_wy, target_wz],
                initial_position=[0.0] * 5
            )
        except Exception as e:
            self.log(f"IK failure: {e}")
            return None

        # Use geometric base heading, then force oscillation offset
        base_nominal_deg = math.degrees(math.atan2(target_wy, target_wx))
        base_deg = base_nominal_deg + base_swivel_deg

        shoulder_deg = math.degrees(wrist_rads[2])
        elbow_deg = math.degrees(wrist_rads[3])
        local_pitch_deg = (180.0 - pitch_deg) - (shoulder_deg + elbow_deg)

        s_b, s_s, s_e, s_wp, s_wr = convert_angles_to_steps(
            base_deg, shoulder_deg, elbow_deg, local_pitch_deg, roll_deg
        )
        self.robot.send_move(s_b, s_s, s_e, s_wp, s_wr)

        # Render commanded pose so sim visibly matches motor command
        full_target_rads = [
            0.0,
            math.radians(base_deg),
            math.radians(shoulder_deg),
            math.radians(elbow_deg),
            math.radians(local_pitch_deg),
            math.radians(roll_deg)
        ]
        actual_rads = full_target_rads if self.robot.simulation_mode else self.robot.actual_joint_rads

        self.trigger_render(
            target_rads=full_target_rads,
            actual_rads=actual_rads,
            z_surf=Z_LAP_SURFACE,
            facet_name=facet_name,
            contact=contact_detected,
            force_grams=self.robot.latest_force_grams
        )
        return full_target_rads


    def task_home(self):
        self.log("Homing steppers...")
        if self.robot.home():
            self.is_homed = True
            self.after(0, lambda: self.btn_cal.config(state=tk.NORMAL))
            self.log("Homing complete.")
            self.trigger_render([0]*6, [0]*6, Z_LAP_SURFACE, "Homed", False, self.robot.latest_force_grams)
        else:
            self.log("ERROR: Homing failed or timed out.")

    def task_calibrate(self):
        global Z_LAP_SURFACE

        self.log("STATUS: Calculating IK for Z-Lap surface probe...")
        profile_key = self.profile_var.get()

        try:
            size_mm = float(self.size_var.get())
            if not (8.0 <= size_mm <= 40.0):
                self.log("ERROR: Finished size must be between 8 and 40 mm.")
                return
        except ValueError:
            self.log("ERROR: Invalid finished size value.")
            return

        # ------------------------------------------------------------------
        # 1) APPROACH pose (perpendicular, clearance above lap)
        # ------------------------------------------------------------------
        approach_z = Z_LAP_SURFACE + LOAD_CLEARANCE_MM

        try:
            # Calibration uses EMPTY DOP reference only
            awx, awy, awz = compute_calibration_wrist_target(
                LAP_CENTER_X, LAP_CENTER_Y, approach_z, pitch_deg=0.0, swivel_offset_y=0.0
            )

            wrist_rads_app = wrist_chain.inverse_kinematics(
                target_position=[awx, awy, awz],
                initial_position=[0.0] * 5
            )

            base_deg = math.degrees(wrist_rads_app[1])
            shoulder_deg = math.degrees(wrist_rads_app[2])
            elbow_deg = math.degrees(wrist_rads_app[3])
            local_pitch_deg = 180.0 - (shoulder_deg + elbow_deg)

            s_b, s_s, s_e, s_wp, s_wr = convert_angles_to_steps(
                base_deg, shoulder_deg, elbow_deg, local_pitch_deg, 0.0
            )

            fk_app = arm.forward_kinematics([
                0.0, wrist_rads_app[1], wrist_rads_app[2], wrist_rads_app[3],
                math.radians(local_pitch_deg), 0.0
            ])
            app_tip_z = float(fk_app[2, 3])

            self.log(f"DEBUG SERVICE IK: S={shoulder_deg:.1f}° E={elbow_deg:.1f}°")
            self.log(f"DEBUG: local_pitch = 180 - ({shoulder_deg:.1f} + {elbow_deg:.1f}) = {local_pitch_deg:.1f}°")
            self.log(f"DEBUG: Approach FK tip Z = {app_tip_z:.2f} mm")

        except Exception as e:
            self.log(f"IK error on approach: {e}")
            return

        perpendicular_pitch_steps = s_wp

        self.log(
            f"Approach angles (deg): Base={base_deg:.1f} Shoulder={shoulder_deg:.1f} "
            f"Elbow={elbow_deg:.1f} Pitch={local_pitch_deg:.1f}"
        )
        self.log(f"Approach steps: B={s_b} S={s_s} E={s_e} P={s_wp} R={s_wr}")

        self.robot.clear_stop()
        self.robot.send_move(s_b, s_s, s_e, s_wp, s_wr)

        self.log("Waiting for approach move to complete...")
        if not self.robot.wait_for_move_complete(timeout=45.0):
            self.log("ERROR: Approach move timed out.")
            return

        self.log("Approach position reached – starting perpendicular probe plunge")

        # ------------------------------------------------------------------
        # 2) PROBE command target (real hardware path)
        # ------------------------------------------------------------------
        probe_target_z = -15.0

        try:
            # Calibration uses EMPTY DOP reference only
            pwx, pwy, pwz = compute_calibration_wrist_target(
                LAP_CENTER_X, LAP_CENTER_Y, probe_target_z, pitch_deg=0.0, swivel_offset_y=0.0
            )

            wrist_rads_probe = wrist_chain.inverse_kinematics(
                target_position=[pwx, pwy, pwz],
                initial_position=[0.0] * 5
            )

            base_deg_probe = math.degrees(wrist_rads_probe[1])
            shoulder_deg_probe = math.degrees(wrist_rads_probe[2])
            elbow_deg_probe = math.degrees(wrist_rads_probe[3])

            s_b_probe, s_s_probe, s_e_probe, _, s_wr_probe = convert_angles_to_steps(
                base_deg_probe, shoulder_deg_probe, elbow_deg_probe, 0.0, 0.0
            )
            s_wp_probe = perpendicular_pitch_steps

        except Exception as e:
            self.log(f"IK error on probe target: {e}")
            return

        self.log(f"Probe steps (pitch locked): B={s_b_probe} S={s_s_probe} E={s_e_probe} P={s_wp_probe} R={s_wr_probe}")
        self.robot.clear_stop()

        # ------------------------------------------------------------------
        # 2b) Execute probe: simulation vs hardware
        # ------------------------------------------------------------------
        if self.robot.simulation_mode:
            plunge_start_z = Z_LAP_SURFACE + LOAD_CLEARANCE_MM
            plunge_target_z = probe_target_z
            plunge_steps = 60
            plunge_duration = 1.2
            step_interval = plunge_duration / plunge_steps

            last_sim_hit_steps = None
            hit_detected = False
            plate_z = Z_LAP_SURFACE
            hit_tolerance_mm = 0.2

            for i in range(plunge_steps + 1):
                if self.abort_flag.is_set():
                    return

                progress = i / plunge_steps
                current_z = plunge_start_z + progress * (plunge_target_z - plunge_start_z)

                try:
                    # Calibration uses EMPTY DOP reference only
                    wx_i, wy_i, wz_i = compute_calibration_wrist_target(
                        LAP_CENTER_X, LAP_CENTER_Y, current_z, pitch_deg=0.0, swivel_offset_y=0.0
                    )

                    wrist_rads_i = wrist_chain.inverse_kinematics(
                        target_position=[wx_i, wy_i, wz_i],
                        initial_position=[0.0] * 5
                    )

                    base_deg_i = math.degrees(wrist_rads_i[1])
                    shoulder_deg_i = math.degrees(wrist_rads_i[2])
                    elbow_deg_i = math.degrees(wrist_rads_i[3])
                    local_pitch_deg_i = 180.0 - (shoulder_deg_i + elbow_deg_i)

                    sb_i, ss_i, se_i, sp_i, sr_i = convert_angles_to_steps(
                        base_deg_i, shoulder_deg_i, elbow_deg_i, local_pitch_deg_i, 0.0
                    )

                    self.robot.current_steps = [sb_i, ss_i, se_i, sp_i, sr_i]
                    self.robot.actual_joint_rads = convert_steps_to_rads(sb_i, ss_i, se_i, sp_i, sr_i)

                    full_target_rads_i = [
                        0.0, wrist_rads_i[1], wrist_rads_i[2], wrist_rads_i[3],
                        math.radians(local_pitch_deg_i), 0.0
                    ]

                    fk_i = arm.forward_kinematics(full_target_rads_i)
                    tip_z_i = float(fk_i[2, 3])

                    self.trigger_render(
                        target_rads=full_target_rads_i,
                        actual_rads=full_target_rads_i,
                        z_surf=Z_LAP_SURFACE,
                        facet_name=f"Probe Plunge Z={current_z:.1f}mm | TipZ={tip_z_i:.1f}",
                        contact=(tip_z_i <= plate_z + hit_tolerance_mm),
                        force_grams=0.0
                    )

                    if tip_z_i <= plate_z + hit_tolerance_mm:
                        last_sim_hit_steps = [sb_i, ss_i, se_i, sp_i, sr_i]
                        hit_detected = True
                        self.log(f"SIM PROBE HIT at tip Z={tip_z_i:.2f} mm (plate Z={plate_z:.2f} mm)")
                        break

                except Exception as e:
                    self.log(f"Sim probe IK frame error @ Z={current_z:.2f}: {e}")
                    return

                time.sleep(step_interval)

            if hit_detected and last_sim_hit_steps is not None:
                self.robot.last_probe_result = (
                    f"PROBE_HIT:{last_sim_hit_steps[0]},"
                    f"{last_sim_hit_steps[1]},"
                    f"{last_sim_hit_steps[2]},"
                    f"{last_sim_hit_steps[3]},"
                    f"{last_sim_hit_steps[4]}"
                )
                self.robot.current_steps = last_sim_hit_steps[:]
                self.robot.actual_joint_rads = convert_steps_to_rads(*last_sim_hit_steps)
                self.robot.last_event = "PROBE_HIT"
            else:
                self.robot.last_probe_result = "FAILED"
                self.robot.last_event = "PROBE_FAILED"

        else:
            self.robot.send_probe(
                s_b_probe, s_s_probe, s_e_probe, s_wp_probe, s_wr_probe, target_z=probe_target_z
            )

        result = self.robot.wait_for_probe(timeout=30.0)

        # ------------------------------------------------------------------
        # 3) Handle probe result + move to service pose (+150 mm)
        # ------------------------------------------------------------------
        if result and result.startswith("PROBE_HIT:"):
            self.log(f"Probe successful! Received: {result}")

            try:
                raw_steps = result.split(":")[1].split(",")
                hit_steps = [int(p) for p in raw_steps[:5]]

                # Dop-tip FK defines lap Z
                hit_rads = convert_steps_to_rads(*hit_steps)
                fk_matrix = arm.forward_kinematics(hit_rads)
                probe_tip_z = float(fk_matrix[2, 3])

                Z_LAP_SURFACE = probe_tip_z
                self.log(f"Z-Lap surface calibrated at Z = {Z_LAP_SURFACE:.2f} mm")
                self.log(f"DEBUG: Probe hit steps = {hit_steps}")
                self.log(f"DEBUG: FK Z at probe contact = {probe_tip_z:.2f} mm")

                service_tip_z = Z_LAP_SURFACE + 150.0
                self.log(f"DEBUG: Service Z target = {service_tip_z:.2f} mm")

            except Exception as e:
                self.log(f"Error computing probe Z coordinate: {e}")
                Z_LAP_SURFACE = 0.0
                return

            self.is_probed = True
            self.after(0, lambda: self.btn_cut.config(state=tk.NORMAL))

            time.sleep(0.3)
            self.robot.clear_stop()
            time.sleep(0.1)

            try:
                # Service pose should also use calibration geometry (empty dop reference)
                wx_srv, wy_srv, wz_srv = compute_calibration_wrist_target(
                    LAP_CENTER_X, LAP_CENTER_Y, service_tip_z, pitch_deg=0.0, swivel_offset_y=0.0
                )

                wrist_rads_service = wrist_chain.inverse_kinematics(
                    target_position=[wx_srv, wy_srv, wz_srv],
                    initial_position=[0.0] * 5
                )

                base_deg_s = math.degrees(wrist_rads_service[1])
                shoulder_deg_s = math.degrees(wrist_rads_service[2])
                elbow_deg_s = math.degrees(wrist_rads_service[3])
                local_pitch_deg_s = 180.0 - (shoulder_deg_s + elbow_deg_s)

                fk_check = arm.forward_kinematics([
                    0.0,
                    wrist_rads_service[1],
                    wrist_rads_service[2],
                    wrist_rads_service[3],
                    math.radians(local_pitch_deg_s),
                    0.0
                ])
                actual_tip_z = float(fk_check[2, 3])

                self.log(f"DEBUG: Requested Tip Z = {service_tip_z:.2f} mm")
                self.log(f"DEBUG: FK check - Actual Tip Z = {actual_tip_z:.2f} mm")
                self.log(f"DEBUG: Difference = {service_tip_z - actual_tip_z:.2f} mm")

                sb, ss, se, sp, sr = convert_angles_to_steps(
                    base_deg_s, shoulder_deg_s, elbow_deg_s, local_pitch_deg_s, 0.0
                )

                self.log(f"Moving to dop-loading pose (Target Tip Z={service_tip_z:.1f}mm)...")
                self.robot.send_move(sb, ss, se, sp, sr)

                if self.robot.simulation_mode:
                    service_target_rads = [
                        0.0, wrist_rads_service[1], wrist_rads_service[2],
                        wrist_rads_service[3], math.radians(local_pitch_deg_s), 0.0
                    ]
                    self.trigger_render(
                        target_rads=service_target_rads,
                        actual_rads=service_target_rads,
                        z_surf=Z_LAP_SURFACE,
                        facet_name=f"Dop-loading pose (+150mm) Z={service_tip_z:.1f}",
                        contact=False,
                        force_grams=self.robot.latest_force_grams
                    )

                if self.robot.wait_for_move_complete(timeout=30.0):
                    self.log("Dop-loading position reached (150mm above bed).")
                else:
                    self.log("Warning: Timed out moving to dop-loading position.")

            except Exception as e:
                self.log(f"IK error on service pose: {e}")

        else:
            self.log("ERROR: Probe timed out or reached target without registering contact.")


    def task_cut(self):
        global Z_LAP_SURFACE
        prof_key = self.profile_var.get()
        prof = DICE_PROFILES[prof_key]

        try:
            size_mm = float(self.size_var.get())
            if not (8.0 <= size_mm <= 40.0):
                self.log("ERROR: Finished size must be between 8 and 40 mm.")
                return
        except ValueError:
            self.log("ERROR: Invalid finished size value.")
            return

        self.log(f"Starting {prof['name']} Faceting Sequence. Target Size: {size_mm} mm")
        self.robot.clear_stop()

        for stage in prof["stages"]:
            if self.abort_flag.is_set():
                self.log("Job aborted by user.")
                return

            if stage["name"] == "FLIP_DOP":
                if self.robot.simulation_mode:
                    self.robot.set_sim_target_force(0.0)
                self.log("--- WORK STOP: RE-DOP FIXTURE REQUIRED ---")
                self.log("Flip stone fixture and press 'RESUME JOB' when secured.")

                wx_ret, wy_ret, wz_ret = compute_wrist_target_coordinates(
                    LAP_CENTER_X, LAP_CENTER_Y, Z_LAP_SURFACE + LOAD_CLEARANCE_MM,
                    0.0, 0.0, size_mm, prof_key, 0.0
                )
                self.execute_wrist_motion(wx_ret, wy_ret, wz_ret, 0.0, 0.0, prof_key, "PAUSED FOR RE-DOP")

                self.after(0, lambda: self.btn_resume.config(state=tk.NORMAL))
                self.resume_event.clear()
                self.resume_event.wait()
                if self.abort_flag.is_set():
                    return
                self.robot.clear_stop()
                continue

            self.log(f"Executing: {stage['name']}")
            pitch = stage["pitch"]
            r_offset = compute_geometry_target_offset(size_mm, prof_key, stage)

            for roll in stage["rolls"]:
                if self.abort_flag.is_set():
                    self.log("Job aborted by user.")
                    return

                # 1) Approach
                current_z_offset = LOAD_CLEARANCE_MM
                if self.robot.simulation_mode:
                    self.robot.set_sim_target_force(0.0)

                wx_app, wy_app, wz_app = compute_wrist_target_coordinates(
                    LAP_CENTER_X, LAP_CENTER_Y, Z_LAP_SURFACE + current_z_offset + r_offset,
                    pitch, roll, size_mm, prof_key, 0.0
                )
                self.execute_wrist_motion(
                    wx_app, wy_app, wz_app, pitch, roll, prof_key,
                    f"Approach R={roll}° P={pitch}°"
                )
                time.sleep(0.25)

                # 2) Force-controlled plunge
                self.log(f"Plunging (Target Pressure: {TARGET_CUTTING_FORCE_GRAMS}g)")
                step_size = 0.5
                plunge_start_time = time.time()

                while current_z_offset > 0.0:
                    if self.abort_flag.is_set():
                        self.log("Plunge aborted.")
                        return

                    force = self.robot.latest_force_grams
                    if force >= FORCE_ABORT_THRESHOLD_GRAMS:
                        self.log(f"FORCE ABORT (software): {force:.1f} g")
                        self.robot.abort()
                        return

                    if self.robot.last_event == "STOP_TRIGGERED":
                        self.log("Hardware STOP line triggered during plunge.")
                        return

                    if force < TARGET_CUTTING_FORCE_GRAMS:
                        current_z_offset = max(0.0, current_z_offset - step_size)

                    if self.robot.simulation_mode:
                        if current_z_offset <= 0.0:
                            self.robot.set_sim_target_force(TARGET_CUTTING_FORCE_GRAMS)
                        else:
                            self.robot.set_sim_target_force(30.0)

                    elapsed = time.time() - plunge_start_time

                    # Base swivel: rotate lap center about origin
                    base_swivel_deg = BASE_SWIVEL_DEG * math.sin(2.0 * math.pi * BASE_SWIVEL_FREQUENCY_HZ * elapsed)
                    theta = math.radians(base_swivel_deg)
                    lap_x_sw = LAP_CENTER_X * math.cos(theta) - LAP_CENTER_Y * math.sin(theta)
                    lap_y_sw = LAP_CENTER_X * math.sin(theta) + LAP_CENTER_Y * math.cos(theta)

                    wx, wy, wz = compute_wrist_target_coordinates(
                        lap_x_sw, lap_y_sw, Z_LAP_SURFACE + current_z_offset + r_offset,
                        pitch, roll, size_mm, prof_key, 0.0
                    )
                    self.execute_wrist_motion(
                        wx, wy, wz, pitch, roll, prof_key,
                        f"Plunging R={roll}° Z_off={current_z_offset:.1f} Bsw={base_swivel_deg:+.2f}°",
                        contact_detected=(force > FORCE_TOUCH_THRESHOLD_GRAMS)
                    )
                    time.sleep(0.02)

                # 3) Spark-out
                self.log(f"Target reached. {SPARK_OUT_TIME_SEC}s Spark-out R={roll}°...")
                dwell_start = time.time()

                while time.time() - dwell_start < SPARK_OUT_TIME_SEC:
                    if self.abort_flag.is_set() or self.robot.last_event == "STOP_TRIGGERED":
                        return

                    elapsed = time.time() - plunge_start_time

                    # Base swivel: rotate lap center about origin
                    base_swivel_deg = BASE_SWIVEL_DEG * math.sin(2.0 * math.pi * BASE_SWIVEL_FREQUENCY_HZ * elapsed)
                    theta = math.radians(base_swivel_deg)
                    lap_x_sw = LAP_CENTER_X * math.cos(theta) - LAP_CENTER_Y * math.sin(theta)
                    lap_y_sw = LAP_CENTER_X * math.sin(theta) + LAP_CENTER_Y * math.cos(theta)

                    wx, wy, wz = compute_wrist_target_coordinates(
                        lap_x_sw, lap_y_sw, Z_LAP_SURFACE + r_offset,
                        pitch, roll, size_mm, prof_key, 0.0
                    )

                    if self.robot.simulation_mode:
                        self.robot.set_sim_target_force(TARGET_CUTTING_FORCE_GRAMS)

                    time_left = max(0, int(SPARK_OUT_TIME_SEC - (time.time() - dwell_start)))
                    self.execute_wrist_motion(
                        wx, wy, wz, pitch, roll, prof_key,
                        f"Spark-out {time_left}s R={roll}° Bsw={base_swivel_deg:+.2f}°",
                        contact_detected=True
                    )
                    time.sleep(0.03)

                # 4) Retract
                if self.robot.simulation_mode:
                    self.robot.set_sim_target_force(0.0)

                wx_ret, wy_ret, wz_ret = compute_wrist_target_coordinates(
                    LAP_CENTER_X, LAP_CENTER_Y, Z_LAP_SURFACE + LOAD_CLEARANCE_MM + r_offset,
                    pitch, roll, size_mm, prof_key, 0.0
                )
                self.execute_wrist_motion(
                    wx_ret, wy_ret, wz_ret, pitch, roll, prof_key,
                    f"Retract R={roll}° P={pitch}°"
                )
                time.sleep(0.25)

        self.log("Faceting Routine Complete. Returning Home.")
        self.task_home()

    def on_close(self):
        if self.robot:
            self.robot.close()
        self.destroy()

if __name__ == "__main__":
    app = LapidaryHMI()
    app.mainloop()
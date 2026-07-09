#!/usr/bin/env python3
"""
UDP to WebSocket Bridge with Grid Map SLAM
Clean architecture with atomic shared state and generation counters
Modified to use a reverse WebSocket connection to a relay server.
Supports waypoint following with continuous replanning.

Now uses RPLIDAR directly for scanning and serial communication for robot control.
"""

import asyncio
import json
import websockets
import numpy as np
import math
import time
import threading
import heapq
import queue
import ssl
import serial
import serial.tools.list_ports
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, Any, List
from scipy.ndimage import binary_dilation, generate_binary_structure

# RPLidar imports
from rplidar import RPLidar

# ============ Configuration ============
# Serial communication settings (for robot control)
SERIAL_PORT = "/dev/ttyUSB0"          # Change to your port (e.g., COM3 on Windows)
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1

# RPLidar settings
LIDAR_PORT = '/dev/ttyUSB0'                   # Change to your LiDAR port
LIDAR_BAUDRATE = 115200
LIDAR_SCAN_TYPE = "normal"           # "express" or "standard"

BROADCAST_POSE_HZ = 30  # 30 Hz pose updates
BROADCAST_MAP_HZ = 1    # 1 Hz map updates

# Planner settings
COARSE_FACTOR = 4  # COARSE_FACTOR^2 fine cells per coarse cell
ROBOT_WIDTH = 0.41  # meters

# Map parameters
MAP_SIZE = 200  # pixels
MAP_RESOLUTION = 0.05  # meters per pixel
MAP_ORIGIN_X = -MAP_SIZE * MAP_RESOLUTION / 2
MAP_ORIGIN_Y = -MAP_SIZE * MAP_RESOLUTION / 2

# Occupancy grid parameters
LOG_ODDS_OCCUPIED = 0.8
LOG_ODDS_FREE = -0.4
MAX_LOG_ODDS = 3.0
MIN_LOG_ODDS = -3.0
OCCUPIED_THRESHOLD = 0.6

# Sensor parameters
MAX_RANGE = 4.0
MIN_RANGE = 0.1

# SLAM / scan-matching parameters
SCAN_MATCH_STRIDE = 3
SCAN_MATCH_MIN_FEATURES = 150
SCAN_MATCH_TRANSLATION_RANGE = 0.20
SCAN_MATCH_TRANSLATION_STEP = 0.05
SCAN_MATCH_MIN_IMPROVEMENT = 0.05

# Angular velocity threshold for scan matching and map updates (rad/s)
ANGULAR_VEL_THRESHOLD = 0.2

# Coordinate system fix
FLIP_THETA_FOR_VISUALIZATION = True
FLIP_ROBOT_Y_FROM_SIM = True

# Correction settings
CORRECTION_WEIGHT = 0.05
ENABLE_SCAN_MATCHING = True

# ============ Relay Configuration ============
RELAY_URL = "wss://kmo-relayserver.yudhisthereal.workers.dev"
BRIDGE_ID = "my_robot_01"                # Unique identifier for this robot
BRIDGE_TOKEN = "kmo-bridge-token1"       # Must match relay's token
RECONNECT_DELAY = 3.0                    # seconds
MAX_RECONNECT_ATTEMPTS = 0               # 0 = infinite

# ============ End Configuration ============

CONFIG_FILE = "robot_config.json"
DEFAULT_CONFIG = {
    "wheel_radius": 0.0975,
    "wheel_base": 0.33,
    "lidar_offset_x": 0.0,
    "lidar_offset_y": 0.0,
    "max_speed": 4.0,
    "robot_width": 0.41,
    "stop_distance": 0.3
}

robot_config = DEFAULT_CONFIG.copy()

def load_config():
    global robot_config
    try:
        with open(CONFIG_FILE, 'r') as f:
            saved = json.load(f)
            robot_config.update(saved)
            print(f"[Config] Loaded from {CONFIG_FILE}")
    except FileNotFoundError:
        print(f"[Config] No config file found, using defaults")
    except Exception as e:
        print(f"[Config] Error loading config: {e}")

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(robot_config, f, indent=2)
        print(f"[Config] Saved to {CONFIG_FILE}")
    except Exception as e:
        print(f"[Config] Error saving config: {e}")

def send_config_to_robot():
    global command_forwarder
    """Send all robot-relevant parameters via a single CONFIG command."""
    config_str = (
        f"{robot_config['wheel_radius']},"
        f"{robot_config['wheel_base']},"
        f"{robot_config['max_speed']},"
        f"{robot_config['robot_width']},"
        f"{robot_config['stop_distance']}"
    )
    command_forwarder.send_command('config', config_str, priority=5)
    print(f"[Config] Sent to robot: {config_str}")

@dataclass(frozen=True)
class SimulationPacket:
    """Immutable raw data from LiDAR scan with odometry from robot"""
    packet_id: int
    timestamp: float
    robot_x: float
    robot_y: float
    robot_theta: float
    ranges: Tuple[float, ...]
    angles: Tuple[float, ...]
    left_speed: float = 0
    right_speed: float = 0
    linear_vel: float = 0
    angular_vel: float = 0
    auto_navigate: bool = True


@dataclass(frozen=True)
class SlamState:
    """Immutable estimated state from SLAM processor"""
    state_id: int
    timestamp: float
    x: float
    y: float
    theta: float
    match_score: float
    packet_id_processed: int
    ranges: Tuple[float, ...] = field(default_factory=tuple)
    angles: Tuple[float, ...] = field(default_factory=tuple)
    raw_robot_x: float = 0
    raw_robot_y: float = 0
    raw_robot_theta: float = 0
    left_speed: float = 0
    right_speed: float = 0
    auto_navigate: bool = True
    linear_vel: float = 0
    angular_vel: float = 0
    scan_matching_skipped: bool = False


@dataclass
class Command:
    """Command with priority for immediate execution"""
    priority: int  # Lower = higher priority (0 = emergency)
    timestamp: float
    command_type: str  # 'cmd', 'auto', 'path', 'config'
    payload: Any
    callback: Optional[callable] = None  # Optional acknowledgment callback


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


print("=" * 60)
print("UDP to WebSocket Bridge with Grid Map SLAM")
print("Reverse WebSocket Relay Mode")
print("RPLIDAR direct acquisition + Serial robot control")
print("=" * 60)
print(f"Map: {MAP_SIZE}x{MAP_SIZE} cells, {MAP_RESOLUTION*100:.0f}cm resolution")
print(f"Serial Port: {SERIAL_PORT} @ {SERIAL_BAUDRATE} baud")
print(f"LiDAR Port: {LIDAR_PORT} @ {LIDAR_BAUDRATE} baud")
print(f"Pose broadcast: {BROADCAST_POSE_HZ} Hz")
print(f"Map broadcast: {BROADCAST_MAP_HZ} Hz")
print(f"Correction Weight: {CORRECTION_WEIGHT * 100:.0f}%")
print(f"Angular velocity threshold: {ANGULAR_VEL_THRESHOLD} rad/s ({ANGULAR_VEL_THRESHOLD * 180 / math.pi:.1f} deg/s)")
print(f"Relay URL: {RELAY_URL}")
print(f"Bridge ID: {BRIDGE_ID}")
print("=" * 60)


planner = None
command_forwarder = None


class AtomicSharedPacket:
    """Thread-safe container for the latest packet with generation counter"""

    def __init__(self):
        self._packet: Optional[SimulationPacket] = None
        self._generation: int = 0
        self._lock = threading.Lock()

    def update(self, packet: SimulationPacket):
        with self._lock:
            self._packet = packet
            self._generation += 1

    def get_latest(self):
        with self._lock:
            if self._packet is None:
                return None, 0
            return self._packet, self._generation


class AtomicSharedState:
    """Thread-safe container for the latest SLAM state"""

    def __init__(self):
        self._state: Optional[SlamState] = None
        self._state_id: int = 0
        self._lock = threading.Lock()

    def update(self, state: SlamState):
        with self._lock:
            self._state = state

    def get_latest(self):
        with self._lock:
            return self._state


class OccupancyGrid:
    """2D occupancy grid map using log-odds with dynamic expansion"""

    def __init__(self, width, height, resolution, offset_x=0.0, offset_y=0.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        self.occupancy = np.full((height, width), -1, dtype=np.int8)
        self.last_update_packet_id = -1
        self.map_update_id = 0  # incremented on each successful update
        # LiDAR mounting offset (in robot's local frame)
        self.lidar_offset_x = offset_x
        self.lidar_offset_y = offset_y

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def is_ready_for_scan_matching(self):
        """Return True once the map has enough structure."""
        return np.count_nonzero(np.abs(self.log_odds) > 0.25) >= SCAN_MATCH_MIN_FEATURES

    def sample_log_odds(self, x, y):
        """Bilinearly sample the log-odds grid at world coordinates."""
        fx = (x - self.origin_x) / self.resolution
        fy = (y - self.origin_y) / self.resolution

        x0 = int(math.floor(fx))
        y0 = int(math.floor(fy))
        x1 = x0 + 1
        y1 = y0 + 1

        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            return 0.0

        tx = fx - x0
        ty = fy - y0

        v00 = float(self.log_odds[y0, x0])
        v10 = float(self.log_odds[y0, x1])
        v01 = float(self.log_odds[y1, x0])
        v11 = float(self.log_odds[y1, x1])

        v0 = v00 * (1.0 - tx) + v10 * tx
        v1 = v01 * (1.0 - ty) + v11 * ty
        return v0 * (1.0 - ty) + v1 * ty

    def score_scan_pose(self, lidar_x, lidar_y, robot_theta, ranges, angles):
        """
        Score how well a scan aligns with the current map.
        lidar_x, lidar_y: the world position of the LiDAR origin.
        """
        score = 0.0
        valid_points = 0

        for i in range(0, len(ranges), SCAN_MATCH_STRIDE):
            r = ranges[i]
            angle = angles[i]

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            beam_angle = robot_theta + angle
            cos_beam = math.cos(beam_angle)
            sin_beam = math.sin(beam_angle)

            quarter_x = lidar_x + 0.25 * r * cos_beam
            quarter_y = lidar_y + 0.25 * r * sin_beam
            mid_x = lidar_x + 0.50 * r * cos_beam
            mid_y = lidar_y + 0.50 * r * sin_beam
            end_x = lidar_x + r * cos_beam
            end_y = lidar_y + r * sin_beam

            score += 1.8 * self.sample_log_odds(end_x, end_y)
            score -= 0.6 * self.sample_log_odds(mid_x, mid_y)
            score -= 0.3 * self.sample_log_odds(quarter_x, quarter_y)
            valid_points += 1

        if valid_points == 0:
            return float("-inf")

        return score / valid_points

    def refine_pose(self, robot_x, robot_y, robot_theta, ranges, angles):
        """
        Refine the supplied robot-center pose with a small local search.
        Internally converts to LiDAR world position using the offset.
        """
        if not self.is_ready_for_scan_matching():
            return robot_x, robot_y, robot_theta, 0.0

        # Helper: compute LiDAR position from robot center pose
        def lidar_from_robot(rx, ry, rt):
            lx = rx + self.lidar_offset_x * math.cos(rt) - self.lidar_offset_y * math.sin(rt)
            ly = ry + self.lidar_offset_x * math.sin(rt) + self.lidar_offset_y * math.cos(rt)
            return lx, ly

        # Score the starting pose
        lx0, ly0 = lidar_from_robot(robot_x, robot_y, robot_theta)
        best_score = self.score_scan_pose(lx0, ly0, robot_theta, ranges, angles)
        best_pose = (robot_x, robot_y, robot_theta)

        # Translation search (in robot center space)
        dx_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE, SCAN_MATCH_TRANSLATION_RANGE + 1e-6, SCAN_MATCH_TRANSLATION_STEP)
        dy_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE, SCAN_MATCH_TRANSLATION_RANGE + 1e-6, SCAN_MATCH_TRANSLATION_STEP)

        for dx in dx_values:
            for dy in dy_values:
                cand_x = robot_x + float(dx)
                cand_y = robot_y + float(dy)
                lx, ly = lidar_from_robot(cand_x, cand_y, robot_theta)
                candidate_score = self.score_scan_pose(lx, ly, robot_theta, ranges, angles)

                if candidate_score > best_score:
                    best_score = candidate_score
                    best_pose = (cand_x, cand_y, robot_theta)

        # Fine search
        if best_pose != (robot_x, robot_y, robot_theta):
            base_x, base_y, _ = best_pose
            dx_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE * 0.5, SCAN_MATCH_TRANSLATION_RANGE * 0.5 + 1e-6, SCAN_MATCH_TRANSLATION_STEP * 0.5)
            dy_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE * 0.5, SCAN_MATCH_TRANSLATION_RANGE * 0.5 + 1e-6, SCAN_MATCH_TRANSLATION_STEP * 0.5)

            for dx in dx_values:
                for dy in dy_values:
                    cand_x = base_x + float(dx)
                    cand_y = base_y + float(dy)
                    lx, ly = lidar_from_robot(cand_x, cand_y, robot_theta)
                    candidate_score = self.score_scan_pose(lx, ly, robot_theta, ranges, angles)

                    if candidate_score > best_score:
                        best_score = candidate_score
                        best_pose = (cand_x, cand_y, robot_theta)

        return best_pose[0], best_pose[1], best_pose[2], best_score

    def mark_occupied_with_neighbors(self, gx, gy):
        """Mark a cell and its 8 neighbors as occupied"""
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return

        self.log_odds[gy, gx] += LOG_ODDS_OCCUPIED
        self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], MIN_LOG_ODDS, MAX_LOG_ODDS)

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.log_odds[ny, nx] += LOG_ODDS_OCCUPIED * 0.75
                    self.log_odds[ny, nx] = np.clip(self.log_odds[ny, nx], MIN_LOG_ODDS, MAX_LOG_ODDS)

    def update(self, lidar_x, lidar_y, robot_theta, ranges, angles, packet_id):
        """
        Update occupancy grid with new LiDAR scan.
        lidar_x, lidar_y: world position of the LiDAR origin.
        """
        if packet_id <= self.last_update_packet_id:
            return False

        self.last_update_packet_id = packet_id

        for i in range(len(ranges)):
            r = ranges[i]
            angle = angles[i]

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            end_x = lidar_x + r * math.cos(robot_theta + angle)
            end_y = lidar_y + r * math.sin(robot_theta + angle)

            gx, gy = self.world_to_grid(end_x, end_y)
            self.mark_occupied_with_neighbors(gx, gy)

            steps = int(r / self.resolution)
            for step in range(steps):
                t = step * self.resolution / r
                ray_x = lidar_x + r * t * math.cos(robot_theta + angle)
                ray_y = lidar_y + r * t * math.sin(robot_theta + angle)

                gx, gy = self.world_to_grid(ray_x, ray_y)
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    self.log_odds[gy, gx] += LOG_ODDS_FREE
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], MIN_LOG_ODDS, MAX_LOG_ODDS)

        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        self.occupancy = np.where(
            self.log_odds == 0, -1,
            np.where(prob > OCCUPIED_THRESHOLD, 100, 0)
        )

        self.map_update_id += 1
        return True

    def get_map(self):
        """Get occupancy map for visualization"""
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "data": self.occupancy.flatten().tolist()
        }

    def get_map_update_id(self):
        return self.map_update_id


class SlamProcessor:
    """SLAM processor that consumes packets and produces pose estimates"""

    def __init__(self, shared_packet: AtomicSharedPacket, shared_state: AtomicSharedState):
        global robot_config
        self.shared_packet = shared_packet
        self.shared_state = shared_state
        self.map_grid = OccupancyGrid(
            MAP_SIZE, MAP_SIZE, MAP_RESOLUTION,
            offset_x=robot_config.get("lidar_offset_x", 0.0),
            offset_y=robot_config.get("lidar_offset_y", 0.0)
        )

        self.last_generation = 0

        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_theta = 0.0
        self.slam_initialized = False

        self.processed_count = 0
        self.state_id = 0
        self.skipped_count = 0

    def process_loop(self, stop_event: threading.Event):
        """Main processing loop - runs in its own thread"""
        while not stop_event.is_set():
            packet, generation = self.shared_packet.get_latest()
            if packet is None or generation == self.last_generation:
                time.sleep(0.0001)
                continue
            self.last_generation = generation
            self.process_packet(packet)

    def process_packet(self, packet: SimulationPacket):
        global robot_config
        """Process a single packet and update SLAM state"""
        self.processed_count += 1

        angular_vel_abs = abs(packet.angular_vel)
        is_rotating_fast = angular_vel_abs > ANGULAR_VEL_THRESHOLD

        if is_rotating_fast:
            self.skipped_count += 1

        current_x = packet.robot_x
        current_y = -packet.robot_y if FLIP_ROBOT_Y_FROM_SIM else packet.robot_y
        current_theta = packet.robot_theta
        match_score = 0.0

        if not is_rotating_fast and ENABLE_SCAN_MATCHING and self.map_grid.is_ready_for_scan_matching():
            refined_x, refined_y, refined_theta, match_score = self.map_grid.refine_pose(
                current_x, current_y, current_theta, packet.ranges, packet.angles
            )

            current_x = (1 - CORRECTION_WEIGHT) * current_x + CORRECTION_WEIGHT * refined_x
            current_y = (1 - CORRECTION_WEIGHT) * current_y + CORRECTION_WEIGHT * refined_y
            theta_diff = wrap_angle(refined_theta - current_theta)
            current_theta = wrap_angle(current_theta + CORRECTION_WEIGHT * 0.5 * theta_diff)

        if not self.slam_initialized:
            self.slam_x = current_x
            self.slam_y = current_y
            self.slam_theta = current_theta
            self.slam_initialized = True
        else:
            self.slam_x = current_x
            self.slam_y = current_y
            self.slam_theta = current_theta

        # --- LiDAR offset correction: compute LiDAR world position ---
        offset_x = robot_config.get("lidar_offset_x", 0.0)
        offset_y = robot_config.get("lidar_offset_y", 0.0)
        lidar_x = self.slam_x + offset_x * math.cos(self.slam_theta) - offset_y * math.sin(self.slam_theta)
        lidar_y = self.slam_y + offset_x * math.sin(self.slam_theta) + offset_y * math.cos(self.slam_theta)

        if not is_rotating_fast:
            self.map_grid.update(
                lidar_x, lidar_y, self.slam_theta,
                packet.ranges, packet.angles, packet.packet_id
            )

        self.state_id += 1
        new_state = SlamState(
            state_id=self.state_id,
            timestamp=packet.timestamp,
            x=self.slam_x,
            y=self.slam_y,
            theta=self.slam_theta,
            match_score=match_score if not is_rotating_fast else 0.0,
            packet_id_processed=packet.packet_id,
            ranges=packet.ranges,
            angles=packet.angles,
            raw_robot_x=packet.robot_x,
            raw_robot_y=packet.robot_y,
            raw_robot_theta=packet.robot_theta,
            left_speed=packet.left_speed,
            right_speed=packet.right_speed,
            auto_navigate=packet.auto_navigate,
            linear_vel=packet.linear_vel,
            angular_vel=packet.angular_vel,
            scan_matching_skipped=is_rotating_fast
        )

        self.shared_state.update(new_state)

    def get_map(self):
        """Get the current occupancy grid"""
        return self.map_grid.get_map()

    def get_map_update_id(self):
        return self.map_grid.get_map_update_id()


class SerialCommandForwarder:
    """
    Command forwarder that sends commands over the serial port to the robot.
    Uses a priority queue to ensure critical commands (e.g., STOP) are sent immediately.
    All commands are written as strings terminated with newline.
    """

    def __init__(self, serial_port: str, baudrate: int):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.command_queue = queue.PriorityQueue()
        self.stop_event = threading.Event()
        self._thread = None
        self._ser = None
        self._lock = threading.Lock()

    def start(self):
        """Open serial port and start the command sender thread."""
        if self._thread and self._thread.is_alive():
            return

        # Open serial port
        try:
            self._ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self._ser.flushInput()
            self._ser.flushOutput()
            print(f"[SerialForwarder] Opened {self.serial_port} at {self.baudrate} baud")
        except Exception as e:
            print(f"[SerialForwarder] Failed to open serial port: {e}")
            return

        self.stop_event.clear()
        self._thread = threading.Thread(target=self._forward_loop, daemon=True)
        self._thread.start()
        print("[SerialForwarder] Command sender thread started")

    def stop(self):
        """Stop the command sender thread and close serial port."""
        self.stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
                print("[SerialForwarder] Serial port closed")

    def send_command(self, command_type: str, payload: Any, priority: int = 10, callback: Optional[callable] = None):
        """
        Queue a command to be sent.

        Args:
            command_type: 'cmd', 'auto', 'path', 'config'
            payload: Command payload (string for 'cmd'/'auto'/'config', list for 'path')
            priority: Lower = higher priority (0 = emergency stop)
            callback: Optional callback for acknowledgment (not used in serial)
        """
        cmd = Command(
            priority=priority,
            timestamp=time.time(),
            command_type=command_type,
            payload=payload,
            callback=callback
        )
        self.command_queue.put((priority, time.time(), cmd))

    def _forward_loop(self):
        """Main loop: take commands from queue and send over serial."""
        while not self.stop_event.is_set():
            try:
                try:
                    _, _, cmd = self.command_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

                # Build the command string
                message = self._build_command(cmd.command_type, cmd.payload)
                if message is None:
                    continue

                # Send over serial
                with self._lock:
                    if self._ser and self._ser.is_open:
                        try:
                            self._ser.write((message + '\n').encode('utf-8'))
                            self._ser.flush()
                        except Exception as e:
                            print(f"[SerialForwarder] Write error: {e}")
                    else:
                        print("[SerialForwarder] Serial port not open, command dropped")

                if cmd.callback:
                    try:
                        cmd.callback(True)
                    except Exception as e:
                        print(f"[SerialForwarder] Callback error: {e}")

            except Exception as e:
                print(f"[SerialForwarder] Forward loop error: {e}")

    def _build_command(self, cmd_type: str, payload) -> Optional[str]:
        """Build the command string to send to the robot."""
        if cmd_type == 'cmd':
            return f"CMD:{payload}"
        elif cmd_type == 'auto':
            value = "1" if payload else "0"
            return f"AUTO:{value}"
        elif cmd_type == 'speed':
            return f"SPEED:{payload}"
        elif cmd_type == 'path':
            if isinstance(payload, list) and payload:
                path_str = ';'.join([f"{p[0]:.3f},{p[1]:.3f}" for p in payload])
                return f"PATH:{path_str}"
            else:
                print("[SerialForwarder] Invalid path payload")
                return None
        elif cmd_type == 'config':
            return f"CONFIG:{payload}"
        else:
            print(f"[SerialForwarder] Unknown command type: {cmd_type}")
            return None


def lidar_receiver(shared_packet: AtomicSharedPacket, stop_event: threading.Event,
                   lidar_port: str, baudrate: int, scan_type: str = "express"):
    """
    Receiver thread: reads LiDAR scans directly from RPLidar and updates shared_packet.
    Accumulates scans until a full 360-degree rotation is complete or a timeout occurs.
    Also reads odometry data from the robot via serial.
    """
    packet_count = 0
    packet_id = 0
    
    # Open LiDAR
    try:
        lidar = RPLidar(lidar_port, baudrate=baudrate)
        info = lidar.get_info()
        print(f"[LiDAR] Connected! Model: {info.get('model', 'unknown')}, Firmware: {info.get('firmware', 'unknown')}")
        scan_generator = lidar.iter_scans(scan_type=scan_type)
    except Exception as e:
        print(f"[LiDAR] Failed to open LiDAR: {e}")
        return

    # Open serial for odometry data from robot
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.01)
        ser.flushInput()
        print(f"[Odometry] Opened serial port {SERIAL_PORT} for odometry")
    except Exception as e:
        print(f"[Odometry] Failed to open serial port: {e}")

    # Current robot state (updated from odometry messages)
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0
    left_speed = 0.0
    right_speed = 0.0
    linear_vel = 0.0
    angular_vel = 0.0
    auto_navigate = True
    last_odom_update = 0

    # Buffer for incomplete serial lines
    line_buffer = ""

    # Statistics for logging
    scan_counter = 0
    last_log_time = time.time()
    scans_in_interval = 0
    points_in_interval = 0
    low_quality_points_in_interval = 0

    # Scan accumulation buffers
    accumulated_ranges = []
    accumulated_angles = []
    accumulated_timestamp = None
    accumulated_scan_count = 0
    MIN_SCANS_TO_ACCUMULATE = 3  # Minimum number of scans to accumulate
    MAX_ACCUMULATION_TIME = 0.5  # Maximum time to accumulate (seconds)
    last_accumulation_start = time.time()

    # Track angle range to detect full rotation
    MIN_ANGLE_COVERAGE = 350  # Degrees - require at least this much coverage
    accumulated_angle_min = float('inf')
    accumulated_angle_max = -float('inf')

    while not stop_event.is_set():
        # ---- Read LiDAR scan ----
        try:
            scan = next(scan_generator)
            packet_id += 1
            scan_counter += 1
            
            # Convert scan to ranges and angles
            ranges = []
            angles = []
            low_quality_count = 0
            low_quality_points = []
            
            for quality, angle, distance in scan:
                if distance > 0:  # Only include valid readings
                    # Check for low quality readings
                    if quality < 15:
                        low_quality_count += 1
                        low_quality_points.append((quality, angle, distance))
                        # Still include low quality points but with a warning
                        # (uncomment below to filter out low quality points)
                        # continue
                    
                    # Convert to meters (RPLidar returns mm)
                    ranges.append(distance / 1000.0)
                    angles.append(math.radians(angle))
            
            # Log low quality readings
            if low_quality_count > 0:
                print(f"[LiDAR WARNING] Scan #{scan_counter}: {low_quality_count} low quality points (quality < 15)")
                # Show first 3 low quality points as examples
                for i, (q, a, d) in enumerate(low_quality_points[:3]):
                    print(f"  - Point {i+1}: quality={q}, angle={a:.1f} deg, distance={d/1000.0:.3f}m")
                if low_quality_count > 3:
                    print(f"  - ... and {low_quality_count - 3} more low quality points")
            
            # ---- Accumulate scan data ----
            if len(ranges) > 0:
                # Initialize accumulation on first scan
                if accumulated_timestamp is None:
                    accumulated_timestamp = time.time()
                    last_accumulation_start = accumulated_timestamp
                    accumulated_angle_min = float('inf')
                    accumulated_angle_max = -float('inf')
                
                # Add points to accumulation
                accumulated_ranges.extend(ranges)
                accumulated_angles.extend(angles)
                accumulated_scan_count += 1
                
                # Track angle coverage (convert from radians to degrees for easier comparison)
                for angle_rad in angles:
                    angle_deg = math.degrees(angle_rad) % 360
                    if angle_deg < accumulated_angle_min:
                        accumulated_angle_min = angle_deg
                    if angle_deg > accumulated_angle_max:
                        accumulated_angle_max = angle_deg
                
                # Calculate angle coverage
                angle_coverage = accumulated_angle_max - accumulated_angle_min
                if angle_coverage > 360:
                    angle_coverage = 360
                
                # Determine if we should send the accumulated data
                current_time = time.time()
                time_elapsed = current_time - last_accumulation_start
                
                should_send = False
                send_reason = ""
                
                # Condition 1: We have enough scans AND good angle coverage
                if (accumulated_scan_count >= MIN_SCANS_TO_ACCUMULATE and 
                    angle_coverage >= MIN_ANGLE_COVERAGE):
                    should_send = True
                    send_reason = f"coverage {angle_coverage:.1f} deg with {accumulated_scan_count} scans"
                
                # Condition 2: Maximum accumulation time reached
                elif time_elapsed >= MAX_ACCUMULATION_TIME and accumulated_scan_count >= 2:
                    should_send = True
                    send_reason = f"timeout ({time_elapsed:.2f}s) with coverage {angle_coverage:.1f} deg"
                
                # Condition 3: We have many scans even if coverage isn't perfect
                elif accumulated_scan_count >= 10:
                    should_send = True
                    send_reason = f"many scans ({accumulated_scan_count}) with coverage {angle_coverage:.1f} deg"
                
                if should_send:
                    # Log accumulation details
                    print(f"[LiDAR] Accumulated {len(accumulated_ranges)} points from {accumulated_scan_count} scans")
                    print(f"  - Angle coverage: {angle_coverage:.1f} degrees")
                    print(f"  - Reason: {send_reason}")
                    print(f"  - Point count: {len(accumulated_ranges)}")
                    
                    # Create packet with accumulated data
                    packet = SimulationPacket(
                        packet_id=packet_id,
                        timestamp=time.time(),
                        robot_x=robot_x,
                        robot_y=robot_y,
                        robot_theta=robot_theta,
                        ranges=tuple(accumulated_ranges),
                        angles=tuple(accumulated_angles),
                        left_speed=left_speed,
                        right_speed=right_speed,
                        linear_vel=linear_vel,
                        angular_vel=angular_vel,
                        auto_navigate=auto_navigate
                    )
                    shared_packet.update(packet)
                    packet_count += 1
                    
                    print(f"[LiDAR] Packet #{packet_id} updated shared state: {len(accumulated_ranges)} points")
                    
                    # Reset accumulation
                    accumulated_ranges = []
                    accumulated_angles = []
                    accumulated_scan_count = 0
                    accumulated_timestamp = None
                    accumulated_angle_min = float('inf')
                    accumulated_angle_max = -float('inf')
            
            # Update statistics
            scans_in_interval += 1
            points_in_interval += len(ranges)
            low_quality_points_in_interval += low_quality_count
            
            # Log statistics every 10 seconds
            now = time.time()
            if now - last_log_time >= 10.0:
                elapsed = now - last_log_time
                low_quality_ratio = low_quality_points_in_interval / points_in_interval if points_in_interval > 0 else 0
                print(f"[LiDAR] Statistics (last {elapsed:.1f}s):")
                print(f"  - Scans: {scans_in_interval} ({scans_in_interval/elapsed:.1f} scans/s)")
                print(f"  - Points: {points_in_interval} ({points_in_interval/elapsed:.1f} points/s)")
                print(f"  - Low quality points: {low_quality_points_in_interval} ({low_quality_ratio*100:.1f}%)")
                print(f"  - Total scans: {scan_counter}")
                print(f"  - Total packets: {packet_id}")
                last_log_time = now
                scans_in_interval = 0
                points_in_interval = 0
                low_quality_points_in_interval = 0
                    
        except StopIteration:
            # Scan generator ended, restart it
            print(f"[LiDAR] Scan generator ended after {scan_counter} scans, restarting...")
            try:
                lidar.stop()
                lidar.disconnect()
                lidar = RPLidar(lidar_port, baudrate=baudrate)
                scan_generator = lidar.iter_scans(scan_type=scan_type)
                print("[LiDAR] Successfully reconnected")
            except Exception as e:
                print(f"[LiDAR] Reconnect error: {e}")
                time.sleep(1)
            continue
        except Exception as e:
            print(f"[LiDAR] Scan error: {e}")
            time.sleep(0.01)
            continue

        # ---- Read odometry data from robot ----
        if ser and ser.is_open:
            try:
                # Read all available data
                while ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    try:
                        text = data.decode('utf-8')
                        line_buffer += text
                    except UnicodeDecodeError:
                        # Partial unicode, skip
                        pass
                    
                    # Process complete lines
                    while '\n' in line_buffer:
                        line, line_buffer = line_buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        # Parse odometry message from robot
                        # Expected format: "Position: (X.XXX m, Y.XXX m) | Theta: XXX.X deg"
                        if line.startswith("Position:"):
                            try:
                                # Extract position
                                pos_start = line.find('(') + 1
                                pos_end = line.find(')')
                                if pos_start > 0 and pos_end > pos_start:
                                    pos_str = line[pos_start:pos_end]
                                    parts = pos_str.split(',')
                                    if len(parts) >= 2:
                                        robot_x = float(parts[0].strip().replace('m', '').strip())
                                        robot_y = float(parts[1].strip().replace('m', '').strip())
                                
                                # Extract theta
                                theta_start = line.find('Theta:')
                                if theta_start > 0:
                                    theta_str = line[theta_start + 6:].strip()
                                    theta_str = theta_str.replace('deg', '').strip()
                                    robot_theta = math.radians(float(theta_str))
                                
                                last_odom_update = time.time()
                            except Exception as e:
                                pass
            except Exception as e:
                pass

    # Cleanup
    try:
        lidar.stop()
        lidar.disconnect()
        print("[LiDAR] Disconnected cleanly")
    except:
        pass
    if ser and ser.is_open:
        ser.close()
    
    print(f"[Receiver] Stopped. Total scans: {scan_counter}, total packets: {packet_count}, last packet_id: {packet_id}")
    
def coarse_grid_from_map(map_data, coarse_factor=COARSE_FACTOR, robot_width=None):
    """
    Create a downsampled occupancy grid (coarse) and then inflate obstacles
    on that coarse grid by a number of coarse cells derived from robot radius.
    """
    if not map_data:
        return None
    if robot_width is None:
        robot_width = robot_config.get('robot_width', 0.41)

    width = map_data['width']
    height = map_data['height']
    res = map_data['resolution']
    data = np.array(map_data['data'], dtype=np.int8).reshape((height, width))

    # 1. Downsample to coarse (original method)
    cf = int(coarse_factor)
    cw = max(1, width // cf)
    ch = max(1, height // cf)
    cres = res * cf

    # Build coarse binary occupancy (1 = occupied, 0 = free)
    coarse = np.zeros((ch, cw), dtype=np.uint8)   # use uint8 for boolean ops
    for cy in range(ch):
        for cx in range(cw):
            fx0 = cx * cf
            fy0 = cy * cf
            fx1 = min(width, fx0 + cf)
            fy1 = min(height, fy0 + cf)
            block = data[fy0:fy1, fx0:fx1]
            if np.any(block == 100):
                coarse[cy, cx] = 1

    # 2. Compute inflation radius in coarse cells
    robot_radius = robot_width / 2.0
    inflation_cells = max(1, int(robot_radius / cres))

    # 3. Dilate the coarse occupancy using 8-connectivity
    struct = generate_binary_structure(2, 2)   # 8-neighbour kernel
    inflated = binary_dilation(coarse, structure=struct, iterations=inflation_cells)

    # Convert back to int8 (0/1) for A* (which expects 0 = free, 1 = occupied)
    inflated = inflated.astype(np.int8)

    return {
        'width': cw,
        'height': ch,
        'resolution': cres,
        'origin_x': -width * res / 2.0,
        'origin_y': -height * res / 2.0,
        'data': inflated,   # now 0/1 (occupied inflated)
    }


def astar_plan(coarse, start_xy, goal_xy):
    """A* on coarse grid. Returns list of world points (x,y)."""
    if coarse is None:
        return []

    w = coarse['width']
    h = coarse['height']
    cres = coarse['resolution']
    ox = coarse['origin_x']
    oy = coarse['origin_y']

    def to_idx(x, y):
        gx = int((x - ox) / cres)
        gy = int((y - oy) / cres)
        return gx, gy

    def to_world(gx, gy):
        x = ox + (gx + 0.5) * cres
        y = oy + (gy + 0.5) * cres
        return x, y

    sx, sy = start_xy
    gx, gy = goal_xy
    si, sj = to_idx(sx, sy)
    gi, gj = to_idx(gx, gy)

    if si < 0 or sj < 0 or si >= w or sj >= h:
        return []
    if gi < 0 or gj < 0 or gi >= w or gj >= h:
        return []
    grid = coarse['data']
    if grid[sj, si] == 1 or grid[gj, gi] == 1:
        return []

    def h_cost(a, b):
        (x1, y1) = a
        (x2, y2) = b
        return math.hypot(x1 - x2, y1 - y2)

    start = (si, sj)
    goal = (gi, gj)

    open_set = []
    heapq.heappush(open_set, (0.0, start))
    came_from = {}
    gscore = {start: 0.0}

    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break

        for dx, dy in neighbors:
            nx = current[0] + dx
            ny = current[1] + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if grid[ny, nx] == 1:
                continue
            tentative_g = gscore[current] + (math.hypot(dx, dy))
            neigh = (nx, ny)
            if tentative_g < gscore.get(neigh, float('inf')):
                came_from[neigh] = current
                gscore[neigh] = tentative_g
                f = tentative_g + h_cost(neigh, goal)
                heapq.heappush(open_set, (f, neigh))

    if goal not in came_from and start != goal:
        return []

    path_idx = [goal]
    cur = goal
    while cur != start:
        cur = came_from.get(cur)
        if cur is None:
            break
        path_idx.append(cur)
    path_idx.reverse()

    path = [to_world(px, py) for (px, py) in path_idx]

    # Simplify path using Bresenham's line
    simplified = []
    def bresenham_clear(a, b):
        ax, ay = a
        bx, by = b
        ai, aj = to_idx(ax, ay)
        bi, bj = to_idx(bx, by)
        di = abs(bi - ai)
        dj = abs(bj - aj)
        si = 1 if ai < bi else -1
        sj = 1 if aj < bj else -1
        err = di - dj
        i = ai
        j = aj
        while True:
            if grid[j, i] == 1:
                return False
            if i == bi and j == bj:
                break
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return True

    if path:
        last = path[0]
        simplified.append(last)
        prev = last
        for p in path[1:]:
            if not bresenham_clear(last, p):
                simplified.append(prev)
                last = prev
            prev = p
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])
        # Remove first point (robot position) to avoid immediate stopping
        if simplified and len(simplified) > 1:
            simplified = simplified[1:]

    return simplified


class PlannerWorker:
    """
    Planner that handles a sequence of waypoints.
    Continuously replans the path from the robot to the current active waypoint
    and sends the updated path to the robot.
    """

    def __init__(self, slam_processor, shared_state, command_forwarder):
        self.slam_processor = slam_processor
        self.shared_state = shared_state
        self.command_forwarder = command_forwarder
        self.lock = threading.Lock()

        # Waypoint management
        self.waypoints = []                # list of (x, y)
        self.current_wp_index = 0          # index into waypoints
        self.loop_mode = False
        self.finished = False              # set when all waypoints done (no loop)

        # Path and goal
        self.path = []
        self._goal = None                  # current active goal (waypoint)

        # Re-planning state
        self.last_plan_time = 0
        self.last_map_update_id = -1
        self.replan_interval = 2.0         # seconds
        self.start_x = 0.0
        self.start_y = 0.0
        self.returning_to_start = False

        # For external requests (e.g., set_goal from relay)
        self.request_queue = deque()
        
    def get_remaining_waypoints(self):
        with self.lock:
            if self.finished or not self.waypoints or self.returning_to_start:
                return []
            return self.waypoints[self.current_wp_index:]

    def set_waypoints(self, waypoints, loop=False):
        with self.lock:
            self.waypoints = waypoints[:]
            self.loop_mode = loop
            self.current_wp_index = 0
            self.finished = False
            self.returning_to_start = False
            # Store current robot position as start
            state = self.shared_state.get_latest()
            if state:
                self.start_x = state.x
                self.start_y = state.y
            else:
                self.start_x = 0.0
                self.start_y = 0.0
            if self.waypoints:
                self._goal = self.waypoints[0]
                self.request_queue.append(('plan',))
            else:
                self._goal = None
                self.path = []

    def set_goal(self, x, y, loop=False):
        """Convenience: set a single goal (clears waypoints)."""
        print(f"[Planner] set_goal called with x={x}, y={y}, loop={loop}")
        
        with self.lock:
            self.waypoints = [(x, y)]
            self.loop_mode = loop
            self.current_wp_index = 0
            self.finished = False
            self._goal = (x, y)
            print(f"[Planner] Single goal set: {self._goal}")
            self.request_queue.append(('plan',))

    def get_goal(self):
        with self.lock:
            return self._goal

    def get_path(self):
        with self.lock:
            return list(self.path)

    def _advance_to_next_waypoint(self):
        with self.lock:
            if not self.waypoints:
                return False

            # If we are already returning to start, finishing that leg means stop.
            if self.returning_to_start:
                self.finished = True
                self._goal = None
                self.returning_to_start = False
                return False

            # Normal waypoint advancement
            if self.current_wp_index + 1 < len(self.waypoints):
                self.current_wp_index += 1
                self._goal = self.waypoints[self.current_wp_index]
                return True
            else:
                print("ALL WAYPOINTS REACHED")
                # Reached the last waypoint: advance index past the end
                self.current_wp_index = len(self.waypoints)
                if self.loop_mode:
                    self.returning_to_start = True
                    self._goal = (self.start_x, self.start_y)
                    return True          # new goal is the start position
                else:
                    self.finished = True
                    self._goal = None
                    return False         # no more goals

    def trim_path(self, robot_x, robot_y):
        """
        Remove all reached waypoints from the front of the path.
        If the path becomes empty, advance to the next waypoint (or stop).
        """
        need_advance = False

        with self.lock:
            if not self.path or self.finished:
                return

            # Remove any number of consecutive path points that are within 0.2 m
            while self.path and math.hypot(self.path[0][0] - robot_x, self.path[0][1] - robot_y) < 0.2:
                _ = self.path.pop(0)
                print("WAYPOINT REMOVED")

            if not self.path:
                need_advance = True

        if need_advance:
            has_next = self._advance_to_next_waypoint()
            if has_next:
                with self.lock:
                    self.request_queue.append(('plan',))
            else:
                # No more waypoints (and not returning to start) – stop robot
                self.command_forwarder.send_command('cmd', 'stop', priority=1)

    def check_replan(self, robot_x, robot_y, map_update_id):
        """
        Called from the main loop to decide if we should re-plan.
        Returns True if a new plan was queued.
        """
        with self.lock:
            if self.finished or self._goal is None:
                return False

            # Force re-plan if map changed or time elapsed
            map_changed = map_update_id != self.last_map_update_id
            time_elapsed = time.time() - self.last_plan_time > self.replan_interval
            
            if map_changed or time_elapsed:
                self.request_queue.append(('plan',))
                self.last_map_update_id = map_update_id
                self.last_plan_time = time.time()
                return True
        return False

    def planner_loop(self, stop_event: threading.Event):
        """Main planning loop – processes planning requests."""
        while not stop_event.is_set():
            # Check for planning requests
            plan_requested = False
            with self.lock:
                if self.request_queue:
                    self.request_queue.popleft()   # discard, just a trigger
                    plan_requested = True

            if not plan_requested:
                time.sleep(0.05)
                continue

            # Get current robot pose
            slam_state = self.shared_state.get_latest()
            if slam_state is None:
                continue
            sx = slam_state.x
            sy = slam_state.y

            # Get the current goal
            with self.lock:
                goal = self._goal
                if goal is None:
                    continue

            # Plan path from robot to goal
            map_data = self.slam_processor.get_map()
            robot_width = robot_config.get('robot_width', 0.41)
            coarse = coarse_grid_from_map(map_data, COARSE_FACTOR, robot_width)
            
            if coarse is None:
                continue
                
            planned = astar_plan(coarse, (sx, sy), goal)
            
            with self.lock:
                self.path = planned
                self.last_plan_time = time.time()

            # Send path to robot (enable auto mode first)
            if planned:
                self.command_forwarder.send_command('auto', True, priority=2)
                # Convert to robot frame (flip Y)
                robot_path = [(x, -y) for x, y in planned]
                self.command_forwarder.send_command('path', robot_path, priority=5)

            time.sleep(0.05)


# ============================================================================
# Relay WebSocket Client functions
# ============================================================================

relay_ws = None
relay_connected = False
relay_lock = threading.Lock()


async def connect_to_relay():
    """Connect to the relay server and register."""
    global relay_ws, relay_connected
    attempts = 0
    while True:
        try:
            ssl_context = ssl.create_default_context()

            print(f"[Bridge] Connecting to relay at {RELAY_URL} ...")
            relay_ws = await websockets.connect(
                RELAY_URL,
                ssl=ssl_context,
            )
            # Register
            register_msg = {
                "type": "register",
                "role": "bridge",
                "bridgeId": BRIDGE_ID,
                "token": BRIDGE_TOKEN
            }

            await relay_ws.send(json.dumps(register_msg))
            response = await relay_ws.recv()
            resp_data = json.loads(response)
            if resp_data.get("type") == "registered":
                print(f"[Bridge] Registered with relay as {BRIDGE_ID}")
                with relay_lock:
                    relay_connected = True
                asyncio.create_task(relay_message_handler(relay_ws))
                return
            else:
                print(f"[Bridge] Registration failed: {response}")
                await relay_ws.close()
                with relay_lock:
                    relay_connected = False
        except Exception as e:
            print(f"[Bridge] Connection error: {e}")
            with relay_lock:
                relay_connected = False
        attempts += 1
        if MAX_RECONNECT_ATTEMPTS > 0 and attempts >= MAX_RECONNECT_ATTEMPTS:
            print("[Bridge] Max reconnect attempts reached. Giving up.")
            break
        await asyncio.sleep(RECONNECT_DELAY)


async def relay_message_handler(ws):
    """Receive and process messages from relay (commands from browser)."""
    global relay_connected
    try:
        async for message in ws:
            try:
                data = json.loads(message)
                await process_relay_message(data)
            except json.JSONDecodeError:
                print(f"[Bridge] Invalid JSON from relay: {message}")
    except websockets.exceptions.ConnectionClosed:
        print("[Bridge] Relay connection closed")
        with relay_lock:
            relay_connected = False
    except Exception as e:
        print(f"[Bridge] Relay handler error: {e}")
        with relay_lock:
            relay_connected = False
    finally:
        if not relay_connected:
            asyncio.create_task(connect_to_relay())


async def process_relay_message(data):
    """Process command messages from the relay."""
    print(f"[Bridge] process_relay_message: Received data: {data}")
    
    if data.get('type') == 'command':
        cmd = data.get('command')
        print(f"[Bridge] Received command: {cmd}")
        if cmd == 'forward':
            command_forwarder.send_command('cmd', 'forward', priority=3)
        elif cmd == 'backward':
            command_forwarder.send_command('cmd', 'backward', priority=3)
        elif cmd == 'left':
            command_forwarder.send_command('cmd', 'left', priority=3)
        elif cmd == 'right':
            command_forwarder.send_command('cmd', 'right', priority=3)
        elif cmd == 'stop':
            command_forwarder.send_command('cmd', 'stop', priority=1)
        elif cmd == 'auto':
            command_forwarder.send_command('cmd', 'auto', priority=2)
        else:
            command_forwarder.send_command('cmd', cmd, priority=10)

    elif data.get('type') == 'set_goal':
        gx = float(data.get('x', 0.0))
        gy = float(data.get('y', 0.0))
        print(f"[Bridge] Goal set: ({gx:.2f},{gy:.2f})")
        planner.set_goal(gx, gy)

    elif data.get('type') == 'set_waypoints':
        waypoints = data.get('waypoints', [])
        loop = data.get('loop', False)
        print(f"[Bridge] Received set_waypoints with {len(waypoints)} waypoints, loop={loop}")
        print(f"[Bridge] Waypoints data: {waypoints}")
        
        if waypoints:
            wp_list = [(p['x'], p['y']) for p in waypoints]
            print(f"[Bridge] Converted to wp_list: {wp_list}")
            print(f"[Bridge] Calling planner.set_waypoints with {len(wp_list)} waypoints")
            planner.set_waypoints(wp_list, loop)
        else:
            print(f"[Bridge] Empty waypoints list received")

    elif data.get('type') == 'set_loop':
        loop = data.get('loop', False)
        print(f"[Bridge] set_loop: Setting loop mode to {loop}")
        with planner.lock:
            planner.loop_mode = loop
        print(f"[Bridge] Loop mode set to {loop}")

    elif data.get('type') == 'autonomy':
        auto = bool(data.get('auto_navigate', True))
        print(f"[Bridge] Autonomy set: {auto}")
        command_forwarder.send_command('auto', auto, priority=2)
        if not auto:
            command_forwarder.send_command('cmd', 'stop', priority=1)
    elif data.get('type') == 'set_speed':
        speed = float(data.get('speed', 4.0))
        speed = max(0.1, min(10.0, speed))   # clamp
        print(f"[Bridge] Setting max speed to {speed:.1f} rad/s")
        command_forwarder.send_command('speed', speed, priority=5)
    elif data.get('type') == 'get_config':
        # Send current config back to the requester
        await relay_ws.send(json.dumps({
            'type': 'config',
            'config': robot_config
        }))

    elif data.get('type') == 'set_config':
        new_config = data.get('config', {})
        for key, value in new_config.items():
            if key in robot_config:
                robot_config[key] = value
        save_config()
        send_config_to_robot()
        await relay_ws.send(json.dumps({
            'type': 'config_updated',
            'config': robot_config
        }))


# ============================================================================
# Broadcaster – sends data to relay
# ============================================================================

async def websocket_broadcaster(shared_state, slam_processor, planner, stop_event):
    """Broadcast pose, map, and path data to the relay connection."""
    global relay_ws, relay_connected

    pose_interval = 1.0 / BROADCAST_POSE_HZ
    map_interval = 1.0 / BROADCAST_MAP_HZ

    last_pose_broadcast = 0
    last_map_broadcast = 0
    last_sent_path_sig = None

    prev_remaining_wp_len = 0
    
    # Statistics for logging
    total_messages_sent = 0
    total_bytes_sent = 0
    last_log_time = time.time()
    messages_in_current_interval = 0
    bytes_in_current_interval = 0

    while not stop_event.is_set():
        now = time.time()

        slam_state = shared_state.get_latest()

        coarse_grid = None
        if slam_processor and slam_processor.map_grid:
            map_data = slam_processor.get_map()
            if map_data:
                robot_width = robot_config.get('robot_width', 0.41)
                coarse = coarse_grid_from_map(map_data, COARSE_FACTOR, robot_width)
                if coarse:
                    coarse_grid = {
                        'width': coarse['width'],
                        'height': coarse['height'],
                        'resolution': coarse['resolution'],
                        'origin_x': coarse['origin_x'],
                        'origin_y': coarse['origin_y'],
                        'data': coarse['data'].flatten().tolist() if isinstance(coarse['data'], np.ndarray) else coarse['data']
                    }

        if slam_state and relay_connected and relay_ws and relay_ws.state == websockets.protocol.State.OPEN:
            if now - last_pose_broadcast >= pose_interval:
                last_pose_broadcast = now
                
                # Build the complete message
                output_message = {
                    "type": "lidar_scan",
                    "timestamp": slam_state.timestamp,
                    "num_points": len(slam_state.ranges),
                    "min_range": MIN_RANGE,
                    "max_range": MAX_RANGE,
                    "fov": 6.283,
                    "ranges": list(slam_state.ranges),
                    "angles": list(slam_state.angles),
                    "robot_x": slam_state.x,
                    "robot_y": slam_state.y,
                    "robot_theta": -slam_state.theta if FLIP_THETA_FOR_VISUALIZATION else slam_state.theta,
                    "raw_robot_x": slam_state.raw_robot_x,
                    "raw_robot_y": slam_state.raw_robot_y,
                    "raw_robot_theta": -slam_state.raw_robot_theta if FLIP_THETA_FOR_VISUALIZATION else slam_state.raw_robot_theta,
                    "pose_source": "slam",
                    "left_speed": slam_state.left_speed,
                    "right_speed": slam_state.right_speed,
                    "auto_navigate": slam_state.auto_navigate,
                    "linear_vel": slam_state.linear_vel,
                    "angular_vel": slam_state.angular_vel,
                    "slam_match_score": slam_state.match_score,
                    "correction_weight": CORRECTION_WEIGHT if ENABLE_SCAN_MATCHING else 0.0,
                    "scan_matching_skipped": slam_state.scan_matching_skipped,
                }

                if now - last_map_broadcast >= map_interval:
                    output_message["map"] = slam_processor.get_map()
                    if coarse_grid:
                        output_message["coarse_grid"] = coarse_grid
                    last_map_broadcast = now

                current_path = planner.get_path() if planner is not None else []
                if current_path:
                    output_message['path'] = [{'x': p[0], 'y': p[1]} for p in current_path]

                if planner.get_goal():
                    output_message["goal"] = {"x": planner.get_goal()[0], "y": planner.get_goal()[1]}

                remaining_wp = planner.get_remaining_waypoints() if planner is not None else []
                if len(remaining_wp) != prev_remaining_wp_len:
                    output_message["remaining_waypoints"] = [{"x": wp[0], "y": wp[1]} for wp in remaining_wp]
                prev_remaining_wp_len = len(remaining_wp)

                # Convert to JSON and calculate size
                json_message = json.dumps(output_message)
                message_size = len(json_message.encode('utf-8'))
                
                # Log the message details
                # print(f"[Broadcaster] Sending LiDAR data to relay:")
                # print(f"  - Packet ID: {slam_state.packet_id_processed}")
                # print(f"  - Timestamp: {slam_state.timestamp:.3f}")
                # print(f"  - Num points: {len(slam_state.ranges)}")
                # print(f"  - Robot pose: ({slam_state.x:.3f}, {slam_state.y:.3f}, {math.degrees(slam_state.theta):.1f} deg)")
                # print(f"  - Robot speed: L={slam_state.left_speed:.2f}, R={slam_state.right_speed:.2f} rad/s")
                # print(f"  - Auto mode: {slam_state.auto_navigate}")
                # print(f"  - Match score: {slam_state.match_score:.3f}")
                
                if 'map' in output_message:
                    map_data = output_message['map']
                    occupied = sum(1 for cell in map_data['data'] if cell == 100)
                    free = sum(1 for cell in map_data['data'] if cell == 0)
                    unknown = sum(1 for cell in map_data['data'] if cell == -1)
                    print(f"  - Map: {occupied} occupied, {free} free, {unknown} unknown")
                
                if 'path' in output_message:
                    print(f"  - Path: {len(output_message['path'])} waypoints")
                
                if 'goal' in output_message:
                    goal = output_message['goal']
                    print(f"  - Goal: ({goal['x']:.3f}, {goal['y']:.3f})")
                
                # print(f"  - Message size: {message_size} bytes")
                
                # Calculate statistics
                total_messages_sent += 1
                total_bytes_sent += message_size
                messages_in_current_interval += 1
                bytes_in_current_interval += message_size
                
                # Log statistics every 10 seconds
                if now - last_log_time >= 10.0:
                    elapsed = now - last_log_time
                    msg_rate = messages_in_current_interval / elapsed
                    byte_rate = bytes_in_current_interval / elapsed
                    # print(f"[Broadcaster] Statistics (last {elapsed:.1f}s):")
                    # print(f"  - Messages: {messages_in_current_interval} ({msg_rate:.1f} msg/s)")
                    # print(f"  - Data: {bytes_in_current_interval/1024:.2f} KB ({byte_rate/1024:.2f} KB/s)")
                    # print(f"  - Total messages: {total_messages_sent}")
                    # print(f"  - Total data: {total_bytes_sent/1024:.2f} KB")
                    last_log_time = now
                    messages_in_current_interval = 0
                    bytes_in_current_interval = 0

                # Send to relay
                try:
                    await asyncio.wait_for(
                        relay_ws.send(json_message),
                        timeout=5.0
                    )
                    # print(f"[Broadcaster] Successfully sent to relay")
                    
                except asyncio.TimeoutError:
                    print(f"[Broadcaster] SEND TIMEOUT - relay may be unresponsive")
                    print(f"  - Failed message size: {message_size} bytes")
                    print(f"  - Failed message type: lidar_scan")
                    with relay_lock:
                        relay_connected = False
                        
                except websockets.exceptions.ConnectionClosed as e:
                    print(f"[Broadcaster] CONNECTION CLOSED while sending: {e}")
                    print(f"  - Failed message size: {message_size} bytes")
                    with relay_lock:
                        relay_connected = False
                        
                except Exception as e:
                    print(f"[Broadcaster] SEND ERROR: {type(e).__name__}: {e}")
                    print(f"  - Failed message size: {message_size} bytes")
                    with relay_lock:
                        relay_connected = False

        elif not relay_connected:
            # Log relay connection status periodically
            if now - last_pose_broadcast >= 10.0:  # Log every 10 seconds if disconnected
                last_pose_broadcast = now
                print(f"[Broadcaster] Relay not connected - cannot send LiDAR data")
                if slam_state:
                    print(f"  - Would have sent: {len(slam_state.ranges)} points at pose ({slam_state.x:.3f}, {slam_state.y:.3f})")

        await asyncio.sleep(0.001)

# ============================================================================
# main
# ============================================================================

async def main():
    global planner, command_forwarder
    shared_packet = AtomicSharedPacket()
    shared_state = AtomicSharedState()
    stop_event = threading.Event()

    # Initialize command forwarder with serial port
    command_forwarder = SerialCommandForwarder(SERIAL_PORT, SERIAL_BAUDRATE)
    command_forwarder.start()

    slam_processor = SlamProcessor(shared_packet, shared_state)

    # Start LiDAR receiver thread (reads LiDAR + odometry from robot)
    lidar_thread = threading.Thread(
        target=lidar_receiver,
        args=(shared_packet, stop_event, LIDAR_PORT, LIDAR_BAUDRATE, LIDAR_SCAN_TYPE),
        daemon=True
    )
    lidar_thread.start()

    # SLAM processor thread
    slam_thread = threading.Thread(target=slam_processor.process_loop, args=(stop_event,), daemon=True)
    slam_thread.start()

    # Planner thread
    planner = PlannerWorker(slam_processor, shared_state, command_forwarder)
    planner_thread = threading.Thread(target=planner.planner_loop, args=(stop_event,), daemon=True)
    planner_thread.start()

    # Connect to relay
    await connect_to_relay()

    # Start broadcaster
    broadcaster_task = asyncio.create_task(
        websocket_broadcaster(shared_state, slam_processor, planner, stop_event)
    )
    
    load_config()

    # Main loop: trim path and check for replan
    try:
        while not stop_event.is_set():
            slam_state = shared_state.get_latest()
            if slam_state and planner:
                planner.trim_path(slam_state.x, slam_state.y)
                map_update_id = slam_processor.get_map_update_id()
                planner.check_replan(slam_state.x, slam_state.y, map_update_id)
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        stop_event.set()
        broadcaster_task.cancel()
        command_forwarder.stop()
        lidar_thread.join(timeout=2)
        slam_thread.join(timeout=2)
        planner_thread.join(timeout=1)
        if relay_ws:
            await relay_ws.close()
        print("[Main] Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")
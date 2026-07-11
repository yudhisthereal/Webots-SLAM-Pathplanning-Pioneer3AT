#!/usr/bin/env python3
"""
UDP to WebSocket Bridge with Grid Map SLAM
New architecture: UI owns state, bridge forwards commands.
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
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, Any, List
from scipy.ndimage import binary_dilation, generate_binary_structure

from rplidar import RPLidar

# ============ Configuration ============
SERIAL_PORT = "COM12"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1

LIDAR_PORT = 'COM6'
LIDAR_BAUDRATE = 115200
LIDAR_SCAN_TYPE = "normal"

BROADCAST_POSE_HZ = 30
BROADCAST_MAP_HZ = 1

COARSE_FACTOR = 4
ROBOT_WIDTH = 0.41

MAP_SIZE = 200
MAP_RESOLUTION = 0.05
MAP_ORIGIN_X = -MAP_SIZE * MAP_RESOLUTION / 2
MAP_ORIGIN_Y = -MAP_SIZE * MAP_RESOLUTION / 2

LOG_ODDS_OCCUPIED = 0.8
LOG_ODDS_FREE = -0.4
MAX_LOG_ODDS = 3.0
MIN_LOG_ODDS = -3.0
OCCUPIED_THRESHOLD = 0.6

MAX_RANGE = 4.0
MIN_RANGE = 0.1

SCAN_MATCH_STRIDE = 3
SCAN_MATCH_MIN_FEATURES = 150
SCAN_MATCH_TRANSLATION_RANGE = 0.20
SCAN_MATCH_TRANSLATION_STEP = 0.05

ANGULAR_VEL_THRESHOLD = 0.2

FLIP_THETA_FOR_VISUALIZATION = False
FLIP_ROBOT_Y_FROM_SIM = False

CORRECTION_WEIGHT = 0.05
ENABLE_SCAN_MATCHING = False

RELAY_URL = "wss://kmo-relayserver.yudhisthereal.workers.dev"
BRIDGE_ID = "my_robot_01"
BRIDGE_TOKEN = "kmo-bridge-token1"
RECONNECT_DELAY = 3.0
MAX_RECONNECT_ATTEMPTS = 0

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

# ============ Global state ============
shared_packet = None
shared_state = None
planner = None
command_forwarder = None
serial_manager = None
relay_ws = None
relay_connected = False
relay_lock = threading.Lock()
slam_processor = None
reset_coordinator = None

# ---- Robot physical dimensions ----
ROBOT_LENGTH = 0.71       # meters
ROBOT_HALF_WIDTH = 0.195  # 0.39/2

# ============ Reset Coordinator ============
class ResetCoordinator:
    """
    Deterministic state machine for SLAM reset.
    Uses asyncio.Event for clean async waiting.
    All state transitions happen through public methods.
    """
    IDLE = 0
    WAITING_ODOM = 1
    DISCARDING_SCAN = 2
    
    def __init__(self, loop):
        self._lock = threading.Lock()
        self._state = self.IDLE
        self.current_epoch = 0
        self.odom_confirmed = asyncio.Event()
        self.scan_discarded = asyncio.Event()
        self._scans_to_discard = 1
        self.loop = loop
        self._aborted = False
    
    def start_reset(self):
        """Non-blocking. Returns True if reset was started."""
        with self._lock:
            if self._state != self.IDLE:
                return False
            self._state = self.WAITING_ODOM
            self._aborted = False
            self.odom_confirmed.clear()
            self.scan_discarded.clear()
            self._scans_to_discard = 1
            print("[ResetCoordinator] Reset started, waiting for odometry zero")
            return True
    
    def notify_odometry_zero(self):
        """Called from SerialManager when odometry reaches zero."""
        with self._lock:
            if self._state == self.WAITING_ODOM and not self._aborted:
                self._state = self.DISCARDING_SCAN
                # Use call_soon_threadsafe since we're in a non-asyncio thread
                try:
                    self.loop.call_soon_threadsafe(self.odom_confirmed.set)
                except RuntimeError as e:
                    print(e)
                    pass
                print("[ResetCoordinator] Odometry confirmed zero, discarding next scan")
    
    def notify_scan_ready(self):
        """
        Called from LiDAR receiver before publishing a scan.
        Returns True if scan should be published, False if should be discarded.
        This is the single source of truth for scan gating.
        """
        with self._lock:
            if self._aborted:
                return True
            
            if self._state == self.DISCARDING_SCAN:
                self._scans_to_discard -= 1
                if self._scans_to_discard <= 0:
                    self.current_epoch += 1
                    self._state = self.IDLE
                    try:
                        self.loop.call_soon_threadsafe(self.scan_discarded.set)
                    except RuntimeError:
                        pass
                    print(f"[ResetCoordinator] Scan discarded, reset complete. Epoch={self.current_epoch}")
                return False
            
            # In WAITING_ODOM or IDLE, allow publishing
            return True
    
    def abort_reset(self):
        """Cleanly abort an in-progress reset. Returns to IDLE."""
        with self._lock:
            if self._state != self.IDLE:
                self._state = self.IDLE
                self._aborted = True
                self._scans_to_discard = 1
                try:
                    self.loop.call_soon_threadsafe(self.odom_confirmed.set)
                    self.loop.call_soon_threadsafe(self.scan_discarded.set)
                except RuntimeError:
                    pass
                print("[ResetCoordinator] Reset aborted")
    
    def get_epoch(self):
        return self.current_epoch
    
    def is_reset_in_progress(self):
        with self._lock:
            return self._state != self.IDLE
    
    def should_discard_before_accumulation(self):
        """
        Returns True only during WAITING_ODOM phase.
        During this phase, scans should be discarded immediately without accumulation.
        During DISCARDING_SCAN, returns False so scans can be accumulated and then
        gated by notify_scan_ready().
        """
        with self._lock:
            return self._state == self.WAITING_ODOM
    
    def clear_accumulation(self):
        """Signal that LiDAR accumulation should be cleared."""
        # This is handled by the receiver checking notify_scan_ready()
        pass

# ============ Config ============
def load_config():
    global robot_config
    try:
        with open(CONFIG_FILE, 'r') as f:
            saved = json.load(f)
            robot_config.update(saved)
    except FileNotFoundError:
        pass
    except Exception as e:
        pass

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(robot_config, f, indent=2)
    except Exception as e:
        pass

def send_config_to_robot():
    global command_forwarder
    config_str = (
        f"{robot_config['wheel_radius']},"
        f"{robot_config['wheel_base']},"
        f"{robot_config['max_speed']},"
        f"{robot_config['robot_width']},"
        f"{robot_config['stop_distance']}"
    )
    command_forwarder.send_command('config', config_str, priority=5)

# ============ Data containers ============
@dataclass(frozen=True)
class SimulationPacket:
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
    reset_epoch: int = 0

@dataclass(frozen=True)
class SlamState:
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
    linear_vel: float = 0
    angular_vel: float = 0
    scan_matching_skipped: bool = False
    reset_epoch: int = 0

@dataclass
class Command:
    priority: int
    timestamp: float
    command_type: str
    payload: Any
    callback: Optional[callable] = None

# ---------- Atomic containers ----------
class AtomicSharedPacket:
    def __init__(self):
        self._packet: Optional[SimulationPacket] = None
        self._generation = 0
        self._lock = threading.Lock()
    def update(self, packet):
        with self._lock:
            self._packet = packet
            self._generation += 1
    def get_latest(self):
        with self._lock:
            return self._packet, self._generation

class AtomicSharedState:
    def __init__(self):
        self._state: Optional[SlamState] = None
        self._lock = threading.Lock()
    def update(self, state):
        with self._lock:
            self._state = state
    def get_latest(self):
        with self._lock:
            return self._state

# ---------- OccupancyGrid ----------
class OccupancyGrid:
    def __init__(self, width, height, resolution, offset_x=0.0, offset_y=0.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        self.occupancy = np.full((height, width), -1, dtype=np.int8)
        self.last_update_packet_id = -1
        self.map_update_id = 0
        self.lidar_offset_x = offset_x
        self.lidar_offset_y = offset_y

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def is_ready_for_scan_matching(self):
        return np.count_nonzero(np.abs(self.log_odds) > 0.25) >= SCAN_MATCH_MIN_FEATURES

    def sample_log_odds(self, x, y):
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
        if not self.is_ready_for_scan_matching():
            return robot_x, robot_y, robot_theta, 0.0

        def lidar_from_robot(rx, ry, rt):
            lx = rx + self.lidar_offset_x * math.cos(rt) - self.lidar_offset_y * math.sin(rt)
            ly = ry + self.lidar_offset_x * math.sin(rt) + self.lidar_offset_y * math.cos(rt)
            return lx, ly

        lx0, ly0 = lidar_from_robot(robot_x, robot_y, robot_theta)
        best_score = self.score_scan_pose(lx0, ly0, robot_theta, ranges, angles)
        best_pose = (robot_x, robot_y, robot_theta)

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
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "data": self.occupancy.flatten().tolist()
        }

    def get_map_update_id(self):
        return self.map_update_id

# ---------- SlamProcessor ----------
class SlamProcessor:
    def __init__(self, shared_packet, shared_state):
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
        self.current_epoch = 0

    def process_loop(self, stop_event):
        while not stop_event.is_set():
            packet, generation = self.shared_packet.get_latest()
            if packet is None or generation == self.last_generation:
                time.sleep(0.0001)
                continue
            
            if packet.reset_epoch < self.current_epoch:
                self.last_generation = generation
                continue
            
            self.last_generation = generation
            self.process_packet(packet)

    def process_packet(self, packet):
        global robot_config
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
            linear_vel=packet.linear_vel,
            angular_vel=packet.angular_vel,
            scan_matching_skipped=is_rotating_fast,
            reset_epoch=packet.reset_epoch
        )
        self.shared_state.update(new_state)
    
    def reset_slam(self):
        global robot_config, reset_coordinator
        self.map_grid = OccupancyGrid(
            MAP_SIZE, MAP_SIZE, MAP_RESOLUTION,
            offset_x=robot_config.get("lidar_offset_x", 0.0),
            offset_y=robot_config.get("lidar_offset_y", 0.0)
        )
        self.slam_initialized = False
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_theta = 0.0
        self.processed_count = 0
        self.skipped_count = 0
        self.state_id = 0
        
        if reset_coordinator:
            self.current_epoch = reset_coordinator.get_epoch()
        
        packet, generation = self.shared_packet.get_latest()
        self.last_generation = generation
        
        print(f"[SlamProcessor] Reset complete (epoch={self.current_epoch})")

    def get_map(self):
        return self.map_grid.get_map()

    def get_map_update_id(self):
        return self.map_grid.get_map_update_id()

# ---------- SerialManager ----------
class SerialManager:
    def __init__(self, port, baudrate, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader_thread = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._left_speed = 0.0
        self._right_speed = 0.0
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._last_odom_update = 0
        self._odom_lock = threading.Lock()
        self._lines_received = 0
        self._commands_sent = 0
        self._parse_errors = 0
        self._line_buffer = ""

    def start(self):
        if self._reader_thread and self._reader_thread.is_alive():
            return True
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._ser.flushInput()
            self._ser.flushOutput()
        except Exception as e:
            return False
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()

    def write(self, data):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((data + '\n').encode('utf-8'))
                    self._ser.flush()
                    self._commands_sent += 1
                    return True
                except Exception as e:
                    return False
            return False

    def get_latest_odometry(self):
        with self._odom_lock:
            return {
                'robot_x': self._robot_x,
                'robot_y': self._robot_y,
                'robot_theta': self._robot_theta,
                'left_speed': self._left_speed,
                'right_speed': self._right_speed,
                'linear_vel': self._linear_vel,
                'angular_vel': self._angular_vel,
                'timestamp': self._last_odom_update
            }

    def _reader_loop(self):
        global reset_coordinator
        while not self._stop_event.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.1)
                    continue
                if self._ser.in_waiting > 0:
                    data = self._ser.read(self._ser.in_waiting)
                    try:
                        text = data.decode('utf-8')
                        self._line_buffer += text
                    except UnicodeDecodeError:
                        pass
                    while '\n' in self._line_buffer:
                        line, self._line_buffer = self._line_buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        self._lines_received += 1
                        self._parse_line(line)
                else:
                    time.sleep(0.001)
            except Exception as e:
                time.sleep(0.1)

    def _parse_line(self, line):
        global reset_coordinator
        if line.startswith("ODOM,"):
            try:
                parts = line.split(',')
                if len(parts) >= 7:
                    x = float(parts[1])
                    y = float(parts[2])
                    theta = float(parts[3])
                    left = float(parts[4])
                    right = float(parts[5])
                    lin = float(parts[6])
                    ang = float(parts[7]) if len(parts) > 7 else 0.0
                    with self._odom_lock:
                        self._robot_x = x
                        self._robot_y = y
                        self._robot_theta = theta
                        self._left_speed = left
                        self._right_speed = right
                        self._linear_vel = lin
                        self._angular_vel = ang
                        self._last_odom_update = time.time()
                    
                    if (reset_coordinator and 
                        reset_coordinator.is_reset_in_progress() and 
                        abs(x) < 0.01 and abs(y) < 0.01 and abs(theta) < 0.02):
                        reset_coordinator.notify_odometry_zero()
            except Exception as e:
                self._parse_errors += 1

# ---------- SerialCommandForwarder ----------
class SerialCommandForwarder:
    def __init__(self, serial_manager):
        self.serial_manager = serial_manager
        self.command_queue = queue.PriorityQueue()
        self.stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self.stop_event.clear()
        self._thread = threading.Thread(target=self._forward_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def send_command(self, command_type, payload, priority=10, callback=None):
        cmd = Command(priority=priority, timestamp=time.time(),
                      command_type=command_type, payload=payload, callback=callback)
        self.command_queue.put((priority, time.time(), cmd))
        
    def _forward_loop(self):
        while not self.stop_event.is_set():
            try:
                _, _, cmd = self.command_queue.get(timeout=0.01)
            except queue.Empty:
                continue
            
            if cmd.command_type == 'raw':
                if self.serial_manager.write(cmd.payload):
                    if cmd.callback:
                        cmd.callback(True)
            else:
                message = self._build_command(cmd.command_type, cmd.payload)
                if message:
                    if self.serial_manager.write(message):
                        if cmd.callback:
                            cmd.callback(True)
                    else:
                        if cmd.callback:
                            cmd.callback(False)
            time.sleep(0.01)

    def _build_command(self, cmd_type, payload):
        if cmd_type == 'mode':
            return f"MODE:{payload}"
        elif cmd_type == 'cmd':
            return f"CMD:{payload}"
        elif cmd_type == 'path':
            if isinstance(payload, list) and payload:
                path_str = ';'.join([f"{p[0]:.3f},{p[1]:.3f}" for p in payload])
                return f"PATH:{path_str}"
            else:
                return None
        elif cmd_type == 'config':
            return f"CONFIG:{payload}"
        elif cmd_type == 'obs':
            return f"OBS:{payload}"
        else:
            return None

# ---------- Helper functions for path planning ----------
def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def coarse_grid_from_map(map_data, coarse_factor=COARSE_FACTOR, robot_width=None):
    if not map_data:
        return None
    if robot_width is None:
        robot_width = robot_config.get('robot_width', 0.41)

    width = map_data['width']
    height = map_data['height']
    res = map_data['resolution']
    data = np.array(map_data['data'], dtype=np.int8).reshape((height, width))

    cf = int(coarse_factor)
    cw = max(1, width // cf)
    ch = max(1, height // cf)
    cres = res * cf

    coarse = np.zeros((ch, cw), dtype=np.uint8)
    for cy in range(ch):
        for cx in range(cw):
            fx0 = cx * cf
            fy0 = cy * cf
            fx1 = min(width, fx0 + cf)
            fy1 = min(height, fy0 + cf)
            block = data[fy0:fy1, fx0:fx1]
            if np.any(block == 100):
                coarse[cy, cx] = 1

    robot_radius = robot_width / 2.0
    inflation_cells = max(1, int(robot_radius / cres))
    struct = generate_binary_structure(2, 2)
    inflated = binary_dilation(coarse, structure=struct, iterations=inflation_cells)
    inflated = inflated.astype(np.int8)

    return {
        'width': cw,
        'height': ch,
        'resolution': cres,
        'origin_x': -width * res / 2.0,
        'origin_y': -height * res / 2.0,
        'data': inflated,
    }

def astar_plan(coarse, start_xy, goal_xy):
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
        return math.hypot(a[0] - b[0], a[1] - b[1])

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
            tentative_g = gscore[current] + math.hypot(dx, dy)
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
        if simplified and len(simplified) > 1:
            simplified = simplified[1:]

    return simplified

# ---------- PlannerWorker ----------
class PlannerWorker:
    def __init__(self, shared_state, command_forwarder):
        self.shared_state = shared_state
        self.command_forwarder = command_forwarder
        self.lock = threading.Lock()
        self.waypoints = []
        self.current_wp_index = 0
        self.loop_mode = False
        self.finished = False
        self.path = []
        self._goal = None
        self.last_plan_time = 0
        self.last_map_update_id = -1
        self.replan_interval = 2.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.returning_to_start = False
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
            state = self.shared_state.get_latest()
            if state:
                self.start_x = state.x
                self.start_y = state.y
            else:
                self.start_x = self.start_y = 0.0
            if self.waypoints:
                self._goal = self.waypoints[0]
                self.request_queue.append(('plan',))
            else:
                self._goal = None
                self.path = []

    def set_goal(self, x, y):
        with self.lock:
            self.waypoints = [(x, y)]
            self.loop_mode = False
            self.current_wp_index = 0
            self.finished = False
            self._goal = (x, y)
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
            if self.returning_to_start:
                self.finished = True
                self._goal = None
                self.returning_to_start = False
                return False
            if self.current_wp_index + 1 < len(self.waypoints):
                self.current_wp_index += 1
                self._goal = self.waypoints[self.current_wp_index]
                return True
            else:
                self.current_wp_index = len(self.waypoints)
                if self.loop_mode:
                    self.returning_to_start = True
                    self._goal = (self.start_x, self.start_y)
                    return True
                else:
                    self.finished = True
                    self._goal = None
                    return False

    def trim_path(self, robot_x, robot_y):
        need_advance = False
        with self.lock:
            if not self.path or self.finished:
                return
            while self.path and math.hypot(self.path[0][0] - robot_x, self.path[0][1] - robot_y) < 0.2:
                self.path.pop(0)
            if not self.path:
                need_advance = True
        if need_advance:
            has_next = self._advance_to_next_waypoint()
            if has_next:
                with self.lock:
                    self.request_queue.append(('plan',))
            else:
                self.command_forwarder.send_command('cmd', 'stop', priority=1)

    def check_replan(self, robot_x, robot_y, map_update_id):
        with self.lock:
            if self.finished or self._goal is None:
                return False
            map_changed = map_update_id != self.last_map_update_id
            time_elapsed = time.time() - self.last_plan_time > self.replan_interval
            if map_changed or time_elapsed:
                self.request_queue.append(('plan',))
                self.last_map_update_id = map_update_id
                self.last_plan_time = time.time()
                return True
        return False

    def planner_loop(self, stop_event):
        global slam_processor
        while not stop_event.is_set():
            plan_requested = False
            with self.lock:
                if self.request_queue:
                    self.request_queue.popleft()
                    plan_requested = True
            if not plan_requested:
                time.sleep(0.05)
                continue
            slam_state = self.shared_state.get_latest()
            if slam_state is None:
                continue
            sx, sy = slam_state.x, slam_state.y
            with self.lock:
                goal = self._goal
                if goal is None:
                    continue
            map_data = slam_processor.get_map()
            robot_width = robot_config.get('robot_width', 0.41)
            coarse = coarse_grid_from_map(map_data, COARSE_FACTOR, robot_width)
            if coarse is None:
                continue
            planned = astar_plan(coarse, (sx, sy), goal)
            with self.lock:
                self.path = planned
                self.last_plan_time = time.time()
            if planned:
                robot_path = [(x, -y) for x, y in planned]
                self.command_forwarder.send_command('path', robot_path, priority=5)
            time.sleep(0.05)

# ---------- LiDAR receiver ----------
def lidar_and_odom_receiver(shared_packet, stop_event, lidar_port, baudrate, scan_type, serial_manager, command_forwarder):
    """
    LiDAR receiver thread.
    Discards scans before accumulation when reset is in progress.
    notify_scan_ready() is the single source of truth for scan gating.
    """
    global reset_coordinator
    packet_count = 0
    packet_id = 0

    try:
        lidar = RPLidar(lidar_port, baudrate=baudrate, timeout=3)
        info = lidar.get_info()
        scan_generator = lidar.iter_scans(scan_type=scan_type)
    except Exception as e:
        return

    scan_counter = 0
    last_log_time = time.time()
    scans_in_interval = 0
    points_in_interval = 0
    low_quality_points_in_interval = 0

    accumulated_ranges = []
    accumulated_angles = []
    accumulated_timestamp = None
    accumulated_scan_count = 0
    MIN_SCANS_TO_ACCUMULATE = 3
    MAX_ACCUMULATION_TIME = 0.5
    last_accumulation_start = time.time()
    MIN_ANGLE_COVERAGE = 350
    accumulated_angle_min = float('inf')
    accumulated_angle_max = -float('inf')

    last_obs_send = 0

    while not stop_event.is_set():
        try:
            scan = next(scan_generator)
            packet_id += 1
            scan_counter += 1

            # Discard scans BEFORE accumulation ONLY during WAITING_ODOM phase.
            # During DISCARDING_SCAN phase, allow accumulation so notify_scan_ready()
            # can gate the completed scan.
            if reset_coordinator and reset_coordinator.should_discard_before_accumulation():
                continue

            ranges = []
            angles = []
            low_quality_count = 0

            for quality, angle, distance in scan:
                if distance > 0:
                    if quality < 15:
                        low_quality_count += 1
                    ranges.append(distance / 1000.0)
                    angles.append(math.radians(angle))

            if len(ranges) > 0:
                if accumulated_timestamp is None:
                    accumulated_timestamp = time.time()
                    last_accumulation_start = accumulated_timestamp
                    accumulated_angle_min = float('inf')
                    accumulated_angle_max = -float('inf')

                accumulated_ranges.extend(ranges)
                accumulated_angles.extend(angles)
                accumulated_scan_count += 1

                for angle_rad in angles:
                    angle_deg = math.degrees(angle_rad) % 360
                    if angle_deg < accumulated_angle_min:
                        accumulated_angle_min = angle_deg
                    if angle_deg > accumulated_angle_max:
                        accumulated_angle_max = angle_deg

                angle_coverage = accumulated_angle_max - accumulated_angle_min
                if angle_coverage > 360:
                    angle_coverage = 360

                current_time = time.time()
                time_elapsed = current_time - last_accumulation_start

                should_send = False

                if (accumulated_scan_count >= MIN_SCANS_TO_ACCUMULATE and 
                    angle_coverage >= MIN_ANGLE_COVERAGE):
                    should_send = True
                elif time_elapsed >= MAX_ACCUMULATION_TIME and accumulated_scan_count >= 2:
                    should_send = True
                elif accumulated_scan_count >= 10:
                    should_send = True

                if should_send:
                    # notify_scan_ready() is the single source of truth
                    # It handles both the discard phase and normal operation
                    allow_publish = True
                    if reset_coordinator:
                        allow_publish = reset_coordinator.notify_scan_ready()
                    
                    if not allow_publish:
                        # Scan was discarded - clear accumulation and continue
                        accumulated_ranges = []
                        accumulated_angles = []
                        accumulated_scan_count = 0
                        accumulated_timestamp = None
                        accumulated_angle_min = float('inf')
                        accumulated_angle_max = -float('inf')
                        continue
                    
                    if serial_manager:
                        odom = serial_manager.get_latest_odometry()
                        robot_x = odom['robot_x']
                        robot_y = odom['robot_y']
                        robot_theta = odom['robot_theta']
                        left_speed = odom['left_speed']
                        right_speed = odom['right_speed']
                        linear_vel = odom['linear_vel']
                        angular_vel = odom['angular_vel']
                    else:
                        robot_x = robot_y = robot_theta = 0.0
                        left_speed = right_speed = linear_vel = angular_vel = 0.0

                    current_epoch = reset_coordinator.get_epoch() if reset_coordinator else 0

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
                        reset_epoch=current_epoch
                    )
                    shared_packet.update(packet)
                    packet_count += 1

                    # Obstacle clearance computation
                    now = time.time()
                    if now - last_obs_send >= 0.2:
                        front_clearance = float('inf')
                        rear_clearance = float('inf')
                        left_clearance = float('inf')
                        right_clearance = float('inf')
                        
                        for r, angle in zip(accumulated_ranges, accumulated_angles):
                            if r < MIN_RANGE or r > MAX_RANGE:
                                continue
                            
                            normalized_angle = angle
                            while normalized_angle > math.pi:
                                normalized_angle -= 2 * math.pi
                            while normalized_angle < -math.pi:
                                normalized_angle += 2 * math.pi
                            
                            angle_deg = math.degrees(normalized_angle)
                            
                            if abs(angle_deg) <= 30:
                                clearance = r * math.cos(normalized_angle) - ROBOT_LENGTH
                                if clearance < front_clearance:
                                    front_clearance = clearance
                            elif abs(abs(angle_deg) - 180) <= 30:
                                clearance = r
                                if clearance < rear_clearance:
                                    rear_clearance = clearance
                            elif 30 <= angle_deg <= 90:
                                clearance = r * math.sin(normalized_angle) - ROBOT_HALF_WIDTH
                                if clearance < left_clearance:
                                    left_clearance = clearance
                            elif -90 <= angle_deg <= -30:
                                clearance = r * abs(math.sin(normalized_angle)) - ROBOT_HALF_WIDTH
                                if clearance < right_clearance:
                                    right_clearance = clearance
                        
                        if front_clearance == float('inf'):
                            front_clearance = MAX_RANGE
                        if rear_clearance == float('inf'):
                            rear_clearance = MAX_RANGE
                        if left_clearance == float('inf'):
                            left_clearance = MAX_RANGE
                        if right_clearance == float('inf'):
                            right_clearance = MAX_RANGE
                        
                        front_clearance = max(0.0, front_clearance)
                        rear_clearance = max(0.0, rear_clearance)
                        left_clearance = max(0.0, left_clearance)
                        right_clearance = max(0.0, right_clearance)
                        
                        if front_clearance < 4.9 or rear_clearance < 4.9 or left_clearance < 4.9 or right_clearance < 4.9:
                            payload = f"{front_clearance:.3f},{rear_clearance:.3f},{left_clearance:.3f},{right_clearance:.3f}"
                            command_forwarder.send_command('obs', payload, priority=10)
                            last_obs_send = now

                    accumulated_ranges = []
                    accumulated_angles = []
                    accumulated_scan_count = 0
                    accumulated_timestamp = None
                    accumulated_angle_min = float('inf')
                    accumulated_angle_max = -float('inf')

            scans_in_interval += 1
            points_in_interval += len(ranges)
            low_quality_points_in_interval += low_quality_count

            now = time.time()
            if now - last_log_time >= 10.0:
                last_log_time = now
                scans_in_interval = 0
                points_in_interval = 0
                low_quality_points_in_interval = 0

        except StopIteration:
            try:
                lidar.stop()
                lidar.disconnect()
                lidar = RPLidar(lidar_port, baudrate=baudrate)
                scan_generator = lidar.iter_scans(scan_type=scan_type)
            except Exception as e:
                time.sleep(1)
            continue
        except Exception as e:
            time.sleep(0.01)
            continue

    try:
        lidar.stop()
        lidar.disconnect()
    except:
        pass

# ---------- Relay handling ----------
async def connect_to_relay():
    global relay_ws, relay_connected
    attempts = 0
    while True:
        try:
            ssl_context = ssl.create_default_context()
            relay_ws = await websockets.connect(RELAY_URL, ssl=ssl_context)
            await relay_ws.send(json.dumps({
                "type": "register",
                "role": "bridge",
                "bridgeId": BRIDGE_ID,
                "token": BRIDGE_TOKEN
            }))
            response = await relay_ws.recv()
            resp = json.loads(response)
            if resp.get("type") == "registered":
                with relay_lock:
                    relay_connected = True
                asyncio.create_task(relay_message_handler(relay_ws))
                return
            else:
                await relay_ws.close()
        except Exception as e:
            pass
        attempts += 1
        if MAX_RECONNECT_ATTEMPTS > 0 and attempts >= MAX_RECONNECT_ATTEMPTS:
            break
        await asyncio.sleep(RECONNECT_DELAY)

async def relay_message_handler(ws):
    global relay_connected
    try:
        async for message in ws:
            try:
                data = json.loads(message)
                await process_relay_message(data)
            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        with relay_lock:
            relay_connected = False
    except Exception as e:
        pass
    finally:
        if not relay_connected:
            asyncio.create_task(connect_to_relay())

# ---------- Process messages from UI ----------
async def process_relay_message(data):
    global command_forwarder, planner, slam_processor, reset_coordinator

    if data.get('type') == 'command':
        cmd = data.get('command')
        command_forwarder.send_command('cmd', cmd, priority=3)

    elif data.get('type') == 'mode':
        mode = data.get('mode')
        if mode in ['auto', 'manual', 'idle']:
            command_forwarder.send_command('mode', mode, priority=1)

    elif data.get('type') == 'set_waypoints':
        waypoints = data.get('waypoints', [])
        loop = data.get('loop', False)
        if waypoints:
            wp_list = [(p['x'], p['y']) for p in waypoints]
            planner.set_waypoints(wp_list, loop)
            robot_path = [(x, -y) for x, y in wp_list]
            command_forwarder.send_command('path', robot_path, priority=5)
        else:
            planner.set_waypoints([], False)
            command_forwarder.send_command('path', [], priority=5)

    elif data.get('type') == 'set_goal':
        gx = float(data.get('x', 0.0))
        gy = float(data.get('y', 0.0))
        planner.set_goal(gx, gy)
        robot_path = [(gx, -gy)]
        command_forwarder.send_command('path', robot_path, priority=5)

    elif data.get('type') == 'set_config':
        new_config = data.get('config', {})
        for key, value in new_config.items():
            if key in robot_config:
                robot_config[key] = value
        save_config()
        send_config_to_robot()
        if relay_ws and relay_connected:
            await relay_ws.send(json.dumps({
                'type': 'config_updated',
                'config': robot_config
            }))

    elif data.get('type') == 'get_config':
        if relay_ws and relay_connected:
            await relay_ws.send(json.dumps({
                'type': 'config',
                'config': robot_config
            }))
    
    elif data.get('type') == 'reset_slam':
        await perform_reset()

async def perform_reset():
    """Synchronous reset state machine driven by asyncio.Events."""
    global reset_coordinator, command_forwarder, slam_processor
    
    if reset_coordinator is None:
        return
    
    if not reset_coordinator.start_reset():
        print("[Bridge] Reset already in progress")
        return
    
    command_forwarder.send_command(
        "cmd",
        "reset",
        priority=1
    )
    print("[Bridge] Reset sent to ESP32, waiting for odometry zero...")
    
    # Wait for odometry to reach zero (asyncio.Event, no executor needed)
    try:
        await asyncio.wait_for(reset_coordinator.odom_confirmed.wait(), timeout=5.0)
    except asyncio.TimeoutError:
        print("[Bridge] Timeout waiting for odometry reset")
        reset_coordinator.abort_reset()
        return
    
    print("[Bridge] Odometry confirmed zero, waiting for fresh scan...")
    
    # Wait for one full scan to be discarded
    try:
        await asyncio.wait_for(reset_coordinator.scan_discarded.wait(), timeout=3.0)
    except asyncio.TimeoutError:
        print("[Bridge] Timeout waiting for scan discard")
        reset_coordinator.abort_reset()
        return
    
    if slam_processor:
        slam_processor.reset_slam()
    
    print("[Bridge] SLAM reset complete")

# ---------- Broadcaster ----------
async def websocket_broadcaster(shared_state, planner, stop_event):
    global relay_ws, relay_connected, slam_processor
    pose_interval = 1.0 / BROADCAST_POSE_HZ
    map_interval = 1.0 / BROADCAST_MAP_HZ
    last_pose = 0
    last_map = 0

    while not stop_event.is_set():
        now = time.time()
        slam_state = shared_state.get_latest()
        if slam_state and relay_ws and relay_connected and relay_ws.state == websockets.protocol.State.OPEN:
            if now - last_pose >= pose_interval:
                last_pose = now
                output = {
                    "type": "lidar_scan",
                    "timestamp": slam_state.timestamp,
                    "num_points": len(slam_state.ranges),
                    "min_range": MIN_RANGE,
                    "max_range": MAX_RANGE,
                    "ranges": list(slam_state.ranges),
                    "angles": list(slam_state.angles),
                    "robot_x": slam_state.x,
                    "robot_y": slam_state.y,
                    "robot_theta": -slam_state.theta if FLIP_THETA_FOR_VISUALIZATION else slam_state.theta,
                    "left_speed": slam_state.left_speed,
                    "right_speed": slam_state.right_speed,
                    "linear_vel": slam_state.linear_vel,
                    "angular_vel": slam_state.angular_vel,
                    "slam_match_score": slam_state.match_score,
                    "scan_matching_skipped": slam_state.scan_matching_skipped,
                }
                if now - last_map >= map_interval:
                    output["map"] = slam_processor.get_map()
                    map_data = slam_processor.get_map()
                    robot_width = robot_config.get('robot_width', 0.41)
                    coarse = coarse_grid_from_map(map_data, COARSE_FACTOR, robot_width)
                    if coarse:
                        output["coarse_grid"] = {
                            'width': coarse['width'],
                            'height': coarse['height'],
                            'resolution': coarse['resolution'],
                            'origin_x': coarse['origin_x'],
                            'origin_y': coarse['origin_y'],
                            'data': coarse['data'].flatten().tolist()
                        }
                    last_map = now
                path = planner.get_path() if planner else []
                if path:
                    output['path'] = [{'x': p[0], 'y': p[1]} for p in path]
                goal = planner.get_goal() if planner else None
                if goal:
                    output['goal'] = {'x': goal[0], 'y': goal[1]}
                remaining = planner.get_remaining_waypoints() if planner else []
                if remaining:
                    output['remaining_waypoints'] = [{'x': wp[0], 'y': wp[1]} for wp in remaining]
                try:
                    await relay_ws.send(json.dumps(output))
                except Exception as e:
                    with relay_lock:
                        relay_connected = False
        await asyncio.sleep(0.001)

# ---------- Main ----------
async def main():
    global shared_packet, shared_state, planner, command_forwarder, serial_manager, slam_processor, reset_coordinator
    shared_packet = AtomicSharedPacket()
    shared_state = AtomicSharedState()
    stop_event = threading.Event()
    
    loop = asyncio.get_event_loop()
    reset_coordinator = ResetCoordinator(loop)

    serial_manager = SerialManager(SERIAL_PORT, SERIAL_BAUDRATE)
    if not serial_manager.start():
        return

    command_forwarder = SerialCommandForwarder(serial_manager)
    command_forwarder.start()

    slam_processor = SlamProcessor(shared_packet, shared_state)

    lidar_thread = threading.Thread(
        target=lidar_and_odom_receiver,
        args=(shared_packet, stop_event, LIDAR_PORT, LIDAR_BAUDRATE,
            LIDAR_SCAN_TYPE, serial_manager, command_forwarder),
        daemon=True
    )
    lidar_thread.start()

    slam_thread = threading.Thread(target=slam_processor.process_loop, args=(stop_event,), daemon=True)
    slam_thread.start()

    planner = PlannerWorker(shared_state, command_forwarder)
    planner_thread = threading.Thread(target=planner.planner_loop, args=(stop_event,), daemon=True)
    planner_thread.start()

    await connect_to_relay()

    broadcaster_task = asyncio.create_task(
        websocket_broadcaster(shared_state, planner, stop_event)
    )

    load_config()
    send_config_to_robot()

    try:
        while not stop_event.is_set():
            slam_state = shared_state.get_latest()
            if slam_state and planner:
                planner.trim_path(slam_state.x, slam_state.y)
                map_update_id = slam_processor.get_map_update_id()
                planner.check_replan(slam_state.x, slam_state.y, map_update_id)
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        broadcaster_task.cancel()
        command_forwarder.stop()
        lidar_thread.join(timeout=2)
        slam_thread.join(timeout=2)
        planner_thread.join(timeout=1)
        if relay_ws:
            await relay_ws.close()
        serial_manager.stop()

if __name__ == "__main__":
    asyncio.run(main())
#!/usr/bin/env python3
"""
UDP to WebSocket Bridge with Grid Map SLAM
Clean architecture with atomic shared state and generation counters
Modified to use a reverse WebSocket connection to a relay server.
Supports waypoint following with continuous replanning.
"""

import asyncio
import socket
import json
import websockets
import numpy as np
import math
import time
import threading
import heapq
import queue
import ssl
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, Any, List
from scipy.ndimage import binary_dilation, generate_binary_structure

# ============ Configuration ============
UDP_PORT = 8765
WEBSOCKET_PORT = 8766  # no longer used as server, kept for reference
BROADCAST_POSE_HZ = 30  # 30 Hz pose updates
BROADCAST_MAP_HZ = 1    # 1 Hz map updates

# Planner settings
COARSE_FACTOR = 4  # COARSE_FACTOR^2 fine cells per coarse cell
ROBOT_WIDTH = 0.41  # meters

# UDP ports for robot communication
ROBOT_DATA_PORT = 8765    # Robot → Bridge (LiDAR/odometry)
ROBOT_CMD_PORT = 8767     # Bridge → Robot (Commands)
ROBOT_PATH_PORT = 8768    # Bridge → Robot (Path following)

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

LIDAR_OFFSET_X = 0.0   # e.g., 0.10 if LiDAR is 10 cm forward
LIDAR_OFFSET_Y = 0.0   # e.g., -0.05 if 5 cm to the right (negative = right)

# Sensor parameters
MAX_RANGE = 12.0
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


@dataclass(frozen=True)
class SimulationPacket:
    """Immutable raw data from Webots"""
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
    command_type: str  # 'cmd', 'auto', 'path'
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
print("=" * 60)
print(f"Map: {MAP_SIZE}x{MAP_SIZE} cells, {MAP_RESOLUTION*100:.0f}cm resolution")
print(f"UDP Receive Port (Robot→Bridge): {ROBOT_DATA_PORT}")
print(f"UDP Command Port (Bridge→Robot): {ROBOT_CMD_PORT}")
print(f"UDP Path Port (Bridge→Robot): {ROBOT_PATH_PORT}")
print(f"Pose broadcast: {BROADCAST_POSE_HZ} Hz")
print(f"Map broadcast: {BROADCAST_MAP_HZ} Hz")
print(f"Correction Weight: {CORRECTION_WEIGHT * 100:.0f}%")
print(f"Angular velocity threshold: {ANGULAR_VEL_THRESHOLD} rad/s ({ANGULAR_VEL_THRESHOLD * 180 / math.pi:.1f}°/s)")
print(f"Relay URL: {RELAY_URL}")
print(f"Bridge ID: {BRIDGE_ID}")
print("=" * 60)


planner = None
command_forwarder = None


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
        v1 = v01 * (1.0 - tx) + v11 * tx
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


class DedicatedCommandForwarder:
    """
    Dedicated command forwarder with separate ports for commands and paths.
    Uses non-blocking UDP sockets for immediate transmission.
    """

    def __init__(self, cmd_port=ROBOT_CMD_PORT, path_port=ROBOT_PATH_PORT):
        self.cmd_port = cmd_port
        self.path_port = path_port
        self.command_queue = queue.PriorityQueue()
        self.path_queue = queue.PriorityQueue()
        self.stop_event = threading.Event()
        self._cmd_thread = None
        self._path_thread = None
        self._cmd_socket = None
        self._path_socket = None
        self._lock = threading.Lock()

    def start(self):
        """Start the command forwarder threads"""
        if self._cmd_thread is not None and self._cmd_thread.is_alive():
            return

        self.stop_event.clear()

        # Create dedicated UDP sockets for each channel
        self._cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmd_socket.setblocking(False)

        self._path_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._path_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._path_socket.setblocking(False)

        # Start separate threads for commands and paths
        self._cmd_thread = threading.Thread(target=self._cmd_forward_loop, daemon=True)
        self._cmd_thread.start()

        self._path_thread = threading.Thread(target=self._path_forward_loop, daemon=True)
        self._path_thread.start()

        # print(f"[CommandForwarder] Started - CMD port: {self.cmd_port}, PATH port: {self.path_port}")

    def stop(self):
        """Stop the command forwarder"""
        self.stop_event.set()
        if self._cmd_thread is not None:
            self._cmd_thread.join(timeout=1.0)
        if self._path_thread is not None:
            self._path_thread.join(timeout=1.0)
        if self._cmd_socket:
            self._cmd_socket.close()
            self._cmd_socket = None
        if self._path_socket:
            self._path_socket.close()
            self._path_socket = None
        # print("[CommandForwarder] Stopped")

    def send_command(self, command_type: str, payload: Any, priority: int = 10, callback: Optional[callable] = None):
        """
        Send a command to the robot immediately.

        Args:
            command_type: 'cmd', 'auto', 'path'
            payload: Command payload (string for 'cmd'/'auto', list for 'path')
            priority: Lower = higher priority (0 = emergency stop)
            callback: Optional callback for acknowledgment
        """
        cmd = Command(
            priority=priority,
            timestamp=time.time(),
            command_type=command_type,
            payload=payload,
            callback=callback
        )

        # Route to appropriate queue
        if command_type == 'path':
            self.path_queue.put((priority, time.time(), cmd))
            # print(f"[CommandForwarder] Queued PATH with priority {priority} on PORT {self.path_port}")
        else:
            self.command_queue.put((priority, time.time(), cmd))
            # print(f"[CommandForwarder] Queued {command_type} with priority {priority} on PORT {self.cmd_port}")

    def _cmd_forward_loop(self):
        """Forward commands on dedicated command port"""
        # print(f"[CommandForwarder] CMD thread started on port {self.cmd_port}")

        while not self.stop_event.is_set():
            try:
                try:
                    _, _, cmd = self.command_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

                if cmd.command_type == 'cmd':
                    message = f"CMD:{cmd.payload}".encode('utf-8')
                    self._send_udp(self._cmd_socket, message, self.cmd_port)
                elif cmd.command_type == 'auto':
                    value = "1" if cmd.payload else "0"
                    message = f"AUTO:{value}".encode('utf-8')
                    self._send_udp(self._cmd_socket, message, self.cmd_port)
                else:
                    print(f"[CommandForwarder] Unknown command type in CMD queue: {cmd.command_type}")

                if cmd.callback:
                    try:
                        cmd.callback(True)
                    except Exception as e:
                        print(f"[CommandForwarder] Callback error: {e}")

            except Exception as e:
                print(f"[CommandForwarder] CMD thread error: {e}")

        # print("[CommandForwarder] CMD thread stopped")

    def _path_forward_loop(self):
        """Forward path commands on dedicated path port"""
        print(f"[CommandForwarder] PATH thread started on port {self.path_port}")

        while not self.stop_event.is_set():
            try:
                try:
                    _, _, cmd = self.path_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

                if cmd.command_type == 'path':
                    if isinstance(cmd.payload, list) and cmd.payload:
                        path_str = ';'.join([f"{p[0]:.3f},{p[1]:.3f}" for p in cmd.payload])
                        message = f"PATH:{path_str}".encode('utf-8')
                        self._send_udp(self._path_socket, message, self.path_port)
                        # print(f"[CommandForwarder] Sent PATH with {len(cmd.payload)} points on port {self.path_port}")
                    else:
                        print(f"[CommandForwarder] Invalid path payload: {cmd.payload}")
                else:
                    print(f"[CommandForwarder] Unknown command type in PATH queue: {cmd.command_type}")

                if cmd.callback:
                    try:
                        cmd.callback(True)
                    except Exception as e:
                        print(f"[CommandForwarder] Callback error: {e}")

            except Exception as e:
                print(f"[CommandForwarder] PATH thread error: {e}")

        # print("[CommandForwarder] PATH thread stopped")

    def _send_udp(self, sock, message: bytes, port: int):
        """Send UDP message to robot (non-blocking)"""
        if not sock:
            print(f"[CommandForwarder] No UDP socket available for port {port}")
            return

        try:
            sock.sendto(message, ('127.0.0.1', port))
        except BlockingIOError:
            print(f"[CommandForwarder] Socket would block on port {port}, retrying...")
            try:
                sock.setblocking(True)
                sock.sendto(message, ('127.0.0.1', port))
                sock.setblocking(False)
            except Exception as e:
                print(f"[CommandForwarder] Retry failed on port {port}: {e}")
        except Exception as e:
            print(f"[CommandForwarder] UDP send error on port {port}: {e}")


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


class SlamProcessor:
    """SLAM processor that consumes packets and produces pose estimates"""

    def __init__(self, shared_packet: AtomicSharedPacket, shared_state: AtomicSharedState):
        self.shared_packet = shared_packet
        self.shared_state = shared_state
        self.map_grid = OccupancyGrid(
            MAP_SIZE, MAP_SIZE, MAP_RESOLUTION,
            offset_x=LIDAR_OFFSET_X,
            offset_y=LIDAR_OFFSET_Y
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
        # print("[SLAM] Processor thread started")

        last_debug = time.time()

        while not stop_event.is_set():
            packet, generation = self.shared_packet.get_latest()

            if packet is None or generation == self.last_generation:
                time.sleep(0.0001)
                continue

            self.last_generation = generation
            self.process_packet(packet)

            # if time.time() - last_debug > 2.0 and self.processed_count > 0:
            #     last_debug = time.time()
            #     print(f"[SLAM] Processed {self.processed_count} scans, {self.skipped_count} skipped, "
            #           f"pose=({self.slam_x:.2f}, {self.slam_y:.2f}, {math.degrees(self.slam_theta):.1f}°), "
            #           f"last packet_id: {packet.packet_id}")

    def process_packet(self, packet: SimulationPacket):
        """Process a single packet and update SLAM state"""
        self.processed_count += 1

        angular_vel_abs = abs(packet.angular_vel)
        is_rotating_fast = angular_vel_abs > ANGULAR_VEL_THRESHOLD

        if is_rotating_fast:
            self.skipped_count += 1
            # if self.skipped_count % 100 == 0:
            #     print(f"[SLAM] Skipping scan matching (angular_vel={angular_vel_abs:.3f} rad/s)")

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
        lidar_x = self.slam_x + LIDAR_OFFSET_X * math.cos(self.slam_theta) - LIDAR_OFFSET_Y * math.sin(self.slam_theta)
        lidar_y = self.slam_y + LIDAR_OFFSET_X * math.sin(self.slam_theta) + LIDAR_OFFSET_Y * math.cos(self.slam_theta)

        map_updated = False
        if not is_rotating_fast:
            map_updated = self.map_grid.update(
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


def coarse_grid_from_map(map_data, coarse_factor=COARSE_FACTOR, robot_width=ROBOT_WIDTH):
    """
    Create a downsampled occupancy grid (coarse) and then inflate obstacles
    on that coarse grid by a number of coarse cells derived from robot radius.
    """
    if not map_data:
        return None

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

    # 3. Dilate the coarse occupancy using 8‑connectivity
    struct = generate_binary_structure(2, 2)   # 8‑neighbour kernel
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
                # Optional: print which point was removed
                # print(f"[Planner] Removed waypoint ({removed[0]:.3f}, {removed[1]:.3f})")

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
        Called from the main loop to decide if we should re‑plan.
        Returns True if a new plan was queued.
        """
        with self.lock:
            if self.finished or self._goal is None:
                # if self.finished:
                #     print(f"[Planner] check_replan: Finished flag True, skipping")
                # elif self._goal is None:
                #     print(f"[Planner] check_replan: No goal set, skipping")
                return False

            # Force re‑plan if map changed or time elapsed
            map_changed = map_update_id != self.last_map_update_id
            time_elapsed = time.time() - self.last_plan_time > self.replan_interval
            
            if map_changed or time_elapsed:
                # print(f"[Planner] check_replan: Replan triggered (map_changed={map_changed}, time_elapsed={time_elapsed:.2f} > {self.replan_interval})")
                # Also ensure we have a path or the path is empty
                self.request_queue.append(('plan',))
                self.last_map_update_id = map_update_id
                self.last_plan_time = time.time()
                return True
        return False

    def planner_loop(self, stop_event: threading.Event):
        """Main planning loop – processes planning requests."""
        # print("[Planner] Thread started")
        while not stop_event.is_set():
            # Check for planning requests
            plan_requested = False
            with self.lock:
                if self.request_queue:
                    self.request_queue.popleft()   # discard, just a trigger
                    plan_requested = True
                    # print(f"[Planner] planner_loop: Planning request popped from queue")

            if not plan_requested:
                time.sleep(0.05)
                continue

            # Get current robot pose
            slam_state = self.shared_state.get_latest()
            if slam_state is None:
                # print(f"[Planner] planner_loop: No SLAM state available yet, skipping")
                continue
            sx = slam_state.x
            sy = slam_state.y
            # print(f"[Planner] planner_loop: Current robot pose: ({sx:.3f},{sy:.3f})")

            # Get the current goal
            with self.lock:
                goal = self._goal
                if goal is None:
                    # print(f"[Planner] planner_loop: No goal set, skipping")
                    continue
                # print(f"[Planner] planner_loop: Current goal: ({goal[0]:.3f},{goal[1]:.3f})")

            # Plan path from robot to goal
            # print(f"[Planner] planner_loop: Starting A* planning from ({sx:.3f},{sy:.3f}) to ({goal[0]:.3f},{goal[1]:.3f})")
            map_data = self.slam_processor.get_map()
            coarse = coarse_grid_from_map(map_data)
            
            if coarse is None:
                # print(f"[Planner] planner_loop: Coarse grid is None (no map data)")
                continue
                
            planned = astar_plan(coarse, (sx, sy), goal)
            
            # if planned:
            #     print(f"[Planner] planner_loop: A* found path with {len(planned)} points")
            #     print(f"[Planner] planner_loop: Path first 5 points: {planned[:5]}")
            # else:
            #     print(f"[Planner] planner_loop: A* found NO path to goal")
            
            with self.lock:
                self.path = planned
                self.last_plan_time = time.time()
                # print(f"[Planner] planner_loop: Stored new path with {len(self.path)} points")

            # Send path to robot (enable auto mode first)
            if planned:
                # print(f"[Planner] planner_loop: Sending AUTO mode command")
                self.command_forwarder.send_command('auto', True, priority=2)
                # Convert to robot frame (flip Y)
                robot_path = [(x, -y) for x, y in planned]
                # print(f"[Planner] planner_loop: Sending PATH command with {len(robot_path)} points")
                self.command_forwarder.send_command('path', robot_path, priority=5)
                # print(f"[Planner] Sent path with {len(planned)} points to goal ({goal[0]:.2f},{goal[1]:.2f})")
            # else:
            #     print(f"[Planner] No path found to goal ({goal[0]:.2f},{goal[1]:.2f})")

            time.sleep(0.05)


def udp_receiver(shared_packet: AtomicSharedPacket, stop_event: threading.Event):
    """UDP receiver thread - receives data from robot on port 8765"""
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind(('0.0.0.0', ROBOT_DATA_PORT))
    udp_socket.settimeout(0.1)

    packet_count = 0
    packet_id = 0
    # print(f"[Receiver] UDP thread started on port {ROBOT_DATA_PORT} (Robot→Bridge)")

    while not stop_event.is_set():
        try:
            data, _ = udp_socket.recvfrom(65535)

            try:
                message = data.decode('utf-8')
                scan_data = json.loads(message)

                if scan_data.get('type') == 'lidar_scan':
                    packet_id += 1

                    ranges = tuple(scan_data.get('ranges', []))
                    angles = tuple(scan_data.get('angles', []))

                    packet = SimulationPacket(
                        packet_id=packet_id,
                        timestamp=scan_data.get('timestamp', time.time()),
                        robot_x=scan_data.get('robot_x', 0),
                        robot_y=scan_data.get('robot_y', 0),
                        robot_theta=scan_data.get('robot_theta', 0),
                        ranges=ranges,
                        angles=angles,
                        left_speed=scan_data.get('left_speed', 0),
                        right_speed=scan_data.get('right_speed', 0),
                        linear_vel=scan_data.get('linear_vel', 0),
                        angular_vel=scan_data.get('angular_vel', 0),
                        auto_navigate=scan_data.get('auto_navigate', True)
                    )

                    shared_packet.update(packet)
                    packet_count += 1

                    # if packet_count % 100 == 0:
                        # print(f"[Receiver] {packet_count} packets received, last packet_id: {packet_id}")

            except json.JSONDecodeError as e:
                print(f"[DEBUG] JSON decode error: {e}")
            except Exception as e:
                print(f"[DEBUG] Error processing packet: {e}")

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] UDP error: {e}")

    udp_socket.close()
    # print(f"[Receiver] Stopped. Total packets: {packet_count}, last packet_id: {packet_id}")


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

    udp_backup_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_backup_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    prev_remaining_wp_len = 0

    while not stop_event.is_set():
        now = time.time()

        slam_state = shared_state.get_latest()

        coarse_grid = None
        if slam_processor and slam_processor.map_grid:
            map_data = slam_processor.get_map()
            if map_data:
                coarse = coarse_grid_from_map(map_data, COARSE_FACTOR)
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
                    # print(f"[Broadcaster] Including path with {len(current_path)} points in output")

                if planner.get_goal():
                    output_message["goal"] = {"x": planner.get_goal()[0], "y": planner.get_goal()[1]}

                # Backup path send via UDP
                if current_path:
                    sig = tuple((round(p[0],3), round(p[1],3)) for p in current_path)
                    if sig != last_sent_path_sig:
                        last_sent_path_sig = sig
                        try:
                            path_payload = 'PATH:' + ';'.join([f"{p[0]:.3f},{p[1]:.3f}" for p in current_path])
                            udp_backup_sock.sendto(path_payload.encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                            # print(f"[Broadcaster] Sent backup UDP path with {len(current_path)} points")
                        except Exception as e:
                            print(f"[Broadcaster] UDP backup send error: {e}")
                
                remaining_wp = planner.get_remaining_waypoints() if planner is not None else []
                if len(remaining_wp) != prev_remaining_wp_len:
                    output_message["remaining_waypoints"] = [{"x": wp[0], "y": wp[1]} for wp in remaining_wp]
                prev_remaining_wp_len = len(remaining_wp)

                # Send to relay
                try:
                    await asyncio.wait_for(
                        relay_ws.send(json.dumps(output_message)),
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    print("[Broadcaster] Send timed out - relay may be unresponsive")
                    with relay_lock:
                        relay_connected = False
                except Exception as e:
                    print(f"[Broadcaster] Send error: {e}")
                    with relay_lock:
                        relay_connected = False

        await asyncio.sleep(0.001)

    udp_backup_sock.close()
    # print("[Broadcaster] Stopped")


# ============================================================================
# main
# ============================================================================

async def main():
    global planner, command_forwarder
    shared_packet = AtomicSharedPacket()
    shared_state = AtomicSharedState()
    stop_event = threading.Event()

    command_forwarder = DedicatedCommandForwarder(
        cmd_port=ROBOT_CMD_PORT,
        path_port=ROBOT_PATH_PORT
    )
    command_forwarder.start()

    slam_processor = SlamProcessor(shared_packet, shared_state)

    udp_thread = threading.Thread(target=udp_receiver, args=(shared_packet, stop_event), daemon=True)
    udp_thread.start()

    slam_thread = threading.Thread(target=slam_processor.process_loop, args=(stop_event,), daemon=True)
    slam_thread.start()

    planner = PlannerWorker(slam_processor, shared_state, command_forwarder)
    planner_thread = threading.Thread(target=planner.planner_loop, args=(stop_event,), daemon=True)
    planner_thread.start()

    # Connect to relay
    await connect_to_relay()

    # Start broadcaster
    broadcaster_task = asyncio.create_task(
        websocket_broadcaster(shared_state, slam_processor, planner, stop_event)
    )

    # Main loop: trim path and check for replan
    try:
        while not stop_event.is_set():
            slam_state = shared_state.get_latest()
            if slam_state and planner:
                # print(f"[Main] Trimming path at robot pose ({slam_state.x:.3f},{slam_state.y:.3f})")
                planner.trim_path(slam_state.x, slam_state.y)
                map_update_id = slam_processor.get_map_update_id()
                planner.check_replan(slam_state.x, slam_state.y, map_update_id)
            await asyncio.sleep(0.05)   # check every 50 ms
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        stop_event.set()
        broadcaster_task.cancel()
        command_forwarder.stop()
        udp_thread.join(timeout=2)
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
#!/usr/bin/env python3
"""
UDP to WebSocket Bridge with Grid Map SLAM
Clean architecture with atomic shared state and generation counters
Maintains compatibility with existing web client
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
from typing import Deque
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# ============ Configuration ============
UDP_PORT = 8765
WEBSOCKET_PORT = 8766
BROADCAST_POSE_HZ = 30  # 30 Hz pose updates
BROADCAST_MAP_HZ = 1    # 1 Hz map updates

# Planner settings
COARSE_FACTOR = 8  # how many fine cells per coarse cell
ROBOT_WIDTH = 0.35  # meters

# UDP port robot listens on for commands/path
ROBOT_CMD_PORT = 8767

# Map parameters
MAP_SIZE = 1880  # pixels
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


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


print("=" * 60)
print("UDP to WebSocket Bridge with Grid Map SLAM")
print("Clean Architecture - Client Compatible")
print("=" * 60)
print(f"Map: {MAP_SIZE}x{MAP_SIZE} cells, {MAP_RESOLUTION*100:.0f}cm resolution")
print(f"UDP Receive Port: {UDP_PORT}")
print(f"WebSocket Port: {WEBSOCKET_PORT}")
print(f"Pose broadcast: {BROADCAST_POSE_HZ} Hz")
print(f"Map broadcast: {BROADCAST_MAP_HZ} Hz")
print(f"Correction Weight: {CORRECTION_WEIGHT * 100:.0f}%")
print(f"Angular velocity threshold: {ANGULAR_VEL_THRESHOLD} rad/s ({ANGULAR_VEL_THRESHOLD * 180 / math.pi:.1f}°/s)")
print("=" * 60)


class OccupancyGrid:
    """2D occupancy grid map using log-odds with dynamic expansion"""

    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        self.occupancy = np.full((height, width), -1, dtype=np.int8)
        self.last_update_packet_id = -1

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

    def score_scan_pose(self, robot_x, robot_y, robot_theta, ranges, angles):
        """Score how well a pose aligns with the current map."""
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

            quarter_x = robot_x + 0.25 * r * cos_beam
            quarter_y = robot_y + 0.25 * r * sin_beam
            mid_x = robot_x + 0.50 * r * cos_beam
            mid_y = robot_y + 0.50 * r * sin_beam
            end_x = robot_x + r * cos_beam
            end_y = robot_y + r * sin_beam

            score += 1.8 * self.sample_log_odds(end_x, end_y)
            score -= 0.6 * self.sample_log_odds(mid_x, mid_y)
            score -= 0.3 * self.sample_log_odds(quarter_x, quarter_y)
            valid_points += 1

        if valid_points == 0:
            return float("-inf")

        return score / valid_points

    def refine_pose(self, robot_x, robot_y, robot_theta, ranges, angles):
        """Refine the supplied pose with a small local scan-matching search."""
        if not self.is_ready_for_scan_matching():
            return robot_x, robot_y, robot_theta, 0.0

        best_pose = (robot_x, robot_y, robot_theta)
        best_score = self.score_scan_pose(robot_x, robot_y, robot_theta, ranges, angles)

        # Translation search
        dx_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE, SCAN_MATCH_TRANSLATION_RANGE + 1e-6, SCAN_MATCH_TRANSLATION_STEP)
        dy_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE, SCAN_MATCH_TRANSLATION_RANGE + 1e-6, SCAN_MATCH_TRANSLATION_STEP)

        for dx in dx_values:
            for dy in dy_values:
                candidate_x = robot_x + float(dx)
                candidate_y = robot_y + float(dy)
                candidate_score = self.score_scan_pose(candidate_x, candidate_y, robot_theta, ranges, angles)

                if candidate_score > best_score:
                    best_score = candidate_score
                    best_pose = (candidate_x, candidate_y, robot_theta)

        # Fine search
        if best_pose != (robot_x, robot_y, robot_theta):
            base_x, base_y, _ = best_pose
            dx_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE * 0.5, SCAN_MATCH_TRANSLATION_RANGE * 0.5 + 1e-6, SCAN_MATCH_TRANSLATION_STEP * 0.5)
            dy_values = np.arange(-SCAN_MATCH_TRANSLATION_RANGE * 0.5, SCAN_MATCH_TRANSLATION_RANGE * 0.5 + 1e-6, SCAN_MATCH_TRANSLATION_STEP * 0.5)

            for dx in dx_values:
                for dy in dy_values:
                    candidate_x = base_x + float(dx)
                    candidate_y = base_y + float(dy)
                    candidate_score = self.score_scan_pose(candidate_x, candidate_y, robot_theta, ranges, angles)

                    if candidate_score > best_score:
                        best_score = candidate_score
                        best_pose = (candidate_x, candidate_y, robot_theta)

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

    def update(self, robot_x, robot_y, robot_theta, ranges, angles, packet_id):
        """Update occupancy grid with new LiDAR scan"""
        if packet_id <= self.last_update_packet_id:
            return False
        
        self.last_update_packet_id = packet_id
        
        for i in range(len(ranges)):
            r = ranges[i]
            angle = angles[i]

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            end_x = robot_x + r * math.cos(robot_theta + angle)
            end_y = robot_y + r * math.sin(robot_theta + angle)

            gx, gy = self.world_to_grid(end_x, end_y)
            self.mark_occupied_with_neighbors(gx, gy)

            steps = int(r / self.resolution)
            for step in range(steps):
                t = step * self.resolution / r
                ray_x = robot_x + r * t * math.cos(robot_theta + angle)
                ray_y = robot_y + r * t * math.sin(robot_theta + angle)

                gx, gy = self.world_to_grid(ray_x, ray_y)
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    self.log_odds[gy, gx] += LOG_ODDS_FREE
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], MIN_LOG_ODDS, MAX_LOG_ODDS)

        prob = 1.0 / (1.0 + np.exp(-self.log_odds))
        self.occupancy = np.where(
            self.log_odds == 0, -1,
            np.where(prob > OCCUPIED_THRESHOLD, 100, 0)
        )
        
        return True

    def get_map(self):
        """Get occupancy map for visualization"""
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "data": self.occupancy.flatten().tolist()
        }


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
        self.map_grid = OccupancyGrid(MAP_SIZE, MAP_SIZE, MAP_RESOLUTION)
        
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
        print("[SLAM] Processor thread started")
        
        last_debug = time.time()
        
        while not stop_event.is_set():
            packet, generation = self.shared_packet.get_latest()
            
            if packet is None or generation == self.last_generation:
                time.sleep(0.0001)
                continue
            
            self.last_generation = generation
            self.process_packet(packet)
            
            if time.time() - last_debug > 2.0 and self.processed_count > 0:
                last_debug = time.time()
                print(f"[SLAM] Processed {self.processed_count} scans, {self.skipped_count} skipped, "
                      f"pose=({self.slam_x:.2f}, {self.slam_y:.2f}, {math.degrees(self.slam_theta):.1f}°), "
                      f"last packet_id: {packet.packet_id}")
    
    def process_packet(self, packet: SimulationPacket):
        """Process a single packet and update SLAM state"""
        self.processed_count += 1
        
        angular_vel_abs = abs(packet.angular_vel)
        is_rotating_fast = angular_vel_abs > ANGULAR_VEL_THRESHOLD
        
        if is_rotating_fast:
            self.skipped_count += 1
            if self.skipped_count % 100 == 0:
                print(f"[SLAM] Skipping scan matching (angular_vel={angular_vel_abs:.3f} rad/s)")
        
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
        
        map_updated = False
        if not is_rotating_fast:
            map_updated = self.map_grid.update(self.slam_x, self.slam_y, self.slam_theta, 
                                               packet.ranges, packet.angles, packet.packet_id)
        
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


def coarse_grid_from_map(map_data, coarse_factor=COARSE_FACTOR):
    """Create a downsampled occupancy grid."""
    if not map_data:
        return None

    width = map_data['width']
    height = map_data['height']
    res = map_data['resolution']
    data = np.array(map_data['data'], dtype=np.int8).reshape((height, width))

    cf = int(coarse_factor)
    cw = max(1, width // cf)
    ch = max(1, height // cf)
    cres = res * cf

    coarse = np.zeros((ch, cw), dtype=np.int8)

    for cy in range(ch):
        for cx in range(cw):
            fx0 = cx * cf
            fy0 = cy * cf
            fx1 = min(width, fx0 + cf)
            fy1 = min(height, fy0 + cf)
            block = data[fy0:fy1, fx0:fx1]
            if np.any(block == 100):
                coarse[cy, cx] = 1
    return {
        'width': cw,
        'height': ch,
        'resolution': cres,
        'origin_x': -width * res / 2.0,
        'origin_y': -height * res / 2.0,
        'data': coarse,
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

    # Simplify path
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
        # Remove first point (robot position)
        if simplified and len(simplified) > 0:
            simplified = simplified[1:]

    return simplified


class PlannerWorker:
    def __init__(self, slam_processor, shared_state):
        self.slam_processor = slam_processor
        self.shared_state = shared_state
        self._goal = None  # Private variable for goal
        self.path = []
        self.start_at_goal = None
        self.lock = threading.Lock()
        self.request_queue = deque()  # <-- Use deque() directly

    def get_goal(self):
        """Get the current goal"""
        with self.lock:
            return self._goal
    
    def set_goal(self, x, y):
        """Set the goal by coordinates"""
        with self.lock:
            self._goal = (x, y)
            self.request_queue.append((x, y))
    
    def get_path(self):
        """Get the current path"""
        with self.lock:
            return list(self.path)

    def planner_loop(self, stop_event: threading.Event):
        """Main planning loop"""
        print("[Planner] Thread started")
        while not stop_event.is_set():
            # Check if we have a goal to process
            goal = None
            with self.lock:
                if self.request_queue:
                    goal = self.request_queue.popleft()
            
            if goal is None:
                time.sleep(0.05)
                continue

            # Get current robot pose
            slam_state = self.shared_state.get_latest()
            if slam_state is None:
                continue
            sx = slam_state.x
            sy = slam_state.y

            self.start_at_goal = (sx, sy)

            # Get map and plan path
            map_data = self.slam_processor.get_map()
            coarse = coarse_grid_from_map(map_data)  # Just downsample, no inflation
            planned = astar_plan(coarse, (sx, sy), goal)

            with self.lock:
                self._goal = goal  # Keep the goal
                self.path = planned

            print(f"[Planner] Planned path with {len(planned)} points to ({goal[0]:.2f},{goal[1]:.2f})")
            time.sleep(0.05)


def udp_receiver(shared_packet: AtomicSharedPacket, stop_event: threading.Event):
    """UDP receiver thread - only receives packets and updates shared state"""
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind(('0.0.0.0', UDP_PORT))
    udp_socket.settimeout(0.1)
    
    packet_count = 0
    packet_id = 0
    print(f"[Receiver] UDP thread started on port {UDP_PORT}")
    
    while not stop_event.is_set():
        try:
            data, _ = udp_socket.recvfrom(65535)
            
            try:
                message = data.decode('utf-8')
                # print(f"[DEBUG] Raw UDP received (first 100 chars): {message[:100]}...")
                
                scan_data = json.loads(message)
                
                if scan_data.get('type') == 'lidar_scan':
                    packet_id += 1
                    # print(f"[DEBUG] LiDAR scan received, auto_navigate={scan_data.get('auto_navigate', 'unknown')}")
                    
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
                    
                    if packet_count % 100 == 0:
                        print(f"[Receiver] {packet_count} packets received, last packet_id: {packet_id}")
                        
            except json.JSONDecodeError as e:
                print(f"[DEBUG] JSON decode error: {e}, raw message: {message[:200]}...")
            except Exception as e:
                print(f"[DEBUG] Error processing packet: {e}")
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] UDP error: {e}")
    
    udp_socket.close()
    print(f"[Receiver] Stopped. Total packets: {packet_count}, last packet_id: {packet_id}")

async def websocket_broadcaster(shared_state: AtomicSharedState, slam_processor: SlamProcessor, planner: PlannerWorker, stop_event: threading.Event):
    """WebSocket broadcaster - sends pose at fixed rate, map at lower rate"""
    
    connected_clients = set()
    
    # UDP socket for sending commands/paths to robot
    udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    async def handle_client(websocket):
        print(f"[Broadcaster] Client connected from {websocket.remote_address}")
        connected_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    if data.get('type') == 'command':
                        cmd = data.get('command')
                        print(f"[Command] Received: {cmd}")
                        
                        # Forward all commands to robot via UDP
                        try:
                            if cmd == 'auto':
                                # Toggle auto mode - send both CMD and AUTO to ensure robot catches it
                                udp_cmd_sock.sendto(f"CMD:auto".encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                                # Also send explicit AUTO toggle
                                # We'll let the robot handle the toggle via CMD:auto
                            else:
                                udp_cmd_sock.sendto(f"CMD:{cmd}".encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                        except Exception as e:
                            print(f"[Broadcaster] UDP cmd send error: {e}")

                    elif data.get('type') == 'set_goal':
                        gx = float(data.get('x', 0.0))
                        gy = float(data.get('y', 0.0))
                        print(f"[Broadcaster] Goal set by client: ({gx:.2f},{gy:.2f})")
                        planner.set_goal(gx, gy)

                    elif data.get('type') == 'autonomy':
                        auto = bool(data.get('auto_navigate', True))
                        print(f"[Broadcaster] Autonomy set: {auto}")
                        try:
                            # Send both the command and the explicit autonomy message
                            if auto:
                                udp_cmd_sock.sendto(f"AUTO:1".encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                            else:
                                udp_cmd_sock.sendto(f"AUTO:0".encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                                # Also send a stop command when disabling auto
                                udp_cmd_sock.sendto(f"CMD:stop".encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                        except Exception as e:
                            print(f"[Broadcaster] UDP auto send error: {e}")
                            
                except json.JSONDecodeError as e:
                    print(f"[Broadcaster] JSON parse error: {e}")
                except Exception as e:
                    print(f"[Broadcaster] Error handling message: {e}")
        except websockets.exceptions.ConnectionClosed:
            print(f"[Broadcaster] Client disconnected")
        finally:
            connected_clients.discard(websocket)

    async with websockets.serve(handle_client, "0.0.0.0", WEBSOCKET_PORT):
        print(f"[Broadcaster] WebSocket server on ws://0.0.0.0:{WEBSOCKET_PORT}")
        
        pose_interval = 1.0 / BROADCAST_POSE_HZ
        map_interval = 1.0 / BROADCAST_MAP_HZ
        
        last_pose_broadcast = 0
        last_map_broadcast = 0
        last_sent_state_id = -1
        last_sent_path_sig = None
        
        while not stop_event.is_set():
            now = time.time()
            
            slam_state = shared_state.get_latest()
            
            # Get the coarse grid for visualization
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

            
            if slam_state and connected_clients:
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
                        "auto_navigate": slam_state.auto_navigate,  # Pass through the robot's auto state
                        "linear_vel": slam_state.linear_vel,
                        "angular_vel": slam_state.angular_vel,
                        "slam_match_score": slam_state.match_score,
                        "correction_weight": CORRECTION_WEIGHT if ENABLE_SCAN_MATCHING else 0.0,
                        "scan_matching_skipped": slam_state.scan_matching_skipped,
                    }
                    
                    if now - last_map_broadcast >= map_interval:
                        output_message["map"] = slam_processor.get_map()
                        if coarse_grid:
                            output_message["coarse_grid"] = coarse_grid  # Downsampled grid for A*
                        last_map_broadcast = now

                    current_path = planner.get_path() if planner is not None else []
                    if current_path:
                        output_message['path'] = [{'x': p[0], 'y': p[1]} for p in current_path]
                    
                    if planner.get_goal():
                        output_message["goal"] = {"x": planner.get_goal()[0], "y": planner.get_goal()[1]}

                    if current_path:
                        sig = tuple((round(p[0],3), round(p[1],3)) for p in current_path)
                        if sig != last_sent_path_sig:
                            last_sent_path_sig = sig
                            try:
                                path_payload = 'PATH:' + ';'.join([f"{p[0]:.3f},{p[1]:.3f}" for p in current_path])
                                udp_cmd_sock.sendto(path_payload.encode('utf-8'), ('127.0.0.1', ROBOT_CMD_PORT))
                                print(f"[Broadcaster] Sent PATH with {len(current_path)} points to robot")
                            except Exception as e:
                                print(f"[Broadcaster] UDP path send error: {e}")
                    
                    payload = json.dumps(output_message)
                    await asyncio.gather(*[client.send(payload) for client in connected_clients], return_exceptions=True)
                    
                    if slam_state.state_id != last_sent_state_id:
                        last_sent_state_id = slam_state.state_id
            
            await asyncio.sleep(0.001)
        
        udp_cmd_sock.close()
        print("[Broadcaster] Stopped")


async def main():
    shared_packet = AtomicSharedPacket()
    shared_state = AtomicSharedState()
    stop_event = threading.Event()
    
    slam_processor = SlamProcessor(shared_packet, shared_state)
    
    udp_thread = threading.Thread(target=udp_receiver, args=(shared_packet, stop_event), daemon=True)
    udp_thread.start()
    
    slam_thread = threading.Thread(target=slam_processor.process_loop, args=(stop_event,), daemon=True)
    slam_thread.start()

    planner = PlannerWorker(slam_processor, shared_state)
    planner_thread = threading.Thread(target=planner.planner_loop, args=(stop_event,), daemon=True)
    planner_thread.start()
    
    try:
        await websocket_broadcaster(shared_state, slam_processor, planner, stop_event)
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        stop_event.set()
        udp_thread.join(timeout=2)
        slam_thread.join(timeout=2)
        print("[Main] Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")
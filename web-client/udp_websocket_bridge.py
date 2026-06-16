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
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

# ============ Configuration ============
UDP_PORT = 8765
WEBSOCKET_PORT = 8766
BROADCAST_POSE_HZ = 30  # 30 Hz pose updates
BROADCAST_MAP_HZ = 1    # 1 Hz map updates

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
    # Store the latest ranges/angles for the web client
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
        # Skip if we've already processed this packet
        if packet_id <= self.last_update_packet_id:
            return False
        
        self.last_update_packet_id = packet_id
        
        for i in range(len(ranges)):
            r = ranges[i]
            angle = angles[i]

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            # Endpoint in world frame
            end_x = robot_x + r * math.cos(robot_theta + angle)
            end_y = robot_y + r * math.sin(robot_theta + angle)

            # Mark endpoint as occupied
            gx, gy = self.world_to_grid(end_x, end_y)
            self.mark_occupied_with_neighbors(gx, gy)

            # Ray casting for free space
            steps = int(r / self.resolution)
            for step in range(steps):
                t = step * self.resolution / r
                ray_x = robot_x + r * t * math.cos(robot_theta + angle)
                ray_y = robot_y + r * t * math.sin(robot_theta + angle)

                gx, gy = self.world_to_grid(ray_x, ray_y)
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    self.log_odds[gy, gx] += LOG_ODDS_FREE
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], MIN_LOG_ODDS, MAX_LOG_ODDS)

        # Convert to occupancy for visualization
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
        """Update to a new packet (increment generation)"""
        with self._lock:
            self._packet = packet
            self._generation += 1
    
    def get_latest(self):
        """Get the latest packet and its generation"""
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
        """Update to a new state"""
        with self._lock:
            self._state = state
    
    def get_latest(self):
        """Get the latest state"""
        with self._lock:
            return self._state


class SlamProcessor:
    """SLAM processor that consumes packets and produces pose estimates"""
    
    def __init__(self, shared_packet: AtomicSharedPacket, shared_state: AtomicSharedState):
        self.shared_packet = shared_packet
        self.shared_state = shared_state
        self.map_grid = OccupancyGrid(MAP_SIZE, MAP_SIZE, MAP_RESOLUTION)
        
        # Track last processed generation
        self.last_generation = 0
        
        # Running pose
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_theta = 0.0
        self.slam_initialized = False
        
        # Statistics
        self.processed_count = 0
        self.state_id = 0
    
    def process_loop(self, stop_event: threading.Event):
        """Main processing loop - runs in its own thread"""
        print("[SLAM] Processor thread started")
        
        last_debug = time.time()
        
        while not stop_event.is_set():
            # Get latest packet and generation
            packet, generation = self.shared_packet.get_latest()
            
            # If no new packet, sleep briefly to avoid spinning
            if packet is None or generation == self.last_generation:
                time.sleep(0.0001)
                continue
            
            # New packet available - process immediately
            self.last_generation = generation
            self.process_packet(packet)
            
            # Debug output every 2 seconds
            if time.time() - last_debug > 2.0 and self.processed_count > 0:
                last_debug = time.time()
                print(f"[SLAM] Processed {self.processed_count} scans, "
                      f"pose=({self.slam_x:.2f}, {self.slam_y:.2f}, {math.degrees(self.slam_theta):.1f}°), "
                      f"last packet_id: {packet.packet_id}")
    
    def process_packet(self, packet: SimulationPacket):
        """Process a single packet and update SLAM state"""
        self.processed_count += 1
        
        # Start with raw odometry
        current_x = packet.robot_x
        current_y = -packet.robot_y if FLIP_ROBOT_Y_FROM_SIM else packet.robot_y
        current_theta  = packet.robot_theta
        match_score = 0.0
        
        # Apply scan matching if enabled and map is ready
        if ENABLE_SCAN_MATCHING and self.map_grid.is_ready_for_scan_matching():
            refined_x, refined_y, refined_theta, match_score = self.map_grid.refine_pose(
                current_x, current_y, current_theta, packet.ranges, packet.angles
            )
            
            # Gentle correction
            current_x = (1 - CORRECTION_WEIGHT) * current_x + CORRECTION_WEIGHT * refined_x
            current_y = (1 - CORRECTION_WEIGHT) * current_y + CORRECTION_WEIGHT * refined_y
            theta_diff = wrap_angle(refined_theta - current_theta)
            current_theta = wrap_angle(current_theta + CORRECTION_WEIGHT * 0.5 * theta_diff)
        
        # Initialize on first packet
        if not self.slam_initialized:
            self.slam_x = current_x
            self.slam_y = current_y
            self.slam_theta = current_theta
            self.slam_initialized = True
        else:
            self.slam_x = current_x
            self.slam_y = current_y
            self.slam_theta = current_theta
        
        # print("angles: ", packet.angles)
        # print("ranges: ", packet.ranges)
        
        # Update occupancy grid
        self.map_grid.update(self.slam_x, self.slam_y, self.slam_theta, 
                            packet.ranges, packet.angles, packet.packet_id)
        
        # Create new immutable SlamState with all data needed for client
        self.state_id += 1
        new_state = SlamState(
            state_id=self.state_id,
            timestamp=packet.timestamp,
            x=self.slam_x,
            y=self.slam_y,
            theta=self.slam_theta,
            match_score=match_score,
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
            angular_vel=packet.angular_vel
        )
        
        # Update shared state for broadcaster
        self.shared_state.update(new_state)
    
    def get_map(self):
        """Get the current occupancy grid"""
        return self.map_grid.get_map()


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
            data, addr = udp_socket.recvfrom(65535)
            
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
                    
                    if packet_count % 100 == 0:
                        print(f"[Receiver] {packet_count} packets received, last packet_id: {packet_id}")
                        
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"[Receiver] Error: {e}")
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] UDP error: {e}")
    
    udp_socket.close()
    print(f"[Receiver] Stopped. Total packets: {packet_count}, last packet_id: {packet_id}")


async def websocket_broadcaster(shared_state: AtomicSharedState, slam_processor: SlamProcessor, stop_event: threading.Event):
    """WebSocket broadcaster - sends pose at fixed rate, map at lower rate"""
    
    connected_clients = set()
    
    async def handle_client(websocket):
        print(f"[Broadcaster] Client connected from {websocket.remote_address}")
        connected_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'command':
                        print(f"[Command] Received: {data.get('command')}")
                except:
                    pass
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
        
        while not stop_event.is_set():
            now = time.time()
            
            slam_state = shared_state.get_latest()
            
            if slam_state and connected_clients:
                # Send FULL lidar_scan message (what the client expects) at fixed rate
                if now - last_pose_broadcast >= pose_interval:
                    last_pose_broadcast = now
                    
                    # Build the exact message format that script.js expects
                    output_message = {
                        "type": "lidar_scan",
                        "timestamp": slam_state.timestamp,
                        "num_points": len(slam_state.ranges),
                        "min_range": MIN_RANGE,
                        "max_range": MAX_RANGE,
                        "fov": 6.283,  # 2*pi
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
                    }
                    
                    # Add map periodically
                    if now - last_map_broadcast >= map_interval:
                        output_message["map"] = slam_processor.get_map()
                        last_map_broadcast = now
                    
                    # Send to all connected clients
                    payload = json.dumps(output_message)
                    await asyncio.gather(*[client.send(payload) for client in connected_clients], return_exceptions=True)
                    
                    if slam_state.state_id != last_sent_state_id:
                        last_sent_state_id = slam_state.state_id
                        # Uncomment for debug:
                        # print(f"[Broadcaster] Sent state_id={slam_state.state_id}, pose=({slam_state.x:.2f}, {slam_state.y:.2f})")
            
            await asyncio.sleep(0.001)
        
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
    
    try:
        await websocket_broadcaster(shared_state, slam_processor, stop_event)
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
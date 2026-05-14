#!/usr/bin/env python3
"""
UDP to WebSocket Bridge with Grid Map SLAM
Uses wheel odometry from Webots for robot pose
"""

import asyncio
import socket
import json
import websockets
import numpy as np
import math
import time
from collections import deque

# ============ Configuration ============
UDP_PORT = 8765
WEBSOCKET_PORT = 8766

# Map parameters
MAP_SIZE = 200  # pixels
MAP_RESOLUTION = 0.05  # meters per pixel (10cm)
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
SCAN_MATCH_THETA_RANGE = math.radians(4)  # Reduced from 8 degrees for more stability
SCAN_MATCH_THETA_STEP = math.radians(1)   # Reduced from 2 degrees
SCAN_MATCH_MIN_IMPROVEMENT = 0.05  # Increased from 0.03 for theta corrections
SCAN_MATCH_THETA_MIN_IMPROVEMENT = 0.10  # Minimum improvement threshold for accepting theta changes

def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


print("=" * 60)
print("UDP to WebSocket Bridge with Grid Map SLAM")
print("=" * 60)
print(f"Map: {MAP_SIZE}x{MAP_SIZE} cells, {MAP_RESOLUTION*100:.0f}cm resolution")
print(f"UDP Receive Port: {UDP_PORT}")
print(f"WebSocket Port: {WEBSOCKET_PORT}")
print("=" * 60)


class OccupancyGrid:
    """2D occupancy grid map using log-odds with dynamic expansion"""

    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2

        # Log-odds grid (0 = unknown)
        self.log_odds = np.zeros((height, width), dtype=np.float32)

        # For visualization
        self.occupancy = np.full((height, width), -1, dtype=np.int8)

    def expand_if_needed(self, robot_x, robot_y):
        """Expand grid if robot is near boundaries"""
        gx, gy = self.world_to_grid(robot_x, robot_y)
        
        # If robot is within 10% of edge, expand
        expand_threshold = int(0.1 * self.width)
        needs_expand = False
        expand_direction = None
        
        if gx < expand_threshold:
            needs_expand = True
            expand_direction = 'left'
        elif gx >= self.width - expand_threshold:
            needs_expand = True
            expand_direction = 'right'
        elif gy < expand_threshold:
            needs_expand = True
            expand_direction = 'bottom'
        elif gy >= self.height - expand_threshold:
            needs_expand = True
            expand_direction = 'top'
        
        if needs_expand:
            self._expand_grid(expand_direction)
            return True
        return False

    def _expand_grid(self, direction):
        """Expand grid in specified direction"""
        old_width = self.width
        old_height = self.height
        
        if direction == 'left':
            new_width = int(old_width * 1.5)
            new_height = old_height
            self.width = new_width
            self.height = new_height
            
            # Expand left
            offset_x = int((new_width - old_width) / 2)
            offset_y = 0
        elif direction == 'right':
            new_width = int(old_width * 1.5)
            new_height = old_height
            self.width = new_width
            self.height = new_height
            
            offset_x = int((new_width - old_width) / 2)
            offset_y = 0
        elif direction == 'bottom':
            new_width = old_width
            new_height = int(old_height * 1.5)
            self.width = new_width
            self.height = new_height
            
            offset_x = 0
            offset_y = int((new_height - old_height) / 2)
        elif direction == 'top':
            new_width = old_width
            new_height = int(old_height * 1.5)
            self.width = new_width
            self.height = new_height
            
            offset_x = 0
            offset_y = int((new_height - old_height) / 2)
        
        # Update origin
        self.origin_x -= offset_x * self.resolution
        self.origin_y -= offset_y * self.resolution
        
        # Create new grids
        new_log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        new_occupancy = np.full((self.height, self.width), -1, dtype=np.int8)
        
        # Copy old data to new grid
        new_log_odds[offset_y:offset_y+old_height, offset_x:offset_x+old_width] = self.log_odds
        new_occupancy[offset_y:offset_y+old_height, offset_x:offset_x+old_width] = self.occupancy
        
        self.log_odds = new_log_odds
        self.occupancy = new_occupancy
        
        print(f"[Map] Expanded to {self.width}×{self.height} cells ({direction})")

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates"""
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y

    def is_ready_for_scan_matching(self):
        """Return True once the map has enough structure to support scan matching."""
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

            # Encourage occupied endpoints and free space along the ray.
            score += 1.8 * self.sample_log_odds(end_x, end_y)
            score -= 0.6 * self.sample_log_odds(mid_x, mid_y)
            score -= 0.3 * self.sample_log_odds(quarter_x, quarter_y)
            valid_points += 1

        if valid_points == 0:
            return float("-inf")

        return score / valid_points

    def refine_pose(self, robot_x, robot_y, robot_theta, ranges, angles):
        """Refine the supplied pose with a small local scan-matching search.
        
        Prioritizes translation refinement over rotation refinement to prevent
        theta drift from creating slanted walls in the occupancy grid.
        """
        if not self.is_ready_for_scan_matching():
            return robot_x, robot_y, robot_theta, 0.0

        best_pose = (robot_x, robot_y, robot_theta)
        best_score = self.score_scan_pose(robot_x, robot_y, robot_theta, ranges, angles)

        # Level 1: Coarse translation search (NO theta adjustment)
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

        # Level 2: Fine translation search
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

        # Level 3: Small theta refinement (only if translation search improved things significantly)
        translation_score = best_score
        base_x, base_y, base_theta = best_pose
        dtheta_values = np.arange(-SCAN_MATCH_THETA_RANGE, SCAN_MATCH_THETA_RANGE + 1e-6, SCAN_MATCH_THETA_STEP)

        for dtheta in dtheta_values:
            candidate_theta = wrap_angle(base_theta + float(dtheta))
            candidate_score = self.score_scan_pose(base_x, base_y, candidate_theta, ranges, angles)

            # Only accept theta if improvement is significant (to prevent theta drift)
            if candidate_score > best_score + SCAN_MATCH_THETA_MIN_IMPROVEMENT:
                best_score = candidate_score
                best_pose = (base_x, base_y, candidate_theta)

        return best_pose[0], best_pose[1], best_pose[2], best_score

    def mark_occupied_with_neighbors(self, gx, gy):
        """Mark a cell and its 8 neighbors as occupied for thickness and error tolerance"""
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return
        
        # Mark center cell
        self.log_odds[gy, gx] += LOG_ODDS_OCCUPIED
        self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], MIN_LOG_ODDS, MAX_LOG_ODDS)
        
        # Mark 8 neighbors (4 cardinal + 4 diagonal)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip center (already marked)
                
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Add less weight to neighbors (75% of center weight)
                    neighbor_weight = LOG_ODDS_OCCUPIED * 0.75
                    self.log_odds[ny, nx] += neighbor_weight
                    self.log_odds[ny, nx] = np.clip(self.log_odds[ny, nx], MIN_LOG_ODDS, MAX_LOG_ODDS)

    def update(self, robot_x, robot_y, robot_theta, ranges, angles):
        """Update occupancy grid with new LiDAR scan"""
        # Expand grid if robot is near boundaries
        self.expand_if_needed(robot_x, robot_y)
        
        for i in range(len(ranges)):
            r = ranges[i]
            angle = angles[i]

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            # Endpoint in world frame
            end_x = robot_x + r * math.cos(robot_theta + angle)
            end_y = robot_y + r * math.sin(robot_theta + angle)

            # Mark endpoint as occupied with neighbors for thickness
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
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx],
                                                     MIN_LOG_ODDS, MAX_LOG_ODDS)

        # Update occupancy for visualization.
        self._update_occupancy()

    def _update_occupancy(self):
        """Convert log-odds to occupancy values for visualization"""
        # Probability = 1 / (1 + exp(-log_odds))
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))

        # Unknown = -1, Free = 0, Occupied = 100
        self.occupancy = np.where(
            self.log_odds == 0, -1,
            np.where(prob > OCCUPIED_THRESHOLD, 100, 0)
        )

    def get_map(self):
        """Get occupancy map for visualization"""
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "data": self.occupancy.flatten().tolist()
        }

# Create UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_socket.bind(('0.0.0.0', UDP_PORT))
udp_socket.setblocking(False)
print(f"[UDP] Listening on 0.0.0.0:{UDP_PORT}")

# Initialize map
map_grid = OccupancyGrid(MAP_SIZE, MAP_SIZE, MAP_RESOLUTION)

# Store connected WebSocket clients
connected_clients = set()
packet_count = 0
scan_count = 0
slam_initialized = False
slam_x = 0.0
slam_y = 0.0
slam_theta = 0.0

async def handle_websocket(websocket, path):
    """Handle new WebSocket client connections"""
    print(f"[WebSocket] Client connected from {websocket.remote_address}")
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
        print(f"[WebSocket] Client disconnected")
    finally:
        connected_clients.discard(websocket)

async def forward_udp_to_websocket():
    """Forward UDP packets and update map"""
    global packet_count, scan_count, slam_initialized, slam_x, slam_y, slam_theta
    loop = asyncio.get_event_loop()
    
    last_map_send = 0
    map_send_interval = 1.0  # Send map every second
    
    while True:
        try:
            data, addr = await loop.sock_recvfrom(udp_socket, 65535)
            packet_count += 1
            
            try:
                message = data.decode('utf-8')
                scan_data = json.loads(message)
                
                if scan_data.get('type') == 'lidar_scan':
                    ranges = scan_data.get('ranges', [])
                    angles = scan_data.get('angles', [])
                    raw_robot_x = scan_data.get('robot_x', 0)
                    raw_robot_y = scan_data.get('robot_y', 0)
                    raw_robot_theta = scan_data.get('robot_theta', 0)
                    match_score = 0.0
                    
                    if ranges and angles:
                        scan_count += 1

                        if not slam_initialized:
                            slam_x = raw_robot_x
                            slam_y = raw_robot_y
                            slam_theta = raw_robot_theta
                            slam_initialized = True
                        else:
                            predicted_x = raw_robot_x
                            predicted_y = raw_robot_y
                            predicted_theta = raw_robot_theta

                            refined_x, refined_y, refined_theta, match_score = map_grid.refine_pose(
                                predicted_x,
                                predicted_y,
                                predicted_theta,
                                ranges,
                                angles,
                            )

                            predicted_score = map_grid.score_scan_pose(predicted_x, predicted_y, predicted_theta, ranges, angles)
                            
                            # Accept refinement only if improvement is significant
                            if match_score >= predicted_score + SCAN_MATCH_MIN_IMPROVEMENT:
                                # Check if theta changed significantly - be conservative
                                theta_delta = abs(wrap_angle(refined_theta - predicted_theta))
                                if theta_delta > math.radians(0.5):
                                    # Only accept large theta changes if they're really beneficial
                                    if match_score >= predicted_score + SCAN_MATCH_THETA_MIN_IMPROVEMENT:
                                        slam_x = refined_x
                                        slam_y = refined_y
                                        slam_theta = refined_theta
                                    else:
                                        # Accept translation but keep theta from odometry
                                        slam_x = refined_x
                                        slam_y = refined_y
                                        slam_theta = predicted_theta
                                else:
                                    # Small theta change, accept it
                                    slam_x = refined_x
                                    slam_y = refined_y
                                    slam_theta = refined_theta
                            else:
                                slam_x = predicted_x
                                slam_y = predicted_y
                                slam_theta = predicted_theta
                        
                        # Update occupancy grid
                        map_grid.update(slam_x, slam_y, slam_theta, ranges, angles)
                        
                        # Prepare message for web client
                        output_message = {
                            "type": "lidar_scan",
                            "timestamp": scan_data.get('timestamp', time.time()),
                            "num_points": len(ranges),
                            "min_range": scan_data.get('min_range', 0.1),
                            "max_range": scan_data.get('max_range', MAX_RANGE),
                            "fov": scan_data.get('fov', 6.283),
                            "ranges": ranges,
                            "angles": angles,
                            "robot_x": slam_x,
                            "robot_y": slam_y,
                            "robot_theta": slam_theta,
                            "raw_robot_x": raw_robot_x,
                            "raw_robot_y": raw_robot_y,
                            "raw_robot_theta": raw_robot_theta,
                            "pose_source": "slam" if slam_initialized else "odometry",
                            "left_speed": scan_data.get('left_speed', 0),
                            "right_speed": scan_data.get('right_speed', 0),
                            "auto_navigate": scan_data.get('auto_navigate', True),
                            "linear_vel": scan_data.get('linear_vel', 0),
                            "angular_vel": scan_data.get('angular_vel', 0),
                            "slam_match_score": match_score if slam_initialized else 0.0,
                        }
                        
                        # Send map periodically (not every frame for performance)
                        current_time = time.time()
                        if current_time - last_map_send >= map_send_interval:
                            output_message["map"] = map_grid.get_map()
                            last_map_send = current_time
                            print(f"[Map] Updated, cells occupied: {np.sum(map_grid.occupancy == 100)}")
                        
                        # Send to WebSocket clients
                        if connected_clients:
                            tasks = [client.send(json.dumps(output_message)) for client in connected_clients]
                            if tasks:
                                await asyncio.gather(*tasks, return_exceptions=True)
                                
                        if scan_count % 30 == 0:
                            print(f"[SLAM] Processed {scan_count} scans, pose: x={slam_x:.2f}, y={slam_y:.2f}, theta={slam_theta:.2f}rad, score={output_message['slam_match_score']:.3f}")
                            
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"[Error] {e}")
                
        except Exception as e:
            print(f"[UDP] Error: {e}")
            await asyncio.sleep(0.01)

async def main():
    async with websockets.serve(handle_websocket, "0.0.0.0", WEBSOCKET_PORT):
        print(f"[WebSocket] Server on ws://0.0.0.0:{WEBSOCKET_PORT}")
        print("[Bridge] READY! Connect to ws://localhost:8766")
        print("[SLAM] Building occupancy grid map with wheel odometry")
        print("=" * 60)
        await forward_udp_to_websocket()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")

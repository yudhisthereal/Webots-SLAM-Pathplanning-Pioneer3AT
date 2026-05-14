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

print("=" * 60)
print("UDP to WebSocket Bridge with Grid Map SLAM")
print("=" * 60)
print(f"Map: {MAP_SIZE}x{MAP_SIZE} cells, {MAP_RESOLUTION*100:.0f}cm resolution")
print(f"UDP Receive Port: {UDP_PORT}")
print(f"WebSocket Port: {WEBSOCKET_PORT}")
print("=" * 60)

class OccupancyGrid:
    """2D occupancy grid map using log-odds"""
    
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
    
    def update(self, robot_x, robot_y, robot_theta, ranges, angles):
        """Update occupancy grid with new LiDAR scan"""
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
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.log_odds[gy, gx] += LOG_ODDS_OCCUPIED
                self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], 
                                                 MIN_LOG_ODDS, MAX_LOG_ODDS)
            
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
        
        # Update occupancy for visualization (every 10 scans for performance)
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
    global packet_count, scan_count
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
                    robot_x = scan_data.get('robot_x', 0)
                    robot_y = scan_data.get('robot_y', 0)
                    robot_theta = scan_data.get('robot_theta', 0)
                    
                    if ranges and angles:
                        scan_count += 1
                        
                        # Update occupancy grid
                        map_grid.update(robot_x, robot_y, robot_theta, ranges, angles)
                        
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
                            "robot_x": robot_x,
                            "robot_y": robot_y,
                            "robot_theta": robot_theta,
                            "left_speed": scan_data.get('left_speed', 0),
                            "right_speed": scan_data.get('right_speed', 0),
                            "auto_navigate": scan_data.get('auto_navigate', True),
                            "linear_vel": scan_data.get('linear_vel', 0),
                            "angular_vel": scan_data.get('angular_vel', 0),
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
                            print(f"[SLAM] Processed {scan_count} scans, robot pose: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}rad")
                            
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

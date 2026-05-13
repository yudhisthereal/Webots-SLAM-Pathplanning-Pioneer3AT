#!/usr/bin/env python3
"""
Simple WebSocket Echo Server for Testing
Run this first to verify the web client works
"""

import asyncio
import websockets
import json
import time
import math

async def handle_client(websocket, path):
    """Handle WebSocket client connection"""
    print(f"[Server] Client connected from {websocket.remote_address}")
    
    try:
        # Send initial connection confirmation
        await websocket.send(json.dumps({
            "type": "connection",
            "message": "Connected to test server",
            "timestamp": time.time()
        }))
        print("[Server] Sent connection confirmation")
        
        scan_count = 0
        
        # Handle incoming messages and send test data
        async for message in websocket:
            print(f"[Server] Received: {message}")
            
            try:
                data = json.loads(message)
                cmd = data.get('command')
                print(f"[Server] Command received: {cmd}")
                
                # Echo command back
                await websocket.send(json.dumps({
                    "type": "command_echo",
                    "command": cmd,
                    "timestamp": time.time()
                }))
                
            except Exception as e:
                print(f"[Server] Error parsing message: {e}")
            
            # Send test LiDAR scan every time we receive a message
            scan_count += 1
            
            # Create test LiDAR scan data
            ranges = []
            angles = []
            for i in range(360):
                angle = i * 2 * math.pi / 360
                # Create some fake obstacles for visualization
                if 60 < i < 120:  # Front obstacle
                    range_val = 2.0
                elif 150 < i < 210:  # Left obstacle
                    range_val = 1.5
                else:
                    range_val = 5.0 + math.sin(angle * 5) * 1.0  # Varying distances
                
                ranges.append(range_val)
                angles.append(angle)
            
            test_scan = {
                "type": "lidar_scan",
                "timestamp": time.time(),
                "num_points": 360,
                "min_range": 0.1,
                "max_range": 12.0,
                "fov": 2 * math.pi,
                "ranges": ranges,
                "angles": angles
            }
            
            await websocket.send(json.dumps(test_scan))
            print(f"[Server] Sent test scan #{scan_count} (360 points)")
            
            # Also send robot info periodically
            if scan_count % 5 == 0:
                robot_info = {
                    "type": "robot_info",
                    "timestamp": time.time(),
                    "left_speed": 0.3,
                    "right_speed": 0.3,
                    "auto_navigate": True,
                    "position": {"x": 0.0, "y": 0.0, "theta": 0.0}
                }
                await websocket.send(json.dumps(robot_info))
                print(f"[Server] Sent robot info")
                
    except websockets.exceptions.ConnectionClosed:
        print(f"[Server] Client disconnected")
    except Exception as e:
        print(f"[Server] Error: {e}")

async def main():
    print("=" * 50)
    print("WebSocket Test Server")
    print("=" * 50)
    print("Starting server on ws://localhost:8765")
    print("Waiting for web client to connect...")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    async with websockets.serve(handle_client, "localhost", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

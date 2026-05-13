#!/usr/bin/env python3
"""
UDP to WebSocket Bridge - Corrected asyncio version
"""

import asyncio
import socket
import json
import websockets

# Configuration
UDP_PORT = 8765
WEBSOCKET_PORT = 8766

print("=" * 60)
print("UDP to WebSocket Bridge")
print("=" * 60)
print(f"UDP Receive Port: {UDP_PORT}")
print(f"WebSocket Port: {WEBSOCKET_PORT}")
print("=" * 60)

# Create UDP socket (use traditional socket with asyncio)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_socket.bind(('0.0.0.0', UDP_PORT))
udp_socket.setblocking(False)
print(f"[UDP] Listening on 0.0.0.0:{UDP_PORT}")

# Store connected WebSocket clients
connected_clients = set()
packet_count = 0

async def handle_websocket(websocket, path):
    """Handle new WebSocket client connections"""
    print(f"[WebSocket] Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    
    try:
        # Keep connection alive
        async for message in websocket:
            print(f"[WebSocket] Received: {message[:100]}")
    except websockets.exceptions.ConnectionClosed:
        print(f"[WebSocket] Client disconnected")
    finally:
        connected_clients.discard(websocket)

async def forward_udp_to_websocket():
    """Forward UDP packets to all connected WebSocket clients"""
    global packet_count
    loop = asyncio.get_event_loop()
    
    print("[UDP] Starting UDP forwarder...")
    
    while True:
        try:
            # Correct way: use loop.sock_recv which returns (data, addr)
            # But sock_recv expects a connected socket. Use recvfrom instead.
            data, addr = await loop.sock_recvfrom(udp_socket, 65535)
            packet_count += 1
            
            if packet_count % 10 == 0:
                print(f"[UDP] Packet #{packet_count} from {addr}, size: {len(data)} bytes")
            
            # Decode and validate
            try:
                message = data.decode('utf-8')
                # Quick validation (don't parse full JSON for speed)
                if message.startswith('{"type":"lidar_scan"') or message.startswith('{"type":"robot_info"'):
                    # Forward to all WebSocket clients
                    if connected_clients:
                        tasks = [client.send(message) for client in connected_clients]
                        if tasks:
                            await asyncio.gather(*tasks, return_exceptions=True)
                else:
                    print(f"[UDP] Unknown message type: {message[:50]}")
                    
            except Exception as e:
                print(f"[UDP] Decode error: {e}")
                
        except Exception as e:
            print(f"[UDP] Error: {e}")
            await asyncio.sleep(0.1)

async def main():
    # Start WebSocket server
    async with websockets.serve(handle_websocket, "0.0.0.0", WEBSOCKET_PORT):
        print(f"[WebSocket] Server on ws://0.0.0.0:{WEBSOCKET_PORT}")
        print("[Bridge] READY! Connect to ws://localhost:8766")
        print("=" * 60)
        await forward_udp_to_websocket()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")
        print(f"[UDP] Total packets received: {packet_count}")

import socket

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(('0.0.0.0', 8765))
print("Listening for UDP on port 8765...")

while True:
    data, addr = udp_socket.recvfrom(65535)
    print(f"Received {len(data)} bytes from {addr}")
    print(f"Data: {data[:200]}")
    print("-" * 50)

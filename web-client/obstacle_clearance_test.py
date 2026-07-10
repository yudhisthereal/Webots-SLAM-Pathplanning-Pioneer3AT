#!/usr/bin/env python3
"""
LiDAR Obstacle Clearance Calculator with Real-Time Visualization
Pure LiDAR-based obstacle detection and clearance measurement.
"""

import math
import time
import threading
import numpy as np
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch, Arc
from matplotlib.animation import FuncAnimation

from rplidar import RPLidar

# ============ Configuration ============
LIDAR_PORT = 'COM6'
LIDAR_BAUDRATE = 115200
LIDAR_SCAN_TYPE = "normal"

# Robot physical dimensions
ROBOT_LENGTH = 0.71        # meters
ROBOT_WIDTH = 0.39         # meters
ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2
ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2

# Clearance sector definitions (angles in degrees from robot heading)
FRONT_SECTOR = 30           # ±30° front
REAR_SECTOR = 30            # ±30° rear (150° to 210°)
LEFT_SECTOR = [30, 90]      # 30° to 90°
RIGHT_SECTOR = [-90, -30]   # -90° to -30°

# LiDAR filtering
MIN_RANGE = 0.1             # meters
MAX_RANGE = 8.0             # meters
MIN_QUALITY = 10            # minimum quality threshold

# Visualization settings
VISUALIZATION_RANGE = 5.0   # meters (plot range)
UPDATE_INTERVAL = 100       # milliseconds
SCAN_POINTS_TO_SHOW = 500   # max points to display for performance

# ============ Visualization Setup ============
class ObstacleClearanceVisualizer:
    def __init__(self):
        self.fig, (self.ax_clearance, self.ax_polar, self.ax_bars) = plt.subplots(
            1, 3, figsize=(18, 6), 
            gridspec_kw={'width_ratios': [1.2, 1.2, 0.6]}
        )
        self.fig.canvas.manager.set_window_title('LiDAR Obstacle Clearance Diagnostic Tool')
        
        # Setup clearance map
        self.ax_clearance.set_xlim(-VISUALIZATION_RANGE, VISUALIZATION_RANGE)
        self.ax_clearance.set_ylim(-VISUALIZATION_RANGE, VISUALIZATION_RANGE)
        self.ax_clearance.set_aspect('equal')
        self.ax_clearance.grid(True, alpha=0.3)
        self.ax_clearance.set_title('Obstacle Clearance Map', fontsize=12, fontweight='bold')
        self.ax_clearance.set_xlabel('X (meters)')
        self.ax_clearance.set_ylabel('Y (meters)')
        
        # Robot body representation
        self.robot_body = Rectangle(
            (-ROBOT_HALF_WIDTH, -ROBOT_HALF_LENGTH), 
            ROBOT_WIDTH, ROBOT_LENGTH,
            fill=True, color='cyan', alpha=0.3, zorder=3
        )
        self.ax_clearance.add_patch(self.robot_body)
        
        # Robot heading arrow
        self.heading_arrow = FancyArrowPatch(
            (0, 0), (0, ROBOT_HALF_LENGTH + 0.2),
            arrowstyle='-|>', mutation_scale=20, 
            color='cyan', linewidth=2, zorder=4
        )
        self.ax_clearance.add_patch(self.heading_arrow)
        
        # Clearance zones
        self.front_zone = FancyArrowPatch(
            (0, 0), (0, 0),
            color='green', alpha=0.2, linewidth=0
        )
        self.rear_zone = FancyArrowPatch(
            (0, 0), (0, 0),
            color='green', alpha=0.2, linewidth=0
        )
        self.left_zone = FancyArrowPatch(
            (0, 0), (0, 0),
            color='green', alpha=0.2, linewidth=0
        )
        self.right_zone = FancyArrowPatch(
            (0, 0), (0, 0),
            color='green', alpha=0.2, linewidth=0
        )
        
        # LiDAR scan scatter
        self.scan_scatter = self.ax_clearance.scatter([], [], 
            c=[], cmap='coolwarm', s=1, alpha=0.6, zorder=2
        )
        
        # Danger circles
        self.danger_circles = []
        for dist, color, alpha in [(0.5, 'red', 0.2), (1.0, 'yellow', 0.1)]:
            circle = Circle((0, 0), dist, fill=True, color=color, 
                          alpha=alpha, linestyle='--', linewidth=0.5)
            self.danger_circles.append(circle)
            self.ax_clearance.add_patch(circle)
        
        # Setup polar plot for sector analysis
        self.ax_polar = plt.subplot(1, 3, 2, projection='polar')
        self.ax_polar.set_theta_zero_location('N')
        self.ax_polar.set_theta_direction(-1)
        self.ax_polar.set_title('Sector Analysis', fontsize=12, fontweight='bold')
        self.ax_polar.set_ylim(0, VISUALIZATION_RANGE)
        
        # Sector boundaries
        for angle_deg, color, label in [
            (FRONT_SECTOR, 'blue', 'Front'),
            (180 - REAR_SECTOR, 'purple', 'Rear'),
            (LEFT_SECTOR[1], 'orange', 'Left'),
            (RIGHT_SECTOR[1], 'brown', 'Right')
        ]:
            self.ax_polar.axvline(x=math.radians(angle_deg), color=color, 
                                 alpha=0.3, linestyle='--', linewidth=1)
        
        self.polar_scatter = self.ax_polar.scatter([], [], c=[], 
            cmap='viridis', s=2, alpha=0.5
        )
        
        # Setup bar chart for clearances
        self.ax_bars.set_ylim(0, VISUALIZATION_RANGE)
        self.ax_bars.set_xlim(-0.5, 3.5)
        self.ax_bars.grid(True, alpha=0.3, axis='y')
        self.ax_bars.set_title('Clearances', fontsize=12, fontweight='bold')
        self.ax_bars.set_ylabel('Distance (m)')
        self.ax_bars.set_xticks([0, 1, 2, 3])
        self.ax_bars.set_xticklabels(['Front', 'Rear', 'Left', 'Right'])
        
        # Danger lines
        self.ax_bars.axhline(y=0.5, color='red', linestyle='--', 
                            linewidth=2, alpha=0.7, label='Danger')
        self.ax_bars.axhline(y=1.0, color='yellow', linestyle='--', 
                            linewidth=2, alpha=0.7, label='Warning')
        self.ax_bars.legend(loc='upper right', fontsize=8)
        
        # Bar colors based on danger level
        self.bar_colors = ['green', 'green', 'green', 'green']
        self.bar_container = None
        
        # Status text
        self.status_text = self.fig.text(0.5, 0.02, 'Initializing...', 
            ha='center', fontsize=10, fontweight='bold', color='white')
        
        plt.tight_layout()
        
        # Data storage
        self.clearance_history = {
            'front': deque(maxlen=100),
            'rear': deque(maxlen=100),
            'left': deque(maxlen=100),
            'right': deque(maxlen=100)
        }
        self.last_scan_points = []
        self.last_scan_angles = []
        
    def update_visualization(self, clearances, scan_points=None, scan_angles=None):
        """Update all visualization elements with new data"""
        front, rear, left, right = clearances
        
        # Store history
        self.clearance_history['front'].append(front)
        self.clearance_history['rear'].append(rear)
        self.clearance_history['left'].append(left)
        self.clearance_history['right'].append(right)
        
        if scan_points is not None and scan_angles is not None:
            self.last_scan_points = scan_points
            self.last_scan_angles = scan_angles
        
        # Update clearance map
        self._update_clearance_map(front, rear, left, right)
        
        # Update polar plot
        self._update_polar_plot()
        
        # Update bar chart
        self._update_bar_chart(front, rear, left, right)
        
        # Update status
        self._update_status(front, rear, left, right)
    
    def _update_clearance_map(self, front, rear, left, right):
        """Update the 2D clearance map"""
        # Clear previous zones
        for artist in self.ax_clearance.patches[1:]:  # Keep robot body
            if hasattr(artist, '_clearance_zone'):
                artist.remove()
        
        # Draw clearance zones
        # Front zone (rectangle in front of robot)
        front_rect = Rectangle(
            (-ROBOT_HALF_WIDTH * 0.8, ROBOT_HALF_LENGTH),
            ROBOT_WIDTH * 0.8, front,
            fill=True, alpha=0.3,
            color='green' if front > 1.0 else 'yellow' if front > 0.5 else 'red',
            zorder=1
        )
        front_rect._clearance_zone = True
        self.ax_clearance.add_patch(front_rect)
        
        # Rear zone
        rear_rect = Rectangle(
            (-ROBOT_HALF_WIDTH * 0.8, -ROBOT_HALF_LENGTH - rear),
            ROBOT_WIDTH * 0.8, rear,
            fill=True, alpha=0.3,
            color='green' if rear > 1.0 else 'yellow' if rear > 0.5 else 'red',
            zorder=1
        )
        rear_rect._clearance_zone = True
        self.ax_clearance.add_patch(rear_rect)
        
        # Left zone
        left_rect = Rectangle(
            (ROBOT_HALF_WIDTH, -ROBOT_HALF_LENGTH * 0.8),
            left, ROBOT_LENGTH * 0.8,
            fill=True, alpha=0.3,
            color='green' if left > 1.0 else 'yellow' if left > 0.5 else 'red',
            zorder=1
        )
        left_rect._clearance_zone = True
        self.ax_clearance.add_patch(left_rect)
        
        # Right zone
        right_rect = Rectangle(
            (-ROBOT_HALF_WIDTH - right, -ROBOT_HALF_LENGTH * 0.8),
            right, ROBOT_LENGTH * 0.8,
            fill=True, alpha=0.3,
            color='green' if right > 1.0 else 'yellow' if right > 0.5 else 'red',
            zorder=1
        )
        right_rect._clearance_zone = True
        self.ax_clearance.add_patch(right_rect)
        
        # Update scan points
        if self.last_scan_points and self.last_scan_angles:
            points_to_show = min(len(self.last_scan_points), SCAN_POINTS_TO_SHOW)
            if points_to_show > 0:
                indices = np.linspace(0, len(self.last_scan_points) - 1, 
                                    points_to_show, dtype=int)
                x_points = [self.last_scan_points[i] * math.cos(self.last_scan_angles[i]) 
                          for i in indices]
                y_points = [self.last_scan_points[i] * math.sin(self.last_scan_angles[i]) 
                          for i in indices]
                colors = [min(1.0, p / VISUALIZATION_RANGE) for p in 
                         [self.last_scan_points[i] for i in indices]]
                
                self.scan_scatter.set_offsets(np.c_[x_points, y_points])
                self.scan_scatter.set_array(np.array(colors))
    
    def _update_polar_plot(self):
        """Update the polar sector analysis plot"""
        self.ax_polar.clear()
        self.ax_polar.set_theta_zero_location('N')
        self.ax_polar.set_theta_direction(-1)
        self.ax_polar.set_ylim(0, VISUALIZATION_RANGE)
        
        # Sector boundaries
        for angle_deg, color, label in [
            (FRONT_SECTOR, 'blue', 'Front'),
            (180 - REAR_SECTOR, 'purple', 'Rear'),
            (LEFT_SECTOR[1], 'orange', 'Left'),
            (RIGHT_SECTOR[1], 'brown', 'Right')
        ]:
            self.ax_polar.axvline(x=math.radians(angle_deg), color=color, 
                                 alpha=0.3, linestyle='--', linewidth=1)
        
        # Plot scan points in polar coordinates
        if self.last_scan_points and self.last_scan_angles:
            points_to_show = min(len(self.last_scan_points), SCAN_POINTS_TO_SHOW)
            if points_to_show > 0:
                indices = np.linspace(0, len(self.last_scan_points) - 1, 
                                    points_to_show, dtype=int)
                thetas = [self.last_scan_angles[i] for i in indices]
                radii = [min(self.last_scan_points[i], VISUALIZATION_RANGE) 
                        for i in indices]
                colors = [min(1.0, r / VISUALIZATION_RANGE) for r in radii]
                
                self.ax_polar.scatter(thetas, radii, c=colors, cmap='viridis', 
                                    s=2, alpha=0.6)
    
    def _update_bar_chart(self, front, rear, left, right):
        """Update the clearance bar chart"""
        self.ax_bars.clear()
        self.ax_bars.set_ylim(0, VISUALIZATION_RANGE)
        self.ax_bars.set_xlim(-0.5, 3.5)
        self.ax_bars.grid(True, alpha=0.3, axis='y')
        self.ax_bars.set_title('Clearances', fontsize=12, fontweight='bold')
        self.ax_bars.set_ylabel('Distance (m)')
        self.ax_bars.set_xticks([0, 1, 2, 3])
        self.ax_bars.set_xticklabels(['Front', 'Rear', 'Left', 'Right'])
        
        # Danger lines
        self.ax_bars.axhline(y=0.5, color='red', linestyle='--', 
                            linewidth=2, alpha=0.7)
        self.ax_bars.axhline(y=1.0, color='yellow', linestyle='--', 
                            linewidth=2, alpha=0.7)
        
        # Plot bars with color coding
        clearances = [front, rear, left, right]
        colors = []
        for c in clearances:
            if c < 0.5:
                colors.append('red')
            elif c < 1.0:
                colors.append('orange')
            elif c < 2.0:
                colors.append('yellow')
            else:
                colors.append('green')
        
        bars = self.ax_bars.bar([0, 1, 2, 3], clearances, color=colors, alpha=0.7)
        
        # Add value labels on bars
        for bar, value in zip(bars, clearances):
            height = bar.get_height()
            self.ax_bars.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.2f}m', ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    def _update_status(self, front, rear, left, right):
        """Update status text based on clearance values"""
        min_clearance = min(front, rear, left, right)
        
        if min_clearance < 0.3:
            status = "⚠️ CRITICAL - Obstacle Very Close!"
            color = 'red'
        elif min_clearance < 0.5:
            status = "⚠️ DANGER - Obstacle Near"
            color = 'red'
        elif min_clearance < 1.0:
            status = "⚠️ WARNING - Proceed with Caution"
            color = 'yellow'
        elif min_clearance < 2.0:
            status = "✓ Clear - Safe to Navigate"
            color = 'lightgreen'
        else:
            status = "✓✓ Clear - Large Clearance"
            color = 'green'
        
        self.status_text.set_text(status)
        self.status_text.set_color(color)

# ============ LiDAR Obstacle Clearance Calculator ============
class ObstacleClearanceCalculator:
    def __init__(self):
        self.lidar = None
        self.scan_generator = None
        self.stop_event = threading.Event()
        self.latest_clearances = (5.0, 5.0, 5.0, 5.0)  # front, rear, left, right
        self.latest_scan_points = []
        self.latest_scan_angles = []
        self.data_lock = threading.Lock()
        
    def connect_lidar(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=3)
            info = self.lidar.get_info()
            print(f"[LiDAR] Connected successfully!")
            print(f"  Model: {info.get('model', 'unknown')}")
            print(f"  Firmware: {info.get('firmware', 'unknown')}")
            print(f"  Hardware: {info.get('hardware', 'unknown')}")
            self.scan_generator = self.lidar.iter_scans(scan_type=LIDAR_SCAN_TYPE)
            return True
        except Exception as e:
            print(f"[LiDAR] Failed to connect: {e}")
            return False
    
    def calculate_clearances(self, ranges, angles):
        """
        Calculate obstacle clearances from LiDAR scan data.
        Returns: (front_clearance, rear_clearance, left_clearance, right_clearance)
        """
        # Initialize clearances to maximum
        front_clearance = float('inf')
        rear_clearance = float('inf')
        left_clearance = float('inf')
        right_clearance = float('inf')
        
        # Valid points counter for debugging
        front_points = 0
        rear_points = 0
        left_points = 0
        right_points = 0
        
        for r, angle in zip(ranges, angles):
            # Filter invalid measurements
            if r < MIN_RANGE or r > MAX_RANGE:
                continue
            
            # Robot heading is 0 radians (forward)
            # Angle is the absolute angle from LiDAR
            
            # Normalize angle to [-pi, pi]
            normalized_angle = angle
            while normalized_angle > math.pi:
                normalized_angle -= 2 * math.pi
            while normalized_angle < -math.pi:
                normalized_angle += 2 * math.pi
            
            # Calculate clearance based on sector
            angle_deg = math.degrees(normalized_angle)
            
            # Front sector: -FRONT_SECTOR to +FRONT_SECTOR degrees
            if abs(angle_deg) <= FRONT_SECTOR:
                # Clearance = distance - half_length (projected)
                clearance = r - ROBOT_HALF_LENGTH / abs(math.cos(normalized_angle))
                if clearance < front_clearance:
                    front_clearance = clearance
                front_points += 1
            
            # Rear sector: 180° ± REAR_SECTOR
            elif abs(abs(angle_deg) - 180) <= REAR_SECTOR:
                # For rear, angle relative to rear is angle - 180
                rear_angle = normalized_angle - math.pi
                clearance = r - ROBOT_HALF_LENGTH / abs(math.cos(rear_angle))
                if clearance < rear_clearance:
                    rear_clearance = clearance
                rear_points += 1
            
            # Left sector: 30° to 90°
            elif LEFT_SECTOR[0] <= angle_deg <= LEFT_SECTOR[1]:
                # Clearance = distance * sin(angle) - half_width
                clearance = r * math.sin(normalized_angle) - ROBOT_HALF_WIDTH
                if clearance < left_clearance:
                    left_clearance = clearance
                left_points += 1
            
            # Right sector: -90° to -30°
            elif RIGHT_SECTOR[0] <= angle_deg <= RIGHT_SECTOR[1]:
                # Clearance = distance * |sin(angle)| - half_width
                clearance = r * abs(math.sin(normalized_angle)) - ROBOT_HALF_WIDTH
                if clearance < right_clearance:
                    right_clearance = clearance
                right_points += 1
        
        # Set default if no points in sector
        if front_points == 0:
            front_clearance = MAX_RANGE
        if rear_points == 0:
            rear_clearance = MAX_RANGE
        if left_points == 0:
            left_clearance = MAX_RANGE
        if right_points == 0:
            right_clearance = MAX_RANGE
        
        # Ensure non-negative values
        front_clearance = max(0.0, front_clearance)
        rear_clearance = max(0.0, rear_clearance)
        left_clearance = max(0.0, left_clearance)
        right_clearance = max(0.0, right_clearance)
        
        # Debug output (can be disabled for performance)
        # print(f"Points - Front: {front_points}, Rear: {rear_points}, "
        #       f"Left: {left_points}, Right: {right_points}")
        
        return front_clearance, rear_clearance, left_clearance, right_clearance
    
    def process_scan(self, scan):
        """Process a single LiDAR scan"""
        ranges = []
        angles = []
        
        for quality, angle, distance in scan:
            if distance > 0 and quality >= MIN_QUALITY:
                ranges.append(distance / 1000.0)  # Convert mm to meters
                angles.append(math.radians(angle))
        
        if len(ranges) < 10:  # Need minimum points
            return None
        
        # Calculate clearances
        clearances = self.calculate_clearances(ranges, angles)
        
        with self.data_lock:
            self.latest_clearances = clearances
            self.latest_scan_points = ranges
            self.latest_scan_angles = angles
        
        return clearances
    
    def get_latest_data(self):
        """Thread-safe getter for latest data"""
        with self.data_lock:
            return (self.latest_clearances, 
                   list(self.latest_scan_points), 
                   list(self.latest_scan_angles))
    
    def run(self):
        """Main loop for LiDAR data acquisition"""
        if not self.connect_lidar():
            return
        
        print("[Calculator] Starting obstacle clearance calculation...")
        print("[Calculator] Press Ctrl+C to stop")
        
        scan_count = 0
        last_print_time = time.time()
        
        try:
            while not self.stop_event.is_set():
                try:
                    scan = next(self.scan_generator)
                    scan_count += 1
                    
                    # Process scan
                    clearances = self.process_scan(scan)
                    
                    # Periodic console output
                    if time.time() - last_print_time >= 1.0:
                        if clearances:
                            front, rear, left, right = clearances
                            print(f"\r[Clearances] Front: {front:.2f}m | "
                                  f"Rear: {rear:.2f}m | "
                                  f"Left: {left:.2f}m | "
                                  f"Right: {right:.2f}m | "
                                  f"Scans: {scan_count}", end='')
                        last_print_time = time.time()
                    
                except StopIteration:
                    print("\n[LiDAR] Scan generator stopped, restarting...")
                    try:
                        self.lidar.stop()
                        self.lidar.disconnect()
                        time.sleep(1)
                        if not self.connect_lidar():
                            break
                    except Exception as e:
                        print(f"[LiDAR] Restart failed: {e}")
                        break
                except Exception as e:
                    print(f"\n[Calculator] Error processing scan: {e}")
                    time.sleep(0.01)
                    
        except KeyboardInterrupt:
            print("\n[Calculator] Stopping...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
                print("[LiDAR] Disconnected")
            except:
                pass

# ============ Main Application ============
def main():
    """Main function to run the diagnostic tool"""
    print("=" * 60)
    print("LiDAR Obstacle Clearance Diagnostic Tool")
    print("=" * 60)
    print(f"Robot dimensions: {ROBOT_LENGTH:.2f}m x {ROBOT_WIDTH:.2f}m")
    print(f"Sectors: Front ±{FRONT_SECTOR}°, Rear ±{REAR_SECTOR}°, "
          f"Left {LEFT_SECTOR[0]}-{LEFT_SECTOR[1]}°, "
          f"Right {RIGHT_SECTOR[0]}-{RIGHT_SECTOR[1]}°")
    print("=" * 60)
    
    # Initialize calculator
    calculator = ObstacleClearanceCalculator()
    
    # Initialize visualizer
    visualizer = ObstacleClearanceVisualizer()
    
    # Start LiDAR in separate thread
    lidar_thread = threading.Thread(target=calculator.run, daemon=True)
    lidar_thread.start()
    
    # Wait for first data
    print("[Main] Waiting for LiDAR data...")
    time.sleep(2)
    
    # Animation update function
    def update_plot(frame):
        """Update function for matplotlib animation"""
        clearances, scan_points, scan_angles = calculator.get_latest_data()
        
        if scan_points:  # Only update if we have data
            visualizer.update_visualization(clearances, scan_points, scan_angles)
        
        return []
    
    # Create animation
    ani = FuncAnimation(
        visualizer.fig, 
        update_plot, 
        interval=UPDATE_INTERVAL,
        blit=False,
        cache_frame_data=False
    )
    
    try:
        print("[Main] Starting visualization...")
        print("[Main] Close the plot window to exit")
        plt.show()
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        calculator.stop_event.set()
        calculator.cleanup()
        lidar_thread.join(timeout=2)
        print("[Main] Shutdown complete")

if __name__ == "__main__":
    main()
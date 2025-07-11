#!/usr/bin/env python3
"""
Real-time UWB Position Monitor
Monitor and visualize UWB position data with Kalman filtering in real-time

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import logging
import json
import threading
from pathlib import Path
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Optional, List, Tuple

# Add src directory to Python path
current_dir = Path(__file__).parent
parent_dir = current_dir.parent
src_dir = parent_dir / 'src'
sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(parent_dir))

from src.uart_comm import UARTCommunicator, SystemData
from src.trilateration import Trilaterator, Position2D
from src.kalman_filter import KalmanFilter2D, FilteredPosition

class RealTimeMonitor:
    """Real-time UWB position monitor with Kalman filtering"""
    
    def __init__(self, config_file: str = "config.json", max_points: int = 1000):
        """Initialize the real-time monitor"""
        self.logger = logging.getLogger(__name__)
        self.running = False
        self.max_points = max_points
        
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize components
        self.uart = UARTCommunicator(
            port=self.config.get('uart_port', '/dev/serial0'),
            baudrate=self.config.get('uart_baudrate', 2000000)
        )
        self.trilaterator = Trilaterator()
        
        # Initialize Kalman Filter
        kalman_config = self.config.get('kalman_filter', {})
        self.kalman_filter = KalmanFilter2D(
            process_noise_pos=kalman_config.get('process_noise_pos', 50.0),
            process_noise_vel=kalman_config.get('process_noise_vel', 100.0),
            measurement_noise=kalman_config.get('measurement_noise', 100.0),
            initial_uncertainty=kalman_config.get('initial_uncertainty', 1000.0)
        )
        
        # Data storage for plotting
        self.lock = threading.Lock()
        self.times = deque(maxlen=max_points)
        self.raw_positions = deque(maxlen=max_points)
        self.filtered_positions = deque(maxlen=max_points)
        self.velocities = deque(maxlen=max_points)
        self.confidences = deque(maxlen=max_points)
        self.innovations = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'valid_positions': 0,
            'filtered_positions': 0,
            'start_time': time.time()
        }
        
        # Data thread
        self.data_thread = None
        
    def _load_config(self, config_file: str) -> dict:
        """Load configuration from JSON file"""
        try:
            config_path = Path(__file__).parent / config_file
            if config_path.exists():
                with open(config_path, 'r') as f:
                    return json.load(f)
            else:
                self.logger.warning(f"Config file {config_file} not found, using defaults")
                return {}
        except Exception as e:
            self.logger.error(f"Error loading config: {e}")
            return {}
    
    def start_data_collection(self):
        """Start collecting data from ESP32"""
        self.logger.info("Starting data collection...")
        
        # Connect to ESP32
        if not self.uart.connect():
            self.logger.error("Failed to connect to ESP32")
            return False
        
        self.running = True
        self.data_thread = threading.Thread(target=self._data_collection_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        return True
    
    def stop_data_collection(self):
        """Stop data collection"""
        self.logger.info("Stopping data collection...")
        self.running = False
        
        if self.data_thread:
            self.data_thread.join(timeout=2.0)
        
        self.uart.disconnect()
    
    def _data_collection_loop(self):
        """Main data collection loop"""
        while self.running:
            try:
                # Read data from ESP32
                system_data = self.uart.read_data()
                
                if system_data:
                    self.stats['total_messages'] += 1
                    
                    # Calculate position
                    position = self._calculate_position(system_data)
                    
                    current_time = time.time() - self.stats['start_time']
                    
                    with self.lock:
                        self.times.append(current_time)
                        
                        # Store distance data
                        uwb = system_data.uwb
                        distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
                        self.distances.append(distances)
                        
                        if position.valid:
                            self.stats['valid_positions'] += 1
                            
                            # Store raw position
                            self.raw_positions.append((position.x, position.y))
                            
                            # Apply Kalman filtering
                            filtered_pos = self._apply_kalman_filter(position)
                            if filtered_pos:
                                self.stats['filtered_positions'] += 1
                                self.filtered_positions.append((filtered_pos.x, filtered_pos.y))
                                self.velocities.append((filtered_pos.vx, filtered_pos.vy))
                                self.confidences.append(filtered_pos.confidence)
                                self.innovations.append(filtered_pos.innovation)
                            else:
                                # Fill with None values to keep arrays aligned
                                self.filtered_positions.append((None, None))
                                self.velocities.append((None, None))
                                self.confidences.append(None)
                                self.innovations.append(None)
                        else:
                            # Fill with None values for invalid positions
                            self.raw_positions.append((None, None))
                            self.filtered_positions.append((None, None))
                            self.velocities.append((None, None))
                            self.confidences.append(None)
                            self.innovations.append(None)
                
                time.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                self.logger.error(f"Error in data collection loop: {e}")
                time.sleep(0.1)
    
    def _calculate_position(self, system_data: SystemData) -> Position2D:
        """Calculate position from UWB data"""
        uwb = system_data.uwb
        
        distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
        anchor_status = [uwb.anchor_status['s1'], uwb.anchor_status['s2'], uwb.anchor_status['s3']]
        
        return self.trilaterator.calculate_position(distances, anchor_status)
    
    def _apply_kalman_filter(self, position: Position2D) -> Optional[FilteredPosition]:
        """Apply Kalman filter to the raw UWB position"""
        try:
            # Only filter valid positions
            if not position.valid:
                return None
            
            # Update Kalman filter with new measurement
            if position.error != float('inf'):
                filtered_pos = self.kalman_filter.process_measurement(
                    position.x, position.y, position.error
                )
            else:
                filtered_pos = self.kalman_filter.process_measurement(
                    position.x, position.y
                )
            
            return filtered_pos
                
        except Exception as e:
            self.logger.error(f"Error in Kalman filter: {e}")
            return None
    
    def create_realtime_plot(self):
        """Create real-time plot"""
        self.logger.info("Starting real-time plot...")
        
        # Setup the plot
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle('Real-time UWB Position Monitor with Kalman Filtering')
        
        ax1 = fig.add_subplot(2, 2, 1)
        ax2 = fig.add_subplot(2, 2, 2)
        ax3 = fig.add_subplot(2, 2, 3)
        ax4 = fig.add_subplot(2, 2, 4)
        
        # Initialize empty plots
        raw_scatter = ax1.scatter([], [], c='red', alpha=0.6, s=10, label='Raw UWB')
        filtered_line, = ax1.plot([], [], 'b-', linewidth=2, label='Kalman Filtered')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_title('Position Trajectory')
        ax1.legend()
        ax1.grid(True)
        ax1.set_xlim(-1000, 1000)
        ax1.set_ylim(-1000, 1000)
        
        # Distance plot
        dist_lines = []
        for i in range(3):
            line, = ax2.plot([], [], label=f'Anchor {i+1}')
            dist_lines.append(line)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance (mm)')
        ax2.set_title('UWB Distances')
        ax2.legend()
        ax2.grid(True)
        ax2.set_ylim(0, 2000)
        
        # Velocity plot
        vel_x_line, = ax3.plot([], [], 'b-', label='Velocity X')
        vel_y_line, = ax3.plot([], [], 'g-', label='Velocity Y')
        speed_line, = ax3.plot([], [], 'r-', label='Speed')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (mm/s)')
        ax3.set_title('Estimated Velocity')
        ax3.legend()
        ax3.grid(True)
        ax3.set_ylim(-500, 500)
        
        # Filter metrics - use single axis for simplicity
        conf_line, = ax4.plot([], [], 'b-', label='Confidence')
        innov_line, = ax4.plot([], [], 'r-', label='Innovation (scaled)')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Value')
        ax4.set_title('Filter Metrics')
        ax4.set_ylim(0, 1)
        ax4.legend()
        ax4.grid(True)
        
        # Store references for animation
        artists = [raw_scatter, filtered_line] + dist_lines + [vel_x_line, vel_y_line, speed_line, conf_line, innov_line]
        
        def animate(frame):
            """Animation function"""
            with self.lock:
                if not self.times:
                    return artists
                
                # Convert to numpy arrays
                times_array = np.array(self.times)
                
                # Update position plot
                raw_x = [pos[0] for pos in self.raw_positions if pos[0] is not None]
                raw_y = [pos[1] for pos in self.raw_positions if pos[1] is not None]
                
                filt_x = [pos[0] for pos in self.filtered_positions if pos[0] is not None]
                filt_y = [pos[1] for pos in self.filtered_positions if pos[1] is not None]
                
                if raw_x and raw_y:
                    raw_scatter.set_offsets(np.column_stack([raw_x, raw_y]))
                
                if filt_x and filt_y:
                    filtered_line.set_data(filt_x, filt_y)
                
                # Update distance plot
                if self.distances:
                    distances_array = np.array(self.distances)
                    time_window = times_array[-len(distances_array):]
                    
                    for i in range(3):
                        if len(distances_array) > 0:
                            dist_lines[i].set_data(time_window, distances_array[:, i])
                    
                    # Auto-scale time axis
                    if len(time_window) > 0:
                        ax2.set_xlim(max(0, time_window[-1] - 30), time_window[-1] + 1)
                
                # Update velocity plot
                vel_x_data = [vel[0] for vel in self.velocities if vel[0] is not None]
                vel_y_data = [vel[1] for vel in self.velocities if vel[1] is not None]
                
                if vel_x_data and vel_y_data:
                    vel_times = times_array[-len(vel_x_data):]
                    speed_data = np.sqrt(np.array(vel_x_data)**2 + np.array(vel_y_data)**2)
                    
                    vel_x_line.set_data(vel_times, vel_x_data)
                    vel_y_line.set_data(vel_times, vel_y_data)
                    speed_line.set_data(vel_times, speed_data)
                    
                    if len(vel_times) > 0:
                        ax3.set_xlim(max(0, vel_times[-1] - 30), vel_times[-1] + 1)
                
                # Update filter metrics
                conf_data = [conf for conf in self.confidences if conf is not None]
                innov_data = [innov for innov in self.innovations if innov is not None]
                
                if conf_data:
                    conf_times = times_array[-len(conf_data):]
                    conf_line.set_data(conf_times, conf_data)
                    
                    if len(conf_times) > 0:
                        ax4.set_xlim(max(0, conf_times[-1] - 30), conf_times[-1] + 1)
                
                if innov_data:
                    innov_times = times_array[-len(innov_data):]
                    # Scale innovation to 0-1 range for display
                    scaled_innov = np.array(innov_data) / 200.0  # Scale down by 200mm
                    innov_line.set_data(innov_times, scaled_innov)
                
                # Update statistics in title
                valid_rate = (self.stats['valid_positions'] / max(1, self.stats['total_messages'])) * 100
                filter_rate = (self.stats['filtered_positions'] / max(1, self.stats['valid_positions'])) * 100 if self.stats['valid_positions'] > 0 else 0
                
                fig.suptitle(f'Real-time UWB Monitor - Messages: {self.stats["total_messages"]}, '
                           f'Valid: {valid_rate:.1f}%, Filtered: {filter_rate:.1f}%')
                
                return artists
        
        # Start animation
        anim = FuncAnimation(fig, animate, interval=100, blit=False)
        plt.tight_layout()
        plt.show()
        
        return anim

def setup_logging():
    """Setup logging configuration"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

def main():
    """Main function"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Starting Real-time UWB Position Monitor...")
    
    monitor = None
    try:
        # Create monitor
        monitor = RealTimeMonitor()
        
        # Start data collection
        if not monitor.start_data_collection():
            logger.error("Failed to start data collection")
            return 1
        
        # Give some time for initial data
        time.sleep(2.0)
        
        # Start real-time plot
        anim = monitor.create_realtime_plot()
        
        # Keep alive
        input("\nPress Enter to stop monitoring...")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Monitor failed: {e}")
        return 1
    finally:
        if monitor is not None:
            monitor.stop_data_collection()
    
    logger.info("Real-time monitor stopped")
    return 0

if __name__ == "__main__":
    sys.exit(main())

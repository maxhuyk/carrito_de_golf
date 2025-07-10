#!/usr/bin/env python3
"""
Golf Cart Control System - Main Entry Point
Raspberry Pi control system for UWB-based golf cart navigation

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import logging
import json
import signal
import threading
from pathlib import Path
from typing import Optional

# Add src directory to Python path
sys.path.append(str(Path(__file__).parent / 'src'))

# Import our modules
from uart_comm import UARTCommunicator, SystemData
from trilateration import Trilaterator, Position2D

class GolfCartController:
    """Main controller for the golf cart system"""
    
    def __init__(self, config_file: str = "config.json"):
        """Initialize the golf cart controller"""
        self.logger = logging.getLogger(__name__)
        self.running = False
        
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize components
        self.uart = UARTCommunicator(
            port=self.config.get('uart_port', '/dev/serial0'),
            baudrate=self.config.get('uart_baudrate', 2000000)
        )
        self.trilaterator = Trilaterator()
        
        # Data storage
        self.last_position = Position2D(x=0, y=0, valid=False, error=float('inf'))
        self.last_system_data: Optional[SystemData] = None
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'valid_positions': 0,
            'motor_commands_sent': 0,
            'start_time': time.time()
        }
    
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
    
    def start(self):
        """Start the golf cart control system"""
        self.logger.info("=== Golf Cart Control System Starting ===")
        
        # Connect to ESP32
        if not self.uart.connect():
            self.logger.error("Failed to connect to ESP32")
            return False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.running = True
        self.logger.info("Golf Cart Controller started successfully")
        
        # Main control loop
        self._control_loop()
        
        return True
    
    def stop(self):
        """Stop the golf cart control system"""
        self.logger.info("Stopping Golf Cart Controller...")
        self.running = False
        
        # Send emergency stop
        self.uart.send_emergency_stop()
        
        # Disconnect
        self.uart.disconnect()
        
        self.logger.info("Golf Cart Controller stopped")
    
    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
    
    def _control_loop(self):
        """Main control loop"""
        while self.running:
            try:
                # Read data from ESP32
                system_data = self.uart.read_data()
                
                if system_data:
                    self.last_system_data = system_data
                    self.stats['total_messages'] += 1
                    
                    # Calculate position
                    position = self._calculate_position(system_data)
                    
                    if position.valid:
                        self.last_position = position
                        self.stats['valid_positions'] += 1
                        
                        # Log position periodically
                        if self.stats['total_messages'] % 50 == 0:
                            self.logger.info(f"Position: X={position.x:.1f}mm, Y={position.y:.1f}mm")
                    
                    # TODO: Add navigation and motor control logic here
                    # This is where you would implement:
                    # - Path planning
                    # - Obstacle avoidance  
                    # - Motor control based on desired trajectory
                    
                    # Example simple motor control (REMOVE IN PRODUCTION)
                    if self.stats['total_messages'] % 100 == 0:
                        self._example_motor_control()
                
                time.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                self.logger.error(f"Error in control loop: {e}")
                time.sleep(0.1)
    
    def _calculate_position(self, system_data: SystemData) -> Position2D:
        """Calculate position from UWB data"""
        uwb = system_data.uwb
        
        distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
        anchor_status = [uwb.anchor_status['s1'], uwb.anchor_status['s2'], uwb.anchor_status['s3']]
        
        return self.trilaterator.calculate_position(distances, anchor_status)
    
    def _example_motor_control(self):
        """Example motor control - REMOVE IN PRODUCTION"""
        # This is just for testing - implement real control logic
        self.uart.send_motor_command(0, 0)  # Keep motors stopped
        self.stats['motor_commands_sent'] += 1
    
    def get_status(self) -> dict:
        """Get system status"""
        uptime = time.time() - self.stats['start_time']
        uart_stats = self.uart.get_statistics()
        
        return {
            'running': self.running,
            'uptime': uptime,
            'total_messages': self.stats['total_messages'],
            'valid_positions': self.stats['valid_positions'],
            'motor_commands_sent': self.stats['motor_commands_sent'],
            'last_position': {
                'x': self.last_position.x,
                'y': self.last_position.y,
                'valid': self.last_position.valid,
                'error': self.last_position.error
            },
            'uart_stats': uart_stats
        }

def setup_logging(level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('golf_cart.log')
        ]
    )

def main():
    """Main entry point"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Starting Golf Cart Control System...")
    
    try:
        controller = GolfCartController()
        controller.start()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

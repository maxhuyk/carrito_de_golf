#!/usr/bin/env python3
"""
Golf Cart Control System - Main Entry Point (Simplified)
Raspberry Pi control system for UWB-based golf cart navigation

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import logging
import json
import signal
import os
from pathlib import Path

def setup_logging():
    """Setup logging configuration"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('golf_cart.log')
        ]
    )

def load_config(config_file="config.json"):
    """Load configuration from JSON file"""
    try:
        config_path = Path(__file__).parent / config_file
        if config_path.exists():
            with open(config_path, 'r') as f:
                return json.load(f)
        else:
            print(f"Config file {config_file} not found, using defaults")
            return {}
    except Exception as e:
        print(f"Error loading config: {e}")
        return {}

class GolfCartController:
    """Main controller for the golf cart system"""
    
    def __init__(self, config):
        """Initialize the golf cart controller"""
        self.logger = logging.getLogger(__name__)
        self.running = False
        self.config = config
        
        # Import modules dynamically
        self._import_modules()
        
        # Initialize components
        uart_port = self.config.get('uart_port', '/dev/serial0')
        uart_baudrate = self.config.get('uart_baudrate', 2000000)
        
        self.uart = self.UARTCommunicator(port=uart_port, baudrate=uart_baudrate)
        self.trilaterator = self.Trilaterator()
        
        # Data storage
        self.last_position = None
        self.last_system_data = None
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'valid_positions': 0,
            'motor_commands_sent': 0,
            'start_time': time.time()
        }
    
    def _import_modules(self):
        """Import required modules dynamically"""
        try:
            # Add src directory to path
            current_dir = Path(__file__).parent
            src_dir = current_dir / 'src'
            sys.path.insert(0, str(src_dir))
            
            # Import modules
            import uart_comm
            import trilateration
            
            self.UARTCommunicator = uart_comm.UARTCommunicator
            self.SystemData = uart_comm.SystemData
            self.Trilaterator = trilateration.Trilaterator
            self.Position2D = trilateration.Position2D
            
            self.logger.info("Modules imported successfully")
            
        except ImportError as e:
            self.logger.error(f"Failed to import modules: {e}")
            sys.exit(1)
    
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
        if self.uart:
            self.uart.send_emergency_stop()
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
                    
                    if position and position.valid:
                        self.last_position = position
                        self.stats['valid_positions'] += 1
                        
                        # Log position periodically
                        if self.stats['total_messages'] % 50 == 0:
                            self.logger.info(f"Position: X={position.x:.1f}mm, Y={position.y:.1f}mm")
                    
                    # Log UWB data periodically
                    if self.stats['total_messages'] % 20 == 0:
                        uwb = system_data.uwb
                        self.logger.info(f"UWB: d1={uwb.distances['d1']:.1f}cm, "
                                       f"d2={uwb.distances['d2']:.1f}cm, "
                                       f"d3={uwb.distances['d3']:.1f}cm, "
                                       f"freq={uwb.frequency:.1f}Hz")
                    
                    # Example motor control (REMOVE IN PRODUCTION)
                    if self.stats['total_messages'] % 100 == 0:
                        self._example_motor_control()
                
                time.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                self.logger.error(f"Error in control loop: {e}")
                time.sleep(0.1)
    
    def _calculate_position(self, system_data):
        """Calculate position from UWB data"""
        try:
            uwb = system_data.uwb
            
            distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
            anchor_status = [uwb.anchor_status['s1'], uwb.anchor_status['s2'], uwb.anchor_status['s3']]
            
            return self.trilaterator.calculate_position(distances, anchor_status)
        except Exception as e:
            self.logger.error(f"Position calculation error: {e}")
            return None
    
    def _example_motor_control(self):
        """Example motor control - REMOVE IN PRODUCTION"""
        try:
            # This is just for testing - implement real control logic
            self.uart.send_motor_command(0, 0)  # Keep motors stopped
            self.stats['motor_commands_sent'] += 1
        except Exception as e:
            self.logger.error(f"Motor control error: {e}")
    
    def get_status(self):
        """Get system status"""
        uptime = time.time() - self.stats['start_time']
        uart_stats = self.uart.get_statistics() if self.uart else {}
        
        status = {
            'running': self.running,
            'uptime': uptime,
            'total_messages': self.stats['total_messages'],
            'valid_positions': self.stats['valid_positions'],
            'motor_commands_sent': self.stats['motor_commands_sent'],
            'uart_stats': uart_stats
        }
        
        if self.last_position:
            status['last_position'] = {
                'x': self.last_position.x,
                'y': self.last_position.y,
                'valid': self.last_position.valid,
                'error': self.last_position.error
            }
        
        return status

def main():
    """Main entry point"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Starting Golf Cart Control System...")
    
    try:
        config = load_config()
        controller = GolfCartController(config)
        controller.start()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

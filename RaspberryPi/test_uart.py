#!/usr/bin/env python3
"""
Test Script - UART Communication with ESP32
Simple test to verify serial communication and data parsing

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import json
import logging
from pathlib import Path

# Add src directory to Python path
sys.path.append(str(Path(__file__).parent / 'src'))

from uart_comm import UARTCommunicator
from trilateration import Trilaterator

def setup_logging():
    """Setup simple logging for test"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

def test_uart_communication():
    """Test UART communication with ESP32"""
    logger = logging.getLogger(__name__)
    
    # Create UART communicator and trilaterator
    uart = UARTCommunicator(port='/dev/serial0', baudrate=2000000)
    trilaterator = Trilaterator()
    
    logger.info("=== UART Communication Test ===")
    logger.info("Connecting to ESP32...")
    
    if not uart.connect():
        logger.error("Failed to connect to ESP32")
        return False
    
    logger.info("Connected! Waiting for data...")
    
    try:
        data_count = 0
        start_time = time.time()
        
        while data_count < 20:  # Read 20 messages then exit
            system_data = uart.read_data()
            
            if system_data:
                data_count += 1
                
                # Print UWB data
                uwb = system_data.uwb
                logger.info(f"UWB #{data_count}: "
                           f"d1={uwb.distances['d1']:.1f}cm, "
                           f"d2={uwb.distances['d2']:.1f}cm, "
                           f"d3={uwb.distances['d3']:.1f}cm, "
                           f"freq={uwb.frequency:.1f}Hz")
                
                # Calculate position using trilateration
                distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
                anchor_status = [uwb.anchor_status['s1'], uwb.anchor_status['s2'], uwb.anchor_status['s3']]
                
                position = trilaterator.calculate_position(distances, anchor_status)
                
                if position.valid:
                    logger.info(f"Position: X={position.x:.1f}mm, Y={position.y:.1f}mm, "
                               f"Error={position.error:.1f}mm")
                else:
                    logger.info("Position: INVALID")
                
                # Print power data
                power = system_data.power
                logger.info(f"Power: battery={power.battery_voltage:.1f}V, "
                           f"motors={power.motor_left_current:.1f}A + {power.motor_right_current:.1f}A")
                
                # Test motor command every 10 messages
                if data_count == 10:
                    logger.info("Sending test motor command...")
                    uart.send_motor_command(30, -30)  # Turn right slowly
                    time.sleep(1)
                    uart.send_motor_command(0, 0)     # Stop
            
            time.sleep(0.01)
        
        # Print statistics
        stats = uart.get_statistics()
        elapsed = time.time() - start_time
        
        logger.info("=== Test Results ===")
        logger.info(f"Test duration: {elapsed:.1f}s")
        logger.info(f"Messages received: {stats['messages_received']}")
        logger.info(f"Messages sent: {stats['messages_sent']}")
        logger.info(f"Parse errors: {stats['parse_errors']}")
        logger.info(f"Data rate: {stats['messages_received']/elapsed:.1f} msg/s")
        
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    finally:
        uart.disconnect()
        logger.info("UART disconnected")
    
    return True

def main():
    """Main test function"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Starting UART communication test...")
    
    try:
        success = test_uart_communication()
        if success:
            logger.info("Test completed successfully!")
            return 0
        else:
            logger.error("Test failed!")
            return 1
    except Exception as e:
        logger.error(f"Test error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())

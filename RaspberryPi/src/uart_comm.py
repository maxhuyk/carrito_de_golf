#!/usr/bin/env python3
"""
UART Communication Module
Handles serial communication with ESP32 for UWB data and motor control

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import serial
import json
import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

@dataclass
class UWBData:
    """UWB measurement data from ESP32"""
    distances: Dict[str, float]  # d1, d2, d3 distances in cm
    anchor_status: Dict[str, bool]  # s1, s2, s3 anchor connection status
    frequency: float  # Measurement frequency in Hz
    count: int  # Total measurement count

@dataclass
class PowerData:
    """Power monitoring data from ESP32"""
    battery_voltage: float  # Battery voltage in V
    motor_left_current: float  # Left motor current in A
    motor_right_current: float  # Right motor current in A

@dataclass
class SystemData:
    """Complete system data from ESP32"""
    timestamp: int  # ESP32 timestamp in ms
    uwb: UWBData
    power: PowerData
    raw_json: Dict[str, Any]  # Original JSON data

class UARTCommunicator:
    """UART communication handler for ESP32"""
    
    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 115200):
        """
        Initialize UART communicator
        
        Args:
            port: Serial port (default for RPi GPIO UART)
            baudrate: Baud rate (must match ESP32 setting)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.logger = logging.getLogger(__name__)
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'messages_sent': 0,
            'parse_errors': 0,
            'connection_errors': 0
        }
    
    def connect(self) -> bool:
        """
        Connect to ESP32 via UART
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Wait for connection to stabilize
            time.sleep(0.1)
            
            # Clear any existing data in buffer
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.logger.info(f"UART connected to {self.port} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to UART: {e}")
            self.stats['connection_errors'] += 1
            return False
    
    def disconnect(self):
        """Disconnect from ESP32"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info("UART disconnected")
    
    def read_data(self) -> Optional[SystemData]:
        """
        Read and parse data from ESP32
        
        Returns:
            SystemData object if successful, None otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            # Read line from ESP32
            line = self.serial_conn.readline().decode('utf-8').strip()
            
            if not line:
                return None
            
            # DEBUG: Print raw data received
            print(f"RAW DATA: {repr(line)}")
            
            # Parse JSON
            data = json.loads(line)
            
            # DEBUG: Print parsed JSON
            print(f"PARSED JSON: {data}")
            
            # Verify this is system data
            if data.get('type') != 'system_data':
                print(f"WARNING: Unknown data type: {data.get('type')}")
                return None
            
            # Extract UWB data
            uwb_data = data.get('uwb', {})
            uwb = UWBData(
                distances={
                    'd1': uwb_data.get('d1') if uwb_data.get('d1') is not None else float('nan'),
                    'd2': uwb_data.get('d2') if uwb_data.get('d2') is not None else float('nan'),
                    'd3': uwb_data.get('d3') if uwb_data.get('d3') is not None else float('nan')
                },
                anchor_status={
                    's1': uwb_data.get('s1', False),
                    's2': uwb_data.get('s2', False),
                    's3': uwb_data.get('s3', False)
                },
                frequency=uwb_data.get('freq', 0.0),
                count=uwb_data.get('count', 0)
            )
            
            # Extract power data
            power_data = data.get('power', {})
            power = PowerData(
                battery_voltage=power_data.get('battery_v', 0.0),
                motor_left_current=power_data.get('motor_l_a', 0.0),
                motor_right_current=power_data.get('motor_r_a', 0.0)
            )
            
            # Create system data object
            system_data = SystemData(
                timestamp=data.get('timestamp', 0),
                uwb=uwb,
                power=power,
                raw_json=data
            )
            
            self.stats['messages_received'] += 1
            return system_data
            
        except json.JSONDecodeError as e:
            self.logger.warning(f"JSON parse error: {e}")
            self.stats['parse_errors'] += 1
            return None
        except Exception as e:
            self.logger.error(f"Error reading data: {e}")
            return None
    
    def send_motor_command(self, left_speed: int, right_speed: int, emergency_stop: bool = False) -> bool:
        """
        Send motor command to ESP32
        
        Args:
            left_speed: Left motor speed (-255 to 255)
            right_speed: Right motor speed (-255 to 255)
            emergency_stop: Emergency stop flag
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            # Limit speeds
            left_speed = max(-255, min(255, left_speed))
            right_speed = max(-255, min(255, right_speed))
            
            # Create command JSON
            command = {
                "type": "motor_command",
                "left_speed": left_speed,
                "right_speed": right_speed,
                "emergency_stop": emergency_stop
            }
            
            # Send command
            command_str = json.dumps(command) + '\n'
            self.serial_conn.write(command_str.encode('utf-8'))
            self.serial_conn.flush()
            
            self.stats['messages_sent'] += 1
            return True
            
        except Exception as e:
            self.logger.error(f"Error sending motor command: {e}")
            return False
    
    def send_emergency_stop(self) -> bool:
        """
        Send emergency stop command to ESP32
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            command = {"type": "emergency_stop"}
            command_str = json.dumps(command) + '\n'
            self.serial_conn.write(command_str.encode('utf-8'))
            self.serial_conn.flush()
            
            self.stats['messages_sent'] += 1
            return True
            
        except Exception as e:
            self.logger.error(f"Error sending emergency stop: {e}")
            return False
    
    def send_ping(self) -> bool:
        """
        Send ping to ESP32 for connection test
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            command = {"type": "ping"}
            command_str = json.dumps(command) + '\n'
            self.serial_conn.write(command_str.encode('utf-8'))
            self.serial_conn.flush()
            
            self.stats['messages_sent'] += 1
            return True
            
        except Exception as e:
            self.logger.error(f"Error sending ping: {e}")
            return False
    
    def get_statistics(self) -> Dict[str, int]:
        """Get communication statistics"""
        return self.stats.copy()
    
    def reset_statistics(self):
        """Reset communication statistics"""
        self.stats = {
            'messages_received': 0,
            'messages_sent': 0,
            'parse_errors': 0,
            'connection_errors': 0
        }

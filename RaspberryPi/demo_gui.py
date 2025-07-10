#!/usr/bin/env python3
"""
Golf Cart GUI Demo Mode
Test the GUI interface with synthetic data (no ESP32 required)

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import math
import threading
import random
from pathlib import Path
from typing import Optional
import tkinter as tk
from tkinter import messagebox

# Add src directory to Python path
current_dir = Path(__file__).parent
src_dir = current_dir / 'src'
sys.path.insert(0, str(src_dir))

# Import the original GUI and data structures
from golf_cart_gui import GolfCartGUI
from src.trilateration import Position2D
from src.kalman_filter import FilteredPosition

class DemoDataGenerator:
    """Generate synthetic data for GUI testing"""
    
    def __init__(self):
        self.time_start = time.time()
        self.battery_voltage = 12.5
        self.battery_direction = -0.01  # Slowly decrease
        
    def generate_position_data(self) -> Position2D:
        """Generate synthetic UWB position data"""
        t = time.time() - self.time_start
        
        # Create a figure-8 pattern scaled to 5m range
        x = 2000 * math.sin(t * 0.2)  # 2m amplitude
        y = 1000 * math.sin(t * 0.4)  # 1m amplitude
        
        # Add some noise
        x += random.gauss(0, 100)  # 100mm noise
        y += random.gauss(0, 100)
        
        return Position2D(
            x=x,
            y=y,
            valid=True,
            error=random.uniform(80, 150)
        )
    
    def generate_system_data(self):
        """Generate synthetic system data"""
        # Simulate battery discharge
        self.battery_voltage += self.battery_direction
        if self.battery_voltage < 10.0:
            self.battery_direction = 0.01  # Start charging
        elif self.battery_voltage > 13.0:
            self.battery_direction = -0.01  # Start discharging
        
        # Synthetic system data structure
        class SyntheticPowerData:
            def __init__(self):
                self.battery_voltage = random.uniform(12.0, 12.8)
                self.motor_left_current = random.uniform(0.5, 3.0)
                self.motor_right_current = random.uniform(0.5, 3.0)
        
        class SyntheticUWBData:
            def __init__(self):
                self.distances = {
                    'd1': random.uniform(1000, 4000),
                    'd2': random.uniform(1000, 4000),
                    'd3': random.uniform(1000, 4000)
                }
                self.anchor_status = {'s1': True, 's2': True, 's3': True}
        
        class SyntheticSystemData:
            def __init__(self):
                self.power = SyntheticPowerData()
                self.uwb = SyntheticUWBData()
        
        return SyntheticSystemData()

class DemoGolfCartGUI(GolfCartGUI):
    """Modified GUI for demo mode"""
    
    def __init__(self):
        # Initialize data generator
        self.demo_generator = DemoDataGenerator()
        
        # Call parent constructor
        super().__init__()
        
        # Override title to indicate demo mode
        self.root.title("Golf Cart UWB Control System - DEMO MODE")
        
        # Add demo indicator
        demo_label = tk.Label(
            self.root,
            text="DEMO MODE - Datos Sintéticos",
            bg='#f39c12', fg='white',
            font=('Arial', 12, 'bold')
        )
        demo_label.pack(side=tk.TOP, fill=tk.X)
    
    def _toggle_connection(self):
        """Override connection to use demo mode"""
        if not self.running:
            # Simulate successful connection
            self.running = True
            self.btn_connect.config(text="Desconectar", bg='#e74c3c')
            self.connection_status.config(text="Demo Mode", foreground='orange')
            
            # Start demo data thread
            self.data_thread = threading.Thread(target=self._demo_data_loop, daemon=True)
            self.data_thread.start()
            
            messagebox.showinfo("Demo Mode", "Modo demostración activado!\nDatos sintéticos generándose...")
        else:
            # Disconnect
            self.running = False
            self.btn_connect.config(text="Conectar ESP32", bg='#27ae60')
            self.connection_status.config(text="Desconectado", foreground='red')
    
    def _demo_data_loop(self):
        """Demo data generation loop"""
        while self.running:
            try:
                # Generate synthetic system data
                system_data = self.demo_generator.generate_system_data()
                
                with self.thread_lock:
                    self.system_data = system_data
                    
                    # Update power readings
                    self.battery_voltage.set(round(system_data.power.battery_voltage, 2))
                    self.motor_left_current.set(round(system_data.power.motor_left_current, 2))
                    self.motor_right_current.set(round(system_data.power.motor_right_current, 2))
                    
                    # Generate and process position
                    position = self.demo_generator.generate_position_data()
                    if position.valid:
                        self.current_position = position
                        
                        # Apply Kalman filtering
                        filtered_pos = self.kalman_filter.process_measurement(
                            position.x, position.y, position.error
                        )
                        
                        self.filtered_position = filtered_pos
                        
                        # Update position variables
                        self.position_x.set(round(filtered_pos.x, 1))
                        self.position_y.set(round(filtered_pos.y, 1))
                        self.velocity_x.set(round(filtered_pos.vx, 1))
                        self.velocity_y.set(round(filtered_pos.vy, 1))
                        self.confidence.set(round(filtered_pos.confidence * 100, 1))
                
                time.sleep(0.05)  # 20Hz update rate
                
            except Exception as e:
                print(f"Error in demo data loop: {e}")
                time.sleep(0.1)
    
    def _control_motor(self, direction: str):
        """Override motor control for demo feedback"""
        # Call parent method
        super()._control_motor(direction)
        
        # Provide visual feedback in demo mode
        if direction != 'stop':
            print(f"Demo: Motor command - {direction}, Power: {self.motor_power}")
            print(f"Demo: Left motor: {self.motor_left}, Right motor: {self.motor_right}")

def main():
    """Main demo function"""
    print("Golf Cart GUI - Demo Mode")
    print("=" * 30)
    print("Este modo demuestra la interfaz sin necesidad de ESP32")
    print("Se generarán datos sintéticos para probar todas las funciones")
    print("")
    
    try:
        # Create and run demo GUI
        app = DemoGolfCartGUI()
        app.run()
    except Exception as e:
        print(f"Error starting demo: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Golf Cart Control GUI
Advanced graphical interface for UWB golf cart control system

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import threading
import math
from pathlib import Path
from typing import Optional, Dict, Tuple
import tkinter as tk
from tkinter import ttk, messagebox, Canvas
import json

# Add src directory to Python path
current_dir = Path(__file__).parent
src_dir = current_dir / 'src'
sys.path.insert(0, str(src_dir))

from src.uart_comm import UARTCommunicator, SystemData
from src.trilateration import Trilaterator, Position2D
from src.kalman_filter import KalmanFilter2D, FilteredPosition

class GolfCartGUI:
    """Advanced GUI for Golf Cart Control System"""
    
    def __init__(self):
        """Initialize the GUI"""
        self.root = tk.Tk()
        self.root.title("Golf Cart UWB Control System")
        self.root.geometry("1400x900")
        self.root.configure(bg='#2c3e50')
        
        # Control variables
        self.running = False
        self.motor_power = 128  # 0-255 range
        self.motor_left = 0     # -255 to 255
        self.motor_right = 0    # -255 to 255
        
        # Data variables
        self.current_position: Optional[Position2D] = None
        self.filtered_position: Optional[FilteredPosition] = None
        self.system_data: Optional[SystemData] = None
        
        # Load configuration
        self.config = self._load_config()
        
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
        
        # GUI Variables
        self.battery_voltage = tk.DoubleVar(value=12.0)
        self.motor_left_current = tk.DoubleVar(value=0.0)
        self.motor_right_current = tk.DoubleVar(value=0.0)
        self.position_x = tk.DoubleVar(value=0.0)
        self.position_y = tk.DoubleVar(value=0.0)
        self.velocity_x = tk.DoubleVar(value=0.0)
        self.velocity_y = tk.DoubleVar(value=0.0)
        self.confidence = tk.DoubleVar(value=0.0)
        
        # Threading
        self.data_thread = None
        self.thread_lock = threading.Lock()
        
        # Create GUI elements
        self._create_widgets()
        self._setup_keyboard_bindings()
        
        # Start periodic updates
        self._schedule_updates()
    
    def _load_config(self) -> dict:
        """Load configuration from JSON file"""
        try:
            config_path = Path(__file__).parent / "config.json"
            if config_path.exists():
                with open(config_path, 'r') as f:
                    return json.load(f)
            return {}
        except Exception as e:
            print(f"Error loading config: {e}")
            return {}
    
    def _create_widgets(self):
        """Create all GUI widgets"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create three main sections
        self._create_radar_section(main_frame)
        self._create_control_section(main_frame)
        self._create_status_section(main_frame)
    
    def _create_radar_section(self, parent):
        """Create the radar/positioning display section"""
        # Radar frame
        radar_frame = ttk.LabelFrame(parent, text="Posición UWB - Radio 5m", padding=10)
        radar_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        
        # Radar canvas (500x500 pixels for 10m x 10m area)
        self.radar_canvas = Canvas(
            radar_frame, 
            width=500, 
            height=500, 
            bg='#1a1a1a',
            highlightthickness=2,
            highlightbackground='#34495e'
        )
        self.radar_canvas.pack()
        
        # Draw radar grid and setup
        self._setup_radar_display()
        
        # Position info below radar
        info_frame = ttk.Frame(radar_frame)
        info_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Position labels
        ttk.Label(info_frame, text="Posición:").grid(row=0, column=0, sticky="w")
        ttk.Label(info_frame, textvariable=self.position_x, width=10).grid(row=0, column=1)
        ttk.Label(info_frame, text="mm X").grid(row=0, column=2)
        ttk.Label(info_frame, textvariable=self.position_y, width=10).grid(row=0, column=3)
        ttk.Label(info_frame, text="mm Y").grid(row=0, column=4)
        
        ttk.Label(info_frame, text="Velocidad:").grid(row=1, column=0, sticky="w")
        ttk.Label(info_frame, textvariable=self.velocity_x, width=10).grid(row=1, column=1)
        ttk.Label(info_frame, text="mm/s X").grid(row=1, column=2)
        ttk.Label(info_frame, textvariable=self.velocity_y, width=10).grid(row=1, column=3)
        ttk.Label(info_frame, text="mm/s Y").grid(row=1, column=4)
        
        ttk.Label(info_frame, text="Confianza:").grid(row=2, column=0, sticky="w")
        ttk.Label(info_frame, textvariable=self.confidence, width=10).grid(row=2, column=1)
        ttk.Label(info_frame, text="%").grid(row=2, column=2)
    
    def _setup_radar_display(self):
        """Setup the radar display with grid and reference points"""
        canvas = self.radar_canvas
        
        # Clear canvas
        canvas.delete("all")
        
        # Draw grid lines (every meter = 50 pixels)
        for i in range(11):  # 0 to 10 meters
            x = i * 50
            y = i * 50
            # Vertical lines
            canvas.create_line(x, 0, x, 500, fill='#3a3a3a', width=1)
            # Horizontal lines
            canvas.create_line(0, y, 500, y, fill='#3a3a3a', width=1)
        
        # Draw center cross (cart position)
        center_x, center_y = 250, 250
        canvas.create_line(center_x-15, center_y, center_x+15, center_y, fill='#e74c3c', width=3)
        canvas.create_line(center_x, center_y-15, center_x, center_y+15, fill='#e74c3c', width=3)
        
        # Draw range circles (1m, 2.5m, 5m)
        for radius_m in [1, 2.5, 5]:
            radius_px = radius_m * 50
            canvas.create_oval(
                center_x - radius_px, center_y - radius_px,
                center_x + radius_px, center_y + radius_px,
                outline='#2ecc71', width=1, fill=''
            )
            # Label the circles
            canvas.create_text(
                center_x + radius_px - 20, center_y - 10,
                text=f"{radius_m}m", fill='#2ecc71', font=('Arial', 8)
            )
        
        # Draw cardinal directions
        canvas.create_text(center_x, 20, text="N", fill='#ecf0f1', font=('Arial', 12, 'bold'))
        canvas.create_text(center_x, 480, text="S", fill='#ecf0f1', font=('Arial', 12, 'bold'))
        canvas.create_text(20, center_y, text="W", fill='#ecf0f1', font=('Arial', 12, 'bold'))
        canvas.create_text(480, center_y, text="E", fill='#ecf0f1', font=('Arial', 12, 'bold'))
        
        # Store canvas center for later use
        self.radar_center = (center_x, center_y)
    
    def _create_control_section(self, parent):
        """Create motor control section"""
        control_frame = ttk.LabelFrame(parent, text="Control de Motores", padding=10)
        control_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        
        # Power control
        power_frame = ttk.Frame(control_frame)
        power_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(power_frame, text="Potencia:").pack(side=tk.LEFT)
        self.power_scale = ttk.Scale(
            power_frame, 
            from_=0, to=255, 
            orient=tk.HORIZONTAL,
            length=200,
            command=self._on_power_change
        )
        self.power_scale.set(128)
        self.power_scale.pack(side=tk.LEFT, padx=5)
        
        self.power_label = ttk.Label(power_frame, text="128")
        self.power_label.pack(side=tk.LEFT, padx=5)
        
        # Direction controls
        direction_frame = ttk.Frame(control_frame)
        direction_frame.pack(pady=10)
        
        # Forward button
        self.btn_forward = tk.Button(
            direction_frame, text="↑\nAdelante\n(W)", 
            width=8, height=3,
            bg='#3498db', fg='white', font=('Arial', 10, 'bold'),
            command=lambda: self._control_motor('forward')
        )
        self.btn_forward.grid(row=0, column=1, padx=2, pady=2)
        
        # Left and Right buttons
        self.btn_left = tk.Button(
            direction_frame, text="←\nIzquierda\n(A)", 
            width=8, height=3,
            bg='#3498db', fg='white', font=('Arial', 10, 'bold'),
            command=lambda: self._control_motor('left')
        )
        self.btn_left.grid(row=1, column=0, padx=2, pady=2)
        
        self.btn_right = tk.Button(
            direction_frame, text="→\nDerecha\n(D)", 
            width=8, height=3,
            bg='#3498db', fg='white', font=('Arial', 10, 'bold'),
            command=lambda: self._control_motor('right')
        )
        self.btn_right.grid(row=1, column=2, padx=2, pady=2)
        
        # Backward button
        self.btn_backward = tk.Button(
            direction_frame, text="↓\nAtrás\n(S)", 
            width=8, height=3,
            bg='#3498db', fg='white', font=('Arial', 10, 'bold'),
            command=lambda: self._control_motor('backward')
        )
        self.btn_backward.grid(row=2, column=1, padx=2, pady=2)
        
        # Stop button (center)
        self.btn_stop = tk.Button(
            direction_frame, text="STOP\n(Space)", 
            width=8, height=3,
            bg='#e74c3c', fg='white', font=('Arial', 10, 'bold'),
            command=lambda: self._control_motor('stop')
        )
        self.btn_stop.grid(row=1, column=1, padx=2, pady=2)
        
        # Speed control
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=10)
        
        self.btn_speed_up = tk.Button(
            speed_frame, text="↑ Velocidad (U)", 
            bg='#2ecc71', fg='white', font=('Arial', 10),
            command=self._increase_speed
        )
        self.btn_speed_up.pack(side=tk.LEFT, padx=2)
        
        self.btn_speed_down = tk.Button(
            speed_frame, text="↓ Velocidad (J)", 
            bg='#f39c12', fg='white', font=('Arial', 10),
            command=self._decrease_speed
        )
        self.btn_speed_down.pack(side=tk.RIGHT, padx=2)
        
        # Motor status
        motor_frame = ttk.LabelFrame(control_frame, text="Estado Motores", padding=5)
        motor_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(motor_frame, text="Motor Izq:").grid(row=0, column=0, sticky="w")
        self.motor_left_label = ttk.Label(motor_frame, text=f"{self.motor_left}")
        self.motor_left_label.grid(row=0, column=1)
        
        ttk.Label(motor_frame, text="Motor Der:").grid(row=1, column=0, sticky="w")
        self.motor_right_label = ttk.Label(motor_frame, text=f"{self.motor_right}")
        self.motor_right_label.grid(row=1, column=1)
    
    def _create_status_section(self, parent):
        """Create system status section"""
        status_frame = ttk.LabelFrame(parent, text="Estado del Sistema", padding=10)
        status_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        
        # Left column - Battery and currents
        left_status = ttk.Frame(status_frame)
        left_status.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # Battery voltage
        battery_frame = ttk.LabelFrame(left_status, text="Batería", padding=5)
        battery_frame.pack(fill=tk.X, pady=2)
        
        voltage_frame = ttk.Frame(battery_frame)
        voltage_frame.pack(fill=tk.X)
        
        ttk.Label(voltage_frame, text="Voltaje:").pack(side=tk.LEFT)
        voltage_label = ttk.Label(voltage_frame, textvariable=self.battery_voltage, font=('Arial', 12, 'bold'))
        voltage_label.pack(side=tk.LEFT, padx=5)
        ttk.Label(voltage_frame, text="V").pack(side=tk.LEFT)
        
        # Create voltage progress bar
        self.voltage_progress = ttk.Progressbar(
            battery_frame, 
            mode='determinate',
            maximum=15.0,  # Max voltage
            value=12.0
        )
        self.voltage_progress.pack(fill=tk.X, pady=2)
        
        # Motor currents
        current_frame = ttk.LabelFrame(left_status, text="Corrientes", padding=5)
        current_frame.pack(fill=tk.X, pady=2)
        
        # Left motor current
        left_current_frame = ttk.Frame(current_frame)
        left_current_frame.pack(fill=tk.X)
        ttk.Label(left_current_frame, text="Motor Izq:").pack(side=tk.LEFT)
        ttk.Label(left_current_frame, textvariable=self.motor_left_current, font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5)
        ttk.Label(left_current_frame, text="A").pack(side=tk.LEFT)
        
        # Right motor current
        right_current_frame = ttk.Frame(current_frame)
        right_current_frame.pack(fill=tk.X)
        ttk.Label(right_current_frame, text="Motor Der:").pack(side=tk.LEFT)
        ttk.Label(right_current_frame, textvariable=self.motor_right_current, font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5)
        ttk.Label(right_current_frame, text="A").pack(side=tk.LEFT)
        
        # Right column - Connection status
        right_status = ttk.Frame(status_frame)
        right_status.pack(side=tk.RIGHT, fill=tk.Y, padx=10)
        
        # Connection controls
        connection_frame = ttk.LabelFrame(right_status, text="Conexión", padding=5)
        connection_frame.pack(fill=tk.X, pady=2)
        
        self.btn_connect = tk.Button(
            connection_frame, text="Conectar ESP32", 
            bg='#27ae60', fg='white', font=('Arial', 10),
            command=self._toggle_connection
        )
        self.btn_connect.pack(fill=tk.X, pady=2)
        
        self.connection_status = ttk.Label(
            connection_frame, 
            text="Desconectado", 
            foreground='red',
            font=('Arial', 10, 'bold')
        )
        self.connection_status.pack(pady=2)
        
        # Emergency stop
        emergency_frame = ttk.LabelFrame(right_status, text="Emergencia", padding=5)
        emergency_frame.pack(fill=tk.X, pady=2)
        
        self.btn_emergency = tk.Button(
            emergency_frame, text="PARADA DE EMERGENCIA", 
            bg='#c0392b', fg='white', font=('Arial', 12, 'bold'),
            command=self._emergency_stop
        )
        self.btn_emergency.pack(fill=tk.X, pady=2)
        
        # Configure grid weights
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=0)
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        parent.grid_columnconfigure(2, weight=0)
    
    def _setup_keyboard_bindings(self):
        """Setup keyboard bindings for motor control"""
        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)
        self.root.focus_set()  # Make sure window can receive key events
        
        # Store pressed keys
        self.pressed_keys = set()
    
    def _on_key_press(self, event):
        """Handle key press events"""
        key = event.keysym.lower()
        self.pressed_keys.add(key)
        
        # Update motor control based on currently pressed keys
        self._update_motor_from_keys()
        
        # Handle non-movement keys
        if key == 'space':
            self._control_motor('stop')
        elif key == 'u':
            self._increase_speed()
        elif key == 'j':
            self._decrease_speed()
    
    def _on_key_release(self, event):
        """Handle key release events"""
        key = event.keysym.lower()
        self.pressed_keys.discard(key)
        
        # Update motor control based on remaining pressed keys
        self._update_motor_from_keys()
    
    def _update_motor_from_keys(self):
        """Update motor control based on currently pressed keys"""
        forward_pressed = 'w' in self.pressed_keys
        backward_pressed = 's' in self.pressed_keys
        left_pressed = 'a' in self.pressed_keys
        right_pressed = 'd' in self.pressed_keys
        
        # Determine primary movement
        if forward_pressed and not backward_pressed:
            self._control_motor('forward')
        elif backward_pressed and not forward_pressed:
            self._control_motor('backward')
        elif left_pressed and not right_pressed and not (forward_pressed or backward_pressed):
            self._control_motor('left')
        elif right_pressed and not left_pressed and not (forward_pressed or backward_pressed):
            self._control_motor('right')
        elif not any(k in self.pressed_keys for k in ['w', 's', 'a', 'd']):
            self._control_motor('stop')
    
    def _on_power_change(self, value):
        """Handle power scale change"""
        self.motor_power = int(float(value))
        if hasattr(self, 'power_label'):
            self.power_label.config(text=str(self.motor_power))
    
    def _control_motor(self, direction: str):
        """Control motor movement with smooth steering"""
        # Check if multiple keys are pressed for combined movements
        forward_pressed = 'w' in self.pressed_keys
        backward_pressed = 's' in self.pressed_keys
        left_pressed = 'a' in self.pressed_keys
        right_pressed = 'd' in self.pressed_keys
        
        if direction == 'forward' or forward_pressed:
            if left_pressed and not right_pressed:
                # Forward + Left: smooth turn while moving forward
                self.motor_left = int(self.motor_power * 0.5)    # Reduce left motor
                self.motor_right = self.motor_power              # Full right motor
            elif right_pressed and not left_pressed:
                # Forward + Right: smooth turn while moving forward
                self.motor_left = self.motor_power               # Full left motor
                self.motor_right = int(self.motor_power * 0.5)   # Reduce right motor
            else:
                # Pure forward
                self.motor_left = self.motor_power
                self.motor_right = self.motor_power
                
        elif direction == 'backward' or backward_pressed:
            if left_pressed and not right_pressed:
                # Backward + Left: smooth turn while moving backward
                self.motor_left = int(-self.motor_power * 0.5)   # Reduce left motor
                self.motor_right = -self.motor_power             # Full right motor
            elif right_pressed and not left_pressed:
                # Backward + Right: smooth turn while moving backward
                self.motor_left = -self.motor_power              # Full left motor
                self.motor_right = int(-self.motor_power * 0.5)  # Reduce right motor
            else:
                # Pure backward
                self.motor_left = -self.motor_power
                self.motor_right = -self.motor_power
                
        elif direction == 'left' and not (forward_pressed or backward_pressed):
            # Pure left turn (stationary) - more aggressive
            self.motor_left = int(-self.motor_power * 0.5)
            self.motor_right = int(self.motor_power * 0.5)
            
        elif direction == 'right' and not (forward_pressed or backward_pressed):
            # Pure right turn (stationary) - more aggressive
            self.motor_left = int(self.motor_power * 0.5)
            self.motor_right = int(-self.motor_power * 0.5)
            
        elif direction == 'stop':
            self.motor_left = 0
            self.motor_right = 0
        
        # Send command to ESP32
        if self.running:
            self.uart.send_motor_command(self.motor_left, self.motor_right)
        
        # Update visual feedback
        self._update_motor_display()
    
    def _increase_speed(self):
        """Increase motor power"""
        self.motor_power = min(255, self.motor_power + 15)
        self.power_scale.set(self.motor_power)
        self.power_label.config(text=str(self.motor_power))
    
    def _decrease_speed(self):
        """Decrease motor power"""
        self.motor_power = max(0, self.motor_power - 15)
        self.power_scale.set(self.motor_power)
        self.power_label.config(text=str(self.motor_power))
    
    def _update_motor_display(self):
        """Update motor status display"""
        if hasattr(self, 'motor_left_label'):
            self.motor_left_label.config(text=f"{self.motor_left}")
        if hasattr(self, 'motor_right_label'):
            self.motor_right_label.config(text=f"{self.motor_right}")
    
    def _toggle_connection(self):
        """Toggle ESP32 connection"""
        if not self.running:
            # Try to connect
            if self.uart.connect():
                self.running = True
                self.btn_connect.config(text="Desconectar", bg='#e74c3c')
                self.connection_status.config(text="Conectado", foreground='green')
                
                # Start data thread
                self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
                self.data_thread.start()
            else:
                messagebox.showerror("Error", "No se pudo conectar al ESP32")
        else:
            # Disconnect
            self.running = False
            self.uart.disconnect()
            self.btn_connect.config(text="Conectar ESP32", bg='#27ae60')
            self.connection_status.config(text="Desconectado", foreground='red')
    
    def _emergency_stop(self):
        """Emergency stop - immediately stop all motors"""
        self.motor_left = 0
        self.motor_right = 0
        
        if self.running:
            self.uart.send_emergency_stop()
        
        # Visual feedback
        messagebox.showwarning("Parada de Emergencia", "Motores detenidos inmediatamente!")
    
    def _data_loop(self):
        """Main data processing loop"""
        while self.running:
            try:
                # Read data from ESP32
                system_data = self.uart.read_data()
                
                if system_data:
                    with self.thread_lock:
                        self.system_data = system_data
                        
                        # Update battery and current readings
                        self.battery_voltage.set(round(system_data.power.battery_voltage, 2))
                        self.motor_left_current.set(round(system_data.power.motor_left_current, 2))
                        self.motor_right_current.set(round(system_data.power.motor_right_current, 2))
                        
                        # Calculate position
                        position = self._calculate_position(system_data)
                        if position.valid:
                            self.current_position = position
                            
                            # Apply Kalman filtering
                            if position.error != float('inf'):
                                filtered_pos = self.kalman_filter.process_measurement(
                                    position.x, position.y, position.error
                                )
                            else:
                                filtered_pos = self.kalman_filter.process_measurement(
                                    position.x, position.y
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
                print(f"Error in data loop: {e}")
                time.sleep(0.1)
    
    def _calculate_position(self, system_data: SystemData) -> Position2D:
        """Calculate position from UWB data"""
        uwb = system_data.uwb
        
        distances = [uwb.distances['d1'], uwb.distances['d2'], uwb.distances['d3']]
        anchor_status = [uwb.anchor_status['s1'], uwb.anchor_status['s2'], uwb.anchor_status['s3']]
        
        return self.trilaterator.calculate_position(distances, anchor_status)
    
    def _update_radar_display(self):
        """Update the radar display with current position"""
        if self.filtered_position is None:
            return
        
        canvas = self.radar_canvas
        center_x, center_y = self.radar_center
        
        # Clear previous position markers
        canvas.delete("position_marker")
        canvas.delete("velocity_vector")
        
        # Convert position to canvas coordinates
        # Scale: 50 pixels = 1000mm
        pos_x_px = center_x + (self.filtered_position.x / 1000.0 * 50)
        pos_y_px = center_y - (self.filtered_position.y / 1000.0 * 50)  # Invert Y for display
        
        # Draw position marker (tag position relative to cart)
        if abs(self.filtered_position.x) <= 5000 and abs(self.filtered_position.y) <= 5000:  # Within 5m range
            # Draw position dot
            canvas.create_oval(
                pos_x_px - 5, pos_y_px - 5,
                pos_x_px + 5, pos_y_px + 5,
                fill='#f1c40f', outline='#f39c12', width=2,
                tags="position_marker"
            )
            
            # Draw velocity vector if significant
            speed = math.sqrt(self.filtered_position.vx**2 + self.filtered_position.vy**2)
            if speed > 50:  # Show vector if speed > 50 mm/s
                # Scale velocity vector (1 m/s = 50 pixels)
                vel_scale = 0.1  # Scale factor for display
                vel_x_px = (self.filtered_position.vx / 1000.0 * 50) * vel_scale
                vel_y_px = -(self.filtered_position.vy / 1000.0 * 50) * vel_scale  # Invert Y
                
                canvas.create_line(
                    pos_x_px, pos_y_px,
                    pos_x_px + vel_x_px, pos_y_px + vel_y_px,
                    fill='#9b59b6', width=3, arrow=tk.LAST,
                    tags="velocity_vector"
                )
            
            # Draw trail (last few positions)
            # This could be enhanced to show a trail of recent positions
    
    def _update_battery_display(self):
        """Update battery voltage progress bar"""
        voltage = self.battery_voltage.get()
        self.voltage_progress['value'] = voltage
        
        # Change color based on voltage level
        if voltage < 10.5:  # Low battery
            self.voltage_progress.configure(style='red.Horizontal.TProgressbar')
        elif voltage < 11.5:  # Medium battery
            self.voltage_progress.configure(style='yellow.Horizontal.TProgressbar')
        else:  # Good battery
            self.voltage_progress.configure(style='green.Horizontal.TProgressbar')
    
    def _schedule_updates(self):
        """Schedule periodic GUI updates"""
        def update():
            self._update_radar_display()
            self._update_battery_display()
            
            # Schedule next update
            self.root.after(100, update)  # 10Hz GUI update
        
        # Start the update cycle
        self.root.after(100, update)
    
    def run(self):
        """Start the GUI"""
        try:
            self.root.mainloop()
        finally:
            # Cleanup
            if self.running:
                self.running = False
                if self.uart:
                    self.uart.send_emergency_stop()
                    self.uart.disconnect()

def main():
    """Main function"""
    try:
        # Create and run the GUI
        app = GolfCartGUI()
        app.run()
    except Exception as e:
        print(f"Error starting GUI: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

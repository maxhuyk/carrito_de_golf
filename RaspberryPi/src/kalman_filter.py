#!/usr/bin/env python3
"""
Kalman Filter Module
Extended Kalman Filter for UWB position tracking with velocity estimation

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import numpy as np
import time
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

@dataclass
class FilteredPosition:
    """Filtered position with velocity and confidence"""
    x: float          # X position in mm
    y: float          # Y position in mm
    vx: float         # X velocity in mm/s
    vy: float         # Y velocity in mm/s
    valid: bool       # True if filter has converged
    confidence: float # Filter confidence (0-1)
    innovation: float # Innovation magnitude (measurement residual)

class KalmanFilter2D:
    """2D Kalman Filter for position and velocity tracking"""
    
    def __init__(self, 
                 process_noise_pos: float = 50.0,    # Position process noise (mm)
                 process_noise_vel: float = 100.0,   # Velocity process noise (mm/s)
                 measurement_noise: float = 100.0,   # Measurement noise (mm)
                 initial_uncertainty: float = 1000.0): # Initial position uncertainty (mm)
        """
        Initialize Kalman Filter for 2D position tracking
        
        Args:
            process_noise_pos: Process noise for position (how much position can change)
            process_noise_vel: Process noise for velocity (how much velocity can change)
            measurement_noise: Expected measurement noise from UWB
            initial_uncertainty: Initial position uncertainty
        """
        self.logger = logging.getLogger(__name__)
        
        # State vector: [x, y, vx, vy] (position and velocity)
        self.state = np.zeros(4)  # [x, y, vx, vy]
        
        # State covariance matrix (4x4)
        self.P = np.eye(4) * initial_uncertainty
        self.P[2, 2] = 1000.0  # Initial velocity uncertainty
        self.P[3, 3] = 1000.0
        
        # Process noise matrix Q (4x4)
        self.Q = np.array([
            [process_noise_pos**2, 0, 0, 0],
            [0, process_noise_pos**2, 0, 0],
            [0, 0, process_noise_vel**2, 0],
            [0, 0, 0, process_noise_vel**2]
        ])
        
        # Measurement noise matrix R (2x2) - we measure x,y positions
        self.R = np.eye(2) * measurement_noise**2
        
        # Measurement matrix H (2x4) - we observe positions, not velocities
        self.H = np.array([
            [1, 0, 0, 0],  # x position
            [0, 1, 0, 0]   # y position
        ])
        
        # State transition matrix F (4x4) - will be updated with dt
        self.F = np.eye(4)
        
        # Filter status
        self.initialized = False
        self.last_update_time = None
        self.update_count = 0
        
        # Statistics
        self.innovation_history = []
        self.max_innovation_history = 50
        
        self.logger.info("Kalman Filter initialized")
        self.logger.info(f"Process noise: pos={process_noise_pos}mm, vel={process_noise_vel}mm/s")
        self.logger.info(f"Measurement noise: {measurement_noise}mm")
    
    def predict(self, dt: float):
        """
        Prediction step of Kalman filter
        
        Args:
            dt: Time step in seconds
        """
        if not self.initialized:
            return
        
        # Update state transition matrix with time step
        self.F = np.array([
            [1, 0, dt, 0],   # x = x + vx*dt
            [0, 1, 0, dt],   # y = y + vy*dt
            [0, 0, 1, 0],    # vx = vx
            [0, 0, 0, 1]     # vy = vy
        ])
        
        # Predict state: x_k|k-1 = F * x_k-1|k-1
        self.state = self.F @ self.state
        
        # Predict covariance: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q * dt
    
    def update(self, measurement: np.ndarray, measurement_noise: Optional[float] = None):
        """
        Update step of Kalman filter
        
        Args:
            measurement: [x, y] position measurement in mm
            measurement_noise: Optional override for measurement noise
        """
        if measurement_noise is not None:
            # Temporarily update measurement noise
            R = np.eye(2) * measurement_noise**2
        else:
            R = self.R
        
        # Innovation: y = z - H * x_k|k-1
        innovation = measurement - self.H @ self.state
        
        # Innovation covariance: S = H * P_k|k-1 * H^T + R
        S = self.H @ self.P @ self.H.T + R
        
        # Kalman gain: K = P_k|k-1 * H^T * S^-1
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.logger.warning("Singular matrix in Kalman gain calculation")
            return
        
        # Update state: x_k|k = x_k|k-1 + K * y
        self.state = self.state + K @ innovation
        
        # Update covariance: P_k|k = (I - K * H) * P_k|k-1
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P
        
        # Store innovation for statistics
        innovation_magnitude = np.linalg.norm(innovation)
        self.innovation_history.append(innovation_magnitude)
        if len(self.innovation_history) > self.max_innovation_history:
            self.innovation_history.pop(0)
        
        self.update_count += 1
        
        if not self.initialized and self.update_count >= 3:
            self.initialized = True
            self.logger.info("Kalman Filter initialized with first measurements")
    
    def process_measurement(self, x: float, y: float, measurement_error: Optional[float] = None) -> FilteredPosition:
        """
        Process a new UWB measurement
        
        Args:
            x, y: Measured position in mm
            measurement_error: Optional measurement error override
            
        Returns:
            FilteredPosition with filtered results
        """
        current_time = time.time()
        
        # Calculate time step
        if self.last_update_time is not None:
            dt = current_time - self.last_update_time
            dt = max(0.001, min(1.0, dt))  # Clamp dt between 1ms and 1s
        else:
            dt = 0.1  # Default time step
        
        # If not initialized, use measurement as initial state
        if not self.initialized:
            self.state = np.array([x, y, 0, 0])  # Position with zero velocity
            self.last_update_time = current_time
        
        # Predict step
        self.predict(dt)
        
        # Update step
        measurement = np.array([x, y])
        self.update(measurement, measurement_error)
        
        self.last_update_time = current_time
        
        # Calculate confidence based on trace of covariance matrix
        position_uncertainty = np.sqrt(self.P[0, 0] + self.P[1, 1])
        confidence = max(0.0, min(1.0, 1.0 - position_uncertainty / 1000.0))
        
        # Get current innovation
        current_innovation = self.innovation_history[-1] if self.innovation_history else 0.0
        
        return FilteredPosition(
            x=float(self.state[0]),
            y=float(self.state[1]),
            vx=float(self.state[2]),
            vy=float(self.state[3]),
            valid=self.initialized,
            confidence=confidence,
            innovation=current_innovation
        )
    
    def get_position(self) -> Tuple[float, float]:
        """Get current filtered position"""
        return float(self.state[0]), float(self.state[1])
    
    def get_velocity(self) -> Tuple[float, float]:
        """Get current estimated velocity in mm/s"""
        return float(self.state[2]), float(self.state[3])
    
    def get_speed(self) -> float:
        """Get current speed magnitude in mm/s"""
        vx, vy = self.get_velocity()
        return float(np.sqrt(vx**2 + vy**2))
    
    def get_uncertainty(self) -> Tuple[float, float]:
        """Get position uncertainty (standard deviation) in mm"""
        return float(np.sqrt(self.P[0, 0])), float(np.sqrt(self.P[1, 1]))
    
    def get_innovation_stats(self) -> dict:
        """Get innovation statistics"""
        if not self.innovation_history:
            return {'mean': 0.0, 'std': 0.0, 'max': 0.0}
        
        innovations = np.array(self.innovation_history)
        return {
            'mean': float(np.mean(innovations)),
            'std': float(np.std(innovations)),
            'max': float(np.max(innovations)),
            'count': len(innovations)
        }
    
    def reset(self):
        """Reset filter to uninitialized state"""
        self.state = np.zeros(4)
        self.P = np.eye(4) * 1000.0
        self.P[2, 2] = 1000.0
        self.P[3, 3] = 1000.0
        self.initialized = False
        self.last_update_time = None
        self.update_count = 0
        self.innovation_history.clear()
        self.logger.info("Kalman Filter reset")
    
    def adjust_noise_parameters(self, 
                              process_noise_pos: Optional[float] = None,
                              process_noise_vel: Optional[float] = None,
                              measurement_noise: Optional[float] = None):
        """
        Adjust noise parameters during operation
        
        Args:
            process_noise_pos: New position process noise
            process_noise_vel: New velocity process noise  
            measurement_noise: New measurement noise
        """
        if process_noise_pos is not None:
            self.Q[0, 0] = process_noise_pos**2
            self.Q[1, 1] = process_noise_pos**2
            
        if process_noise_vel is not None:
            self.Q[2, 2] = process_noise_vel**2
            self.Q[3, 3] = process_noise_vel**2
            
        if measurement_noise is not None:
            self.R = np.eye(2) * measurement_noise**2
            
        self.logger.info("Kalman Filter noise parameters updated")

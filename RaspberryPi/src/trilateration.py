#!/usr/bin/env python3
"""
Trilateration Module
Calculates position using UWB distance measurements from 3 anchors

Author: Sistema UWB Carrito de Golf  
Date: July 2025
"""

import math
import numpy as np
import logging
from typing import Tuple, Optional, List
from dataclasses import dataclass

@dataclass
class Position2D:
    """2D position with validity flag"""
    x: float  # X coordinate in mm
    y: float  # Y coordinate in mm
    valid: bool  # True if position calculation is valid
    error: float  # Estimation error/uncertainty

@dataclass
class AnchorPosition:
    """3D anchor position"""
    x: float  # X coordinate in mm
    y: float  # Y coordinate in mm
    z: float  # Z coordinate in mm
    id: str   # Anchor identifier

class Trilaterator:
    """Trilateration calculator for UWB positioning"""
    
    def __init__(self):
        """Initialize trilaterator with anchor positions"""
        self.logger = logging.getLogger(__name__)
        
        # Configure anchor positions (from ESP32 configuration)
        # Anchor 1: 280mm a la izquierda del centro
        # Anchor 2: 280mm a la derecha del centro  
        # Anchor 3: 165mm a la derecha y 285mm hacia atrás, 140mm más abajo
        self.anchors = [
            AnchorPosition(x=-280.0, y=0.0, z=0.0, id="anchor1"),    # Izquierda
            AnchorPosition(x=280.0, y=0.0, z=0.0, id="anchor2"),     # Derecha
            AnchorPosition(x=165.0, y=-285.0, z=-140.0, id="anchor3") # Atrás-derecha-abajo
        ]
        
        self.logger.info("Trilaterator initialized with 3 anchors")
        self.logger.info(f"Anchor 1: ({self.anchors[0].x}, {self.anchors[0].y}, {self.anchors[0].z}) mm")
        self.logger.info(f"Anchor 2: ({self.anchors[1].x}, {self.anchors[1].y}, {self.anchors[1].z}) mm")
        self.logger.info(f"Anchor 3: ({self.anchors[2].x}, {self.anchors[2].y}, {self.anchors[2].z}) mm")
    
    def calculate_position(self, distances: List[float], anchor_status: List[bool]) -> Position2D:
        """
        Calculate 2D position using trilateration
        
        Args:
            distances: List of 3 distances in cm [d1, d2, d3]
            anchor_status: List of 3 boolean status [s1, s2, s3]
            
        Returns:
            Position2D object with calculated position
        """
        try:
            # Validate inputs
            if not distances or not anchor_status:
                print("DEBUG: Empty distances or anchor_status")
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            if len(distances) != 3 or len(anchor_status) != 3:
                print(f"DEBUG: Invalid array lengths - distances: {len(distances)}, status: {len(anchor_status)}")
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            # Check for None values and replace with NaN
            clean_distances = []
            for i, d in enumerate(distances):
                if d is None:
                    print(f"DEBUG: Distance {i+1} is None, replacing with NaN")
                    clean_distances.append(float('nan'))
                else:
                    clean_distances.append(float(d))
            
            print(f"DEBUG: Clean distances: {clean_distances}, Status: {anchor_status}")
            
            # Convert distances from cm to mm
            distances_mm = [d * 10.0 if not math.isnan(d) else float('nan') for d in clean_distances]
            
            # Check if we have at least 2 valid measurements (minimum for 2D trilateration)
            valid_count = sum(anchor_status)
            print(f"DEBUG: Valid anchors count: {valid_count}")
            if valid_count < 2:
                print("DEBUG: Insufficient valid anchors (need at least 2)")
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            # Use 2D trilateration (assuming tag is at same Z level as anchors 1&2)
            return self._trilaterate_2d(distances_mm, anchor_status)
            
        except Exception as e:
            self.logger.error(f"Position calculation error: {e}")
            return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
    
    def _trilaterate_2d(self, distances: List[float], status: List[bool]) -> Position2D:
        """
        Perform 2D trilateration using least squares method
        
        Args:
            distances: Distances in mm
            status: Anchor status flags
            
        Returns:
            Position2D object
        """
        try:
            # Filter valid measurements
            valid_anchors = []
            valid_distances = []
            
            for i, (anchor, distance, is_valid) in enumerate(zip(self.anchors, distances, status)):
                if is_valid and not math.isnan(distance) and distance > 0:
                    valid_anchors.append(anchor)
                    valid_distances.append(distance)
            
            print(f"DEBUG: Found {len(valid_anchors)} valid anchors")
            
            if len(valid_anchors) < 2:
                print("DEBUG: Need at least 2 valid anchors for trilateration")
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            # Handle 2-anchor case (intersection of two circles)
            if len(valid_anchors) == 2:
                return self._trilaterate_2_anchors(valid_anchors, valid_distances)
            
            # Use first 3 valid anchors for full trilateration
            if len(valid_anchors) > 3:
                valid_anchors = valid_anchors[:3]
                valid_distances = valid_distances[:3]
            
            # Set up equations for least squares
            # (x - x1)² + (y - y1)² = r1²
            # (x - x2)² + (y - y2)² = r2²  
            # (x - x3)² + (y - y3)² = r3²
            
            x1, y1 = valid_anchors[0].x, valid_anchors[0].y
            x2, y2 = valid_anchors[1].x, valid_anchors[1].y
            x3, y3 = valid_anchors[2].x, valid_anchors[2].y
            
            r1, r2, r3 = valid_distances[0], valid_distances[1], valid_distances[2]
            
            # Convert to linear system using circle intersection method
            A = 2 * (x2 - x1)
            B = 2 * (y2 - y1)
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            
            D = 2 * (x3 - x2)
            E = 2 * (y3 - y2)
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            
            # Solve 2x2 system: Ax + By = C, Dx + Ey = F
            denominator = A * E - B * D
            
            if abs(denominator) < 1e-6:
                # Anchors are collinear, can't trilaterate
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            x = (C * E - F * B) / denominator
            y = (A * F - D * C) / denominator
            
            # Calculate estimation error (RMS of distance errors)
            error = self._calculate_position_error(x, y, valid_anchors, valid_distances)
            
            # Consider position valid if error is reasonable (< 50cm)
            valid = error < 500.0  # 50cm in mm
            
            return Position2D(x=x, y=y, valid=valid, error=error)
            
        except Exception as e:
            self.logger.error(f"2D trilateration error: {e}")
            return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
    
    def _trilaterate_2_anchors(self, anchors: List[AnchorPosition], distances: List[float]) -> Position2D:
        """
        Calculate position using 2 anchors (intersection of two circles)
        
        Args:
            anchors: List of 2 anchor positions
            distances: List of 2 distances in mm
            
        Returns:
            Position2D object
        """
        try:
            if len(anchors) != 2 or len(distances) != 2:
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            # Get anchor positions and distances
            x1, y1 = anchors[0].x, anchors[0].y
            x2, y2 = anchors[1].x, anchors[1].y
            r1, r2 = distances[0], distances[1]
            
            print(f"DEBUG: 2-anchor calculation: A1({x1},{y1}) r={r1:.1f}, A2({x2},{y2}) r={r2:.1f}")
            
            # Distance between anchors
            d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Check if circles intersect
            if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
                print(f"DEBUG: Circles don't intersect or are identical. d={d:.1f}, r1+r2={r1+r2:.1f}")
                return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
            
            # Calculate intersection points
            a = (r1**2 - r2**2 + d**2) / (2 * d)
            h = math.sqrt(r1**2 - a**2)
            
            # Point on line between anchors
            px = x1 + a * (x2 - x1) / d
            py = y1 + a * (y2 - y1) / d
            
            # Two intersection points
            x_pos1 = px + h * (y2 - y1) / d
            y_pos1 = py - h * (x2 - x1) / d
            
            x_pos2 = px - h * (y2 - y1) / d
            y_pos2 = py + h * (x2 - x1) / d
            
            # For now, choose the point closer to origin (you can improve this logic)
            dist1 = math.sqrt(x_pos1**2 + y_pos1**2)
            dist2 = math.sqrt(x_pos2**2 + y_pos2**2)
            
            if dist1 <= dist2:
                x_final, y_final = x_pos1, y_pos1
            else:
                x_final, y_final = x_pos2, y_pos2
            
            # Convert back to cm
            x_cm = x_final / 10.0
            y_cm = y_final / 10.0
            
            # Calculate error (simplified for 2 anchors)
            error = self._calculate_position_error(x_final, y_final, anchors, distances)
            
            print(f"DEBUG: 2-anchor result: ({x_cm:.1f}, {y_cm:.1f}) cm, error={error:.1f}")
            
            return Position2D(x=x_cm, y=y_cm, valid=True, error=error)
            
        except Exception as e:
            self.logger.error(f"2-anchor trilateration failed: {e}")
            return Position2D(x=0.0, y=0.0, valid=False, error=float('inf'))
    
    def _calculate_position_error(self, x: float, y: float, 
                                anchors: List[AnchorPosition], 
                                measured_distances: List[float]) -> float:
        """
        Calculate RMS error of position estimate
        
        Args:
            x, y: Calculated position
            anchors: List of anchor positions used
            measured_distances: List of measured distances
            
        Returns:
            RMS error in mm
        """
        try:
            squared_errors = []
            
            for anchor, measured_dist in zip(anchors, measured_distances):
                # Calculate expected distance
                expected_dist = math.sqrt((x - anchor.x)**2 + (y - anchor.y)**2)
                
                # Calculate error
                error = abs(expected_dist - measured_dist)
                squared_errors.append(error**2)
            
            # Return RMS error
            return math.sqrt(sum(squared_errors) / len(squared_errors))
            
        except Exception as e:
            self.logger.error(f"Error calculation failed: {e}")
            return float('inf')
    
    def get_anchor_positions(self) -> List[AnchorPosition]:
        """Get list of configured anchor positions"""
        return self.anchors.copy()
    
    def update_anchor_position(self, anchor_id: int, x: float, y: float, z: float):
        """
        Update anchor position
        
        Args:
            anchor_id: Anchor index (0, 1, or 2)
            x, y, z: New position coordinates in mm
        """
        if 0 <= anchor_id < len(self.anchors):
            self.anchors[anchor_id].x = x
            self.anchors[anchor_id].y = y
            self.anchors[anchor_id].z = z
            self.logger.info(f"Updated anchor {anchor_id} position to ({x}, {y}, {z}) mm")

#!/usr/bin/env python3
"""
Kalman Filter Test and Calibration Tool
Test the Kalman filter with synthetic or real UWB data

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import time
import logging
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Tuple

# Add src directory to Python path
current_dir = Path(__file__).parent
parent_dir = current_dir.parent
src_dir = parent_dir / 'src'
sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(parent_dir))

from src.kalman_filter import KalmanFilter2D, FilteredPosition

def setup_logging():
    """Setup logging for test"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

def generate_synthetic_trajectory(duration: float = 10.0, 
                                 frequency: float = 10.0,
                                 noise_level: float = 50.0) -> List[Tuple[float, float, float]]:
    """
    Generate synthetic trajectory data for testing
    
    Args:
        duration: Duration of trajectory in seconds
        frequency: Data frequency in Hz
        noise_level: Noise level in mm
        
    Returns:
        List of (time, x, y) tuples
    """
    t_points = np.arange(0, duration, 1.0/frequency)
    trajectory = []
    
    for t in t_points:
        # Create a figure-8 pattern for testing
        x_true = 500 * np.sin(t * 0.5)
        y_true = 250 * np.sin(t * 1.0)
        
        # Add noise
        x_noisy = x_true + np.random.normal(0, noise_level)
        y_noisy = y_true + np.random.normal(0, noise_level)
        
        trajectory.append((t, x_noisy, y_noisy))
    
    return trajectory

def test_kalman_filter_synthetic():
    """Test Kalman filter with synthetic data"""
    logger = logging.getLogger(__name__)
    logger.info("Testing Kalman Filter with synthetic data...")
    
    # Generate synthetic trajectory
    trajectory = generate_synthetic_trajectory(duration=20.0, frequency=20.0, noise_level=80.0)
    
    # Initialize filter
    kalman = KalmanFilter2D(
        process_noise_pos=30.0,    # Lower since synthetic data is predictable
        process_noise_vel=50.0,    # Moderate velocity noise
        measurement_noise=80.0,    # Match noise level
        initial_uncertainty=500.0  # Lower initial uncertainty
    )
    
    # Process trajectory
    times = []
    raw_positions = []
    filtered_positions = []
    velocities = []
    confidences = []
    innovations = []
    
    for i, (t, x, y) in enumerate(trajectory):
        # Simulate real-time processing
        if i > 0:
            dt = t - trajectory[i-1][0]
            time.sleep(max(0, dt * 0.1))  # Speed up 10x for testing
        
        # Process measurement
        filtered_pos = kalman.process_measurement(x, y)
        
        # Store results
        times.append(t)
        raw_positions.append((x, y))
        filtered_positions.append((filtered_pos.x, filtered_pos.y))
        velocities.append((filtered_pos.vx, filtered_pos.vy))
        confidences.append(filtered_pos.confidence)
        innovations.append(filtered_pos.innovation)
        
        if i % 50 == 0:
            logger.info(f"t={t:.1f}s: Raw=({x:.1f},{y:.1f}) "
                       f"Filtered=({filtered_pos.x:.1f},{filtered_pos.y:.1f}) "
                       f"Confidence={filtered_pos.confidence:.2f}")
    
    # Plot results
    plot_test_results(times, raw_positions, filtered_positions, 
                     velocities, confidences, innovations)
    
    # Calculate statistics
    calculate_filter_statistics(raw_positions, filtered_positions, velocities)

def plot_test_results(times: List[float], 
                     raw_positions: List[Tuple[float, float]],
                     filtered_positions: List[Tuple[float, float]],
                     velocities: List[Tuple[float, float]],
                     confidences: List[float],
                     innovations: List[float]):
    """Plot test results"""
    
    # Convert to numpy arrays for easier plotting
    times_array = np.array(times)
    raw_x = np.array([pos[0] for pos in raw_positions])
    raw_y = np.array([pos[1] for pos in raw_positions])
    filt_x = np.array([pos[0] for pos in filtered_positions])
    filt_y = np.array([pos[1] for pos in filtered_positions])
    vel_x = np.array([vel[0] for vel in velocities])
    vel_y = np.array([vel[1] for vel in velocities])
    
    # Create subplots
    fig = plt.figure(figsize=(15, 10))
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)
    
    # Trajectory plot
    ax1.plot(raw_x, raw_y, 'r.', alpha=0.5, label='Raw UWB', markersize=2)
    ax1.plot(filt_x, filt_y, 'b-', linewidth=2, label='Kalman Filtered')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Position vs time
    ax2.plot(times_array, raw_x, 'r.', alpha=0.5, label='Raw X', markersize=2)
    ax2.plot(times_array, filt_x, 'b-', linewidth=2, label='Filtered X')
    ax2.plot(times_array, raw_y, 'r.', alpha=0.5, label='Raw Y', markersize=2)
    ax2.plot(times_array, filt_y, 'g-', linewidth=2, label='Filtered Y')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (mm)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # Velocity
    speed = np.sqrt(vel_x**2 + vel_y**2)
    ax3.plot(times_array, vel_x, 'b-', label='Velocity X')
    ax3.plot(times_array, vel_y, 'g-', label='Velocity Y')
    ax3.plot(times_array, speed, 'r-', label='Speed')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (mm/s)')
    ax3.set_title('Estimated Velocity')
    ax3.legend()
    ax3.grid(True)
    
    # Filter metrics - use single axis with scaled innovation
    confidences_array = np.array(confidences)
    innovations_array = np.array(innovations)
    
    # Scale innovation to 0-1 range for display alongside confidence
    scaled_innovations = innovations_array / 200.0  # Scale down by 200mm
    
    ax4.plot(times_array, confidences_array, 'b-', label='Confidence', linewidth=2)
    ax4.plot(times_array, scaled_innovations, 'r-', label='Innovation (scaled)', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Value (0-1)')
    ax4.set_title('Filter Metrics')
    ax4.set_ylim(0, 1)
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    plt.savefig(current_dir / 'kalman_test_results.png', dpi=300, bbox_inches='tight')
    plt.show()

def calculate_filter_statistics(raw_positions: List[Tuple[float, float]],
                              filtered_positions: List[Tuple[float, float]],
                              velocities: List[Tuple[float, float]]):
    """Calculate and print filter performance statistics"""
    logger = logging.getLogger(__name__)
    
    # Convert to numpy arrays
    raw_pos = np.array(raw_positions)
    filt_pos = np.array(filtered_positions)
    
    # Calculate noise reduction
    raw_noise = np.std(raw_pos, axis=0)
    filt_noise = np.std(filt_pos, axis=0)
    noise_reduction = (raw_noise - filt_noise) / raw_noise * 100
    
    # Calculate smoothness (derivative variance)
    if len(filt_pos) > 1:
        raw_diff = np.diff(raw_pos, axis=0)
        filt_diff = np.diff(filt_pos, axis=0)
        raw_smoothness = np.var(raw_diff, axis=0)
        filt_smoothness = np.var(filt_diff, axis=0)
        smoothness_improvement = (raw_smoothness - filt_smoothness) / raw_smoothness * 100
    else:
        smoothness_improvement = [0, 0]
    
    # Velocity statistics
    velocities_array = np.array(velocities)
    avg_speed = np.mean(np.sqrt(velocities_array[:, 0]**2 + velocities_array[:, 1]**2))
    max_speed = np.max(np.sqrt(velocities_array[:, 0]**2 + velocities_array[:, 1]**2))
    
    logger.info("=== Kalman Filter Performance Statistics ===")
    logger.info(f"Noise Reduction X: {noise_reduction[0]:.1f}%")
    logger.info(f"Noise Reduction Y: {noise_reduction[1]:.1f}%")
    logger.info(f"Smoothness Improvement X: {smoothness_improvement[0]:.1f}%")
    logger.info(f"Smoothness Improvement Y: {smoothness_improvement[1]:.1f}%")
    logger.info(f"Average Speed: {avg_speed:.1f} mm/s")
    logger.info(f"Maximum Speed: {max_speed:.1f} mm/s")

def test_filter_parameters():
    """Test different filter parameters to find optimal settings"""
    logger = logging.getLogger(__name__)
    logger.info("Testing different filter parameters...")
    
    # Generate consistent test data
    np.random.seed(42)  # For reproducible results
    trajectory = generate_synthetic_trajectory(duration=10.0, frequency=20.0, noise_level=100.0)
    
    # Parameter sets to test
    parameter_sets = [
        {"name": "Conservative", "process_pos": 20.0, "process_vel": 50.0, "measurement": 100.0},
        {"name": "Moderate", "process_pos": 50.0, "process_vel": 100.0, "measurement": 100.0},
        {"name": "Aggressive", "process_pos": 100.0, "process_vel": 200.0, "measurement": 100.0},
        {"name": "Low Noise", "process_pos": 30.0, "process_vel": 60.0, "measurement": 50.0},
        {"name": "High Noise", "process_pos": 80.0, "process_vel": 150.0, "measurement": 200.0},
    ]
    
    results = []
    
    for params in parameter_sets:
        kalman = KalmanFilter2D(
            process_noise_pos=params["process_pos"],
            process_noise_vel=params["process_vel"],
            measurement_noise=params["measurement"],
            initial_uncertainty=500.0
        )
        
        filtered_positions = []
        confidences = []
        
        for t, x, y in trajectory:
            filtered_pos = kalman.process_measurement(x, y)
            filtered_positions.append((filtered_pos.x, filtered_pos.y))
            confidences.append(filtered_pos.confidence)
        
        # Calculate performance metrics
        raw_pos = np.array([(x, y) for _, x, y in trajectory])
        filt_pos = np.array(filtered_positions)
        
        # Smoothness metric (lower is better)
        if len(filt_pos) > 1:
            smoothness = np.mean(np.var(np.diff(filt_pos, axis=0), axis=0))
        else:
            smoothness = float('inf')
        
        # Average confidence (higher is better)
        avg_confidence = np.mean(confidences)
        
        # Response time (how quickly filter reaches 80% confidence)
        response_time = None
        for i, conf in enumerate(confidences):
            if conf > 0.8:
                response_time = i * 0.05  # 20Hz = 0.05s per sample
                break
        
        results.append({
            "name": params["name"],
            "smoothness": smoothness,
            "avg_confidence": avg_confidence,
            "response_time": response_time,
            "params": params
        })
        
        logger.info(f"{params['name']}: Smoothness={smoothness:.1f}, "
                   f"Confidence={avg_confidence:.2f}, "
                   f"Response={response_time:.2f}s" if response_time else "Response=N/A")
    
    # Find best parameter set
    best = min(results, key=lambda x: x["smoothness"] if x["response_time"] and x["response_time"] < 2.0 else float('inf'))
    logger.info(f"\nRecommended parameters: {best['name']}")
    logger.info(f"Parameters: {best['params']}")

def main():
    """Main test function"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("Kalman Filter Test Suite Starting...")
    
    try:
        # Test with synthetic data
        test_kalman_filter_synthetic()
        
        # Test different parameters
        test_filter_parameters()
        
        logger.info("Kalman Filter tests completed successfully!")
        
    except Exception as e:
        logger.error(f"Test failed: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

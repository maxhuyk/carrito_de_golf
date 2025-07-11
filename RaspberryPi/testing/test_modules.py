#!/usr/bin/env python3
"""
Quick test for the corrected modules without GUI
"""

import sys
import time
from pathlib import Path

# Add src directory to Python path
current_dir = Path(__file__).parent
parent_dir = current_dir.parent
src_dir = parent_dir / 'src'
sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(parent_dir))

def test_imports():
    """Test that all imports work correctly"""
    print("Testing imports...")
    
    try:
        from src.kalman_filter import KalmanFilter2D, FilteredPosition
        print("✓ Kalman filter imported successfully")
        
        from src.trilateration import Trilaterator, Position2D
        print("✓ Trilateration imported successfully")
        
        from src.uart_comm import UARTCommunicator, SystemData
        print("✓ UART communication imported successfully")
        
        # Try importing the main controller
        from main import GolfCartController
        print("✓ Main controller imported successfully")
        
        print("\nAll imports successful!")
        return True
        
    except Exception as e:
        print(f"✗ Import error: {e}")
        return False

def test_kalman_basic():
    """Test basic Kalman filter functionality"""
    print("\nTesting basic Kalman filter...")
    
    try:
        from src.kalman_filter import KalmanFilter2D
        
        # Initialize filter
        kalman = KalmanFilter2D(
            process_noise_pos=50.0,
            process_noise_vel=100.0,
            measurement_noise=100.0,
            initial_uncertainty=1000.0
        )
        
        # Test a few measurements
        measurements = [
            (100.0, 200.0),
            (110.0, 210.0),
            (120.0, 220.0),
            (130.0, 230.0)
        ]
        
        print("Processing test measurements...")
        for i, (x, y) in enumerate(measurements):
            result = kalman.process_measurement(x, y)
            print(f"  Measurement {i+1}: ({x:.1f}, {y:.1f}) -> "
                  f"Filtered: ({result.x:.1f}, {result.y:.1f}), "
                  f"Velocity: ({result.vx:.1f}, {result.vy:.1f}), "
                  f"Confidence: {result.confidence:.2f}")
            
            # Small delay to simulate real-time
            time.sleep(0.1)
        
        print("✓ Kalman filter test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Kalman filter test failed: {e}")
        return False

def test_main_controller():
    """Test main controller initialization"""
    print("\nTesting main controller initialization...")
    
    try:
        from main import GolfCartController
        
        # Create controller (this will test config loading and component initialization)
        controller = GolfCartController()
        
        # Test status method
        status = controller.get_status()
        print(f"✓ Controller status: {len(status)} fields")
        
        # Test filter adjustment
        controller.kalman_filter.adjust_noise_parameters(
            process_noise_pos=60.0,
            measurement_noise=120.0
        )
        print("✓ Filter parameters adjusted successfully")
        
        print("✓ Main controller test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Main controller test failed: {e}")
        return False

def main():
    """Main test function"""
    print("=== Golf Cart UWB System Module Test ===")
    print("Testing corrected modules...\n")
    
    tests = [
        test_imports,
        test_kalman_basic,
        test_main_controller
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print(f"=== Test Results: {passed}/{total} passed ===")
    
    if passed == total:
        print("🎉 All tests passed! The modules are working correctly.")
        return 0
    else:
        print("❌ Some tests failed. Check the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

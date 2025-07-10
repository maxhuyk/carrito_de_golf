#!/usr/bin/env python3
"""
Golf Cart GUI Launcher
Simple launcher script for the golf cart control interface

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import sys
import os
from pathlib import Path

def check_dependencies():
    """Check if all required dependencies are available"""
    try:
        import tkinter
        print("✓ tkinter available")
    except ImportError:
        print("✗ tkinter not available. Install with: sudo apt-get install python3-tk")
        return False
    
    try:
        import serial
        print("✓ pyserial available")
    except ImportError:
        print("✗ pyserial not available. Install with: pip install pyserial")
        return False
    
    try:
        import numpy
        print("✓ numpy available")
    except ImportError:
        print("✗ numpy not available. Install with: pip install numpy")
        return False
    
    return True

def main():
    """Main launcher function"""
    print("Golf Cart UWB Control System - GUI Launcher")
    print("=" * 50)
    
    # Check dependencies
    print("Checking dependencies...")
    if not check_dependencies():
        print("\nSome dependencies are missing. Please install them and try again.")
        return 1
    
    print("\nAll dependencies available!")
    print("Starting Golf Cart GUI...")
    
    # Import and start the GUI
    try:
        from golf_cart_gui import GolfCartGUI
        
        app = GolfCartGUI()
        app.run()
        
    except ImportError as e:
        print(f"Error importing GUI modules: {e}")
        print("Make sure you're running from the correct directory.")
        return 1
    except Exception as e:
        print(f"Error starting GUI: {e}")
        return 1
    
    print("GUI closed successfully.")
    return 0

if __name__ == "__main__":
    sys.exit(main())

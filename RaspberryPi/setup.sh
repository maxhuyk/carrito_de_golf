#!/bin/bash
# Setup script for Golf Cart UWB Control System
# Raspberry Pi setup and configuration

set -e  # Exit on any error

echo "=================================="
echo "Golf Cart UWB System Setup"
echo "=================================="

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "Warning: This script is designed for Raspberry Pi"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
echo "Installing Python dependencies..."
sudo apt install -y python3-pip python3-venv python3-dev python3-tk

# Create virtual environment
echo "Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install Python packages
echo "Installing required Python packages..."
pip install --upgrade pip
pip install -r requirements.txt

# Configure UART
echo "Configuring UART..."
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
    echo "UART enabled in /boot/config.txt"
else
    echo "UART already enabled"
fi

# Disable Bluetooth to free up UART (optional)
read -p "Disable Bluetooth to use UART0 for ESP32? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if ! grep -q "dtoverlay=disable-bt" /boot/config.txt; then
        echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
        echo "Bluetooth disabled"
    fi
    sudo systemctl disable hciuart
fi

# Configure serial console (disable it for UART communication)
sudo systemctl disable serial-getty@ttyS0.service 2>/dev/null || true

# Set up permissions for UART
sudo usermod -a -G dialout $USER

# Create log directory
mkdir -p logs

# Create startup script
cat > start_golf_cart.sh << 'EOF'
#!/bin/bash
# Startup script for Golf Cart UWB System

cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

# Start the main system
echo "Starting Golf Cart UWB Control System..."
python3 main.py

EOF

chmod +x start_golf_cart.sh

# Create GUI launcher script
cat > start_gui.sh << 'EOF'
#!/bin/bash
# GUI launcher script

cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

# Check if display is available
if [ -z "$DISPLAY" ]; then
    echo "Warning: No display detected."
    echo "For GUI operation, you need:"
    echo "1. Desktop environment (LXDE, XFCE, etc.)"
    echo "2. VNC server enabled, or"
    echo "3. SSH with X11 forwarding: ssh -X user@raspberry_pi"
    exit 1
fi

# Start GUI
echo "Starting Golf Cart Control GUI..."
python3 start_gui.py

EOF

chmod +x start_gui.sh
cat > start_monitor.sh << 'EOF'
#!/bin/bash
# Real-time monitoring script

cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

# Check if display is available
if [ -z "$DISPLAY" ]; then
    echo "Warning: No display detected. Monitor requires GUI."
    echo "Run this script in a desktop environment or via VNC/SSH with X11 forwarding."
    exit 1
fi

# Start real-time monitor
echo "Starting Real-time UWB Monitor..."
python3 monitor_realtime.py

EOF

chmod +x start_monitor.sh

# Create test script
cat > run_tests.sh << 'EOF'
#!/bin/bash
# Test script for Golf Cart UWB System

cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

echo "Running UART communication test..."
python3 test_uart.py

echo ""
echo "Running Kalman filter test..."
python3 test_kalman.py

EOF

chmod +x run_tests.sh

# Create systemd service (optional)
read -p "Create systemd service for auto-start? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cat > golf-cart.service << EOF
[Unit]
Description=Golf Cart UWB Control System
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/start_golf_cart.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo mv golf-cart.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable golf-cart.service
    
    echo "Systemd service created and enabled"
    echo "Control with: sudo systemctl {start|stop|status} golf-cart"
fi

echo ""
echo "=================================="
echo "Setup Complete!"
echo "=================================="
echo ""
echo "Available scripts:"
echo "  ./start_golf_cart.sh  - Start main control system"
echo "  ./start_gui.sh        - Start graphical interface"
echo "  ./start_monitor.sh    - Start real-time monitor (requires GUI)"
echo "  ./run_tests.sh        - Run system tests"
echo ""
echo "Demo and testing:"
echo "  python3 demo_gui.py   - Test GUI with synthetic data"
echo "  python3 test_kalman.py - Test Kalman filter"
echo ""
echo "Configuration:"
echo "  Edit config.json to adjust system parameters"
echo "  Check logs/ directory for system logs"
echo ""
echo "Next steps:"
echo "1. Reboot system to apply UART changes: sudo reboot"
echo "2. Connect ESP32 to UART pins (GPIO 14/15)"
echo "3. Run tests: ./run_tests.sh"
echo "4. Start system: ./start_golf_cart.sh"
echo ""
echo "For real-time monitoring:"
echo "  Enable VNC or use SSH with X11 forwarding"
echo "  Run: ./start_monitor.sh"
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "System service installed. Reboot and it will start automatically."
fi

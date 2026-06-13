#!/bin/bash
set -e

echo "=========================================="
echo "  Surveillance System C++ Installer"
echo "=========================================="

# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y \
    build-essential \
    cmake \
    libopencv-dev \
    libgpiod-dev \
    libv4l-dev \
    v4l-utils \
    git

# Create directories
mkdir -p /home/pi/surveillance/{recordings,logs,build}

# Build
cd /home/pi/surveillance/build
cmake ..
make -j$(nproc)

# Install binary
sudo make install

# Create systemd service
sudo tee /etc/systemd/system/surveillance.service > /dev/null <<EOF
[Unit]
Description=High Performance Surveillance System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/surveillance
ExecStart=/usr/local/bin/surveillance
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable surveillance.service
sudo systemctl start surveillance.service

echo "=========================================="
echo "  Installation Complete!"
echo "=========================================="
echo "Stream: http://$(hostname -I | awk '{print $1}'):8080"
echo "Logs:   sudo journalctl -u surveillance -f"
echo "Config: Edit /home/pi/surveillance/config.hpp"
echo "=========================================="

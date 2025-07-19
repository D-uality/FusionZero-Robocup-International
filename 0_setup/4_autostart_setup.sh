#!/bin/bash

SERVICE_NAME="robocup"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "Setting up Robocup autostart service..."

# Get current user and home directory
CURRENT_USER=$(whoami)
HOME_DIR=$(eval echo ~$CURRENT_USER)

# Create the service file
echo "Creating systemd service file for user: $CURRENT_USER..."
sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robocup Autostart
After=network.target

[Service]
Type=simple
User=$CURRENT_USER
WorkingDirectory=$HOME_DIR/FusionZero-Robocup-International/1_international
ExecStart=/usr/bin/python $HOME_DIR/FusionZero-Robocup-International/1_international_tech/main.py
/home/aidan/FusionZero-Robocup-International/

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd daemon
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable the service (start on boot)
echo "Enabling service for autostart..."
sudo systemctl enable "$SERVICE_NAME.service"

# Start the service immediately
echo "Starting service..."
sudo systemctl start "$SERVICE_NAME.service"

# Check service status
echo "Service status:"
sudo systemctl status "$SERVICE_NAME.service" --no-pager
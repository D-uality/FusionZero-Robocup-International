#!/usr/bin/env bash
set -euo pipefail

# Ensure we’re running as root
if [[ $EUID -ne 0 ]]; then
  echo "❌ This script must be run as root." >&2
  exit 1
fi

# 1. Create the ethernet connection
nmcli connection add \
  type ethernet \
  ifname usb0 \
  con-name usb0-connection \
  ip4 192.168.7.2/24 \
  gw4 192.168.7.1

# 2. Write the systemd service unit
cat > /etc/systemd/system/usb0-autoconnect.service << 'EOF'
[Unit]
Description=Force NetworkManager to bring up usb0
After=network.target NetworkManager.service
Requires=NetworkManager.service

[Service]
ExecStart=/usr/bin/nmcli connection up usb0-connection
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# 3. Reload systemd, enable & start the service
systemctl daemon-reload
systemctl enable usb0-autoconnect.service
systemctl start usb0-autoconnect.service

echo "✅ usb0-connection created and usb0-autoconnect.service enabled & started."
echo "🔄 Rebooting now to apply changes..."

reboot
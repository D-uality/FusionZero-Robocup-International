#!/bin/bash
set -e

echo "Enabling USB Ethernet gadget mode for Pi 4B..."

# Ensure script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "Please run as root: sudo $0"
   exit 1
fi

# 1. Modify /boot/config.txt
CONFIG=/boot/config.txt

# USB-gadget overlay
if ! grep -q "^dtoverlay=dwc2" "$CONFIG"; then
  echo "dtoverlay=dwc2,dr_mode=peripheral" >> "$CONFIG"
  echo "Added dwc2 overlay to config.txt"
fi

# I2C 400 kHz bus speed
if ! grep -q "^dtparam=i2c_arm_baudrate=400000" "$CONFIG"; then
  echo "dtparam=i2c_arm_baudrate=400000" >> "$CONFIG"
  echo "Set I2C baudrate to 400 kHz"
fi

# 2. Modify /boot/cmdline.txt (single line!)
CMDLINE=/boot/cmdline.txt
if ! grep -q "modules-load=dwc2,g_ether" "$CMDLINE"; then
  sed -i 's/rootwait/rootwait modules-load=dwc2,g_ether/' "$CMDLINE"
  echo "Updated cmdline.txt with g_ether"
fi

# 3. Create systemd service for static IP
SERVICE=/etc/systemd/system/usb0-static.service
cat << 'EOF' > "$SERVICE"
[Unit]
Description=Configure usb0 with static IP
After=network.target

[Service]
ExecStart=/sbin/ip link set usb0 up
ExecStartPost=/sbin/ip addr add 192.168.7.2/24 dev usb0
Type=oneshot
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE"
systemctl enable usb0-static.service

# 4. Optionally install ssh and avahi
echo "Enabling SSH and .local access..."
systemctl enable ssh
systemctl start ssh
apt-get update
apt-get install -y avahi-daemon

echo
echo "All done! Next steps:"
echo "1) Reboot the Pi"
echo "2) Connect USB-C to your host (use a data-capable cable)"
echo "3) On your host: set IP to 192.168.7.1/24 and ping 192.168.7.2"
echo "4) On your host: set subnet mask to 255.255.255.0"

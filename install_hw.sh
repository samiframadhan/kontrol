#!/bin/bash

# This script creates a udev rule to give persistent names to serial devices.
# It needs to be run with sudo privileges.

# 1. Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "❌ Please run this script with sudo."
  exit 1
fi

# 2. Define the path for the new udev rules file
RULES_FILE="/etc/udev/rules.d/99-persistent-serial.rules"

echo "⚙️  Creating udev rules file at $RULES_FILE..."

# 3. Create the rules file with a heredoc
cat > $RULES_FILE << EOF
# Rule for FTDI device (HMI)
# Vendor: 0403, Product: 6001
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666", SYMLINK+="hmi"

# --- Lidar Devices (CP210x) ---
# You must find the unique serial number for each device and fill it in.
# Plug in ONE lidar, find its name (e.g., /dev/ttyUSB0), and run:
# udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'
# Then, uncomment the line and replace YOUR_SERIAL_HERE with the number.
#
# Vendor: 10c4, Product: ea60
#SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_LIDAR1_SERIAL_HERE", SYMLINK+="lidar1"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_LIDAR2_SERIAL_HERE", SYMLINK+="lidar2"

# --- ST-Link Device ---
# Like the lidars, find its unique serial number.
# Plug in the ST-Link, find its name (e.g., /dev/ttyACM0), and run:
# udevadm info -a -n /dev/ttyACM0 | grep '{serial}'
#
# Vendor: 0483, Product: 3748 (ST-Link V2) or 374b (V2-1)
#SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", ATTRS{serial}=="YOUR_STLINK_SERIAL_HERE", SYMLINK+="stlink"
EOF

# 4. Reload udev rules to apply the changes
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "✅ Success! The rules file has been created."
echo "💡 IMPORTANT: You must now edit '$RULES_FILE' to add the serial numbers for your lidars and ST-Link."
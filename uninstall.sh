#!/bin/bash

# --- Configuration ---
# This list must match the one in the installation script.
PYTHON_NODES=(
    "aruco.py"
    "camera.py"
    "camera_reverse.py"
    "control.py"
    "hmi_node.py"
    "linefollowing_rs.py"
    "linefollowing_rs_reverse.py"
    "llc_interface.py"
    "orchestrator.py"
)
VENV_NAME="venv"
SERVICE_FILES_DIR="/etc/systemd/system"

# --- 0. Check for Sudo/Root Privileges ---
echo "--- Checking for sudo permissions ---"
if [[ $EUID -ne 0 ]]; then
   echo "🚫 This script must be run with sudo to remove system-level services."
   echo "Please run it like this: sudo ./uninstall_system.sh"
   exit 1
fi
echo "✅ Sudo permissions confirmed."
echo

# --- 1. Stop and Disable Systemd Services ---
echo "--- Stopping and disabling system services... ---"
for node_script in "${PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    service_file_path="$SERVICE_FILES_DIR/$service_name"

    # Check if the service file exists before trying to stop/disable it
    if [ -f "$service_file_path" ]; then
        echo "Processing $service_name..."
        # Stop the service if it's running (ignore errors if not)
        systemctl stop "$service_name" >/dev/null 2>&1 || true
        # Disable the service to prevent it from starting on boot (ignore errors if not enabled)
        systemctl disable "$service_name" >/dev/null 2>&1 || true
    else
        echo "Service file for $service_name not found, skipping."
    fi
done
echo "✅ Services stopped and disabled."
echo

# --- 2. Remove Systemd Service Files ---
echo "--- Removing service files... ---"
for node_script in "${PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    service_file_path="$SERVICE_FILES_DIR/$service_name"

    if [ -f "$service_file_path" ]; then
        echo "Deleting $service_file_path"
        rm -f "$service_file_path"
    fi
done
echo "✅ Service files removed."
echo

# --- 3. Reload Systemd Daemon ---
echo "--- Reloading systemd daemon... ---"
systemctl daemon-reload
echo "✅ Daemon reloaded."
echo

# --- 4. Optionally Remove Virtual Environment ---
# This part is run as the original user to avoid permission issues
# on the directory later.
if [ -d "$VENV_NAME" ]; then
    echo "--- Optional: Remove Virtual Environment ---"
    # Temporarily drop sudo privileges to ask the user for input
    sudo -u "${SUDO_USER:-$(whoami)}" bash -c "read -p \"Do you want to permanently delete the '$VENV_NAME' directory? (y/N) \" -n 1 -r; echo; if [[ \$REPLY =~ ^[Yy]$ ]]; then exit 0; else exit 1; fi"

    # The exit code of the above command determines if we proceed
    if [ $? -eq 0 ]; then
        echo "Deleting virtual environment directory: $VENV_NAME"
        rm -rf "$VENV_NAME"
        echo "✅ Virtual environment removed."
    else
        echo "Skipping removal of '$VENV_NAME' directory."
    fi
fi
echo

echo "--- 🎉 Uninstallation Complete! ---"

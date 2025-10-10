#!/bin/bash

# --- Configuration ---
# This configuration MUST match install.sh to ensure the correct services are removed.
VENV_NAME=".venv"

# List of standard nodes from the installer
# Paths are relative to the project directory (where install.sh is)
STANDARD_PYTHON_NODES=(
    "nodes/control_node.py"
    "nodes/hmi_node.py"
    "nodes/imageprocess_node.py"
    "nodes/llc_interface.py"
    "src/orchestrator.py"
)

# --- Boilerplate ---
SERVICE_FILES_DIR="/etc/systemd/system"

# --- 0. Check for Sudo/Root Privileges ---
echo "--- Checking for sudo permissions ---"
if [[ $EUID -ne 0 ]]; then
   echo "🚫 This script must be run with sudo to remove system-level services."
   echo "Please run it like this: sudo ./uninstall.sh"
   exit 1
fi
echo "✅ Sudo permissions confirmed."
echo

# --- Generate the list of all services to be managed ---
echo "--- Determining which services to manage based on install script logic ---"
ALL_SERVICES=()

# Generate service names for standard nodes
for node_script in "${STANDARD_PYTHON_NODES[@]}"; do
    # Extract just the filename to create the correct service name
    ALL_SERVICES+=("$(basename "${node_script%.py}").service")
done

echo "The following services will be uninstalled:"
printf "  - %s\n" "${ALL_SERVICES[@]}"
echo

# --- 1. Stop and Disable Systemd Services ---
echo "--- Stopping and disabling system services... ---"
for service_name in "${ALL_SERVICES[@]}"; do
    service_file_path="$SERVICE_FILES_DIR/$service_name"

    if [ -f "$service_file_path" ]; then
        echo "Processing $service_name..."
        systemctl stop "$service_name" >/dev/null 2>&1 || true
        systemctl disable "$service_name" >/dev/null 2>&1 || true
    else
        echo "Service file for $service_name not found, skipping."
    fi
done
echo "✅ Services stopped and disabled."
echo

# --- 2. Remove Systemd Service Files ---
echo "--- Removing service files... ---"
for service_name in "${ALL_SERVICES[@]}"; do
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
if [ -d "$VENV_NAME" ]; then
    echo "--- Optional: Remove Virtual Environment ---"
    # Run the prompt as the original user to avoid permission issues with read
    sudo -u "${SUDO_USER:-$(whoami)}" bash -c "read -p \"Do you want to permanently delete the '$VENV_NAME' directory? (y/N) \" -n 1 -r; echo; if [[ \$REPLY =~ ^[Yy]$ ]]; then exit 0; else exit 1; fi"
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
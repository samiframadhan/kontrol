#!/bin/bash

# --- Configuration ---
VENV_NAME=".venv"
PYTHON_NODES=(
    "aruco.py"
    "camera.py"
    "control.py"
    "hmi_node.py"
    "linefollowing_rs.py"
    "linefollowing_rs_reverse.py"
    "llc_interface.py"
    "orchestrator.py"
)
# System services are stored here
SERVICE_FILES_DIR="/etc/systemd/system"
PROJECT_DIR=$(pwd)

# --- 0. Check for Sudo/Root Privileges ---
echo "--- Checking for sudo permissions ---"
if [[ $EUID -ne 0 ]]; then
   echo "🚫 This script must be run with sudo to install services that start on boot."
   echo "Please run it like this: sudo ./install.sh"
   exit 1
fi
echo "✅ Sudo permissions confirmed."
echo

# Determine the user who is running the script with sudo
RUN_AS_USER=${SUDO_USER:-$(whoami)}
RUN_AS_GROUP=$(id -gn "$RUN_AS_USER")
echo "Script will install services to run as user: $RUN_AS_USER"
echo

# --- 1. Create Virtual Environment ---
echo "--- Checking for virtual environment: $VENV_NAME ---"
if [ -d "$VENV_NAME" ]; then
    echo "✅ Virtual environment '$VENV_NAME' already exists. Skipping creation."
else
    echo "--- Creating virtual environment: $VENV_NAME ---"
    # Create the venv as the original user, not as root
    sudo -u "$RUN_AS_USER" python3 -m venv "$VENV_NAME" --system-site-packages
    echo "✅ Virtual environment created."
fi
# Ensure correct ownership of the venv directory
chown -R "$RUN_AS_USER":"$RUN_AS_GROUP" "$VENV_NAME"
echo

# --- 2. Install Requirements ---
echo "--- Installing required modules from requirements.txt ---"
# Activate the venv to install packages
source "$VENV_NAME/bin/activate"
pip install -r requirements.txt
deactivate
echo "✅ Requirements installed."
echo

# --- 3. Create and Enable Systemd Services ---
echo "--- Creating and enabling systemd services ---"

for node_script in "${PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    service_file_path="$SERVICE_FILES_DIR/$service_name"
    venv_python_path="$PROJECT_DIR/$VENV_NAME/bin/python"
    script_path="$PROJECT_DIR/$node_script"

    echo "Creating service file: $service_file_path"

    # Create the service file using tee to write with sudo
    tee "$service_file_path" > /dev/null << EOL
[Unit]
Description=Service for ${node_script}
# Start after the network is available
After=network.target

[Service]
# Run the script as the user who invoked sudo
User=$RUN_AS_USER
Group=$RUN_AS_GROUP

# Set the working directory to the project folder
WorkingDirectory=$PROJECT_DIR

# Command to execute the python script using the virtual environment
ExecStart=$venv_python_path $script_path

# Restart the service automatically if it fails
Restart=on-failure
RestartSec=5

[Install]
# Make the service start at boot in a multi-user environment
WantedBy=multi-user.target
EOL

done

echo
echo "Reloading systemd daemon and enabling services..."
# Reload systemd to recognize the new services
systemctl daemon-reload

# Enable each service to start on boot
for node_script in "${PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    systemctl enable "$service_name"
done
echo "✅ Services enabled."
echo

# --- 4. Final Instructions ---
echo "--- 🎉 Installation Complete! ---"
echo
echo "The services have been created and enabled to run automatically on boot."
echo "You can start them immediately without rebooting by running:"
echo
for node_script in "${PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    echo "   sudo systemctl start $service_name"
done
echo
echo "-> To check the status of a service: sudo systemctl status <service_name>"
echo "-> To view live logs: sudo journalctl -u <service_name> -f"

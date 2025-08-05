#!/bin/bash

# --- Configuration ---
VENV_NAME=".venv"

# List of standard nodes that run as a single instance
STANDARD_PYTHON_NODES=(
    "aruco.py"
    "control.py"
    "hmi_node.py"
    "linefollowing_node.py"
    "llc_interface.py"
    "orchestrator.py"
)

# --- Boilerplate ---
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
    sudo -u "$RUN_AS_USER" python3 -m venv "$VENV_NAME"
    echo "✅ Virtual environment created."
fi
chown -R "$RUN_AS_USER":"$RUN_AS_GROUP" "$VENV_NAME"
echo

# --- 2. Install Requirements ---
echo "--- Installing required modules from requirements.txt ---"
source "$VENV_NAME/bin/activate"
pip3 install -r requirements.txt
deactivate
echo "✅ Requirements installed."
echo

# --- 3. Create and Enable Systemd Services ---
echo "--- Creating and enabling systemd services ---"

# --- Create services for standard nodes ---
for node_script in "${STANDARD_PYTHON_NODES[@]}"; do
    service_name="${node_script%.py}.service"
    service_file_path="$SERVICE_FILES_DIR/$service_name"
    venv_python_path="$PROJECT_DIR/$VENV_NAME/bin/python"
    script_path="$PROJECT_DIR/$node_script"

    echo "Creating service file: $service_file_path"
    tee "$service_file_path" > /dev/null << EOL
[Unit]
Description=Kontrol Service for ${node_script}
After=network.target

[Service]
User=$RUN_AS_USER
Group=$RUN_AS_GROUP
WorkingDirectory=$PROJECT_DIR
ExecStart=$venv_python_path $script_path
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOL
done

echo
echo "Reloading systemd daemon and enabling services..."
systemctl daemon-reload

# --- Enable all created services ---
ALL_SERVICES=()
for node_script in "${STANDARD_PYTHON_NODES[@]}"; do
    ALL_SERVICES+=("${node_script%.py}.service")
done

for service_name in "${ALL_SERVICES[@]}"; do
    systemctl enable "$service_name"
done
echo "✅ Services enabled."
echo

# --- 4. Final Instructions ---
echo "--- 🎉 Installation Complete! ---"
echo
echo "The following services have been created and enabled to run on boot:"
for service_name in "${ALL_SERVICES[@]}"; do
    echo "  - ${service_name}"
done
echo
echo "You can start them immediately without rebooting, for example:"
echo "   sudo systemctl start orchestrator.service"
echo
echo "-> To check the status of a service: sudo systemctl status <service_name>"
echo "-> To view live logs: sudo journalctl -u <service_name> -f"

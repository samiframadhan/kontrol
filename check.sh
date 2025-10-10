#!/bin/bash

# This script checks the status of all systemd services
# that were created by the accompanying install.sh script.

# --- Configuration ---
# This list of services is derived directly from the `STANDARD_PYTHON_NODES`
# array in the install.sh script. If you add or remove nodes there,
# you must update this list as well.
SERVICES_TO_CHECK=(
    "control_node.service"
    "hmi_node.service"
    "imageprocess_node.service"
    "llc_interface.service"
    "orchestrator.service"
)

# --- Script Logic ---
echo "--- 🔎 Checking Status of Installed Services ---"
echo

# Loop through each service and display its status using systemctl
for service in "${SERVICES_TO_CHECK[@]}"; do
    echo "================================================="
    echo "Checking status for: $service"
    echo "================================================="
    systemctl status "$service"
    # The 'systemctl status' command has a non-zero exit code if the
    # service is not 'active', 'reloading', 'inactive', or 'activating'.
    # We add a separator regardless of the exit code.
    echo
done

echo "--- ✅ Check Complete ---"
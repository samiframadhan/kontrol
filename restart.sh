#!/bin/bash

# --- Script to Restart Multiple Systemd Services ---
# This script iterates through a predefined list of services
# and restarts each one using systemctl.
# It requires sudo privileges to run.

# Define the list of services to be restarted
services=(
  "aruco.service"
  "control.service"
  "hmi_node.service"
  "linefollowing_node.service"
  "llc_interface.service"
  "orchestrator.service"
)

echo "--- Starting to restart services ---"

# Loop through each service in the array
for service in "${services[@]}"
do
  echo "Restarting ${service}..."
  # Use sudo to restart the service. The command will fail
  # if the script is not run with sufficient privileges.
  sudo systemctl restart "${service}"
  echo "${service} has been restarted."
  echo "-------------------------------------"
done

echo "--- All services have been restarted. ---"

# You can optionally add a command to check the status of all services afterwards
echo "--- Current status of services: ---"
for service in "${services[@]}"
do
  sudo systemctl status "${service}" --no-pager
done
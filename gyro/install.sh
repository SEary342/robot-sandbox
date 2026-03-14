#!/bin/bash

# --- 1. Basic Setup & Dependencies ---
echo "--- Installing System Dependencies ---"
sudo apt-get update
sudo apt-get install -y python3-pip python3-smbus i2c-tools python3-venv libatlas-base-dev

# --- 2. Enable I2C ---
echo "--- Enabling I2C ---"
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo "I2C enabled. REBOOT REQUIRED after this script finishes."
fi

# Load i2c-dev module for the current session
sudo modprobe i2c-dev

# --- 3. Virtual Environment & Python Libraries ---
echo "--- Setting up Virtual Environment ---"
# Navigate to the script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

# Create venv if it doesn't exist
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

# Install required Python packages
source .venv/bin/activate
pip install --upgrade pip
pip install robotpy[ntcore] smbus math

# --- 4. User Setup (Match gyro.service) ---
# The service file expects a user named 'photon'
TARGET_USER="photon"
if ! id "$TARGET_USER" &>/dev/null; then
    echo "Warning: User '$TARGET_USER' does not exist. Creating it now..."
    sudo useradd -m -s /bin/bash "$TARGET_USER"
    sudo usermod -aG i2c "$TARGET_USER"
else
    echo "Ensuring user '$TARGET_USER' is in the 'i2c' group..."
    sudo usermod -aG i2c "$TARGET_USER"
fi

# Fix permissions for the current directory
sudo chown -R $TARGET_USER:$TARGET_USER "$DIR"

# --- 5. Systemd Service Setup ---
echo "--- Installing Systemd Service ---"
SERVICE_FILE="gyro.service"

if [ -f "$SERVICE_FILE" ]; then
    # Update WorkingDirectory and ExecStart in the service file to match actual paths
    sed -i "s|WorkingDirectory=.*|WorkingDirectory=$DIR|" "$SERVICE_FILE"
    sed -i "s|ExecStart=.*|ExecStart=$DIR/.venv/bin/python gyro_service.py|" "$SERVICE_FILE"
    
    # Copy to systemd folder
    sudo cp "$SERVICE_FILE" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable gyro.service
    echo "Service 'gyro.service' installed and enabled."
else
    echo "Error: $SERVICE_FILE not found in the current directory."
fi

echo "--- Installation Complete ---"
echo "PLEASE REBOOT your Raspberry Pi to ensure I2C is fully active."
echo "After reboot, you can check the service status with: sudo systemctl status gyro.service"

#!/bin/bash

# --- 1. Basic Setup & Dependencies ---
echo "--- Installing System Dependencies ---"
sudo apt-get update
sudo apt-get install -y i2c-tools libatlas-base-dev curl

# --- 2. Install UV ---
echo "--- Installing UV ---"
if ! command -v uv &> /dev/null; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # Ensure uv is in path for this session
    export PATH="$HOME/.local/bin:$PATH"
else
    echo "UV already installed."
fi

# --- 3. Enable I2C ---
echo "--- Enabling I2C ---"
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo "I2C enabled. REBOOT REQUIRED after this script finishes."
fi

# Load i2c-dev module for the current session
sudo modprobe i2c-dev

# --- 4. Virtual Environment & Python Libraries ---
echo "--- Setting up Virtual Environment with UV ---"
# Navigate to the script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

# Use UV to setup the environment with Python 3.13
uv python install 3.13
uv sync --python 3.13

# --- 5. User Setup (Match gyro.service) ---
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

# --- 6. Systemd Service Setup ---
echo "--- Installing Systemd Service ---"
SERVICE_FILE="gyro.service"

if [ -f "$SERVICE_FILE" ]; then
    # Update WorkingDirectory and ExecStart in the service file to match actual paths
    sed -i "s|WorkingDirectory=.*|WorkingDirectory=$DIR|" "$SERVICE_FILE"
    # UV sync creates .venv/bin/python
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

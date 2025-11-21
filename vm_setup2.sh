#!/bin/bash

#==============================================================================
#               Ubuntu 22.04 VM Setup Script for AutoDRIVE
#==============================================================================
# This script automates the setup of an Ubuntu 22.04 virtual machine with a
# desktop environment, drivers, ROS 2, Docker, and other development tools.
#==============================================================================

# Exit immediately if a command fails
set -e

# Prevent interactive prompts during package installation
export DEBIAN_FRONTEND=noninteractive

# Set 'needrestart' to automatically restart services without prompting
export NEEDRESTART_MODE=a 

# Ensure script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo."
  exit 1
fi

## Section 1: System Update & Core Packages
echo "### Updating system and installing core packages... ###"

# --- FIX FOR COLORD ERROR ---
# Pre-create the colord group and user to prevent tmpfiles.d errors during install
# We use '|| true' to prevent the script from exiting if the group/user already exists
addgroup --system colord || true
adduser --system --ingroup colord --home /var/lib/colord colord || true
# ----------------------------

apt-get update
apt-get upgrade -y

# Install essential packages and the desktop environment in a single command
apt-get install -y \
    ubuntu-desktop \
    software-properties-common \
    curl \
    wget \
    unzip \
    git \
    python3-pip \
    python3.10-venv

# ---

## Section 2: Drivers & Third-Party Repositories
echo "### Installing drivers and adding third-party software repositories... ###"

# --- NVIDIA Drivers ---
echo "Installing NVIDIA drivers..."
ubuntu-drivers install -y

# --- ROS 2 Humble ---
echo "Setting up ROS 2 Humble repository..."
add-apt-repository universe -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# --- Docker Engine ---
echo "Setting up Docker repository..."
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) stable" > /etc/apt/sources.list.d/docker.list

# --- Google (Chrome Remote Desktop) ---
echo "Setting up Google repository..."
curl -fsSL https://dl.google.com/linux/linux_signing_key.pub | gpg --dearmor -o /etc/apt/trusted.gpg.d/chrome-remote-desktop.gpg
echo "deb [arch=amd64] https://dl.google.com/linux/chrome-remote-desktop/deb stable main" | tee /etc/apt/sources.list.d/chrome-remote-desktop.list

# --- Microsoft (VS Code) ---
echo "Setting up Microsoft repository..."
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list

# ---

## Section 3: Application Installation
echo "### Installing applications from all configured repositories... ###"

# Update package list after adding all new repositories
apt-get update

# Install all applications in a single command
apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-tf-transformations \
    ros-humble-rviz-imu-plugin \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin \
    chrome-remote-desktop \
    code 

# --- Foxglove Studio (via .deb) ---
echo "Installing Foxglove Studio..."
wget -O /tmp/foxglove.deb https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
apt-get install -y /tmp/foxglove.deb
rm /tmp/foxglove.deb

# ---

## Section 4: AutoDRIVE Environment Setup
echo "### Setting up the AutoDRIVE environment... ###"

# Create the main directory structure
mkdir -p AutoDrive/{DevKit,Explore,Practice,Compete}

# Download and unpack the AutoDRIVE DevKit
echo "Downloading AutoDRIVE DevKit..."
wget -q --show-progress -P AutoDrive/DevKit https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_devkit.zip
unzip -q AutoDrive/DevKit/autodrive_devkit.zip -d AutoDrive/DevKit

# Loop through simulator types to download, unpack, and set permissions
for SIM_TYPE in Explore Practice Compete; do
    echo "Downloading AutoDRIVE Simulator: $SIM_TYPE..."
    LOWER_SIM_TYPE=$(echo "$SIM_TYPE" | tr '[:upper:]' '[:lower:]')
    wget -q --show-progress -P AutoDrive/$SIM_TYPE https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_${LOWER_SIM_TYPE}_linux.zip
    unzip -q AutoDrive/$SIM_TYPE/autodrive_simulator_${LOWER_SIM_TYPE}_linux.zip -d AutoDrive/$SIM_TYPE
    chmod +x AutoDrive/$SIM_TYPE/autodrive_simulator/AutoDRIVE\ Simulator.x86_64
done

# Set correct ownership instead of using 'chmod 777' for better security
echo "Setting permissions for AutoDRIVE directory..."
chown -R $SUDO_USER:$SUDO_USER ./AutoDrive

# Pull AutoDRIVE Docker images
echo "Pulling AutoDRIVE Docker images..."
docker pull autodriveecosystem/autodrive_roboracer_api:2025-icra-compete
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-icra-compete

# Add the user to the 'docker' group to run Docker commands without sudo
usermod -aG docker $SUDO_USER

# ---

## Section 5: Install ROS2 Packages
echo "### Installing ROS2 Packages... ###"

# Instal SLAM
apt install -y ros-humble-slam-toolbox

# ---

## Section 6: Python Dependencies
echo "### Installing Python dependencies... ###"

# Run pip as the original user to install packages in the user's home directory
sudo -u $SUDO_USER pip3 install --upgrade pip packaging
sudo -u $SUDO_USER pip3 install setuptools==79.0.1
sudo -u $SUDO_USER pip3 install -r ./AutoDrive/DevKit/autodrive_devkit/requirements_python_3.10.txt
sudo -u $SUDO_USER pip3 install numpy==1.23.5 opencv-contrib-python==4.8.1.78 attrdict
# Install PyTorch with specific CUDA version
sudo -u $SUDO_USER pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
# sudo -u $SUDO_USER pip3 install stable-baselines

# ---

## ✅ Finalization
echo ""
echo "Installation Complete!"
echo "Please RESTART the virtual machine for all changes to take effect."
echo "After restarting, you can run 'docker' commands without 'sudo'."

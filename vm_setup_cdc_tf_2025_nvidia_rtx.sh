#!/bin/bash

#==============================================================================
#             Ubuntu 22.04 VM Setup Script for AutoDRIVE (RTX vWS)
#==============================================================================
# This script automates the setup of an Ubuntu 22.04 virtual machine with:
# - NVIDIA RTX Virtual Workstation (GRID) Drivers & CUDA
# - ROS 2 Humble
# - Docker & AutoDRIVE Ecosystem
# - VS Code & Chrome Remote Desktop
#==============================================================================

# Exit immediately if a command fails
set -e

# Prevent interactive prompts during package installation
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a 

# Ensure script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo."
  exit 1
fi

## Section 1: System Update & Core Dependencies
echo "### Updating system and installing core packages... ###"

# --- FIX FOR MISSING USERS (Colord & Speech-Dispatcher) ---
addgroup --system colord || true
adduser --system --ingroup colord --home /var/lib/colord colord || true
addgroup --system speech-dispatcher || true
adduser --system --ingroup speech-dispatcher --no-create-home --home /var/run/speech-dispatcher speech-dispatcher || true
# ----------------------------------------------------------

apt-get update
apt-get upgrade -y

# Install essential packages and desktop environment
# Note: linux-headers and build tools moved to Section 2 to group with Driver setup
apt-get install -y \
    ubuntu-desktop \
    software-properties-common \
    curl \
    wget \
    unzip \
    git \
    python3-pip \
    python3.10-venv \
    libvulkan1 \
    pkg-config \
    libglvnd-dev

# ---

## Section 2: NVIDIA RTX Virtual Workstation (GRID) Setup
echo "### Installing NVIDIA GRID Drivers and CUDA... ###"

# 1. Purge conflicting standard drivers
echo "Purging old NVIDIA drivers..."
apt-get purge -y nvidia* libnvidia* || true
apt-get autoremove -y

# 2. PREPARE COMPILER (User Requested Updates)
echo "Configuring GCC-12 for Driver Installation..."
apt install -y gcc-12 g++-12
apt install -y linux-headers-$(uname -r) build-essential

# Setup update-alternatives to point 'gcc' to 'gcc-12'
# We install gcc-12 with priority 100 to ensure it becomes the auto-default
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100

# Note: 'update-alternatives --config gcc' is interactive and breaks scripts.
# We use '--set' instead to explicitly force the selection without user input.
update-alternatives --set gcc /usr/bin/gcc-12
update-alternatives --set g++ /usr/bin/g++-12

# Explicitly export CC for the current shell just in case
export CC=/usr/bin/gcc-12

# 3. Disable GSP Firmware (CRITICAL for T4/L4 vWS stability)
echo "Disabling GSP Firmware..."
bash -c 'echo "options nvidia NVreg_EnableGpuFirmware=0" > /etc/modprobe.d/nvidia-gsp.conf'

# 4. Download & Install NVIDIA GRID Driver (vGPU 19.1)
GRID_VER="580.82.07"
GRID_URL="https://storage.googleapis.com/nvidia-drivers-us-public/GRID/vGPU19.1/NVIDIA-Linux-x86_64-${GRID_VER}-grid.run"

echo "Downloading NVIDIA GRID Driver ${GRID_VER}..."
wget -O /tmp/nvidia_grid_driver.run "${GRID_URL}"
chmod +x /tmp/nvidia_grid_driver.run

echo "Installing NVIDIA GRID Driver..."
# The installer will now find gcc-12 automatically due to the steps above
/tmp/nvidia_grid_driver.run --silent --no-questions
rm /tmp/nvidia_grid_driver.run

# 5. Install CUDA Toolkit 12.4 (Without overwriting the driver)
CUDA_RUNFILE="cuda_12.4.1_550.54.15_linux.run"
CUDA_URL="https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/${CUDA_RUNFILE}"

echo "Downloading CUDA Toolkit 12.4..."
wget -O /tmp/${CUDA_RUNFILE} "${CUDA_URL}"
echo "Installing CUDA Toolkit..."
sh /tmp/${CUDA_RUNFILE} --silent --toolkit
rm /tmp/${CUDA_RUNFILE}

# 6. Set Environment Variables
echo 'export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}' >> /etc/profile.d/cuda.sh
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> /etc/profile.d/cuda.sh

# ---

## Section 3: Third-Party Repositories
echo "### Adding third-party software repositories... ###"

# --- ROS 2 Humble ---
add-apt-repository universe -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# --- Docker Engine ---
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) stable" > /etc/apt/sources.list.d/docker.list

# --- Google (Chrome Remote Desktop) ---
curl -fsSL https://dl.google.com/linux/linux_signing_key.pub | gpg --dearmor -o /etc/apt/trusted.gpg.d/chrome-remote-desktop.gpg
echo "deb [arch=amd64] https://dl.google.com/linux/chrome-remote-desktop/deb stable main" | tee /etc/apt/sources.list.d/chrome-remote-desktop.list

# --- Microsoft (VS Code) ---
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list

# ---

## Section 4: Application Installation
echo "### Installing applications... ###"

apt-get update

# Install applications
apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-tf-transformations \
    ros-humble-rviz-imu-plugin \
    ros-humble-slam-toolbox \
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

## Section 5: AutoDRIVE Environment Setup
echo "### Setting up the AutoDRIVE environment... ###"

# Create directories
mkdir -p AutoDrive/{DevKit,Explore,Practice}

# Download and unpack DevKit
echo "Downloading AutoDRIVE DevKit..."
wget -q --show-progress -P AutoDrive/DevKit https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-cdc-tf/autodrive_devkit.zip
unzip -q AutoDrive/DevKit/autodrive_devkit.zip -d AutoDrive/DevKit

# Download and unpack Simulators
for SIM_TYPE in Explore Practice; do
    echo "Downloading AutoDRIVE Simulator: $SIM_TYPE..."
    LOWER_SIM_TYPE=$(echo "$SIM_TYPE" | tr '[:upper:]' '[:lower:]')
    wget -q --show-progress -P AutoDrive/$SIM_TYPE https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-cdc-tf/autodrive_simulator_${LOWER_SIM_TYPE}_linux.zip
    unzip -q AutoDrive/$SIM_TYPE/autodrive_simulator_${LOWER_SIM_TYPE}_linux.zip -d AutoDrive/$SIM_TYPE
    chmod +x AutoDrive/$SIM_TYPE/autodrive_simulator/AutoDRIVE\ Simulator.x86_64
done

# Set permissions
echo "Setting permissions for AutoDRIVE directory..."
chown -R $SUDO_USER:$SUDO_USER ./AutoDrive

# Pull Docker images
echo "Pulling AutoDRIVE Docker images..."
docker pull autodriveecosystem/autodrive_roboracer_api:2025-cdc-tf-practice
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-cdc-tf-practice

# Add user to docker group
usermod -aG docker $SUDO_USER

# ---

## Section 6: Python Dependencies
echo "### Installing Python dependencies... ###"

# Run pip as the original user
sudo -u $SUDO_USER pip3 install --upgrade pip packaging
# sudo -u $SUDO_USER pip3 install setuptools==79.0.1
sudo -u $SUDO_USER pip3 install -r ./AutoDrive/DevKit/autodrive_devkit/requirements_python_3.10.txt
# sudo -u $SUDO_USER pip3 install numpy==1.23.5 opencv-contrib-python==4.8.1.78 attrdict
sudo -u $SUDO_USER pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
# sudo -u $SUDO_USER pip3 install stable-baselines3

# ---

## ✅ Finalization
echo ""
echo "Installation Complete!"
echo "Please RESTART the virtual machine for the NVIDIA drivers to load."
echo "Command: sudo reboot"

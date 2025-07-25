#!/bin/bash

# These setup commands are intended to be used on Ubuntu 22.04 PRO Version hosted on Google Cloud
export DEBIAN_FRONTEND=noninteractive

# Update and Upgrade
# sudo apt update && sudo NEEDRESTART_MODE=a apt upgrade -y

# Update Packages
sudo apt update
sudo NEEDRESTART_MODE=a apt upgrade -y

# Install Ubuntu Desktop
sudo apt-get install -y ubuntu-desktop

# Install Nvidia Drivers
sudo ubuntu-drivers install

echo "Ubuntu Drivers Installed"

# Install ROS 2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools

# Install Chrome Remote Desktop
wget -O /tmp/chrome-remote-desktop_current_amd64.deb https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y /tmp/chrome-remote-desktop_current_amd64.deb

# Install Visual Studio Code
curl -L -o /tmp/vsc.deb "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y /tmp/vsc.deb

# Install Foxglove
wget -O /tmp/foxglove-studio-latest-linux-amd64.deb https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y /tmp/foxglove-studio-latest-linux-amd64.deb

mkdir ./AutoDrive/
mkdir ./AutoDrive/DevKit
mkdir ./AutoDrive/Explore
mkdir ./AutoDrive/Practice
mkdir ./AutoDrive/Compete
mkdir ./AutoDrive/DevKit//autodrive_devkit
mkdir ./AutoDrive/Practice/autodrive_simulator
mkdir ./AutoDrive/Compete/autodrive_simulator

sudo chmod 777 -v ./AutoDrive/
sudo chmod 777 -v ./AutoDrive/DevKit
sudo chmod 777 -v ./AutoDrive/Explore
sudo chmod 777 -v ./AutoDrive/Practice
sudo chmod 777 -v ./AutoDrive/Compete
sudo chmod 777 -v ./AutoDrive/DevKit//autodrive_devkit
sudo chmod 777 -v ./AutoDrive/Explore/autodrive_simulator
sudo chmod 777 -v ./AutoDrive/Practice/autodrive_simulator
sudo chmod 777 -v ./AutoDrive/Compete/autodrive_simulator

# Get AutoDrive Files
wget -P ./AutoDrive/Explore https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_explore_linux.zip
unzip ./AutoDrive/Explore/autodrive_simulator_explore_linux.zip -d ./AutoDrive/Explore
sudo chmod +x "./AutoDrive/Explore/autodrive_simulator/AutoDRIVE Simulator.x86_64"

wget -P ./AutoDrive/Practice https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_practice_linux.zip
unzip ./AutoDrive/Practice/autodrive_simulator_practice_linux.zip -d ./AutoDrive/Practice
sudo chmod +x "./AutoDrive/Practice/autodrive_simulator/AutoDRIVE Simulator.x86_64"

wget -P ./AutoDrive/Compete https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_compete_linux.zip
unzip ./AutoDrive/Compete/autodrive_simulator_compete_linux.zip -d ./AutoDrive/Compete
sudo chmod +x "./AutoDrive/Compete/autodrive_simulator/AutoDRIVE Simulator.x86_64"

wget -P ./AutoDrive/DevKit https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_devkit.zip
unzip ./AutoDrive/DevKit/autodrive_devkit.zip -d ./AutoDrive/DevKit


# Install Docker Engine
# Add Docker's official GPG key:
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Get Docker Images
docker pull autodriveecosystem/autodrive_roboracer_api:2025-icra-compete
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-icra-compete

# Install TF Tranformations
sudo apt-get install -y ros-humble-tf-transformations

# Install rviz imu plugin
sudo apt-get install -y ros-humble-rviz-imu-plugin

# Install pip3
sudo apt-get install -y python3-pip
sudo apt-get install -y python3.10-venv

https://code.visualstudio.com/download#
https://go.microsoft.com/fwlink/?LinkID=760868

# Install Python Dependencies
# Numpy==1.23.5, opencv-contrib-python==4.8.1.78, attrdict
pip3 install --upgrade pip setuptools packaging
pip3 install -r ./AutoDrive/DevKit/autodrive_devkit/requirements_python_3.10.txt
pip3 install numpy==1.23.5 opencv-contrib-python==4.8.1.78 attrdict

# Update Packages
sudo apt update
sudo NEEDRESTART_MODE=a apt upgrade -y

echo "Installation Complete. Please restart the VM"

# Source image ubuntu-pro-2204-jammy-v20250701
# GPUS: V100, L4

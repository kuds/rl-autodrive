# These setup commands are intended to be used on Ubuntu 22.04 PRO Version hosted on Google Cloud 

# Update Packages
sudo apt update && sudo NEEDRESTART_MODE=a apt upgrade -y

# Install Ubuntu Desktop
sudo apt install -y ubuntu-desktop

# Install Nvidia Drivers
sudo ubuntu-drivers install

# Install Python3 Pip
# sudo apt install -y python3-pip

# Get AutoDrive Files
wget -P ./Documents/ https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_explore_linux.zip
unzip ./Documents/autodrive_simulator_explore_linux.zip -d ./Documents/
sudo chmod +x ./Documents/autodrive_simulator/AutoDRIVE Simulator.x86_64
wget -P ./Documents/ https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_devkit.zip
unzip ./Documents/autodrive_devkit.zip -d ./Documents/

# Install ROS 2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo NEEDRESTART_MODE=a apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools 

# Get Chrome Remote Desktop
wget https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb
sudo apt-get install --assume-yes ./chrome-remote-desktop_current_amd64.deb

# Update Packages
sudo apt update && sudo NEEDRESTART_MODE=a apt upgrade -y

sudo reboot

# Source image ubuntu-pro-2204-jammy-v20250701
# GPUS: V100, L4

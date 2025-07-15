# These setup commands are intended to be used on Ubuntu 22.04 PRO Version hosted on Google Cloud 

wget https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb
sudo apt-get install --assume-yes ./chrome-remote-desktop_current_amd64.deb
sudo apt update && sudo NEEDRESTART_MODE=a apt upgrade -y
sudo apt install -y ubuntu-desktop
sudo ubuntu-drivers install
sudo apt install -y python3-pip
wget /Documents/ https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_simulator_explore_linux.zip
unzip autodrive_simulator_explore_linux.zip
wget /Documents/ https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2025-icra/autodrive_devkit.zip
unzip autodrive_devkit.zip
sudo reboot

# Need to install ros2
# Source image ubuntu-pro-2204-jammy-v20250701
# GPUS: V100, L4

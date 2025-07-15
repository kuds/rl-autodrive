# These setup commands are intended to be used on Ubuntu 22.04 PRO Version hosted on Google Cloud 

wget https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb
sudo apt-get install --assume-yes ./chrome-remote-desktop_current_amd64.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ubuntu-desktop
sudo ubuntu-drivers install
sudo apt install -y python3-pip
sudo reboot

# Need to install ros2
# Source image ubuntu-pro-2204-jammy-v20250701
# GPUS: V100, L4

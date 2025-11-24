# Setup Instructions

## Google Cloud VM Spec
- Machine type: g2-standard-16 (16 vCPUs, 64 GB Memory)
- Architecture: x86/64
- Operating System: Ubuntu Pro 22.04 LTS (Jammy)
- GPUs: NVIDIA L4, V100, A100
- Optional: NVIDIA GRID (10%~20% increase in rendering speed) 

## Script Setup

### Pre-work

When launching a new instance, it is always good practice to get latest on the libraries
```bash
sudo apt update && sudo apt upgrade
```

### ICRA 2025
This script will setup all the dependencies include docker if virtualization is enabled on Google Cloud VM Instance
```bash
curl -s https://raw.githubusercontent.com/kuds/rl-autodrive/refs/heads/main/vm_setup.sh | sudo bash
```

### CDC-TF 2025
This script will setup all the dependencies include docker if virtualization is enabled on Google Cloud VM Instance
```bash
curl -s https://raw.githubusercontent.com/kuds/rl-autodrive/refs/heads/main/vm_setup_cdc_tf_2025.sh | sudo bash
```

This script will setup all the dependencies including RTX Nvidia Virtual Workstations
```bash
curl -s https://raw.githubusercontent.com/kuds/rl-autodrive/refs/heads/main/vm_setup_cdc_tf_2025_nvidia_rtx.sh | sudo bash
```

Once everything is built and running, run this test command to send a basic thorttle amount
```bash
ros2 topic pub --once /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 "{data: 0.001}"
```

## Helpful Command

### Run AutoDrive Viewer/Map

#### Setup ROS2 Humble Source in Terminal
```bash
source /opt/ros/humble/setup.bash
```

#### Launch AutoDriver Viewer
```bash
./AutoDRIVE\ Simulator.x86_64
```

### Launch AutoDriver in Headless Mode
```bash
xvfb-run -a ./AutoDRIVE\ Simulator.x86_64 -batchmode -ip 127.0.0.1 -port 4567
```

### Launch AutoDriver in No Graphics Mode
```bash
xvfb-run -a -s "-screen 0 1280x720x24" \
  ./autodrive_simulator.x86_64 \
  -batchmode \
  -nographics \  <-- REMOVE THIS if using camera sensors/rendering
  -force-vulkan \
  -noaudio \
  -logFile /tmp/unity_log.txt
```
Important Note on -nographics: If your simulation uses Cameras/Lidar to capture data for Reinforcement Learning, do not use the -nographics flag. That flag stops the GPU from rendering the frame buffer, returning black images to your sensors. Use -batchmode combined with xvfb as shown above.


### Run AutoDrive DevKit

#### Build DevKit
```bash
# ./Autodrive_Devkit
colcon build
```

#### Run DevKit
```bash
# source install/setup.bash
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```

## Helpful Links
- [How to stop ubuntu pop-up "Daemons using outdated libraries" when using apt to install or update packages? [closed]](https://stackoverflow.com/questions/73397110/how-to-stop-ubuntu-pop-up-daemons-using-outdated-libraries-when-using-apt-to-i)
- [Launch Ubuntu 22.04 Desktop on Google Cloud](https://ubuntu.com/blog/launch-ubuntu-22-04-desktop-on-google-cloud)
- [Chrome Remote Desktop](https://remotedesktop.google.com/unsupported-browser?target=%2F)
- [ROS 2 Documentation: Humble - Ubuntu Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Docker Documentation](https://docs.docker.com)
- [Google Cloud Platform - Enable Nested Virtualization](https://cloud.google.com/compute/docs/instances/nested-virtualization/enabling)
- [Techincal Guide - AutoDRIVE Ecosystem](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2025/)
- [Orientation | RoboRacer Sim Racing League @ ICRA 2025](https://www.youtube.com/watch?v=Mit9c8B-06o)

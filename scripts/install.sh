#!/bin/bash

# Define colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting ROS2 Humble installation...${NC}"

# Install ROS2 Humble
echo -e "${YELLOW}Updating package list and installing curl...${NC}"
sudo apt update && sudo apt install curl -y

echo -e "${YELLOW}Adding ROS2 GPG key...${NC}"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo -e "${YELLOW}Adding ROS2 repository to sources list...${NC}"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package database
echo -e "${YELLOW}Updating package database...${NC}"
sudo apt update
sudo apt -y upgrade

# Install ROS2 Desktop (Full)
echo -e "${YELLOW}Installing ROS2 Desktop (Full)...${NC}"
sudo apt -y install ros-humble-desktop-full

# Install ROS2 Gazebo and TurtleBot3 packages
echo -e "${YELLOW}Installing ROS2 Gazebo and TurtleBot3 packages...${NC}"
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
sudo apt -y install ros-humble-turtlebot4-desktop

# Install colcon tab completion 
echo -e "${YELLOW}Installing colcon tab completion...${NC}"
sudo apt -y install python3-colcon-common-extensions

# Install Clang and cpplint
echo -e "${YELLOW}Installing Clang and cpplint...${NC}"
sudo apt install build-essential cmake ccache bear clangd clang-tidy cpplint

# Install Catch2 for ROS Humble
echo -e "${YELLOW}Installing Catch2 for ROS Humble...${NC}"
sudo apt install ros-humble-catch-ros2

# Install ROS2 Humble dependencies
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo -e "${GREEN}ROS2 Humble installation and all dependencies have been successfully installed!${NC}"
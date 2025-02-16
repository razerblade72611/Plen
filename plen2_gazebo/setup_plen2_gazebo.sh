#!/bin/bash

set -e  # Exit immediately if any command fails

# Define constants
CONTAINER_NAME="plen2_gazebo_container"
IMAGE_NAME="osrf/ros:iron-desktop"
PLEN_DIR="$HOME/PLEN"
GITHUB_REPO="https://github.com/YOUR_GITHUB_USERNAME/PLEN.git"
GAZEBO_MODEL_PATH="$PLEN_DIR/install/plen2_gazebo/share/plen2_gazebo"

echo "========================="
echo " PLEN2 Gazebo Setup Script"
echo "========================="

# Step 1: Clone or update the PLEN repository from GitHub
if [ ! -d "$PLEN_DIR" ]; then
    echo "[INFO] Cloning PLEN2 repository from GitHub..."
    git clone $GITHUB_REPO $PLEN_DIR
else
    echo "[INFO] PLEN2 directory exists. Updating repository..."
    cd $PLEN_DIR
    git pull origin main
fi

# Ensure the plen2_gazebo package exists
if [ ! -d "$PLEN_DIR/src/plen2_gazebo" ]; then
    echo "[ERROR] plen2_gazebo package not found in repository!"
    exit 1
fi

# Step 2: Install Docker if not present
if ! command -v docker &> /dev/null
then
    echo "[ERROR] Docker is not installed. Installing Docker..."
    sudo apt update
    sudo apt install -y docker.io
    sudo systemctl start docker || sudo service docker start || sudo dockerd &
    sudo systemctl enable docker || echo "[WARNING] systemd not available, skipping enable."
    echo "[INFO] Docker installed successfully."
fi

# Ensure Docker is running
if ! systemctl is-active --quiet docker && ! pgrep -x "dockerd" > /dev/null
then
    echo "[WARNING] Docker service is not running. Starting Docker..."
    sudo service docker start || sudo dockerd &
fi

# Verify Docker works
if ! docker info &> /dev/null; then
    echo "[ERROR] Docker is not working. Please restart your system or check the installation."
    exit 1
fi

# Step 3: Pull the ROS2/Gazebo Docker image
echo "[INFO] Checking if Docker image exists..."
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "[INFO] Pulling Docker image $IMAGE_NAME..."
    docker pull $IMAGE_NAME
fi

# Step 4: Start the container if it's not running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "[INFO] Container is already running."
else
    echo "[INFO] Starting new container: $CONTAINER_NAME..."
    docker run -itd --name $CONTAINER_NAME --net=host --privileged \
        -v $PLEN_DIR:/root/PLEN \
        -e DISPLAY=$DISPLAY \
        -e GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH \
        --device /dev/dri \
        --device /dev/snd \
        $IMAGE_NAME bash
    echo "[INFO] Container started successfully."
fi

# Step 5: Copy missing model files
echo "[INFO] Checking model files..."
docker exec $CONTAINER_NAME bash -c "
    mkdir -p $GAZEBO_MODEL_PATH
    if [ ! -f $GAZEBO_MODEL_PATH/model.config ]; then
        cp /root/PLEN/src/plen2_gazebo/model.config $GAZEBO_MODEL_PATH/
        echo '[INFO] Copied missing model.config'
    fi
    if [ ! -f $GAZEBO_MODEL_PATH/model.sdf ]; then
        cp /root/PLEN/src/plen2_gazebo/model.sdf $GAZEBO_MODEL_PATH/
        echo '[INFO] Copied missing model.sdf'
    fi
"

# Step 6: Build the plen2_gazebo package inside the container
echo "[INFO] Cleaning and rebuilding plen2_gazebo..."
docker exec $CONTAINER_NAME bash -c "
    cd /root/PLEN
    rm -rf build/ install/ log/
    colcon build --symlink-install --packages-select plen2_gazebo
    source install/setup.bash
"

# Step 7: Set up environment variables inside the container
echo "[INFO] Setting up environment variables..."
docker exec $CONTAINER_NAME bash -c "
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH' >> ~/.bashrc
    source ~/.bashrc
"

# Step 8: Fix missing Gazebo model paths
echo "[INFO] Fixing missing Gazebo model paths..."
docker exec $CONTAINER_NAME bash -c "
    mkdir -p /root/.gazebo/models
    ln -s $GAZEBO_MODEL_PATH /root/.gazebo/models/plen2_gazebo
"

# Step 9: Launch Gazebo with PLEN2
echo "[INFO] Launching Gazebo with PLEN2..."
docker exec -it $CONTAINER_NAME bash -c "
    source /root/PLEN/install/setup.bash
    ros2 launch plen2_gazebo gazebo.launch.py
"

echo "[SUCCESS] PLEN2 Gazebo is running inside Docker!"


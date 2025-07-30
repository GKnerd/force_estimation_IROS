#!/bin/sh
echo "Run Moveit2 Humble Container"

# Not needed for docker on WSL2, as WSL handles GUI applications natively.
xhost + local:root

# Mount the .X11.unix driver to be able to open GUI applications.
# Allow GPU passthrough to get hardware acceleration
docker run \
    --name moveit2_humble  \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/georgios-katranis/IROS_force_estimation/moveit_jacobian_ws:/home/docker/moveit_jacobian_ws \
    -v /home/georgios-katranis/IROS_force_estimation/datasets/:/home/docker/robot_data \
    --privileged \
    -it \
    --ipc host \
    --rm \
    moveit2:humble

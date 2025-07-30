##############################################################################
##                                 Base Image                               ##
##############################################################################

# Use the MoveIt 2 Humble release image as the base image
FROM moveit/moveit2:humble-release

# Install additional dependencies (e.g., Python 3 and utilities)
RUN apt-get update && \
     apt-get install --no-install-recommends -y \
     apt-utils bash nano util-linux \
     htop git tmux sudo wget gedit bsdmainutils \
     dirmngr gnupg2 lsb-release can-utils iproute2 \
     mesa-utils x11-utils python3-pip \
     nlohmann-json3-dev

RUN apt update && apt upgrade -y
    #\
    # software-properties-common \
    # dirmngr gnupg2 lsb-release can-utils iproute2 \
    # apt-utils bash nano aptitude util-linux \
    # htop git tmux sudo wget gedit bsdmainutils \
    # mesa-utils x11-utils python3-pip python3.12-venv 
    # # Mesa Graphics Workaround
    # add-apt-repository ppa:kisak/kisak-mesa -y && \
    # apt update && \
    # apt upgrade -y && \
    # # Clear apt-cache to reduce image size
    # rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################

ARG USER=docker
ARG PASSWORD=docker
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=17
ARG VIDEO_GID=44
ENV ROS_DOMAIN_ID=${DOMAIN_ID}
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}

# Create the user and add necessary groups
RUN groupadd -g "$GID" "$USER" && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    groupadd realtime && \
    groupmod -g ${VIDEO_GID} video && \
    usermod -aG video "$USER" && \
    usermod -aG dialout "$USER" && \
    usermod -aG realtime "$USER" && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

# Set up ROS environment variables for the user
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

USER $USER

##############################################################################
##                                 Python Dependencies                      ##
##############################################################################

# Set the working directory
WORKDIR /home/$USER/

RUN pip install numpy scipy matplotlib

# The default command to start the container and enter bash
CMD /bin/bash
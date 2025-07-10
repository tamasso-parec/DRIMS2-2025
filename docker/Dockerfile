FROM ros:humble-perception

ENV DEBIAN_FRONTEND=noninteractive

# Set timezone (avoid tzdata prompt)
RUN echo 'tzdata tzdata/Areas select Etc' | debconf-set-selections \
 && echo 'tzdata tzdata/Zones/Etc select UTC' | debconf-set-selections

# Base tools
RUN apt-get update && apt-get install -y \
    build-essential nano tmux iputils-ping gdb cmake git openssl libssl-dev libmysqlclient-dev \
    python3-colcon-common-extensions \
    python3-pip \
    libboost-all-dev \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-moveit ros-humble-depthai-ros \
    ros-humble-behaviortree-cpp* ros-humble-ros-testing freeglut3-dev ros-humble-moveit-resources-* \
    libprotobuf-dev protobuf-compiler ros-humble-tf*
    
RUN apt-get install -y python3-pip python3-colcon-common-extensions python3-pybind11 cmake
    

# Python dependencies
RUN pip3 install --upgrade scipy depthai trimesh transforms3d

# Create user and group with correct IDs
ARG GROUP_NAME=drims2
ARG GROUP_ID=42042
ARG USER_NAME=drims
ARG USER_ID=1001

RUN groupadd -g $GROUP_ID $GROUP_NAME && \
    useradd -ms /bin/bash -u $USER_ID -g $GROUP_ID $USER_NAME && \
    echo "$USER_NAME:$USER_NAME" | chpasswd

# Setup workspace as root
RUN mkdir -p /home/$USER_NAME/static/drims2_ws/src

WORKDIR /home/$USER_NAME/static/drims2_ws/src

# Clone code as root
RUN git clone https://github.com/SamueleSandrini/drims2_motion_control.git
RUN vcs import < drims2_motion_control/dependencies.repos
RUN git clone https://github.com/CNR-STIIMA-IRAS/drims2_dice_simulator.git
RUN git clone https://github.com/CNR-STIIMA-IRAS/drims2_cells
COPY deps.repos drims2_cells/deps.repos
RUN vcs import < drims2_cells/deps.repos

COPY abb_librws_2.0 ./abb_librws_2.0
COPY abb_omnicore_ros2 ./abb_omnicore_ros2

RUN bash ./abb_librws_2.0/setup_poco_ppa.sh

# Fix ownership of everything
RUN chown -R $USER_NAME:$GROUP_NAME /home/$USER_NAME

# Switch to the user
USER $USER_NAME
WORKDIR /home/$USER_NAME/static/drims2_ws


# 1st build: just build interfaces and messages
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #--packages-select drims2_msgs


# Add ROS and workspace to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f ~/static/drims2_ws/install/setup.bash ]; then source ~/static/drims2_ws/install/setup.bash; fi" >> ~/.bashrc

# Add the check script
COPY check_script.sh /home/$USER_NAME/check_script.sh
USER root
RUN chmod +x /home/$USER_NAME/check_script.sh

WORKDIR /home/$USER_NAME

#Letsgo
CMD ["bash"]


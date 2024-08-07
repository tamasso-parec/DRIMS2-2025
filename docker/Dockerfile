FROM ros:noetic-perception-focal

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Preconfigure debconf for any package that might ask for interactive inputs
RUN echo 'tzdata tzdata/Areas select Etc' | debconf-set-selections \
 && echo 'tzdata tzdata/Zones/Etc select UTC' | debconf-set-selections
 
RUN apt-get update && apt-get install -y build-essential nano 

RUN apt-get update && apt-get install -y ros-noetic-desktop-full ros-noetic-depthai*

RUN apt-get update && apt-get install -y tmux

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# merlin part

RUN apt-get update && apt install -y ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers
RUN apt-get update && apt install -y ros-${ROS_DISTRO}-moveit
RUN apt-get update && apt install -y build-essential gdb cmake git
RUN apt-get update && apt-get install -y openssl libssl-dev
RUN apt-get update && apt-get install -y libmysqlclient-dev

# Install Boost C++
RUN apt-get update && apt-get install -y libboost-all-dev

# Clone and compile Poco libraries
RUN git clone https://github.com/pocoproject/poco.git /tmp/poco \
 && cd /tmp/poco \
 && mkdir cmake-build \
 && cd cmake-build \
 && cmake .. \
 && cmake --build . --config Release \
 && cmake --build . --target install \
 && cp /usr/local/lib/*Poco* /usr/lib \
 && rm -rf /tmp/poco

# Set the group name and GID
ARG GROUP_NAME=drims2
ARG GROUP_ID=42042
ARG USER_NAME=drims
ARG USER_ID=1001

# Create the group and user 
RUN useradd -ms /bin/bash drims && \
    echo "drims:drims" | chpasswd
    
RUN groupadd -g $GROUP_ID $GROUP_NAME 

# Add the user 'drims' to the created group
RUN usermod -aG $GROUP_NAME drims    

RUN mkdir /home/drims/drims_ws \
  && chown -R drims:drims2 /home/drims/drims_ws
    
# Clone the ABB Omnicore ROS Driver repository and build it
RUN mkdir -p /home/drims/catkin_ws/src \
 && cd /home/drims/catkin_ws/src \
 && git clone https://github.com/MerlinLaboratory/ABB_omnicore_ros_driver.git 
 
# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/drims/catkin_ws; catkin_make"

# Environments sources 
RUN echo "source /home/drims/catkin_ws/devel/setup.bash" >> /home/drims/.bashrc
RUN echo "source /home/drims/drims_ws/devel/setup.bash --extend" >> /home/drims/.bashrc

# Add the check script
COPY check_script.sh /check_script.sh
RUN chmod +x check_script.sh

# Letsgo
CMD ["bash"]

FROM ros:humble-desktop

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Preconfigure debconf for any package that might ask for interactive inputs
RUN echo 'tzdata tzdata/Areas select Etc' | debconf-set-selections \
 && echo 'tzdata tzdata/Zones/Etc select UTC' | debconf-set-selections
 
RUN apt-get update && \
    apt-get install -y build-essential nano \
    python3-colcon-common-extensions \
    tmux \
    iputils-ping
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Additional ROS 2 tools
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-moveit \
    libsdformat11-dev \
    build-essential gdb cmake git \
    openssl libssl-dev \
    libmysqlclient-dev

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
 
 # Clone and compile EIPScanner
RUN git clone https://github.com/nimbuscontrols/EIPScanner.git /tmp/EIPScanner \
 && cd /tmp/EIPScanner \
 && mkdir build \
 && cd build \
 && cmake .. \
 && cmake --build . --target install \
 && rm -rf /tmp/EIPScanner

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


# Add scipy
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install scipy
RUN pip3 install depthai
RUN pip3 install --upgrade scipy

# Source folder containing EIPScanner installation
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> /home/drims/.bashrc


# Environments sources 
RUN echo "source /opt/ros/humble/setup.bash" >> /home/drims/.bashrc
RUN echo "if [ -f /home/drims/drims_ws/install/setup.bash ]; then source /home/drims/drims_ws/install/setup.bash; fi" >> /home/drims/.bashrc


# Add the check script
COPY check_script.sh /check_script.sh
RUN chmod +x check_script.sh
# Letsgo
CMD ["bash"]


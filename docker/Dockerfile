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

CMD ["bash"]

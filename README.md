
# DRIMS2 Docker Image

This repository contains the Dockerfiles to build the Docker image for the DRIMS2 summer school, a set of scripts to start the container and check the installation, and an example node to process camera data.

## How to Get Ready for DRIMS2 Summer School

To use ROS, you will need Docker. This allows you to avoid being tied to a specific Ubuntu version to satisfy the ROS-Ubuntu compatibility.

1.  First, install Docker by following the official Docker tutorial: [Install Docker on Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
2.  Test the installation by running the hello-world container: `docker run hello-world`
3.  If you get a permission denied error, add your user to the Docker group: `sudo usermod -aG docker $USER`
4.  Then reboot and try running the hello-world container again.

Once Docker is properly installed, you can get the Docker image for the DRIMS2 summer school:

1.  Clone the repository
2.  Run the check script:
    -   `./check.sh`
3.  The script will automatically download the latest Docker image and start the container, checking that all requirements are satisfied and external folders are mounted properly.
4.  If no errors are printed on the terminal, you are ready for the DRIMS2 summer school.

## Extras

The `check.sh` script is a simple script that starts the container and performs a set of checks on the environment. Other scripts and source code are provided:

-   `start.sh`: Starts the container in interactive mode
-   The `docker` folder contains the Dockerfile to build the DRIMS2 image and a script to build the image for multiple architectures using Buildx.
-   The `drims_ws` folder is the workspace used to develop new code, mounted on `/drims_ws` inside the Docker container. It contains an example node that acquires data from a camera and performs basic image processing.
- The *bags* folder is a suppoprt folder to allow bags recordings and see provided data inside the docker container. It is mounted under /bags in the docker container.

## Windows/MacOS
ROS can work inside docker on Windows and MacOS. Unfortunatly USB and network interfaces are not properly mapped from outside the docker container to inside the container. Therefore if you are not running an Ubuntu system you will require a virtual machine.
1. Follow the instructions on the Ubuntu website to [configure Virtualbox ](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
2. Follow the above steps, like if you were using Ubuntu:
  - install docker 
  - clone the repository 
  - run the check script
2. Once you have created the ubuntu virtualbox and installed all the required software you might want to enable bridged network to comunicate with other devices on the same network. To do so go to Settings->Network and select Bridged Network on the drop-down meny for you network interface
3. When you will start using real cameras for algorithm testing you will also want to enable the usb ports. To do so follow the [Luxonis Camera guide](https://docs.luxonis.com/software/depthai/manual-install/#Manual%20DepthAI%20installation-Installing%20dependencies-VirtualBox)


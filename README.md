

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
2. Run the script `setup.sh` this will add the udev rules for the Luxonis camera and add the drims2 group to your system. Once the script runs succesfully reboot the system.
3.  Run the check script:
    -   `./check.sh`
4.  The script will automatically download the latest Docker image and start the container, checking that all requirements are satisfied and external folders are mounted properly.
5.  If no errors are printed on the terminal, you are ready for the DRIMS2 summer school.

## Developing your ROS nodes
The drims_ws folder and the bag folder are mounted inside the docker container and are used for code development. All your code will go inside the drims_ws/src folder, where you can already find example nodes. 
To run your node you will have to start the container and compile the environment, to do so:
1. Run the script `start.sh`
2. Then move inside the workspace directory `cd drims_ws`
3. Compile the environment `catkin_make`

Now that you have your node compiled you can run them. Since you have only one terminal inside docker you can use `tmux` to create multiple terminals and run all the required commands. For a guide of all basic tmux commands you can reference the [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

An alternative solution is to open new terminals and connect to the running container. To do so make sure you have started the container with `start.sh`, then in a new terminal run `connect.sh`, the new terminal is now running inside the container. While you can only run the `start.sh` once, because you can only have one running container named `drims2`, you can use the `connect.sh` script as many time as you want, since you are connecting to a running container.

## Extras

The `check.sh` script is a simple script that starts the container and performs a set of checks on the environment. Other scripts and source code are provided:

-   `start.sh`: Starts the container in interactive mode
-   The `docker` folder contains the Dockerfile to build the DRIMS2 image and a script to build the image for multiple architectures using Buildx.
-   The `drims_ws` folder is the workspace used to develop new code, mounted on `/drims_ws` inside the Docker container. It contains an example node that acquires data from a camera and performs basic image processing.
- The *bags* folder is a suppoprt folder to allow bags recordings and see provided data inside the docker container. It is mounted under /bags in the docker container.

## Windows/MacOS
ROS can work inside docker on Windows and MacOS. Unfortunatly USB and network interfaces are not properly mapped from outside the docker container to inside the container. Therefore if you are not running an Ubuntu system you will require a virtual machine.
1. Follow the instructions on the Ubuntu website to [configure Virtualbox ](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
2. Once your Ubuntu Virtualbox is running, follow the above steps, like if you were using Ubuntu:
	  - install docker
	  - clone the repository
	  - run the check script
2. Once you have created the ubuntu virtualbox and installed all the required software you might want to enable bridged network to comunicate with other devices on the same network. To do so go to Settings->Network and select Bridged Network on the drop-down meny for you network interface
3. When you will start using real cameras for algorithm testing you will also want to enable the usb ports. To do so follow the [Luxonis Camera guide](https://docs.luxonis.com/software/depthai/manual-install/#Manual%20DepthAI%20installation-Installing%20dependencies-VirtualBox)

## VS-Code integration	
To write your code you can use nano/vim directly inside docker.
If you want to use any ide, they usually provide plugins to connect to a running docker container.
For VS-code: 
1. Install the Dev Containers plugin
2. Click on the green `><` icon in the bottom-left corner of the VS Code window to open the Remote - Containers menu
3. Select `Remote-Containers: Attach to Running Container...`
4. A list of running containers will appear. Select the container you want to connect to.
5. VS Code will then attach to the selected container and open a new VS Code window connected to that container.
6. You can also install the c++ and ros extensions of vs code to have autocomplete and errors hilights 

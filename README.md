# Clone the project

```bash
git clone --recurse-submodules git@github.com:tamasso-parec/DRIMS2-2025.git
```
## Simulate
 Simulation instructions are described in [the simulate.md file](./drims_ws/simulate.md)

# DRIMS2 Docker Image

This repository contains the Dockerfiles to build the Docker image for the DRIMS2 summer school together with a set of scripts to start the container and check the installation.

The first part of the guide assume you are working on some version of Linux (**highly recommended**), if you are on windows go to the end of this guide, follow the Windows instructions 



## How to Get Ready for DRIMS2 Summer School

To use ROS 2 you will need Docker. This allows you to avoid being tied to a specific Ubuntu version to satisfy the ROS–Ubuntu compatibility.

1.  First, install Docker by following the official Docker tutorial: [Install Docker on Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
2.  Test the installation by running the hello-world container: `docker run hello-world`
3.  If you get a permission denied error, add your user to the Docker group: `sudo usermod -aG docker $USER`
4.  Then reboot and try running the hello-world container again.

Once Docker is properly installed, you can get the Docker image for the DRIMS2 summer school:

1. Clone the repository
2. Run the script `setup.sh` this will add the udev rules for the Luxonis camera and add the drims2 group to your system. Once the script runs succesfully reboot the system.
3. Run the check script:
    -   `./check.sh`
4.  The script will automatically download the latest Docker image and start the container, checking that all requirements are satisfied and external folders are mounted properly.
5.  If no errors are printed on the terminal, you are ready for the DRIMS2 summer school.

## Developing your ROS 2 nodes
The `drims_ws` folder and the `bags` folder are mounted inside the Docker container and are used for code development. All your code will go inside the `drims_ws/src` folder. The workspace starts empty so you can create your own packages.
To run your node you will have to start the container and compile the environment, to do so:
1. Run the script `start.sh`
2. Then move inside the workspace directory `cd drims_ws`
3. Compile the environment with `colcon build`

Now that you have your node compiled you can run them. Since you have only one terminal inside docker you can use `tmux` to create multiple terminals and run all the required commands. For a guide of all basic tmux commands you can reference the [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

An alternative solution is to open new terminals and connect to the running container. To do so make sure you have started the container with `start.sh`, then in a new terminal run `connect.sh`, the new terminal is now running inside the container. While you can only run the `start.sh` once, because you can only have one running container named `drims2`, you can use the `connect.sh` script as many time as you want, since you are connecting to a running container.

## Extras

The `check.sh` script is a simple script that starts the container and performs a set of checks on the environment. Other scripts and source code are provided:

-   `start.sh`: Starts the container in interactive mode
-   The `docker` folder contains the Dockerfile to build the DRIMS2 image and a script to build the image for multiple architectures using Buildx.
-   The `drims_ws` folder is the workspace used to develop new code, mounted on `/drims_ws` inside the Docker container.
- The *bags* folder is a support folder to allow bags recordings and see provided data inside the docker container. It is mounted in your home, under /home/drims/bags in the docker container.

## Windows users

This whole section is dedicated to properly configure windows to connect to the Luxonis USB cameras. Follow each steps of this section, from [Enable WSL2](#enable-wsl2) to [Test USB Camera](#test-usb-camera), which you can do when you will have a real camera. 

All files required for this part can be found in the windows_users

### <a id="enable-wsl2"></a> Enable WSL2

1. Open PowerShell as Administrator:
   - Search for "PowerShell" in Start menu
   - Right-click and select "Run as administrator"

2. Install WSL and set WSL2 as default:
```
   wsl --install -d Ubuntu-22.04
```
   This command will:
   - Enable Windows Subsystem for Linux
   - Install the latest Linux kernel
   - Set WSL2 as the default version
   - Install Ubuntu 22.04

3. Restart your computer when prompted.

Note: The first installation attempt might fail. After reboot, run the command again if needed.

### <a id="initial-configuration"></a> Initial Configuration

1. When first launching Ubuntu, you'll be prompted to:
   - Create a Linux username
   - Set a password

2. Update the system:
```

   sudo apt update && sudo apt upgrade -y
```
### <a id="kenel-update"></a> Kernel Update

1. Copy the "Sources" folder containing the modified kernel to C: drive
2. Copy the .wslconfig file to C:\Users\<your_username>

For manual kernel building, refer to:
https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers

###  <a id="docker-installation"></a> Docker Installation

Follow the official Ubuntu installation guide:
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

Test the installation:
```
docker run hello-world

```
If you get a permission denied error:
```
sudo usermod -aG docker $USER

```
Then:
1. Close WSL
2. In PowerShell run `wsl --shutdown`
3. Reopen WSL and try the hello-world container again

### <a id="drims-container-setup"></a> DRIMS Container
Install the container following the provided instructions.

### <a id="usb-device-support"></a> USB Device Support

#### Windows Installation

1. Install via Windows Package Manager:
```

   winget install --interactive --exact dorssel.usbipd-win
```
   This installs:
   - usbipd service (USBIP Device Host)
   - usbipd CLI tool
   - Firewall rule for local connections

2. Verify installation:
```
   usbipd list
```

#### Ubuntu Installation

```
sudo apt update

sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip $(command -v ls /usr/lib/linux-tools/*/usbip | tail -n1) 20
```
If you get a "usbipd not found for kernel" error, in PowerShell run:
```
wsl --shutdown
wsl --update
```
#### Share Devices with WSL

##### Automatic Method (Recommended)
1. Copy attach_usb.ps1 to your PC
2. Run in PowerShell:
```
   Start-Process PowerShell -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"C:\<path_to_file>\attach_usb.ps1`""
```
**NB: This script automatically attach the usb camera to wsl and is needed because when first plugged in, Luxonis cameras are detected as standard USB cameras and Windows assigns generic USB camera drivers.**
**When the camera is initialized (via `depthai` or ROS2) the device _changes its USB profile_ and driver.**
**This causes Windows to treat it as a new device and WSL loses connection to the original USB device.**
**Without intervention, the camera becomes unavailable and requires manual USB reattachment via `usbipd` commands.**
##### Manual Method
1. List devices in PowerShell (Admin): `usbipd list`
2. Bind the device: `usbipd bind --busid <busid>`
3. Attach to WSL: `usbipd attach --wsl Ubuntu-22.04 --busid <busid>`
4. Verify in WSL:  `lsusb`

#### <a id="test-usb-camera"></a>Test USB Camera
In Ubuntu 22, start the Docker container and run:
```
ros2 launch depthai_examples stereo.launch.py

```


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

## GUIs
Docker is primarily designed to be used from the terminal, which means it doesn't natively support graphical user interfaces (GUIs). However, when working with ROS, Gazebo, and MoveIt, having a GUI can be very helpful.

### Unsafe Method (Not Recommended)
The easiest but least secure method is to grant permission to all local users, including root, to connect to the X server. This can be done by running the following command before starting your Docker container:

`xhost +local:root`

This method is not recommended as it opens up your X server to any local user, which can be a significant security risk.

### Safer Method
A safer option is to allow only the current user to connect to the X server. You can do this by running:

`xhost +si:localuser:$(whoami)`

This command limits access to the X server to your current user only, making it a more secure alternative.

### Automating X Server Access Setup
If you don't want to manually run the xhost command every time you open a terminal, you can automate it by adding the command to your .bashrc file. This way, the command will run automatically whenever you start a new terminal session.
To do this, run the following command once:

`echo "xhost +si:localuser:$(whoami) > /dev/null 2>&1" >> ~/.bashrc`

This will add the safer xhost command to your .bashrc file, ensuring it runs without displaying any output in your terminal.

Nevertheless, the start script already enable gui, so if you use the provided script you don't have to set any additional variable 






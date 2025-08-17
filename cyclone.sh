#!/bin/bash

# Add the RMW_IMPLEMENTATION environment variable to bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Add the echo of the cyclone_config.xml path to bashrc
echo 'echo $HOME/cyclone_config.xml' >> ~/.bashrc

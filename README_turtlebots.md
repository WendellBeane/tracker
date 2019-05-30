# CS69: Prey Capture - Final Project
**Developers**: Jennifer Jain and Wendell Beane

**Report**: <insert link here>

NOTE: THIS DOCUMENT IS STILL UNDER DEVELOPMENT

## Setup
These setup instructions assume that you have the ability to ssh into the Raspberry Pi controlling the Turtlebot's in the Robotics Lab at Dartmouth.

### MultiRobot Networking Setup

#### On leader (turtlebot 8):
1) Copy the package `tracker` into `catkin_ws/src`

2) Run `catkin_make` in the `catkin_ws` folder

3) Run`source devel/setup.bash` in the `catkin_ws` folder


#### On followers:

1) Copy the following lines into a bash script called `dongle.sh`. Edit it to include the corect interface name and ssid you want to connect:

2) Make the script executable and run the bash script using `sudo bash dongle.sh`

    #!/bin/bash
    systemctl stop network-manager
    ifconfig wlx000f004dbac7 down
    iwconfig wlx000f004dbac7 essid "turtle8"
    ifconfig wlx000f004dbac7 up
    dhclient -r wlx000f004dbac7
    dhclient -v wlx000f004dbac7
    iwconfig

3) Replace the contents of `/etc/wpa_supplicant/wpa_supplicant.conf` using `sudo` with the following lines (replace with another ssid if not using turtle8 as the leader:
    
    network={
      ssid="turtle8"
      psk="Robotics&7"
    }

4) Type `iwconfig` to get the correct interface name: (wlx...)

5) Add to the end of `/etc/network/interfaces` the following lines using `sudo` and the correct interface name:
    
    auto wlx000f004dbac7
    iface wlx000f004dbac7
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf

6) Run `sudo reboot`


7) In terminal run:

    export ROS_MASTER_URI=http://192.168.108.8:11311
    export ROS_IP=192.168.108.8


#### Testing
To test the configuration, on the leader (turtle8) run `ROS_NAMESPACE=leader roslaunch tracker follow.launch`. Once the launch file is running, you should be able to run `rostopic list` on the follower to get the list of topics
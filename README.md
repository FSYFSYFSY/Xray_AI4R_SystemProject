# AI For Robotics ELEN90095

Software for running the robot that is the focus of the AI For Robotics Subject

## Setting up ROS Components:

- Ensure you are running a Ubuntu 22.04 LTS either in a virtual machine or natively and install ROS2 Humble following the instructions here: https://docs.ros.org/en/humble/Installation.html
- Clone this repository locally to your computer
- Open a terminal and enter the folder using `cd [path to the repository]/ros2_ws`
- If you have not added `source /opt/ros/humble/setup.bash` to your .bashrc you must run this in each terminal before running ros related commands
- run `sudo rosdep init`
- run `rosdep update`
- run `colcon build`
- run `source install/setup.bash` (this adds the new ros commands to path after building, it also needs to be done in each new terminal so you can add it to your .bashrc however you will need to open a new terminal after using colcon build)

You should now have managed to build the packages.

### Sensor PCB Parser Node
- In order for the parser node to work, you must allow read write access on the computer running ros to the file where the sensor pcb publishes/reads data. This file is located at /dev/ttyACM0 or /dev/ttyACM1
- To do this we add a udev rule for each file
- Create a file ttyACM0.rules containing `KERNEL=="ttyACM0", MODE="0666"` at /etc/udev/rules.d/ttyACM0.rules
- Create a file ttyACM1.rules containing `KERNEL=="ttyACM1", MODE="0666"` at /etc/udev/rules.d/ttyACM1.rules

To do that we run the following commands:
- `echo 'KERNEL=="ttyACM0", MODE="0666"' | sudo tee /etc/udev/rules.d/ttyACM0.rules`
- `echo 'KERNEL=="ttyACM1", MODE="0666"' | sudo tee /etc/udev/rules.d/ttyACM1.rules`
- `sudo udevadm control --reload-rules` 
- `sudo udevadm trigger`

### Trouble Shooting:

If you encounter a build error like `Could not find package configuration file provided by "rosidl_default_generators" with any of the following names:" ...` 

The solution is to open a terminal and run `cd [path to the repository]` and then run `sudo chmod 777 -R .` to update permissions to allow full read/write/execute access.


## Setting up Pico SDK Workspace
In order to build the project in the mcu_ws you will need to run the following commands
- cd `[PATH_TO_REPO]/mcu_ws/build`
- cmake .. & make

There will now be two uf2 files within the build folder that can be flashed to each pico respectively. Optionally you may use `make ai4r-sensors-rp1` or `make ai4r-sensors-rp2` to only build the files for respective pico.

To flash the uf2 file to the pico you can hold down the boot button on the pico and then plug it in which will mount it like a storage device, then you copy the uf2 file onto the mounted storage device. Alternatively you can use pico-tool from the Raspberry Pi Foundation. You can run `cmake..; make; sudo picotool load -F [filetoflash].uf2; sudo picotool reboot` from the build folder to automatically build and copy it over, if it fails you will need to manually put it into bootsel mode.

Alternatively you could try use the pico-vscode extension but importing an existing repository does not work properly at this stage due to the different build system (ninja) used!


## Setting up PCB Workspace
The PCB's were designed using KiCAD. This can be downloaded for free and then the project can be directly opened. It may require some manual relinking of the project specific libraries.


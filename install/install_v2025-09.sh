#!/bin/bash

# ============================================ #
# USER SPECIFICATIONS

# NOTE THE FOLLOWING
# > A true/false flag will ONLY be processed as
#   true if it is set to exactly the string
#   "true".

# PIP INSTALLS AS SYSTEM-WIDE OR FOR THE USER
SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE="false"

# DISPLAY SD CARD CLONE COMMANDS
SHOULD_DISPLAY_SD_CARD_CLONE_COMMANDS="false"

# ROS
SHOULD_ROS_INSTALL="false"
ROS_CONFIGURATION="desktop"
# > Options for config (see here for details: )
# "desktop-full"  (Recommended for a GUI-based operating system on a "normal" machine)(Includes: ROS, RViz, demos, tutorials.)
# "ros-base"      (Recommended for a "headless" operating system, i.e., when running ubuntu server)(Includes: Communication libraries, message packages, command line tools. No GUI tools.)

# FOXGLOVE
SHOULD_FOXGLOVE_BRIDGE_INSTALL="false"

# PYTHON3 PIP
SHOULD_PYTHON3_PIP_INSTALL="false"

# OPENCV
SHOULD_OPENCV_INSTALL="false"
OPENCV_CONFIGURATION="python-headless"
# > Options for config (see here for details: https://pypi.org/project/opencv-python/)
# "python"                  Main modules package
# "contib-python"           Full package (contains both main modules and contrib/extra modules
# "python-headless"         Headless main modules package
# "contib-python-headless"  Headless full package (contains both main modules and contrib/extra modules)

# DEPTH AI
SHOULD_DEPTHAI_V2_DEPENDENCIES_INSTALL="false"
SHOULD_DEPTHAI_V2_INSTALL="false"
SHOULD_DEPTHAI_V3_INSTALL="false"

# RPLIDAR
SHOULD_RPLIDAR_CLONE_AND_BUILD_PACKAGE="false"

# I2C Install
SHOULD_I2C_INSTALL_AND_SETUP="false"

# GPIOD INSTALL
SHOULD_GPIOD_INSTALL_AND_SETUP="false"

# PICO SDK DEPENDENCIES
SHOULD_PICO_SDK_INSTALL_DEPENDENCIES="false"

# PICO SDK
SHOULD_PICO_SDK_CLONE_AND_SETUP="false"

# PICO SDK
SHOULD_PICOTOOL_CLONE_AND_BUILD_AND_INSTALL="false"

# AI4R-SYSTEM CLONE AND CONFIG AND BUILD
SHOULD_AI4R_SYSTEM_CLONE="false"
SHOULD_AI4R_SYSTEM_BRANCH="main"

SHOULD_AI4R_SYSTEM_CONFIG="false"
SHOULD_AI4R_SYSTEM_BUILD_ROS_WORKSPACE="false"
SHOULD_AI4R_SYSTEM_BUILD_PICO_WORKSPACE="false"

# SET HOMENAME
SHOULD_HOSTNAME_SET="false"
HOSTNAME_TO_SET="asc33-rpi"

# END OF: USER SPECIFICATIONS
# ============================================ #



# ============================================ #
# GET HOME DIRECTORY (AND EXIT IF ANY PROBLEMS)
# >> The environment variable "EUID" is the "effective user ID"
# >> The "EUID" is 0 when a script is run as root
if [[ "${EUID}" -eq 0 ]]
then
	echo ""
	echo "ERROR: this installation script is NOT allowed to be"
	echo "       run by the root user. Either login as a different"
	echo "       user, or do not use \"sudo\" to call this"
	echo "       installation script."
	echo ""
	exit 1
else
	# Set a variable for the home directory
	# >> The "HOME" environment variable almost always exits
	# >> Setting this to "USER_HOME" is for easier tracability
	#    within this script
	# >> The ":?" part is to cause the script to exit if the
	#    "HOME" variable happens to be empty
	USER_HOME="${HOME:?}"
fi
# END OF: GET HOME DIRECTORY (AND EXIT IF ANY PROBLEMS)
# ============================================ #



# ============================================ #
# SET STRICT EXECUTION IF YOU WISH
set -euo pipefail
# -e : stop script on errors,
# -u : stop script on unset vars,
# -o pipefall : stop on hidden pipeline failures.
# ============================================ #



# ============================================ #
# GET THE UBUNTU VERSION DETAILS
# > By using regular expressions to on the file
#     /etc/os-release
# > See the following website for details and
#   testing of regular expressions:
#   https://regex101.com/

# > Get the Ubuntu major version number
UBUNTU_VERSION_MAJOR="$(grep -o --perl-regexp '(?<=VERSION_ID=")(\d{2,2})(?=\.\d{2,2}")' /etc/os-release)"

# > Get the Ubuntu minor version number
UBUNTU_VERSION_MINOR="$(grep -o --perl-regexp '(?<=VERSION_ID="\d{2,2}\.)(\d{2,2})(?=")' /etc/os-release)"

# > Get the Ubuntu codename
UBUNTU_VERSION_CODENAME="$(grep -o --perl-regexp '(?<=VERSION_CODENAME=)(\w{0,})' /etc/os-release)"

# > Display the details about the Ubuntu version
echo ""
echo "According to the file /etc/os-release,"
echo "the Ubuntu operating system version is:"
echo "${UBUNTU_VERSION_MAJOR}.${UBUNTU_VERSION_MINOR} (${UBUNTU_VERSION_CODENAME})"

# END OF: GET THE UBUNTU VERSION DETAILS
# ============================================ #



# ============================================ #
# CONVERT THE UBUNTU VERSION TO ITS MATCHING ROS VERSION
case "${UBUNTU_VERSION_MAJOR}" in
	20)
		ROS_VERSION_CODENAME="foxy"
		;;
	22)
		ROS_VERSION_CODENAME="humble"
		;;
	24)
		ROS_VERSION_CODENAME="jazzy"
		;;
	*)
		echo ""
		echo ""
		echo "[ERROR] The ROS installation cannot be completed because"
		echo "        Ubuntu ${UBUNTU_VERSION_MAJOR}.${UBUNTU_VERSION_MINOR} is NOT supported by this"
		echo "        installation script."
		echo ""
		echo ""
		SHOULD_ROS_INSTALL="false"
		ROS_VERSION_CODENAME="none"
		;;
esac
# END OF: CONVERT THE UBUNTU VERSION TO ITS MATCHING ROS VERSION
# ============================================ #



# ============================================ #
# DISPLAY SD CARD CLONE COMMANDS
if [[ ${SHOULD_DISPLAY_SD_CARD_CLONE_COMMANDS} = "true" ]]
then
	echo ""
	echo "========================================="
	echo "BEGINNING: DISPLAY SD CARD CLONE COMMANDS"
	echo ">> These steps are taken from:"
	echo "   https://askubuntu.com/questions/227924/sd-card-cloning-using-the-dd-command#631758"

	echo "LIST:   sudo fdisk -l"
	echo "UNMONT: sudo umount /dev/mmcblk0"
	echo "CLONE:  sudo dd if=/dev/mmcblk0 of=~/sd-card-copy.img bs=4M status=progress"

	echo ""
	echo "FINISHED: DISPLAY SD CARD CLONE COMMANDS"
	echo "========================================"
fi
# END OF: DISPLAY SD CARD CLONE COMMANDS
# ============================================ #

# ============================================ #
# ROS INSTALLATION
if [[ ${SHOULD_ROS_INSTALL} = "true" ]]
then
	echo ""
	echo "==========================="
	echo "BEGINNING: ROS INSTALLATION"
	echo ">> These steps are taken from:"
	echo "   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"

	echo ""
	echo ">> Now setting the locale"
	sudo apt update && sudo apt install --yes locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	echo ""
	echo ">> Now enabling required repositories"

	# Ensure that the Ubuntu Universe repository is enabled.
	sudo apt install --yes software-properties-common
	sudo add-apt-repository universe

	# The ros-apt-source packages provide keys and apt source
	# configuration for the various ROS repositories.
	# - Installing the ros2-apt-source package will configure
	#   ROS 2 repositories for your system. Updates to repository
	#   configuration will occur automatically when new versions
	#   of this package are released to the ROS repositories.
	sudo apt update && sudo apt install --yes curl
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
	sudo dpkg -i /tmp/ros2-apt-source.deb

	# Update your apt repository caches after setting up the
	# repositories.
	sudo apt update
	# ROS 2 packages are built on frequently updated Ubuntu
	# systems. It is always recommended that you ensure your
	# system is up to date before installing new packages.
	sudo apt upgrade --yes

	echo ""
	echo ">> Now installing the development tools"
	# If you are going to build ROS packages or otherwise do
	# development, you can also install the development tools.
	sudo apt install --yes ros-dev-tools

	echo ""
	echo ">> Now installing ROS2 (specifically, package ros-${ROS_VERSION_CODENAME}-${ROS_CONFIGURATION})"
	sudo apt install --yes ros-${ROS_VERSION_CODENAME}-${ROS_CONFIGURATION}

	echo ""
	echo ">> Now sourcing the environment"
	# Set up your environment by sourcing the following file.
	source /opt/ros/${ROS_VERSION_CODENAME}/setup.bash

	# Add the ROS environment setup to the .bashrc
	# > Note: added together with a description in comments
	echo ""
	echo ">> Now adding the sourcing of the ROS2 environment variables to the ~/.bashrc file"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# SOURCE THE ROS setup.bash FILE" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "source /opt/ros/${ROS_VERSION_CODENAME}/setup.bash" >> ${USER_HOME}/.bashrc

	echo ""
	echo "FINISHED: ROS INSTALLATION"
	echo "=========================="
fi
# END OF: ROS INSTALLATION
# ============================================ #



# ============================================ #
# FOXGLOVE BRIDGE INSTALLATION
if [[ ${SHOULD_FOXGLOVE_BRIDGE_INSTALL} = "true" ]]
then
	echo ""
	echo "======================================="
	echo "BEGINNING: FOXGLOVE BRIDGE INSTALLATION"
	echo ">> These steps are taken from:"
	echo "   https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge"

	echo ""
	echo ">> Now installing Foxglove bridge"
	sudo apt update
	sudo apt install --yes ros-${ROS_VERSION_CODENAME}-foxglove-bridge

	echo ""
	echo "FINISHED: FOXGLOVE BRIDGE INSTALLATION"
	echo "======================================"
fi
# END OF: FOXGLOVE INSTALLATION
# ============================================ #



# ============================================ #
# PYTHON3 PIP INSTALLATION
if [[ ${SHOULD_PYTHON3_PIP_INSTALL} = "true" ]]
then
	echo ""
	echo "==================================="
	echo "BEGINNING: PYTHON3 PIP INSTALLATION"
	echo ">> These steps will:"
	echo "   >> Install python3-pip package from apt"
	echo "   >> Upgrade pip"
	echo "   >> Set the --break-system-packages flag as necessary (due to PEP 668 restrictions in Ubuntu 22.04 and later)"

	echo ""
	echo ">> Now installing \"python3-pip\""
	sudo apt update
	sudo apt install --yes python3-pip

	echo ""
	echo ">> Now upgrading pip (with --break-system-packages flag if necessary)"
	if [[ ${SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE} = "true" ]]
	then
		sudo python3 -m pip install --upgrade pip --break-system-packages
	else
		python3 -m pip install --upgrade pip --break-system-packages
	fi

	# Notes:
	# > On older Ubuntu (20.04 and earlier), --break-system-packages doesn’t exist, hence this might fail.
	# > On newer Ubuntu (22.04 and later), PEP 668 restrictions protect from global installs,
	#   hence need the --break-system-packages flag to install globally.

	echo ""
	echo ">> Now writing out the currently installed python packages for record keeping"
	CURRENT_TIME_STAMP=$(date +"%Y-%m-%d_%H-%M-%S")
	PIP_FREEZE_OUTPUT_FILE="pip_freeze_output_as_at_${STAMP}.txt"
	python3 -m pip freeze > "${PIP_FREEZE_OUTPUT_FILE}"
	echo ">> pip freeze output written to: ${PIP_FREEZE_OUTPUT_FILE}"

	echo ""
	echo "FINISHED: PYTHON3 PIP INSTALLATION"
	echo "=================================="

	# Notes for returning to the "current" status
	# List into a file the currently installed packages:
	#    pip freeze > requirements.txt
	# Recreate the system-wide environment:
	#    sudo python3 -m pip install -r requirements.txt --break-system-packages
fi
# END OF: OPENCV PYTHON INSTALLATION
# ============================================ #



# ============================================ #
# OPENCV PYTHON INSTALLATION
if [[ ${SHOULD_OPENCV_INSTALL} = "true" ]]
then
	echo ""
	echo "====================================="
	echo "BEGINNING: OPENCV PYTHON INSTALLATION"
	echo ">> These steps are taken from:"
	echo "   https://pypi.org/project/opencv-python/"
	echo ">> Full OpenCV documentation here:"
	echo "   https://docs.opencv.org/4.x/index.html"

	# Notes:
	# Should run a pip upgrade before OpenCV installation
	# This is taken care of by the "python3 pip" installation block above

	echo ""
	echo ">> Now upgrading SciPy to ensure compatibility with NumPy 2.x"
	echo ">> (This is required because the OpenCV installation will force upgrade to NumPy 2.x"
	if [[ ${SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE} = "true" ]]
	then
		sudo python3 -m pip install --upgrade scipy --break-system-packages
	else
		python3 -m pip install --upgrade scipy --break-system-packages
	fi

	echo ""
	echo ">> Now installing OpenCV"
	if [[ ${SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE} = "true" ]]
	then
		sudo python3 -m pip install opencv-${OPENCV_CONFIGURATION} --break-system-packages
	else
		python3 -m pip install opencv-${OPENCV_CONFIGURATION} --break-system-packages
	fi

	# Notes:
	# > On older Ubuntu (20.04 and earlier), --break-system-packages doesn’t exist, hence this might fail.
	# > On newer Ubuntu (22.04 and later), PEP 668 restrictions protect from global installs,
	#   hence need the --break-system-packages flag to install globally.

	echo ""
	echo ">> Now testing the OpenCV installation using the command:"
	echo "   python3 -c \"import cv2; print('OpenCV version:', cv2.__version__)\""
	echo ">> You should now see an output of the OpenCV version:"
	python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"

	echo ""
	echo "FINISHED: OPENCV PYTHON INSTALLATION"
	echo "===================================="

	# Notes for version switching
	# To uninstall:
	#    python3 -m pip uninstall opencv-{OLD_CONFIGURATION}
fi
# END OF: OPENCV PYTHON INSTALLATION
# ============================================ #



# ============================================ #
# DEPTHAI V2 DEPENDENCIES INSTALLATION
if [[ ${SHOULD_DEPTHAI_V2_DEPENDENCIES_INSTALL} = "true" ]]
then
	echo ""
	echo "==============================================="
	echo "BEGINNING: DEPTHAI V2 DEPENDENCIES INSTALLATION"
	echo ">> These steps will install DepthAI (v2) via pip"
	echo ">> Reference for v2 installation: https://docs.luxonis.com/software/depthai/manual-install/"
	echo ">> Reference for dependencies: https://raw.githubusercontent.com/luxonis/depthai-python/refs/heads/main/docs/install_dependencies.sh"

	# According to the dependencies script, should install all of the following
	# on a Raspberry Pi:

	echo ""
	echo ">> Now installing dependencies for DepthAI (v2)"

	# LINUX PACKAGES
	# python3
    # python3-pip
    # udev
    # cmake
    # git
    # python3-numpy

	# DEBIAN ARM PACKAGES
	# # https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
    # build-essential
    sudo apt install --yes libgtk2.0-dev
    # pkg-config
    # libavcodec-dev
    # libavformat-dev
    # libswscale-dev
    # python3-dev
    # libtbb-dev
    # libjpeg-dev
    # libpng-dev
    # libtiff-dev
    # # https://stackoverflow.com/questions/55313610
    sudo apt install --yes ffmpeg
    # libsm6
    # libxext6
    # python3-pyqt5
    sudo apt install --yes python3-pyqt5.qtquick
    sudo apt install --yes qml-module-qtquick-controls2
    sudo apt install --yes qml-module-qt-labs-platform
    # qtdeclarative5-dev
    sudo apt install --yes qml-module-qtquick2
    # qtbase5-dev
    # qtchooser
    # qt5-qmake
    # qtbase5-dev-tools
    sudo apt install --yes qml-module-qtquick-layouts
    sudo apt install --yes qml-module-qtquick-window2
    # # https://stackoverflow.com/a/53402396/5494277
    # libhdf5-dev
    # libhdf5-dev
    sudo apt install --yes libatlas-base-dev
    # # https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi/issues/18#issuecomment-433953426
    # libilmbase-dev # <--- PACKAGE NOT FOUND!
    # libopenexr-dev
    sudo apt install --yes libgstreamer1.0-dev

	# DEBIAN PACKAGES 23
	# libdc1394-dev
    # libgl1-mesa-dev
    # libtbbmalloc2

	# JASPBER (for JPEG-200)
	# libjasper-dev # <--- PACKAGE NOT FOUND!

	# REMOVE uvcdynctrl
	# uvcdynctrl # <--- PACKAGE ALREADY NOT INSTALLED
	
	echo ""
	echo "FINISHED: DEPTHAI V2 DEPENDENCIES INSTALLATION"
	echo "=============================================="
fi
# END OF: DEPTHAI V2 DEPENDENCIES INSTALLATION
# ============================================ #



# ============================================ #
# DEPTHAI V2 PYTHON INSTALLATION
if [[ ${SHOULD_DEPTHAI_V2_INSTALL} = "true" ]]
then
	echo ""
	echo "========================================="
	echo "BEGINNING: DEPTHAI V2 PYTHON INSTALLATION"
	echo ">> These steps will install DepthAI (v2) via pip"
	echo ">> Reference for v2 installation: https://docs.luxonis.com/software/depthai/manual-install/"
	echo ">> Reference for dependencies: https://raw.githubusercontent.com/luxonis/depthai-python/refs/heads/main/docs/install_dependencies.sh"

	# Notes:
	# Should run a pip upgrade before DepthAI installation
	# This is taken care of by the "python3 pip" installation block above

	echo ""
	echo ">> Now installing DepthAI (v2)"
	if [[ ${SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE} = "true" ]]
	then
		sudo python3 -m pip install --pre "depthai>=2,<3" --force-reinstall --break-system-packages
	else
		python3 -m pip install "depthai>=2,<3" --force-reinstall --break-system-packages
	fi

	# Notes for version switching
	# To uninstall DepthAI v3:
	#    python3 -m pip uninstall depthai --break-system-packages
	#
	# To install DepthAI v2 instead:
	#    python3 -m pip install depthai==2.*
	#
	# To list all available versions:
	#    pip index versions depthai

	# Notes:
	# > On older Ubuntu (20.04 and earlier), --break-system-packages doesn’t exist, hence this might fail.
	# > On newer Ubuntu (22.04 and later), PEP 668 restrictions protect from global installs,
	#   hence need the --break-system-packages flag to install globally.

	echo ""
	echo "FINISHED: DEPTHAI V2 PYTHON INSTALLATION"
	echo "========================================"
fi
# END OF: DEPTHAI V2 PYTHON INSTALLATION
# ============================================ #


# ============================================ #
# DEPTHAI V3 PYTHON INSTALLATION
if [[ ${SHOULD_DEPTHAI_V3_INSTALL} = "true" ]]
then
	echo ""
	echo "========================================="
	echo "BEGINNING: DEPTHAI V3 PYTHON INSTALLATION"
	echo ">> These steps will install DepthAI (v3) via pip"
	echo ">> Reference for v3 installation: https://docs.luxonis.com/software-v3/depthai/"
	echo ">> Reference for v2 installation: https://docs.luxonis.com/software/depthai/manual-install/"
	echo ">> Reference for udev rule: https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide"
	echo ">> Reference for code to display all cameras: https://docs.luxonis.com/software-v3/depthai/examples/camera/camera_all/"

	# Notes:
	# Should install apt package "libusb-1.0-0"
	# This is likely already installed, which you can check with:
	#     apt list --installed libusb*

	# Notes:
	# Should run a pip upgrade before DepthAI installation
	# This is taken care of by the "python3 pip" installation block above

	echo ""
	echo ">> Now installing DepthAI (v3)"
	if [[ ${SHOULD_PREFORM_PIP_INSTALLS_AS_SYSTEM_WIDE} = "true" ]]
	then
		sudo python3 -m pip install --pre depthai --force-reinstall --break-system-packages
	else
		python3 -m pip install --pre depthai --force-reinstall --break-system-packages
	fi

	# Notes for version switching
	# To uninstall DepthAI v3:
	#    python3 -m pip uninstall depthai --break-system-packages
	#
	# To install DepthAI v2 instead:
	#    python3 -m pip install depthai==2.*

	# Notes:
	# > On older Ubuntu (20.04 and earlier), --break-system-packages doesn’t exist, hence this might fail.
	# > On newer Ubuntu (22.04 and later), PEP 668 restrictions protect from global installs,
	#   hence need the --break-system-packages flag to install globally.

	# Ensure "plugdev" group exists
	if getent group plugdev >/dev/null; then
		echo ">> Group \"plugdev\" already exists"
	else
		echo ">> Now creating the \"plugdev\" group"
		sudo groupadd plugdev
	fi

	# Ensure the user is in the "plugdev" group
	if id -nG "$USER" | grep -qw plugdev; then
		echo ">> User \"$USER\" is already in the \"plugdev\" group"
	else
		echo ">> Now adding user \"$USER\" to the \"plugdev\" group"
		sudo usermod -aG plugdev "$USER"
		echo ""
		echo ">> NOTE: You need to log-out and back in (or reboot) for the \"plugedv\" group addition to take effect."
	fi


	echo ""
	echo ">> Now configuring a symbolic link for the OAK device by"
	echo ">> configuring a udev rule in file \"/etc/udev/rules.d/80-movidius.rules\""
	echo ">> (Vendor IDs: 03e7 = Intel Movidius, 2bc5 = Luxonis boot/DFU)"
	echo "# Configure the OAK device port be a fixed symbolic link" | sudo tee /etc/udev/rules.d/80-movidius.rules
	echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE:=\"0666\", GROUP=\"plugdev\"" | sudo tee -a /etc/udev/rules.d/80-movidius.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

	# Notes:
	# To check the details of plugged in USB devices, use the command:
	#     lsusb | egrep -i 'movidius|luxonis|03e7|2bc5'

	echo ""
	echo ">> Now reloading, restarting, and triggering \"udev\":"
	echo ""
	sudo service udev reload
	sudo service udev restart
	sudo udevadm control --reload-rules && sudo udevadm trigger

	echo ""
	echo ">> NOTE: you need to unplug and replug the OAK device to apply the new rule."

	echo ""
	echo ">> Now testing the DepathAI installation using the command:"
	echo "   python3 -c \"import depthai as dai, sys, inspect; print(\"DepthAI version:\", dai.__version__)\""
	echo ">> You should now see an output of the DepthAI version (plus a few other details in the check):"
	python3 -c 'import depthai as dai, sys, inspect; print("DepthAI version:", dai.__version__); print("DepthAI module path:", inspect.getfile(dai)); print("Python executable:", sys.executable)'

	echo ""
	echo "FINISHED: DEPTHAI V3 PYTHON INSTALLATION"
	echo "========================================"
fi
# END OF: DEPTHAI V3 PYTHON INSTALLATION
# ============================================ #



# ============================================ #
# RPLIDAR DEVICE 
if [[ ${SHOULD_RPLIDAR_CLONE_AND_BUILD_PACKAGE} = "true" ]]
then
	echo ""
	echo "=================================="
	echo "BEGINNING: RPLIDAR CLONE AND BUILD"
	echo ">> These steps are taken from:"
	echo "   https://github.com/Slamtec/rplidar_ros/tree/ros2"
	echo ">> Note that we use the \"ros2\" branch."

	echo ""
	echo ">> Now creating a ROS2 workspace directory for the RPLidar package"
	mkdir -p ${USER_HOME}/rplidar_ws/src
	
	echo ""
	echo ">> Now cloning from github the \"ros2\" branch of the \"rplidar_ros\" package"
	cd ${USER_HOME}/rplidar_ws/src
	git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

	echo ""
	echo ">> Now calling \"colcon build\" for the \"rplidar_ros\" package"
	cd ${USER_HOME}/rplidar_ws/
	colcon build --symlink-install

	echo ""
	echo ">> Now sourcing the \"install/setup.bash\" of the \"rplidar_ros\" package"
	source ${USER_HOME}/rplidar_ws/install/setup.bash

	# Add the ROS package setup to the .bashrc
	# > Note: added together with a description in comments
	echo ""
	echo ">> Now adding the sourcing of the \"rplidar_ros\" package to the ~/.bashrc file"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# SOURCE THE \"rplidar_ros\" package setup.bash FILE" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "source ${USER_HOME}/rplidar_ws/install/setup.bash" >> ${USER_HOME}/.bashrc

	echo ""
	echo ">> Now configuring a symbolic link for the RPLidar device by"
	echo ">> configuring a udev rule in file \"/etc/udev/rules.d/rplidar.rules\""
	echo ""
	echo "# Configure the rplidar device port be a fixed symbolic link" | sudo tee /etc/udev/rules.d/rplidar.rules
	echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0777\", SYMLINK+=\"rplidar\"" | sudo tee -a /etc/udev/rules.d/rplidar.rules

	echo ""
	echo ">> Now reloading, restarting, and triggering \"udev\":"
	echo ""
	sudo service udev reload
	sudo service udev restart
	sudo udevadm control --reload && sudo udevadm trigger

	echo ""
	echo ">> NOTE: you need to unplug and replug the RPLidar device to apply the new rule."
	echo ">> NOTE: To test that the \"rplidar_ros\" package is working, launch a node with the following:"
	echo ">>     ros2 launch rplidar_ros rplidar_a1_launch.py"

	echo ""
	echo "FINISHED: RPLIDAR CLONE AND BUILD"
	echo "================================="
fi
# END OF: RPLIDAR PACKAGE
# ============================================ #



# ============================================ #
# I2C INSTALL AND SETUP

if [[ ${SHOULD_I2C_INSTALL_AND_SETUP} = "true" ]]
then
	echo ""
	echo "=================================="
	echo "BEGINNING: I2C INSTALL AND SETUP"

	# Install the "i2c-tools" pacakge
	echo ""
	echo ">> Now installing \"i2c-tools\" package:"
	echo ""
	sudo apt -y install i2c-tools

	# Install the "libi2c-dev" pacakge
	echo ""
	echo ">> Now installing \"libi2c-dev\" package:"
	echo ""
	sudo apt -y install libi2c-dev

	# Add the logged in user to the "i2c" group
	echo ""
	echo ">> Now adding you (i.e., the ${USER} user) to the \"i2c\" group."
	sudo usermod -a -G i2c ${USER}

	# Add a "udev" rule to give the "i2c" group access
	# to "i2c-dev1"
	# > First check if there is already such a rule
	if [[ -f "/etc/udev/rules.d/50-i2c.rules" ]]
	then
		echo ""
		echo ">> A \"udev\" rule named \"50-i2c.rules\" already exists and has the contents:"
		echo ""
		cat /etc/udev/rules.d/50-i2c.rules

		echo ""
		echo ">> This file will be removed"
		sudo rm /etc/udev/rules.d/50-i2c.rules
	fi

	echo ""
	echo ">> Now adding a \"udev\" rule named \"50-i2c.rules\" with the following content:"

	# > Add the "udev" rule
	echo "# udev rules for giving i2c bus access to the i2c group" | sudo tee -a /etc/udev/rules.d/50-i2c.rules
	echo "# This allows use of certain i2c tools and libi2c functions without sudo" | sudo tee -a /etc/udev/rules.d/50-i2c.rules
	echo "SUBSYSTEM==\"i2c\", KERNEL==\"i2c-1\", GROUP=\"i2c\", MODE=\"0660\"" | sudo tee -a /etc/udev/rules.d/50-i2c.rules

	# > Display the "udev" rule that was added
	echo ""
	echo ">> As a double check, the full contents of \"50-i2c.rules\" is now:"
	cat /etc/udev/rules.d/50-i2c.rules

	# Inform the user, and check that the installation worked
	echo ""
	echo ">> The I2C congiruation is now complete."
	echo ">> Now checking the installation was successful by running the command:"
	echo ">> sudo i2cdetect -y -r 1"
	echo ""
	sudo i2cdetect -y -r 1

	echo ""
	echo "FINISHED: I2C INSTALL AND SETUP"
	echo "==============================="
fi
# END OF: I2C INSTALL AND SETUP
# ============================================ #



# ============================================ #
# GPIOD INSTALL AND SETUP
if [[ ${SHOULD_GPIOD_INSTALL_AND_SETUP} = "true" ]]
then
	echo ""
	echo "=================================="
	echo "BEGINNING: GPIOD INSTALL AND SETUP"

	echo ""
	echo ">> Now installing \"gpiod\" package:"
	echo ""
	sudo apt --yes install gpiod
	
	# Install the "libgpiod-dev" pacakge
	echo ""
	echo ">> Now installing \"libgpiod-dev\" package:"
	echo ""
	sudo apt --yes install libgpiod-dev

	echo ""
	echo ">> Now installing \"libgpiod-doc\" package:"
	echo ""
	sudo apt --yes install libgpiod-doc

	echo ""
	echo ">> Now creating a new user group named \"gpiod\"."
	sudo groupadd gpiod

	echo ""
	echo ">> Now adding you (i.e., the ${USER} user) to the \"gpiod\" group."
	sudo usermod -a -G gpiod ${USER}

	echo ""
	echo ">> The following is a list of all groups that the ${USER} user currently belongs to:"
	groups ${USER}

	echo ""
	echo ">> The GPIO installation (and configuration) is now complete."
	echo ">> Now checking the installation was successful by running the command:"
	echo ">> sudo gpiodetect"
	echo ""
	sudo gpiodetect


	echo ""
	echo "FINISHED: GPIOD INSTALL AND SETUP"
	echo "================================="
fi
# END OF: GPIOD INSTALL AND SETUP
# ============================================ #



# ============================================ #
# PICO SDK INSTALL DEPENDENCIES
if [[ ${SHOULD_PICO_SDK_INSTALL_DEPENDENCIES} = "true" ]]
then
	echo ""
	echo "========================================"
	echo "BEGINNING: PICO SDK INSTALL DEPENDENCIES"
	echo ">> These steps are taken from:"
	echo "   https://github.com/raspberrypi/pico-sdk"
	echo ">> Note that we use the \"main\" branch."
	echo ">> In the Readme, see:"
	echo "   >> Section \"Quick-start your own project\""
	echo "   >> Then sub-section \"Unix command line\""
	echo "   >> Then step 1"

	echo ""
	echo ">> Now installatin the apt packages necessary for the pico-sdk"
	echo ">> These packages are:"
	echo "   cmake"
	echo "   python3"
	echo "   build-essential"
	echo "   gcc-arm-none-eabi"
	echo "   libnewlib-arm-none-eabi"
	echo "   libstdc++-arm-none-eabi-newlib"
	echo ">> (This can take approx 20 mins on a Raspberry Pi 4)"
	echo ""
	sudo apt update
	echo ""
	sudo apt upgrade --yes
	echo ""
	sudo apt install --yes cmake build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

	echo ""
	echo "FINISHED: PICO SDK INSTALL DEPENDENCIES"
	echo "======================================="
fi
# END OF: PICO SDK INSTALL DEPENDENCIES
# ============================================ #


# ============================================ #
# PICO SDK CLONE AND SETUP
if [[ ${SHOULD_PICO_SDK_CLONE_AND_SETUP} = "true" ]]
then
	echo ""
	echo "==================================="
	echo "BEGINNING: PICO SDK CLONE AND SETUP"
	echo ">> These steps are loosely taken from:"
	echo "   https://github.com/raspberrypi/pico-sdk"

	echo ""
	echo ">> Now cloning the \"pico-sdk\" repository"
	echo ""
	cd ${USER_HOME}
	git clone https://github.com/raspberrypi/pico-sdk

	echo ""
	echo ">> Now initializing the necessary submodules of \"lib/mbedtls\" and \"lib/tinyusb\""
	echo ""
	cd ${USER_HOME}/pico-sdk
	git submodule update --init lib/mbedtls
	echo ""
	git submodule update --init lib/tinyusb

	echo ""
	echo ">> Now adding the \"PICO_SDK_PATH\" environment variable to the ~/.bashrc file"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# Path to the clone of the Pico SDK repository" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "export PICO_SDK_PATH=${USER_HOME}/pico-sdk" >> ${USER_HOME}/.bashrc

	echo ""
	echo "FINISHED: PICO SDK CLONE AND SETUP"
	echo "=================================="
fi
# END OF: PICO SDK INSTALL DEPENDENCIES
# ============================================ #



# ============================================ #
# PICOTOOL CLONE AND BUILD AND INSTALL
if [[ ${SHOULD_PICOTOOL_CLONE_AND_BUILD_AND_INSTALL} = "true" ]]
then
	echo ""
	echo "==============================================="
	echo "BEGINNING: PICOTOOL CLONE AND BUILD AND INSTALL"
	echo ">> These steps are loosely taken from:"
	echo "   https://github.com/raspberrypi/picotool"
	echo ">> Details in the \"BUILDING.md\" file."

	echo ""
	echo ">> Now cloning the \"picotool\" repository"
	echo ""
	cd ${USER_HOME}
	git clone https://github.com/raspberrypi/picotool

	echo ""
	echo ">> Now creating the build folder within the picotool clone"
	echo ""
	cd ${USER_HOME}/picotool
	mkdir build
	cd build

	echo ""
	echo ">> Now calling \"cmake\" from within the build folder"
	echo ""
	cd ${USER_HOME}/picotool/build
	cmake ..

	echo ""
	echo ">> Now calling \"make\" from within the build folder"
	echo ""
	cd ${USER_HOME}/picotool/build
	make

	echo ""
	echo ">> Now copying udev rules from the picotool respository into \"/etc/udev/rules.d/\""
	cd ${USER_HOME}/picotool/
	sudo cp udev/60-picotool.rules /etc/udev/rules.d/

	echo ""
	echo ">> Now installing \"picotool\" so that the Pico SDK can find (and so we can use it from command line)"
	echo ""
	cd ${USER_HOME}/picotool/build
	cmake -DCMAKE_INSTALL_PREFIX=~/.local ..
	echo ""
	make install

	echo ""
	echo "FINISHED: PICOTOOL CLONE AND BUILD AND INSTALL"
	echo "=============================================="
fi
# END OF: PICOTOOL CLONE AND BUILD AND INSTALL
# ============================================ #



# ============================================ #
# AI4R SYSTEM CLONE
if [[ ${SHOULD_AI4R_SYSTEM_CLONE} = "true" ]]
then
	echo ""
	echo "======================================="
	echo "BEGINNING: AI4R SYSTEM CLONE"

	# Make the ai4r directory under the users home directory
	# > Note: the -p option means: no error if existing, make parent directories as needed
	#echo ""
	#echo ">> Now creating, if necessary, the directory: ${USER_HOME}"
	#mkdir -p ${USER_HOME}

	# Change directory
	cd ${USER_HOME}

	# Clone the asclinic-system git repository
	echo ""
	echo ">> Now cloning the \"ai4r-system\" repository"
	echo ""
	if [[ ${SHOULD_AI4R_SYSTEM_BRANCH} = "main" ]]
	then
		git clone https://gitlab.unimelb.edu.au/ai4r/ai4r-system.git
	else
		git clone -b ${SHOULD_AI4R_SYSTEM_BRANCH} https://gitlab.unimelb.edu.au/ai4r/ai4r-system.git
	fi

	echo ""
	echo "FINISHED: AI4R SYSTEM CLONE"
	echo "==========================="
fi
# END OF: AI4R SYSTEM CLONE
# ============================================ #



# ============================================ #
# AI4R SYSTEM CLONE AND CONFIG
if [[ ${SHOULD_AI4R_SYSTEM_CONFIG} = "true" ]]
then
	echo ""
	echo "=============================="
	echo "BEGINNING: AI4R SYSTEM CONFIG"

	# Add the ROS package setup to the .bashrc
	# > Note: added together with a description in comments
	echo ""
	echo ">> Now adding the sourcing of the \"ai4r_pkg\" package to the ~/.bashrc file"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# SOURCE THE \"ai4r_pkg\" package setup.bash FILE" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "source ${USER_HOME}/ai4r-system/ros2_ws/install/setup.bash" >> ${USER_HOME}/.bashrc

	echo ""
	echo ">> Now adding the required ROS environment variables to the ~/.bashrc file"
	echo ">> Adding:"
	echo "   #export ROS_DOMAIN_ID=1"
	echo "   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
	echo "   #export ROS_STATIC_PEERS='192.168.0.1;remote.com'"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# Set the necessary ROS environment variables" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "#export ROS_DOMAIN_ID=1" >> ${USER_HOME}/.bashrc
	echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ${USER_HOME}/.bashrc
	echo "#export ROS_STATIC_PEERS='192.168.0.1;remote.com'" >> ${USER_HOME}/.bashrc

	echo ""
	echo ">> Now adding aliases for ROS launch commands to the ~/.bashrc file"
	echo ">> Adding:"
	echo "   alias launch_foxglove='ros2 launch ai4r_pkg foxglove_bridge_launch.xml'"
	echo "   alias launch_traxxas='ros2 launch ai4r_pkg traxxas_launch.py'"
	echo "   alias launch_policy='ros2 launch ai4r_pkg policy_launch.py'"
	echo "   alias launch_stream='ros2 launch ai4r_pkg camera_stream_launch.py'"
	echo "   alias launch_detector='ros2 run ai4r_pkg cone_detector_node.py'"
	echo "   alias launch_imu='ros2 run ai4r_pkg sensor_pcb_parser'"
	echo "   alias launch_tof='ros2 launch ai4r_pkg vl53l5cx_launch.py'"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# Define aliases for the ai4r-system ROS launch commands" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "alias launch_foxglove='ros2 launch ai4r_pkg foxglove_bridge_launch.xml'" >> ${USER_HOME}/.bashrc
	echo "alias launch_traxxas='ros2 launch ai4r_pkg traxxas_launch.py'" >> ${USER_HOME}/.bashrc
	echo "alias launch_policy='ros2 launch ai4r_pkg policy_launch.py'" >> ${USER_HOME}/.bashrc
	echo "alias launch_stream='ros2 launch ai4r_pkg camera_stream_launch.py'" >> ${USER_HOME}/.bashrc
	echo "alias launch_detector='ros2 run ai4r_pkg cone_detector_node.py'" >> ${USER_HOME}/.bashrc
	echo "alias launch_imu='ros2 run ai4r_pkg sensor_pcb_parser'" >> ${USER_HOME}/.bashrc
	echo "alias launch_tof='ros2 launch ai4r_pkg vl53l5cx_launch.py'" >> ${USER_HOME}/.bashrc

	echo ""
	echo ">> Now adding aliases for convenience of running bash script to the ~/.bashrc file"
	echo ">> Adding:"
	echo "   alias build_ros2_workspace='bash ${HOME}/ai4r-system/install/build_ros_workspace.sh'"
	echo "   alias build_pico_workspace='bash ${HOME}/ai4r-system/install/build_pico_workspace.sh'"
	echo "   alias flash_picos='bash ${HOME}/ai4r-system/install/flash_picos.sh'"
	echo "   alias reboot_picos='bash ${HOME}/ai4r-system/install/reboot_picos.sh'"
	echo "" >> ${USER_HOME}/.bashrc
	echo "# Define aliases for the ai4r-system ROS launch commands" >> ${USER_HOME}/.bashrc
	echo "# (Note: this was added as part of the ai4r-system installation)" >> ${USER_HOME}/.bashrc
	echo "alias build_ros2_workspace='bash ${HOME}/ai4r-system/install/build_ros_workspace.sh'" >> ${USER_HOME}/.bashrc
	echo "alias build_pico_workspace='bash ${HOME}/ai4r-system/install/build_pico_workspace.sh'" >> ${USER_HOME}/.bashrc
	echo "alias flash_picos='bash ${HOME}/ai4r-system/install/flash_picos.sh'" >> ${USER_HOME}/.bashrc
	echo "alias reboot_picos='bash ${HOME}/ai4r-system/install/reboot_picos.sh'" >> ${USER_HOME}/.bashrc

	echo "" >> ${USER_HOME}/.bashrc

	echo ""
	echo "FINISHED: AI4R SYSTEM CONFIG"
	echo "============================"
fi
# END OF: AI4R SYSTEM CONFIG
# ============================================ #


# ============================================ #
# AI4R SYSTEM BUILD ROS WORKSPACE
if [[ ${SHOULD_AI4R_SYSTEM_BUILD_ROS_WORKSPACE} = "true" ]]
then
	# Inform the user
	echo ""
	echo "=========================================="
	echo "BEGINNING: AI4R SYSTEM BUILD ROS WORKSPACE"

	echo ""
	echo ">> Now calling \"colcon build\" for the \"ai4r_pkg\" package"
	echo ">> (This can take approx 5 mins on a Raspberry Pi 4)"
	echo ""
	cd ${USER_HOME}/ai4r-system/ros2_ws/
	colcon build --symlink-install

	echo ""
	echo "FINISHED: AI4R SYSTEM BUILD ROS WORKSPACE"
	echo "========================================="
fi
# END OF: AI4R SYSTEM BUILD ROS WORKSPACE
# ============================================ #



# ============================================ #
# AI4R SYSTEM BUILD PICO WORKSPACE
if [[ ${SHOULD_AI4R_SYSTEM_BUILD_PICO_WORKSPACE} = "true" ]]
then
	# Inform the user
	echo ""
	echo "==========================================="
	echo "BEGINNING: AI4R SYSTEM BUILD PICO WORKSPACE"
	echo ">> These steps are taken from:"
	echo "   https://github.com/raspberrypi/pico-sdk"
	echo ">> Note that we use the \"main\" branch."
	echo ">> In the Readme, see:"
	echo "   >> Section \"Quick-start your own project\""
	echo "   >> Then sub-section \"Unix command line\""
	echo "   >> Then steps 4 and 5"

	echo ""
	echo ">> Now calling \"cmake -S . -B build\" from the \"mcu_ws\" directory"
	echo "   >> The cmake -S flag indicates the source directory; and"
	echo "   >> The -B flag tells cmake the name of the output-directory to create."
	echo ""
	cd ${USER_HOME}/ai4r-system/mcu_ws/
	cmake -S . -B build

	echo ""
	echo ">> Now calling \"cmake --build build\" from the \"mcu_ws\" directory"
	echo "   >> This will produce the *.uf2 and *.elf files (into the build folder)"
	echo "   >> The *.uf2 files are the ones that we later need to copy across to the pico"
	echo ""
	cd ${USER_HOME}/ai4r-system/mcu_ws/
	cmake --build build

	echo ""
	echo "FINISHED: AI4R SYSTEM BUILD PICO WORKSPACE"
	echo "=========================================="
fi
# END OF: AI4R SYSTEM BUILD PICO WORKSPACE
# ============================================ #



# ============================================ #
# HOSTNAME SET
if [[ ${SHOULD_HOSTNAME_SET} = "true" ]]
then
	# Inform the user
	echo ""
	echo "======================="
	echo "BEGINNING: HOSTNAME SET"
	echo ">> Setting hostname to: ${HOSTNAME_TO_SET}"

	echo ""
	echo ">> Now rewriting the file /etc/hostname"
	echo "${HOSTNAME_TO_SET}" | sudo tee /etc/hostname >/dev/null

	if command -v hostnamectl >/dev/null 2>&1; then
		echo ">> Now applying hostname for current session using hostnamectl"
		sudo hostnamectl set-hostname "${HOSTNAME_TO_SET}"
	fi

	echo ""
	echo ">> Now updating the 127.0.1.1 line in the file /etc/hosts"
	
	# Make a back up of the file
	#sudo cp /etc/hosts /etc/hosts.bak
	
	if grep -qE '^\s*127\.0\.1\.1\b' /etc/hosts
	then
		echo ">> (127.0.1.1 line found; hence updating now)"
		sudo sed -i -E "s|^\s*127\.0\.1\.1\b.*|127.0.1.1 ${HOSTNAME_TO_SET}|" /etc/hosts
	else
		echo ">> (No 127.0.1.1 line found; hence appending one now)"
		sudo sh -c "echo '127.0.1.1 ${HOSTNAME_TO_SET}' >> /etc/hosts"
	fi

	# 3) If /etc/cloud exists, write preserve_hostname flag
	echo ""
	if [[ -d /etc/cloud ]]; then
		echo ">> Directory \"/etc/cloud\" detected."
		echo ">> Now setting configuration \"preserve_hostname: true\" into file \"/etc/cloud/cloud.cfg.d/99-preserve-hostname.cfg\""
		sudo mkdir -p /etc/cloud/cloud.cfg.d
		printf '%s\n' 'preserve_hostname: true' | sudo tee /etc/cloud/cloud.cfg.d/99-preserve-hostname.cfg >/dev/null
	else
		echo ">> Directory \"/etc/cloud\" not present; hence skipping cloud-init configuration"
	fi

	echo ""
	echo "> To check the files that were edited, use:"
	echo "cat /etc/hostname"
	echo "cat /etc/hosts"
	echo "cat /etc/cloud/cloud.cfg.d/99-preserve-hostname.cfg"
	
	echo ""
	echo "FINISHED: HOSTNAME SET"
	echo "======================"
fi
# END OF: HOSTNAME SET
# ============================================ #



# HOW TO ADD WIFI TO NETPLAN
# sudo cat /etc/netplan/50-cloud-init.yaml
# network:
#   version: 2
#   wifis:
#     wlan0:
#       optional: true
#       dhcp4: true
#       access-points:
#         "TP-Link_BFA0":
#           hidden: true
#           auth:
#             key-management: "psk"
#             password: "19957f3a31c4634fdff020f22d24b3fda4527d7b6d8b070b7a678c3212d16e1e"

# Modern equivalents of some "classic" networking commands:
# ifconfig → ip addr, ip link, ip -s link
# netstat → ss -tulwn, ss -s
# route → ip route
# arp → ip neigh

# Useful command for checking the network details (from another computer)
# nslookup asc33-rpi.eng.unimelb.edu.au
# nslookup <hostname or IP address>
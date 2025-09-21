#!/bin/bash

# ============================================ #
# SET STRICT EXECUTION
set -euo pipefail
# -e : stop script on errors,
# -u : stop script on unset vars,
# -o pipefall : stop on hidden pipeline failures.
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
# AI4R SYSTEM BUILD ROS WORKSPACE
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
# END OF: AI4R SYSTEM BUILD ROS WORKSPACE
# ============================================ #
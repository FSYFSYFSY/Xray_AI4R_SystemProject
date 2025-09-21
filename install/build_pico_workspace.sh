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
# AI4R SYSTEM BUILD PICO WORKSPACE
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
# END OF: AI4R SYSTEM BUILD PICO WORKSPACE
# ============================================ #
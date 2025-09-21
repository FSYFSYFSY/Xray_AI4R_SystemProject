#!/usr/bin/env bash
set -euo pipefail

#
# ai4r Pico flasher (RP1/RP2)
# ---------------------------------
# Purpose:
#   Copy the correct UF2 file onto each Raspberry Pi Pico by addressing the
#   device by its stable USB "by-path" (physical port), not by volatile bus/device
#   numbers. This makes flashing deterministic even after unplug/replug cycles.
#
# Usage: ./install/flash_picos.sh [0|1|2|both]
#   - 1: flash RP1 only
#   - 2: flash RP2 only
#   - both: flash both boards sequentially
#   - 0: print manual picotool commands for the selected board(s) instead of flashing
#
# Behavior summary:
#   - Prompts you to put the target Pico into BOOTSEL (USB mass storage).
#   - Waits for the BOOTSEL drive whose /dev/disk/by-path symlink contains a
#     configured substring that identifies the physical port.
#   - Mounts it, copies the UF2, syncs, unmounts; the Pico reboots automatically.
#   - If TIMEOUT_SEC elapses without finding the BOOTSEL drive, it warns and skips.
#   - If picotool is installed, the script will attempt to reboot the target
#     Pico into BOOTSEL automatically via USB (no manual BOOTSEL needed).
#
# Requirements:
#   - sudo rights for mount/umount of the BOOTSEL drive.
#   - lsblk (for detecting RPI-RP2 volume label).
#   - The two Picos are always connected to the same physical USB ports.

# Configure these two lines for your Pi's USB ports (from /dev/serial/by-path)
# Examples seen:
#   platform-...-usb-0:1.3:1.0 -> ../../ttyACM1  => use substr "usb-0:1.3"
#   platform-...-usb-0:1.4:1.0 -> ../../ttyACM0  => use substr "usb-0:1.4"
RP1_USB_PATH_SUBSTR="usb-0:1.3"
RP2_USB_PATH_SUBSTR="usb-0:1.4"

# Timeout (seconds) to wait for a BOOTSEL drive to appear per target
# Can be overridden: TIMEOUT_SEC=45 ./install/flash_picos.sh both
TIMEOUT_SEC=${TIMEOUT_SEC:-30}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Expected UF2 outputs from your Pico build in mcu_ws/build

UF2_RP1="${ROOT_DIR}/mcu_ws/build/ai4r-sensors-rp1.uf2"
UF2_RP2="${ROOT_DIR}/mcu_ws/build/ai4r-sensors-rp2.uf2"

# CLI argument: choose which board(s) to flash
want=${1:-both}
if [[ "${want}" != "0" && "${want}" != "1" && "${want}" != "2" && "${want}" != "both" ]]; then
  echo "Usage: $0 [0|1|2|both]" >&2
  exit 1
fi

# Small helper: ensure file exists or exit with a clear message
require_file() {
  local f="$1"
  [[ -f "${f}" ]] || { echo "[flash] Missing file: ${f}" >&2; exit 1; }
}

require_file "${UF2_RP1}"
require_file "${UF2_RP2}"

# Find the BOOTSEL block device by matching /dev/disk/by-path symlink that
# contains the configured substring (identifying the physical port), and confirm
# the device is the RPI-RP2 mass‑storage volume via its filesystem label.
find_bootsel_block_by_substr() {
  local substr="$1"
  # Look for RP2 mass-storage device whose by-path contains the substring
  for link in /dev/disk/by-path/*; do
    [[ -L "${link}" ]] || continue
    if [[ "${link}" != *"${substr}"* ]]; then
      continue
    fi
    local real
    real=$(readlink -f "${link}") || continue
    # Make sure it is the RPI-RP2 volume
    if lsblk -no LABEL "${real}" 2>/dev/null | grep -q "RPI-RP2"; then
      echo "${real}"
      return 0
    fi
  done
  return 1
}

# Try to find the running Pico's tty by matching /dev/serial/by-path with the
# configured substring. Prints tty name (e.g. ttyACM0) on success.
find_serial_tty_by_substr() {
  local substr="$1"
  for link in /dev/serial/by-path/*; do
    [[ -L "${link}" ]] || continue
    if [[ "${link}" == *"${substr}"* ]]; then
      local real
      real=$(readlink -f "${link}") || continue
      echo "${real##*/}"
      return 0
    fi
  done
  return 1
}

# Given a tty like ttyACM0, walk sysfs to locate the parent USB device and
# extract busnum/devnum, which picotool accepts for device selection.
get_bus_addr_from_tty() {
  local tty="$1"
  local node
  node=$(readlink -f "/sys/class/tty/${tty}/device") || return 1
  local d="${node}"
  for _ in {1..8}; do
    # ascend until we find busnum/devnum
    if [[ -r "${d}/../busnum" && -r "${d}/../devnum" ]]; then
      d=$(readlink -f "${d}/..")
      break
    fi
    d=$(readlink -f "${d}/..") || return 1
  done
  if [[ -r "${d}/busnum" && -r "${d}/devnum" ]]; then
    local bus addr
    bus=$(cat "${d}/busnum")
    addr=$(cat "${d}/devnum")
    echo "${bus} ${addr}"
    return 0
  fi
  return 1
}

get_bus_addr_from_block_dev() {
  local dev="$1"
  local sys
  sys=$(readlink -f "/sys/class/block/${dev##*/}") || return 1
  local d
  d=$(readlink -f "${sys}/device") || return 1
  for _ in {1..12}; do
    if [[ -r "${d}/busnum" && -r "${d}/devnum" ]]; then
      break
    fi
    d=$(readlink -f "${d}/..") || return 1
  done
  if [[ -r "${d}/busnum" && -r "${d}/devnum" ]]; then
    local bus addr
    bus=$(cat "${d}/busnum")
    addr=$(cat "${d}/devnum")
    echo "${bus} ${addr}"
    return 0
  fi
  return 1
}

# Full flow using picotool only:
# 1) Map running device (CDC) -> tty -> BUS:ADDR
# 2) picotool reboot --usb -F on BUS:ADDR (into BOOTSEL)
# 3) Wait for BOOTSEL block device by-path substring; map -> BUS:ADDR
# 4) picotool load UF2 on BUS:ADDR
# 5) Re-map BOOTSEL BUS:ADDR (may change) and picotool reboot (to application)
flash_with_picotool() {
  local uf2="$1" substr="$2" which="$3"
  echo "[flash] === Flashing RP${which} via picotool (match: ${substr}) ==="

  if ! command -v picotool >/dev/null 2>&1; then
    echo "[flash] ERROR: picotool not found in PATH" >&2
    return 1
  fi

  # Step 1: find running tty and map to BUS:ADDR
  local tty
  if ! tty=$(find_serial_tty_by_substr "${substr}" 2>/dev/null); then
    echo "[flash] WARNING: Could not find running serial device for '${substr}'. Ensure the Pico is connected and running. Skipping RP${which}." >&2
    return 0
  fi
  local ba bus addr
  if ! ba=$(get_bus_addr_from_tty "${tty}" 2>/dev/null); then
    echo "[flash] WARNING: Could not resolve BUS:ADDR for ${tty}. Skipping RP${which}." >&2
    return 0
  fi
  bus="${ba%% *}"; addr="${ba##* }"
  echo "[flash] Reboot to BOOTSEL: picotool reboot --usb -F (bus=${bus}, addr=${addr})"
  picotool reboot --usb -F --bus "${bus}" --address "${addr}" || true

  # Step 3: wait for BOOTSEL device on same physical port
  local dev="" polls=$(( TIMEOUT_SEC * 4 )) i
  for (( i=0; i<polls; i++ )); do
    if dev=$(find_bootsel_block_by_substr "${substr}"); then
      break
    fi
    sleep 0.25
  done
  if [[ -z "${dev}" ]]; then
    echo "[flash] WARNING: Timed out (${TIMEOUT_SEC}s) waiting for BOOTSEL drive for '${substr}'. Skipping RP${which}." >&2
    return 0
  fi
  # Map BOOTSEL block device -> BUS:ADDR for picotool load
  if ! ba=$(get_bus_addr_from_block_dev "${dev}" 2>/dev/null); then
    echo "[flash] WARNING: Could not resolve BUS:ADDR from BOOTSEL device ${dev}. Skipping RP${which}." >&2
    return 0
  fi
  bus="${ba%% *}"; addr="${ba##* }"
  echo "[flash] Loading UF2 via picotool (bus=${bus}, addr=${addr})"
  picotool load "${uf2}" --bus "${bus}" --address "${addr}"

  # After load, the device may re-enumerate; re-resolve and reboot to application
  sleep 0.5
  dev=""
  for (( i=0; i<polls; i++ )); do
    if dev=$(find_bootsel_block_by_substr "${substr}"); then
      break
    fi
    sleep 0.25
  done
  if [[ -n "${dev}" ]]; then
    if ba=$(get_bus_addr_from_block_dev "${dev}" 2>/dev/null); then
      bus="${ba%% *}"; addr="${ba##* }"
      echo "[flash] Reboot to application via picotool (bus=${bus}, addr=${addr})"
      picotool reboot --bus "${bus}" --address "${addr}" || true
    fi
  fi
  echo "[flash] RP${which}: Done"
}

# Print the sequence of manual picotool commands a user would run.
# We populate the first reboot command with the current BUS:ADDR if found.
# Subsequent steps re-enumerate; advise the user to re-check BUS:ADDR using
# "picotool list -v" for the same physical port (${substr}).
print_manual_commands() {
  local uf2="$1" substr="$2" which="$3"
  echo "[manual] === Manual steps for RP${which} (match: ${substr}) ==="
  local tty ba bus addr
  if tty=$(find_serial_tty_by_substr "${substr}" 2>/dev/null) && ba=$(get_bus_addr_from_tty "${tty}" 2>/dev/null); then
    bus="${ba%% *}"; addr="${ba##* }"
    echo "[manual] 1) Reboot into BOOTSEL:"
    echo "       picotool reboot --usb -F --bus ${bus} --address ${addr}"
  else
    echo "[manual] 1) Reboot into BOOTSEL (fill BUS/ADDR from 'picotool list -v'):"
    echo "       picotool reboot --usb -F --bus <BUS> --address <ADDR>"
  fi
  echo "[manual] 2) After reboot, the device re-enumerates. Find new BUS/ADDR for the BOOTSEL device on port '${substr}' (use 'picotool list -v'). Then load UF2:"
  echo "       picotool load \"${uf2}\" --bus <BUS_AFTER_REBOOT> --address <ADDR_AFTER_REBOOT>"
  echo "[manual] 3) It may re-enumerate again. Find current BUS/ADDR for the BOOTSEL device and reboot to application:"
  echo "       picotool reboot --bus <BUS_AFTER_LOAD> --address <ADDR_AFTER_LOAD>"
}

case "${want}" in
  0)
    print_manual_commands "${UF2_RP1}" "${RP1_USB_PATH_SUBSTR}" 1
    print_manual_commands "${UF2_RP2}" "${RP2_USB_PATH_SUBSTR}" 2
    ;;
  1)
    flash_with_picotool "${UF2_RP1}" "${RP1_USB_PATH_SUBSTR}" 1 ;;
  2)
    flash_with_picotool "${UF2_RP2}" "${RP2_USB_PATH_SUBSTR}" 2 ;;
  both)
    flash_with_picotool "${UF2_RP1}" "${RP1_USB_PATH_SUBSTR}" 1
    flash_with_picotool "${UF2_RP2}" "${RP2_USB_PATH_SUBSTR}" 2
    ;;
esac

echo "[flash] All done."

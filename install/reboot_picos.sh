#!/usr/bin/env bash
set -euo pipefail

# ai4r reboot_pico.sh — reboot Pico(s) from BOOTSEL back to application
#
# Usage: ./install/reboot_pico.sh [0|1|2|both]
#   - 1: reboot RP1 only
#   - 2: reboot RP2 only
#   - both (default): reboot both
#   - 0: print manual picotool commands for the selected board(s)

# Configure stable USB by-path substrings for each Pico (physical ports)
RP1_USB_PATH_SUBSTR="usb-0:1.3"
RP2_USB_PATH_SUBSTR="usb-0:1.4"

# Timeout (seconds) to wait for BOOTSEL device to appear
TIMEOUT_SEC=${TIMEOUT_SEC:-20}

want=${1:-both}
if [[ "${want}" != "0" && "${want}" != "1" && "${want}" != "2" && "${want}" != "both" ]]; then
  echo "Usage: $0 [0|1|2|both]" >&2
  exit 1
fi

if ! command -v picotool >/dev/null 2>&1; then
  echo "[reboot] ERROR: picotool not found in PATH" >&2
  exit 1
fi

# Find running serial tty for a Pico by matching /dev/serial/by-path substring
find_serial_tty_by_substr() {
  local substr="$1"
  local alt1="$substr" alt2="$substr"
  if [[ "$substr" == usb-* ]]; then
    alt2="usbv2-${substr#usb-}"
  elif [[ "$substr" == usbv2-* ]]; then
    alt2="usb-${substr#usbv2-}"
  fi
  for link in /dev/serial/by-path/*; do
    [[ -L "${link}" ]] || continue
    if [[ "${link}" == *"${alt1}"* || "${link}" == *"${alt2}"* ]]; then
      local real
      real=$(readlink -f "${link}") || continue
      echo "${real##*/}"
      return 0
    fi
  done
  return 1
}

# Map a tty (e.g., ttyACM0) to USB BUS:ADDR using sysfs
get_bus_addr_from_tty() {
  local tty="$1"
  local node
  node=$(readlink -f "/sys/class/tty/${tty}/device") || return 1
  local d="${node}"
  for _ in {1..8}; do
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

find_bootsel_block_by_substr() {
  local substr="$1"
  # Accept either usb-... or usbv2-... variants of the same physical port
  local alt1="$substr"
  local alt2="$substr"
  if [[ "$substr" == usb-* ]]; then
    alt2="usbv2-${substr#usb-}"
  elif [[ "$substr" == usbv2-* ]]; then
    alt2="usb-${substr#usbv2-}"
  fi
  for link in /dev/disk/by-path/*; do
    [[ -L "${link}" ]] || continue
    if [[ "${link}" != *"${alt1}"* && "${link}" != *"${alt2}"* ]]; then
      continue
    fi
    local real
    real=$(readlink -f "${link}") || continue
    if lsblk -no LABEL "${real}" 2>/dev/null | grep -q "RPI-RP2"; then
      echo "${real}"
      return 0
    fi
  done
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

reboot_from_bootsel() {
  local which="$1" substr="$2"
  echo "[reboot] === RP${which}: reboot to application (match: ${substr}) ==="
  local dev="" polls=$(( TIMEOUT_SEC * 4 )) i
  for (( i=0; i<polls; i++ )); do
    if (( i % 8 == 0 )); then
      echo -n "."
    fi
    if dev=$(find_bootsel_block_by_substr "${substr}"); then
      break
    fi
    sleep 0.25
  done
  echo
  if [[ -z "${dev}" ]]; then
    echo "[reboot] WARNING: Timed out (${TIMEOUT_SEC}s) waiting for BOOTSEL drive for '${substr}'. Skipping RP${which}." >&2
    return 0
  fi
  local ba bus addr
  if ! ba=$(get_bus_addr_from_block_dev "${dev}" 2>/dev/null); then
    echo "[reboot] WARNING: Could not resolve BUS:ADDR from BOOTSEL device ${dev}. Skipping RP${which}." >&2
    return 0
  fi
  bus="${ba%% *}"; addr="${ba##* }"
  echo "[reboot] picotool reboot --bus ${bus} --address ${addr}"
  picotool reboot --bus "${bus}" --address "${addr}" || true
}

reboot_any_state() {
  local which="$1" substr="$2"
  echo "[reboot] === RP${which}: reboot to application (match: ${substr}) ==="
  # First, check if already in BOOTSEL and reboot from there
  local dev
  if dev=$(find_bootsel_block_by_substr "${substr}"); then
    local ba bus addr
    if ba=$(get_bus_addr_from_block_dev "${dev}" 2>/dev/null); then
      bus="${ba%% *}"; addr="${ba##* }"
      echo "[reboot] (BOOTSEL) picotool reboot --bus ${bus} --address ${addr}"
      picotool reboot --bus "${bus}" --address "${addr}" || true
      return 0
    fi
  fi

  # Not in BOOTSEL: try to locate running CDC device and bounce via BOOTSEL
  local tty
  if tty=$(find_serial_tty_by_substr "${substr}" 2>/dev/null); then
    local ba bus addr
    if ba=$(get_bus_addr_from_tty "${tty}" 2>/dev/null); then
      bus="${ba%% *}"; addr="${ba##* }"
      echo "[reboot] (RUN) picotool reboot --usb -F --bus ${bus} --address ${addr}"
      picotool reboot --usb -F --bus "${bus}" --address "${addr}" || true
      # Wait for BOOTSEL on same port, then reboot to app
      local polls=$(( TIMEOUT_SEC * 4 ))
      for (( i=0; i<polls; i++ )); do
        if (( i % 8 == 0 )); then echo -n "."; fi
        if dev=$(find_bootsel_block_by_substr "${substr}"); then break; fi
        sleep 0.25
      done
      echo
      if [[ -n "${dev}" ]] && ba=$(get_bus_addr_from_block_dev "${dev}" 2>/dev/null); then
        bus="${ba%% *}"; addr="${ba##* }"
        echo "[reboot] (BOOTSEL) picotool reboot --bus ${bus} --address ${addr}"
        picotool reboot --bus "${bus}" --address "${addr}" || true
        return 0
      else
        echo "[reboot] WARNING: BOOTSEL device not found after reboot request (port ${substr})." >&2
        return 0
      fi
    fi
  fi

  echo "[reboot] WARNING: Could not locate device on port '${substr}'. Skipping RP${which}." >&2
}

print_manual_commands() {
  local which="$1" substr="$2"
  echo "[manual] === Manual steps for RP${which} (match: ${substr}) ==="
  echo "[manual] 1) Ensure the Pico is in BOOTSEL mode (RPI-RP2 drive visible)."
  echo "[manual] 2) Find BUS:ADDR with: picotool list -v"
  echo "[manual] 3) Reboot to application:"
  echo "       picotool reboot --bus <BUS> --address <ADDR>"
}

case "${want}" in
  0)
    print_manual_commands 1 "${RP1_USB_PATH_SUBSTR}"
    print_manual_commands 2 "${RP2_USB_PATH_SUBSTR}"
    ;;
  1)
    reboot_any_state 1 "${RP1_USB_PATH_SUBSTR}" ;;
  2)
    reboot_any_state 2 "${RP2_USB_PATH_SUBSTR}" ;;
  both)
    reboot_any_state 1 "${RP1_USB_PATH_SUBSTR}"
    reboot_any_state 2 "${RP2_USB_PATH_SUBSTR}"
    ;;
esac

echo "[reboot] All done."

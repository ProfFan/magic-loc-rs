#!/bin/zsh

# Parse the command line arguments
DEVICES=()
ESPFLASH="$HOME/.cargo/bin/espflash"
IMAGE="./target/xtensa-esp32s3-none-elf/release/magic-loc-rs"

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      echo "Usage: flash-all.sh [-h|--help] [-v|--verbose] [device1 device2 ... deviceN]"
      exit 0
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    -i|--image)
      IMAGE="$2"
      shift
      shift
      ;;
    *)
      DEVICES+=("$1")
      shift
      ;;
  esac
done

if [[ -z $DEVICES ]]; then
  echo "No devices specified. Please specify one or more devices to flash."
  exit 1
fi

if [[ -n $VERBOSE ]]; then
  echo "Flashing devices: $DEVICES"
fi

if [[ -z $DEFMT_LOG ]]; then
  echo "DEFMT_LOG environment variable not set. Please set it to enable defmt logging."
  exit 1
fi

# Build package
echo "Building package..."

cargo build --release

# check partition table
if [[ ! -f partitions.csv ]]; then
  echo "Partitions file not found. Please create a partitions.csv file."
  exit 1
fi

# Flash devices
for device in $DEVICES; do
  echo "Flashing $device..."
  PORT=$(esp-serial-find -s $device)
  if [[ -z $PORT ]]; then
    echo "Device $device not found. Skipping..."
    continue
  fi
  $ESPFLASH flash --partition-table partitions.csv --port $PORT $IMAGE
done

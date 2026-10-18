#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage:"
  echo "  $0 /path/to/bag_directory"
  exit 1
fi

ros2 bag info "$1"

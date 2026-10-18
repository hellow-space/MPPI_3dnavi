#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 BAG_DIR"
  exit 1
fi
ros2 bag info "$(realpath "$1")"

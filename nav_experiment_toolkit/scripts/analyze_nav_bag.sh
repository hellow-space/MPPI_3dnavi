#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 BAG_DIR [extra analyze_nav_bag.py args...]"
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG="$1"
shift
python3 "${ROOT_DIR}/analysis/analyze_nav_bag.py" "$BAG" "$@"

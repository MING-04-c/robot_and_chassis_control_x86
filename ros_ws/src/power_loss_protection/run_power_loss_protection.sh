#!/usr/bin/env bash
# Formal foreground entry point; systemd uses this same executable.
set -euo pipefail
protection_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$EUID" -ne 0 ]]; then
  echo 'Run with sudo: protection needs permission to stop services and power off this computer.' >&2
  exit 1
fi
exec /usr/bin/python3 "$protection_dir/power_loss_protection.py" run \
  --config "${1:-$protection_dir/config.json}"

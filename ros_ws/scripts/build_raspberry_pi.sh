#!/usr/bin/env bash
# Compatibility entry point; supports both ARM64 and amd64.
set -e
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/build_robot.sh" "$@"

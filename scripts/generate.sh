#!/usr/bin/env bash
# Generates Go bindings (msgs/) for the ROS 2 interfaces this node uses.
# Requires a sourced ROS 2 environment and a workspace providing custom_msgs
# (source <mira>/install/setup.bash first).
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
	echo "AMENT_PREFIX_PATH is not set: source ROS 2 and the mira workspace first" >&2
	exit 1
fi

root_args=()
IFS=':' read -ra prefixes <<< "$AMENT_PREFIX_PATH"
for p in "${prefixes[@]}"; do
	[ -n "$p" ] && root_args+=(-r "$p")
done

# rclgo-gen only officially supports Humble; the interfaces it generates
# bindings for are unchanged in Jazzy.
go run github.com/tiiuae/rclgo/cmd/rclgo-gen generate \
	--ignore-ros-distro-mismatch \
	-d msgs \
	--message-module-prefix github.com/davidnoronha1/mira-control-a1/msgs \
	--include-go-package-deps . \
	--cgo-flags-path "" \
	"${root_args[@]}" 2>&1 | { grep -v '^Generating' || true; }

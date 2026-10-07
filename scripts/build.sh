#!/usr/bin/env bash
# Builds the master binary against the sourced ROS 2 distribution.
# Usage: scripts/build.sh [output path]   (default: ./master)
set -euo pipefail
cd "$(dirname "$0")/.."
out="${1:-master}"

./scripts/generate.sh

# rclgo's own cgo flags point at /opt/ros/humble. Add the include directory
# of every package in the sourced prefixes so it builds against Jazzy (rcl
# there also includes type_description_interfaces, service_msgs, ...).
cflags=()
ldflags=()
IFS=':' read -ra prefixes <<< "$AMENT_PREFIX_PATH"
for p in "${prefixes[@]}"; do
	index="$p/share/ament_index/resource_index/packages"
	[ -d "$index" ] || continue
	for pkg in "$index"/*; do
		pkg="$(basename "$pkg")"
		[ -d "$p/include/$pkg" ] && cflags+=("-I$p/include/$pkg")
	done
	[ -d "$p/lib" ] && ldflags+=("-L$p/lib" "-Wl,-rpath,$p/lib")
done

export CGO_ENABLED=1
export CGO_CFLAGS="${CGO_CFLAGS:-} ${cflags[*]} -Wno-deprecated-declarations"
export CGO_LDFLAGS="${CGO_LDFLAGS:-} ${ldflags[*]}"
go build -o "$out" .

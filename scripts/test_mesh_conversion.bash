#!/usr/bin/env bash
# Quick way to eyeball the MESHCONV_* switches (meshasstl.h) without going through
# flexbe. Converts a .osim straight to a static URDF+meshes and shows it in rviz.
#
# Usage:
#   ./test_mesh_conversion.bash /path/to/model.osim [fixed_frame]
#
# To test MESHCONV_SKIP_OBJ_INTERMEDIATE, export MESHCONV_SOURCE_OVERRIDE_DIR first
# (a directory containing <body_name>.dae files) - see meshasstl.h for details.
# The MESHCONV_* switches themselves are compile-time; rebuild osrt_ros between tests.
#
# This only shows the model at its default/zero pose (all joints in the generated
# URDF are "floating" by design, see osim_to_simple_urdf.cpp - not meaningfully
# drivable by joint_state_publisher). For a ground-truth comparison unaffected by
# this conversion pipeline, run view_osim_native.py on the same .osim alongside this.

set -euo pipefail

if [ $# -lt 1 ]; then
	echo "Usage: $0 /path/to/model.osim [fixed_frame]" >&2
	exit 1
fi

MODEL="$1"
FIXED_FRAME="${2:-link_0}"

if [ ! -f "$MODEL" ]; then
	echo "No such file: $MODEL" >&2
	exit 1
fi

echo "Converting $MODEL -> /tmp/converted_model.urdf (meshes -> /tmp/*.stl)"
rosrun osrt_ros osim_to_simple_urdf "$MODEL"

echo "Launching rviz with fixed_frame=$FIXED_FRAME"
roslaunch osrt_ros test_mesh_conversion.launch fixed_frame:="$FIXED_FRAME"

#!/usr/bin/env python3
"""Ground-truth reference for the mesh-orientation bug: opens a .osim in
OpenSim's own Simbody visualizer at the model's default pose. This loads mesh
geometry directly, completely bypassing osrt_ros's dae->obj->stl conversion
pipeline (meshasstl.h / urdf_to_osim.cpp) - so if this window looks correct
but rviz (via test_mesh_conversion.bash on the same .osim) looks twisted, that
confirms the bug is in the conversion pipeline, not the .osim model itself.

Usage: view_osim_native.py /path/to/model.osim
"""
import sys

import opensim


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} model.osim", file=sys.stderr)
        return 1

    model = opensim.Model(sys.argv[1])
    model.setUseVisualizer(True)
    state = model.initSystem()
    model.realizePosition(state)
    model.getVisualizer().show(state)

    print("Opened OpenSim's native visualizer at the model's default pose.")
    print("Compare this window against rviz (test_mesh_conversion.bash on the same .osim).")
    input("Press Enter to close...")
    return 0


if __name__ == "__main__":
    sys.exit(main())

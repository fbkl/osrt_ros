#!/usr/bin/env python3

import opensim
import os, sys

def meat(model, body, prefix, location_in_frame):
    body_name = body.getName()

    marker_name = f"{prefix}{body_name}"

    try:
        frame = model.getComponent(f"/bodyset/{body_name}")
    except Exception:
        # Fallback to look for the frame anywhere in the model components, but i have no clue if it works
        frame = model.getFrame(body_name)

    # 4. Initialize the Marker object
    new_marker = opensim.Marker()
    new_marker.setName(marker_name)
    new_marker.set_location(location_in_frame)

    # Connect the marker to the physical frame
    new_marker.connectSocket_parent_frame(frame)

    return new_marker, {marker_name:body_name}

def create_model_with_markers(model, output_path, bodies, singles, prefix):

    markers = {}

    for body in bodies:

        if singles:
            location_in_frame = opensim.Vec3(0.0, 0.0, 0.0) # X, Y, Z offsets in meters
            new_marker, mm = meat(model, body, prefix, location_in_frame)
            model.addMarker(new_marker)
            markers.update(mm)
        else:
            for pos, l_r in enumerate(["L_", "R_", "C_"]):
                location_in_frame = opensim.Vec3(0.05, 0.02, -0.04) # X, Y, Z offsets in meters
                if pos == 1:
                    location_in_frame = opensim.Vec3(0.05, 0.02, 0.04) # X, Y, Z offsets in meters
                elif pos == 2:
                    location_in_frame = opensim.Vec3(0.05, 0.0, 0.0) # X, Y, Z offsets in meters

                new_marker, mm = meat(model, body, l_r, location_in_frame)
                model.addMarker(new_marker)
                markers.update(mm)

    # 6. Finalize and save the updated model
    model.finalizeConnections()
    model.printToXML(output_path)

    print(f"Successfully added markers to bodies like :\n '{markers}'\n and saved to:\n\n {output_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(prog="create_markerset.py", description="Create a markerset for a model")
    parser.add_argument('model',help="Path of the osim model")
    parser.add_argument('--bodylist','-b',nargs="+", help="List of bodies you want to add markers to. Default is the whole  bodyset")
    parser.add_argument('--outputpath','-o', help="Where you want your new model to be saved (default is /tmp/<<model>>_custom.osim)")
    parser.add_argument('--singlemarker','-s', help="Adds a single marker per body (default is 3)", action='store_true')
    parser.add_argument('--markerprefix', default= "vio_",help="the prefix to be added to single marker setups")
    args = parser.parse_args()
    model_path = args.model
    output_path = args.outputpath
    if not output_path:
        model_name = os.path.basename(model_path)
        output_path = f"/tmp/{model_name}_custom.osim"

    model = opensim.Model(model_path)

    # Initialize system (required before accessing state-dependent quantities)
    state = model.initSystem()

    # Get the ground (global) body
    ground = model.getGround()

    bodies = [*model.getBodyList()]

    if args.bodylist:
        new_bodies = []
        for bb in bodies:
            if bb.getName() in args.bodylist:
                new_bodies.append(bb)
        ## now replace it
        if not new_bodies:
            raise(f"I didn't match any bodies with the list you gave me {arg.bodylist}")
        bodies = new_bodies

    is_single_marker = args.singlemarker

    create_model_with_markers(model, output_path, bodies, is_single_marker, args.markerprefix)

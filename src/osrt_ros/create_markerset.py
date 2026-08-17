#!/usr/bin/env python3

import opensim
import os, sys


def create_model_with_markers(model_path):

    model = opensim.Model(model_path)

    # Initialize system (required before accessing state-dependent quantities)
    state = model.initSystem()

    # Get the ground (global) body
    ground = model.getGround()

    body_set = model.getBodySet()
    markers = []
    body_names = []
    for ith_body in range(body_set.getSize()):
        for pos, l_r in enumerate(["L_", "R_", "C_"]):
            # gets the body
            body = body_set.get(ith_body)

            body_name = body.getName()

            # 2. Define your marker properties
            marker_name = f"{l_r}{body_name}"
            
            # this is the body name now
            #frame_name = "link_0"  # The body/frame name to attach the marker to
            
            location_in_frame = opensim.Vec3(0.05, 0.02, -0.04) # X, Y, Z offsets in meters
            if pos == 1:
                location_in_frame = opensim.Vec3(0.05, 0.02, 0.04) # X, Y, Z offsets in meters
            elif pos == 2:
                location_in_frame = opensim.Vec3(0.05, 0.0, 0.0) # X, Y, Z offsets in meters

            # 3. Get a reference to the physical frame
            # Note: For older OpenSim 3.3 models, you would use model.getBodySet().get()
            try:
                frame = model.getComponent(f"/bodyset/{body_name}")
            except Exception:
                # Fallback to look for the frame anywhere in the model components
                frame = model.getFrame(frame_name)

            # 4. Initialize the Marker object
            new_marker = opensim.Marker()
            new_marker.setName(marker_name)
            new_marker.set_location(location_in_frame)

            # Connect the marker to the physical frame
            new_marker.connectSocket_parent_frame(frame)

            # 5. Add the marker to the model
            model.addMarker(new_marker)

            # 5.1 if it is all okay then we also add the list names to know what we did

            markers.append(marker_name)

            body_names.append(body_name)

    # 6. Finalize and save the updated model
    model.finalizeConnections()
    model_name = os.path.basename(model_path)
    output_path = f"/tmp/{model_name}_custom.osim"
    model.printToXML(output_path)

    print(f"Successfully added markers:\n '{markers}'\n to:\n '{body_names}'\n and saved to:\n\n {output_path}")

if __name__ == "__main__":
    # 1. Load your existing model
    #model_path = "iiwa14.osim"
    try:
        create_model_with_markers(sys.argv[1])
    except:
        print(f"Usage:\n {sys.argv[0]} model.osim")

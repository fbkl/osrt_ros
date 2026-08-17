#!/usr/bin/env python3
"""Generate markers.yaml from an .osim MarkerSet XML block.
Usage: python generate_markers_yaml.py model.osim > config/markers.yaml

This is also a filthy hack. You should edit the UIMU node to parse it from the model, probably, idk.
"""
import sys
import xml.etree.ElementTree as ET
import yaml
import opensim as osim
import rospy


def parse_markers(osim_path):
    """
    Parses the .osim file and returns a dict with the params for the UIMU node
    """
    osim.Logger.setLevel(osim.Logger.Level_Off)
    osim.Logger.removeFileSink()
    tree = ET.parse(osim_path)
    root = tree.getroot()
    markers = []

    ##okay, this already broke, we need to parse the model to get them in global positions i think

    # Load model
    model = osim.Model(osim_path)

    # Initialize system (required before accessing state-dependent quantities)
    state = model.initSystem()

    # Get the ground (global) body
    ground = model.getGround()

    # Get marker set
    marker_set = model.getMarkerSet()

    #print("Marker positions in GLOBAL coordinates (default pose):\n")

    a = {}
    # Loop through markers
    for i in range(marker_set.getSize()):
        marker_ = marker_set.get(i)
        
        name = marker_.getName()
        
        # This gives location in ground frame
        global_pos = marker_.getLocationInGround(state)
        
        x = global_pos.get(0)
        y = global_pos.get(1)
        z = global_pos.get(2)
        
        #print(f"{name}: ({x:.6f}, {y:.6f}, {z:.6f})")

        a[name] = [x,y,z]
    for marker in root.iter('Marker'):
        name = marker.get('name')
        frame = marker.find('socket_parent_frame').text.strip()
        body = frame.split('/')[-1]  # strip /bodyset/
        loc = [float(x) for x in marker.find('location').text.split()]
        markers.append({
            'marker_name': name,
            'marker_tf': body,
            'default_position': a[name]
        })

    #return {'observation_order': markers}
    return markers

if __name__ == '__main__':
    DATA = parse_markers(sys.argv[1])
    rospy.init_node("gen_marker_yaml")
    print("# AUTO-GENERATED from .osim — do not edit by hand")
    print("# Regenerate: python scripts/generate_markers_yaml.py model.osim > config/markers.yaml")
    if not DATA:
         rospy.logfatal(f"You have NO MARKERS in this model. Maybe you want to generate some dummy ones with\n\n\tcreate_markerset.py  {sys.argv[1]}\n")       
    print(yaml.dump(DATA, default_flow_style=None, allow_unicode=True, sort_keys=False))


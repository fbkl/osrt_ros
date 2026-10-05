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


def parse_markers(osim_path, inc=False, exc=False, prefix=""):
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
        ##for readability we are separating these clauses. 
        ## 
        if exc and name.startswith(prefix):
            continue    

        if inc and not name.startswith(prefix):
            continue
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
    
    import argparse
    parser = argparse.ArgumentParser(description="creates the yaml for the ik node")
    parser.add_argument('model',help='Path of the osim model')
    parser.add_argument('--prefix', default="",help="Prefix to check against for inclusion/exclusion of markers")
    parser.add_argument('--inc', '-i', help="Prefix is inclusive, i.e. reads markers only if the include the prefix", action='store_true')
    parser.add_argument('--exc', '-e', help="Prefix is exclusive, i.e. reads markers only if they exclude the prefix, do not have the prefix", action='store_true')
    
    args = parser.parse_args()
    model = args.model
    if args.inc and args.exc:
        raise(Exception("You cannot set both inclusive and exclusive!"))

    if args.inc or args.exc:
        if not args.prefix:
            raise(Exception("When including or excluding prefixes, prefix needs to be set"))

    DATA = parse_markers(model,inc=args.inc, exc=args.exc, prefix=args.prefix)
    rospy.init_node("gen_marker_yaml")
    print("# AUTO-GENERATED from .osim — do not edit by hand")
    print("# Regenerate: python scripts/generate_markers_yaml.py model.osim > config/markers.yaml")
    if not DATA:
         rospy.logfatal(f"You have NO MARKERS in this model. Maybe you want to generate some dummy ones with\n\n\tcreate_markerset.py  {sys.argv[1]}\n")       
    print(yaml.dump(DATA, default_flow_style=None, allow_unicode=True, sort_keys=False))


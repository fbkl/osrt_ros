#!/usr/bin/env python3
"""DOES SOMRTHING.
"""
import sys
import xml.etree.ElementTree as ET
import yaml
import opensim as osim

def parse_bodies(osim_path):
    """
    Parses the .osim file and returns a dict with the bodies?
    """
    osim.Logger.setLevel(osim.Logger.Level_Off)
    osim.Logger.removeFileSink()
    tree = ET.parse(osim_path)
    root = tree.getroot()
    bodies = []

    ##okay, this already broke, we need to parse the model to get them in global positions i think

    # Load model
    model = osim.Model(osim_path)

    # Initialize system (required before accessing state-dependent quantities)
    state = model.initSystem()

    # Get the ground (global) body
    ground = model.getGround()

    # Get body set
    body_set = model.getBodySet()

    #print("Body positions in GLOBAL coordinates (default pose):\n")

    a = {}
    # Loop through bodies
    for i in range(body_set.getSize()):
        body_ = body_set.get(i)
        
        name = body_.getName()
        
        bodies.append(name)
 #       print(name)
        # This gives location in ground frame
        #global_pos = body_.getLocationInGround(state)
        
        #x = global_pos.get(0)
        #y = global_pos.get(1)
        #z = global_pos.get(2)
        
        #print(f"{name}: ({x:.6f}, {y:.6f}, {z:.6f})")

        #a[name] = [x,y,z]
    #for body in root.iter('Body'):
    #    name = body.get('name')
    #    frame = body.find('socket_parent_frame').text.strip()
    #    body = frame.split('/')[-1]  # strip /bodieset/
    #    loc = [float(x) for x in body.find('location').text.split()]
    #    bodies.append({
    #        'body_name': name,
    #        'body_tf': body,
    #        'default_position': a[name]
    #    })

    #return {'observation_order': bodies}
    return bodies

if __name__ == '__main__':
    DATA = parse_bodies(sys.argv[1])
#    print("# AUTO-GENERATED from .osim — do not edit by hand")
#    print("# Regenerate: python scripts/generate_bodies_yaml.py model.osim > config/bodies.yaml")
#    print(yaml.dump(DATA, default_flow_style=None, allow_unicode=True, sort_keys=False))

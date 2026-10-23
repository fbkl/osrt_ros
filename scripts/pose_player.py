#!/usr/bin/env python3
"""Push a named pose from a yaml file into a model, via ~set_pose.

The point of this is the arm: when the subject's pose is known from the outside
(a robot's encoders, a jig, a photograph of the smartPAD) there is no reason to
let calibration assume the model's default pose. Record the pose once, replay it
whenever you calibrate.

    rosrun osrt_ros pose_player.py poses.yaml --list
    rosrun osrt_ros pose_player.py poses.yaml --pose parked
    rosrun osrt_ros pose_player.py poses.yaml --pose parked --service /vio/ik/set_pose
    rosrun osrt_ros pose_player.py --set joint_0_coord_0=12.5 --degrees

File format -- `units` is optional and defaults to radians:

    units: degrees
    poses:
      parked:
        joint_0_coord_0: 0.0
        joint_1_coord_0: 17.3

A pose may name any subset of the coordinates; everything it does not mention
keeps the value it already has.
"""
import argparse
import math
import sys

import rospy
import yaml
from sensor_msgs.msg import JointState
from opensimrt_msgs.srv import SetModelPoseSrv, SetModelPoseSrvRequest

DEFAULT_SERVICE = "/vio/ik/set_pose"


def load_poses(path):
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    if not isinstance(doc, dict) or "poses" not in doc:
        raise ValueError(f"{path}: expected a mapping with a top-level 'poses' key")
    units = str(doc.get("units", "radians")).lower()
    if units not in ("radians", "rad", "degrees", "deg"):
        raise ValueError(f"{path}: units must be radians or degrees, got '{units}'")
    poses = doc["poses"] or {}
    if not isinstance(poses, dict):
        raise ValueError(f"{path}: 'poses' must be a mapping of name -> {{coordinate: value}}")
    return poses, units.startswith("deg")


def to_radians(pose, degrees):
    # Translational coordinates are metres and must NOT be converted. This script
    # cannot tell them apart from rotational ones without the model, so --degrees
    # is all-or-nothing. The iiwa is seven revolute joints, so it is fine there;
    # on a model with a free base, write that pose in radians/metres instead.
    if not degrees:
        return dict(pose)
    return {k: math.radians(v) for k, v in pose.items()}


def main():
    p = argparse.ArgumentParser(prog="pose_player.py", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file", nargs="?", help="yaml file of named poses")
    p.add_argument("--pose", "-p", help="which pose in the file to send")
    p.add_argument("--list", "-l", action="store_true", help="list the poses in the file and exit")
    p.add_argument("--set", "-s", nargs="+", metavar="COORD=VALUE", default=[],
                   help="set coordinates directly, instead of (or on top of) a named pose")
    p.add_argument("--degrees", "-d", action="store_true",
                   help="values are in degrees; overrides the file's own units key")
    p.add_argument("--service", default=DEFAULT_SERVICE, help=f"default {DEFAULT_SERVICE}")
    p.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the service")
    args = p.parse_args()

    pose = {}
    degrees = args.degrees

    if args.file:
        poses, file_in_degrees = load_poses(args.file)
        if args.list:
            for name in sorted(poses):
                print(f"{name}  ({len(poses[name])} coordinates)")
            return 0
        if not args.pose and not args.set:
            p.error("give --pose NAME (or --list) when passing a file")
        if args.pose:
            if args.pose not in poses:
                p.error(f"no pose '{args.pose}' in {args.file}. have: {', '.join(sorted(poses))}")
            chosen = poses[args.pose] or {}
            # A null in the yaml means "we have not measured this yet". Say so,
            # instead of letting it become a TypeError three frames down.
            blank = sorted(k for k, v in chosen.items() if v is None)
            if blank:
                p.error(f"pose '{args.pose}' in {args.file} is not filled in yet: "
                        f"{', '.join(blank)}")
            pose.update(chosen)
            degrees = degrees or file_in_degrees
    elif args.list:
        p.error("--list needs a file")

    for item in args.set:
        if "=" not in item:
            p.error(f"--set wants COORD=VALUE, got '{item}'")
        k, v = item.split("=", 1)
        pose[k.strip()] = float(v)

    if not pose:
        p.error("nothing to send: give a file with --pose, or use --set")

    pose = to_radians(pose, degrees)

    rospy.init_node("pose_player", anonymous=True)
    rospy.loginfo("waiting for %s", args.service)
    try:
        rospy.wait_for_service(args.service, timeout=args.timeout)
    except rospy.ROSException:
        rospy.logerr("%s did not show up in %.1fs -- is the ik node running?",
                     args.service, args.timeout)
        return 1

    req = SetModelPoseSrvRequest()
    req.pose = JointState()
    req.pose.header.stamp = rospy.Time.now()
    # Sorted so the log line is readable and two runs of the same pose look alike.
    for name in sorted(pose):
        req.pose.name.append(name)
        req.pose.position.append(pose[name])

    try:
        res = rospy.ServiceProxy(args.service, SetModelPoseSrv)(req)
    except rospy.ServiceException as e:
        rospy.logerr("service call failed: %s", e)
        return 1

    if not res.ok:
        rospy.logerr("refused: %s", res.message)
        return 1

    rospy.loginfo("posed %d coordinate(s)%s", len(req.pose.name),
                  " (converted from degrees)" if degrees else "")
    for n, v in zip(req.pose.name, req.pose.position):
        rospy.loginfo("  %-20s %+.6f rad  (%+.2f deg)", n, v, math.degrees(v))
    return 0


if __name__ == "__main__":
    sys.exit(main())

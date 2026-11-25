#!/usr/bin/env python3
"""Push a named pose from a yaml file into a model, via ~set_pose.

The point of this is the arm: when the subject's pose is known from the outside
(a robot's encoders, a jig, a photograph of the smartPAD) there is no reason to
let calibration assume the model's default pose. Record the pose once, replay it
whenever you calibrate.

    rosrun osrt_ros pose_player.py --list
    rosrun osrt_ros pose_player.py --pose parked
    rosrun osrt_ros pose_player.py other_poses.yaml --pose parked
    rosrun osrt_ros pose_player.py --set joint_0_coord_0=12.5 --degrees

With no file given it reads DEFAULT_POSE_FILE from this package's config dir.

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
import os
import sys

import rospy
import yaml
from sensor_msgs.msg import JointState
from opensimrt_msgs.srv import SetModelPoseSrv, SetModelPoseSrvRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

DEFAULT_SERVICE = "/vio/ik/set_pose"
DEFAULT_POSE_FILE = "config/iiwa14_poses.yaml"

#this should be an import since i am using this twice and it is pretty useful
def resolve(path):
    """package://pkg/rest -> absolute. Resolved here, in the process that opens
    the file, so it still works when a node is sent to another machine."""
    if not path.startswith("package://"):
        return os.path.expanduser(path)
    pkg, _, rest = path[len("package://"):].partition("/")
    import rospkg
    return os.path.join(rospkg.RosPack().get_path(pkg), rest)


##this is sort of bad
def default_pose_file():
    """This package's own pose file, or None if we cannot find the package."""
    try:
        import rospkg
        return os.path.join(rospkg.RosPack().get_path("osrt_ros"), DEFAULT_POSE_FILE)
    except Exception:
        # Not run through ROS, or the package is not on the path. Fall back to
        # walking up from this script, which works in a plain source checkout.
        here = os.path.dirname(os.path.abspath(__file__))
        guess = os.path.join(os.path.dirname(here), DEFAULT_POSE_FILE)
        return guess if os.path.exists(guess) else None


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
    p.add_argument("file", nargs="?",
                   help=f"yaml file of named poses (default: osrt_ros/{DEFAULT_POSE_FILE})")
    p.add_argument("--pose", "-p", help="which pose in the file to send")
    p.add_argument("--list", "-l", action="store_true", help="list the poses in the file and exit")
    p.add_argument("--set", "-s", nargs="+", metavar="COORD=VALUE", default=[],
                   help="set coordinates directly, instead of (or on top of) a named pose")
    p.add_argument("--degrees", "-d", action="store_true",
                   help="values are in degrees; overrides the file's own units key")
    p.add_argument("--serve", action="store_true", help=f"advertises ~pose_it and spins instead of oneshot setting the pose ")
    p.add_argument("--service", default=DEFAULT_SERVICE, help=f"default {DEFAULT_SERVICE}")
    p.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the service")
    args = p.parse_args(rospy.myargv()[1:])

    pose = {}
    degrees = args.degrees

    pose_file = resolve(args.file) or default_pose_file()
    if args.list and not pose_file:
        p.error("--list needs a file, and I could not find the package default")

    if pose_file and (args.list or args.pose or not args.set):
        if not os.path.exists(pose_file):
            p.error(f"no such pose file: {pose_file}")
        poses, file_in_degrees = load_poses(pose_file)
        if args.list:
            for name in sorted(poses):
                print(f"{name}  ({len(poses[name])} coordinates)")
            return 0
        if not args.pose and not args.set:
            p.error(f"give --pose NAME (or --list). {pose_file} has: "
                    f"{', '.join(sorted(poses))}")
        if args.pose:
            if args.pose not in poses:
                p.error(f"no pose '{args.pose}' in {pose_file}. have: {', '.join(sorted(poses))}")
            chosen = poses[args.pose] or {}
            # A null in the yaml means "we have not measured this yet". Say so,
            # instead of letting it become a TypeError three frames down.
            blank = sorted(k for k, v in chosen.items() if v is None)
            if blank:
                p.error(f"pose '{args.pose}' in {pose_file} is not filled in yet: "
                        f"{', '.join(blank)}")
            pose.update(chosen)
            degrees = degrees or file_in_degrees

    for item in args.set:
        if "=" not in item:
            p.error(f"--set wants COORD=VALUE, got '{item}'")
        k, v = item.split("=", 1)
        pose[k.strip()] = float(v)

    if not pose:
        p.error("nothing to send: give a file with --pose, or use --set")

    pose = to_radians(pose, degrees)

    rospy.init_node("pose_player")
    
    if not args.serve: # if we are serving, then i want to go straight to spinning and hope for the best
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

    def this_callback(empty_req): ## we don't use empty req for anything
        try:
            rospy.wait_for_service(args.service, timeout=args.timeout)
        except rospy.ROSException:
            raise rospy.ServiceException("%s did not show up in %.1fs -- is the ik node running?"%  (args.service, args.timeout))

        req.pose.header.stamp = rospy.Time.now()
        
        res = rospy.ServiceProxy(args.service, SetModelPoseSrv)(req)

        if not res.ok:
            rospy.logerr("refused: %s", res.message)
            raise rospy.ServiceException(f"refused: {res.message}")

        rospy.loginfo("posed %d coordinate(s)%s", len(req.pose.name),
                      " (converted from degrees)" if degrees else "")
        for n, v in zip(req.pose.name, req.pose.position):
            rospy.loginfo("  %-20s %+.6f rad  (%+.2f deg)", n, v, math.degrees(v))
        
        return EmptyResponse()

    if args.serve:
        rospy.Service("~pose_it", Empty, this_callback)
    
        rospy.spin()
    else:
        try:
            this_callback(EmptyRequest())
        except rospy.ServiceException as e:
            rospy.logerr("%s", e)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

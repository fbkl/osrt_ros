#!/usr/env python3

import rospy
import roslaunch
import rosparam
package = 'osrt_ros'
executable = 'human_publisher_mini_node'
node = roslaunch.core.Node(package, executable, args="/srv/host_data/fk.osim", output="log")

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
#print process.is_alive()
rospy.sleep(1)

model_str = open("/tmp/converted_model.urdf").read()
rosparam.set_param("/osim_model", model_str)
rospy.init_node("a", anonymous=True)
rospy.spin() ## if i let it close it closes the launch file

process.stop()


##################################################
## Author: Karla Stepanova
## Email: karla.stepanova@cvut.cz
## Date of creation: 15.6.2019
##################################################

# !/usr/bin/python
import rospy
import time
from math import radians, pi
from geometry_msgs.msg import Pose
from capek_pycommander.capek_robot import CapekRobotCommander
from std_msgs.msg import Float64MultiArray
import tf
import geometry_msgs
import sys
import tf2_ros

def above_chessboard_center():
    pose = Pose()
    pose.position.x = 0.59
    pose.position.y = 1
    pose.position.z = 0.5
    pose.orientation.x = 0
    pose.orientation.y =1
    pose.orientation.z = 0
    pose.orientation.w = 0

    return pose

def above_chess_piece():
    pose = Pose()
    pose.position.x = 0.668-0.4275
    pose.position.y = 1+0.11
    pose.position.z = 0.2
    pose.orientation.x = 0
    pose.orientation.y =1
    pose.orientation.z = 0
    pose.orientation.w = 0

    return pose

rospy.init_node("move")
crc = CapekRobotCommander("r2")
crc.move_l_position()

# camera to robot transformation
# translation:
#   x: 0.909480344525
#   y: -0.266042768948
#   z: 1.29520475995
# rotation:
#   x: -0.718862965166
#   y: -0.672505426905
#   z: 0.127747039845
#   w: 0.121050327991
# translation:
#   x: 0.906409291874
#   y: -0.245099493827
#   z: 1.29990429444
# rotation:
#   x: -0.701589685131
#   y: -0.692603050646
#   z: 0.132803067756
#   w: 0.102158079214

# broadcaster.sendTransform(static_transformStamped)

cartesian_plan, _ = crc.compute_cartesian_path_by_poses([above_chessboard_center()])

crc.display_trajectory(cartesian_plan)
crc.execute_plan(cartesian_plan)

cartesian_plan, _ = crc.compute_cartesian_path_by_poses([above_chess_piece()])

crc.display_trajectory(cartesian_plan)
crc.execute_plan(cartesian_plan)




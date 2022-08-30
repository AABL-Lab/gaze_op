#!/usr/bin/env python3
# asynch move arm based on combined input
# sub to relative_velocity
# pub to arm topics to move
# ----------------------------------------------------------------------------
import arm_stripped

# ROS
import rospy
from gaze_op.msg import VelocityInfo
# ----------------------------------------------------------------------------
GEN3LITE_NAME = "my_gen3_lite" # kinova gen3 lite name as used by ros-kortex
_VEL_SCALE_FACTOR = 0.08
# ----------------------------------------------------------------------------

def initialize():
    rospy.init_node('move_arm')

def move():
    arm = arm_stripped.Arm(GEN3LITE_NAME)
    while not rospy.is_shutdown():
        relvel = rospy.wait_for_message("/relative_velocity", VelocityInfo)
        # coordinate system used is as follows:
        # when facing robot, value passed as "linear_x" is what moves robot to front/back
        # when facing robot, value passed as "linear_y" is what moves robot to right/left
        # when facing robot, value passed as "linear_z" is what moves robot to up/down
        # observer x, y, z have been right/left, up/down, and front/back respectively
        # hence: robo_x = obs_z ; robo_y = obs_x ; robo_z = obs_y
        lin_x = relvel.z*_VEL_SCALE_FACTOR
        lin_y = relvel.x*_VEL_SCALE_FACTOR
        lin_z = relvel.y*_VEL_SCALE_FACTOR
        arm.set_cartesian_velocity([lin_x, lin_y, lin_z, 0, 0, 0])

if __name__ == '__main__':
    try:
        initialize()
        move()
    except rospy.ROSInterruptException:
        pass
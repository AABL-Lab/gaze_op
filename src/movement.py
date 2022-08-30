#!/usr/bin/env python3
# move thing once you get input
# node movement_detector
# subscribed to facial_landmarks
# publish to relative_velocity
# ----------------------------------------------------------------------------
# ROS
from multiprocessing.context import ForkContext
import rospy
from gaze_op.msg import *
# ----------------------------------------------------------------------------
_SCALE_FACTOR = 1000
_MIN_DISP = 5
_x = 0
_y = 0
_z = 0
_d_x = 0
_d_y = 0
_d_z = 0
# ----------------------------------------------------------------------------

def initialize():
  print("INFO: Initializing ROS node  \"movement_processor\"")
  rospy.init_node("movement_processor")
  return

def get_dir(disp):
  disp = disp * _SCALE_FACTOR
  # check if displacement is significant
  if(abs(disp) < _MIN_DISP):
    return 0
  # get direction
  if(disp == 0): return 0
  elif(disp > 0): return 1
  else: return -1

def data_callback(data):
  # referencing global vairables
  global _x, _y, _z, _d_x, _d_y, _d_z

  # get movement information
  # Note: as typical in using 2D matrix for images, origin is at top-left
  # of an image x increases to the right and y increases downwards
  x_disp = -(data.x - _x) # we want to mirror image
  y_disp = -(data.y - _y) # y starts at top right i
  z_disp = (data.z - _z)
  x_dir = get_dir(x_disp)
  y_dir = get_dir(y_disp)
  z_dir = get_dir(z_disp)

  # check direction change and publish
  if(x_dir != _d_x or y_dir != _d_y or z_dir != _d_z):
    # publish relative velocity change
    publish_movement(x_dir, y_dir, z_dir)
    # update reference d_x,d_y,d_z
    _d_x = x_dir
    _d_y = y_dir
    _d_z = z_dir

  # update reference x,y,z
  _x = data.x
  _y = data.y
  _z = data.z

def detect_movement():
  rospy.Subscriber("/facial_landmarks", LandmarkPos, data_callback)

  rospy.spin()

def publish_movement(x, y, z):
  # publishing to topic: relative_velocity
  velocity_pub = rospy.Publisher('/relative_velocity', VelocityInfo, queue_size=10)
  rate = rospy.Rate(1000) # 1000 Hz
  vel = VelocityInfo()
  vel.x = x
  vel.y = y
  vel.z = z
  velocity_pub.publish(vel)
  rate.sleep()
  return

if __name__ == '__main__':
  try:
    initialize()
    detect_movement()
  except rospy.ROSInterruptException:
    pass

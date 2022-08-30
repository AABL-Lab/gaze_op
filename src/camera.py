#!/usr/bin/env python3
# this gets live video, extracts relevant facial landmarks, 
# and publishes them to facial_landmarks topic
# ----------------------------------------------------------------------------
import time

# ROS
import rospy
from gaze_op.msg import LandmarkPos

# mediapipe
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_mesh = mp.solutions.face_mesh
# ----------------------------------------------------------------------------
_NOSE_TIP_LANDMARK_INDEX = 4
# ----------------------------------------------------------------------------

def initialize():
  print("INFO: Booting up...")
  time.sleep(3) # for kortex_driver to boot first when roslaunch used
  print("INFO: Initializing ROS node \"face_analyser\"")
  rospy.init_node("face_analyser")
  return

def begin_tracking():
  print("INFO: Beginning facial landmarks processing")
  cap = cv2.VideoCapture(0)
  # setting mediaipipe facemesh configuration options
  with mp_face_mesh.FaceMesh(
      max_num_faces=1,
      refine_landmarks=False,
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5) as face_mesh:
    # begin video capture and process frame-by-frame (each frame is an image)
    while cap.isOpened():
      # get frame image
      success, image = cap.read()
      if not success:
        print("Ignoring empty camera frame.")
        continue

      # convert image to RGB format and process Face Mesh
      image.flags.writeable = False # to improve performance
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
      results = face_mesh.process(image)

      # processing face landmarks from face mesh
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
          # draw the face mesh annotations on the image.
          mp_drawing.draw_landmarks(
              image=image,
              landmark_list=face_landmarks,
              connections=mp_face_mesh.FACEMESH_TESSELATION,
              landmark_drawing_spec= None,
              connection_drawing_spec=mp_drawing_styles
              .get_default_face_mesh_tesselation_style())
          mp_drawing.draw_landmarks(
              image=image,
              landmark_list=face_landmarks,
              connections=mp_face_mesh.FACEMESH_CONTOURS,
              landmark_drawing_spec=None,
              connection_drawing_spec=mp_drawing_styles
              .get_default_face_mesh_contours_style())

          # extracting information about headpose changes note: visibility is assumed 
          nosetip = list(face_landmarks.landmark)[_NOSE_TIP_LANDMARK_INDEX]
          
          # call function to publish landmark information
          publish_pos(nosetip.x, nosetip.y, abs(nosetip.z)) # note: z is always -0._ so this is a hack

      cv2.imshow('MediaPipe Face Mesh', cv2.flip(image, 1))
      if cv2.waitKey(5) & 0xFF == 27:
        break
  cap.release()
  return

# function that publishes landmark position to facial_landmarks topic
def publish_pos(x, y, z):
  # publishing to topic: facial_landmarks 
  # -- note: misleading name. simply publishing nose tip position
  landmark_pub = rospy.Publisher('facial_landmarks', LandmarkPos, queue_size=10)
  rate = rospy.Rate(1000)
  position = LandmarkPos()
  position.x = x
  position.y = y
  position.z = z
  landmark_pub.publish(position)
  rate.sleep()
  return

if __name__ == '__main__':
  try:
    initialize()
    begin_tracking()
  except rospy.ROSInterruptException:
    pass

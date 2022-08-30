# arm class with fucntionality required for this project (rest is stripped)
# ----------------------------------------------------------------------------
import numpy as np
import time 

# ROS
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kortex_driver.srv import *
from kortex_driver.msg import *
# ----------------------------------------------------------------------------

class Arm:
  def __init__(self, name):
    try:
        self.HOME_ACTION_IDENTIFIER = 2

        self.action_topic_sub = None
        self.all_notifs_succeeded = True

        self.all_notifs_succeeded = True

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', name)
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

        # setup defult translation and orientation speed 
        # when moving in cartesian space
        self.cartesian_speed = CartesianSpeed()
        self.cartesian_speed.translation = .1 # m/s
        self.cartesian_speed.orientation = 15  # deg/s

        # Init the action topic subscriber
        self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
        rospy.wait_for_service(get_product_configuration_full_name)
        self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

        validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

        self.clear_faults()
        self.set_cartesian_reference_frame()
        self.subscribe_to_a_robot_notification()

    except:
      self.is_init_success = False
    else:
      self.is_init_success = True

  def set_cartesian_velocity(self, values):
    """
    Sends a carteian velocity command

    ---------
    values: list or np array of the form: [x,y,z, x-twist, y-twist, z-twist, w-twist]

    """
    cartesian_vel_publisher = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=10, latch=True)
    stop_publisher = rospy.Publisher("/my_gen3_lite/in/stop", std_msgs.msg.Empty, queue_size=10, latch=True)
    empty_message = std_msgs.msg.Empty()
    cartesian_command = TwistCommand()

    twists = euler_from_quaternion((values[3:]))
    cartesian_command.twist.linear_x = values[0]
    cartesian_command.twist.linear_y = values[1]
    cartesian_command.twist.linear_z = values[2]
    cartesian_command.twist.angular_x = twists[0]
    cartesian_command.twist.angular_y = twists[1]
    cartesian_command.twist.angular_z = twists[2]

    cartesian_vel_publisher.publish(cartesian_command)
    # this sleep is necessary to process the sleep before the next potential command
    time.sleep(0.3)
    # TODO reduce this duration to 0.00000001 for video of case where gaze is confusing
    # stop_publisher.publish(empty_message)
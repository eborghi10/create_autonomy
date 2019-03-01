#!/usr/bin/env python

import math
import rospy
# Import ROS messages
from geometry_msgs.msg import Twist

class Kinematics(object):

  def __init__(self):
    # ROS inits
    rospy.init_node('kinematics_example')
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  
  # Forward kinematics
  def forward(self, duration=0.0, linear=0.0, angular=0.0):
    rospy.loginfo("Moving robot [{}, {}]".format(linear, angular))
    rate_hz = rospy.Rate(10)
    timeout = rospy.get_time() + duration
    while rospy.get_time() <= timeout:
      # Move robot with linear velocity
      vel_msg = Twist()
      vel_msg.linear.x = linear
      vel_msg.angular.z = angular
      self.pub.publish(vel_msg)
      rate_hz.sleep()
    # Stop the robot
    self.pub.publish(Twist())
  
  # Inverse kinematics
  def inverse(self):
    pass

if __name__ == "__main__":
  try:
    kine = Kinematics()
    # Straight
    # V * dt = dX --> dX = 1m
    kine.forward(duration=5.0, linear=0.2)
    # Rotate
    # dR = pi/2
    kine.forward(duration=5.0, angular=math.pi/10.)
  except rospy.ROSInterruptException:
    pass

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


sub_ranges = []

def callback(msg):
    global sub_ranges
    sub_ranges = msg.ranges

sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback)   # Create a Subscriber object that will listen to the /counter


rate = rospy.Rate(2)

forward = Twist()
forward.linear.x = 0.2

turn_left = Twist()
turn_left.linear.x = 0.1
turn_left.angular.z = 0.5

turn_right = Twist()
turn_right.linear.x = 0.1
turn_right.angular.z = -0.5
 

while not rospy.is_shutdown():

  twist = forward
  
  if len(sub_ranges) > 0:
      dist_front = sub_ranges[359]
      dist_right = sub_ranges[19]
      dist_left = sub_ranges[709]

      if dist_front < 1 or dist_right < 1:
          twist = turn_left

      if dist_left < 1:
          twist = turn_right
  
  pub.publish(twist)
  rate.sleep()
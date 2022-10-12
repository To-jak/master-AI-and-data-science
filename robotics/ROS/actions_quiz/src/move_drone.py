#! /usr/bin/env python
import rospy

import actionlib

from std_msgs.msg import Empty
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgFeedback, CustomActionMsgResult

class DroneActionServerClass(object):

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    self._as.start()
    self.r = rospy.Rate(1)

    # Feedback Message
    self._feedback = CustomActionMsgFeedback()

    # Publishers
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)

  def take_off(self):
    self._feedback.feedback = 'TAKING OFF'

    sec = 0
    while sec < 3:
      self._pub_takeoff.publish(Empty())
      self._as.publish_feedback(self._feedback)
      self.r.sleep()
      sec += 1

  def land(self):
    self._feedback.feedback = 'LANDING'

    sec = 0
    while sec < 3:
      self._pub_land.publish(Empty())
      self._as.publish_feedback(self._feedback)
      self.r.sleep()
      sec += 1

  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    
    # helper variable
    success = True
    
    action = goal.goal

    # publish info to the console for the user
    rospy.loginfo('Goal: ' + action)

    # check that preempt (cancelation) has not been requested by the action client
    if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        # the following line, sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
    elif action == 'TAKEOFF':
        self.take_off()
    elif action == 'LAND':
        self.land()
    else:
        rospy.loginfo('Invalid goal')
        success = False
    
    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    if success:
      rospy.loginfo('Action succeeded')
      self._as.set_succeeded(CustomActionMsgResult())
      
if __name__ == '__main__':
  rospy.init_node('Drone_Action_Server')
  DroneActionServerClass()
  rospy.spin()
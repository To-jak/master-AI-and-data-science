#! /usr/bin/env python

import rospy
from unit_4_services.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist 
import time

def stop_move(twist):
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.z = 0

    pub.publish(twist)
    return twist

def forward_move(side):
    move = Twist()
    move.linear.x = 0.2
    pub.publish(move)
    time.sleep(2*side)

    stop_move(Twist())
    time.sleep(3)

def turn_move():
    move = Twist()
    move.angular.z = 0.26
    pub.publish(move)
    time.sleep(3)

    stop_move(Twist())
    time.sleep(1)

def move_in_square(side):
    for i in range(4):
        forward_move(side)
        turn_move()
    
def my_callback(request):
    print "Request Data==> side "+str(request.side)+" repetitions "+str(request.repetitions)
    response = BB8CustomServiceMessageResponse()
    
    try:
        for i in range(request.repetitions):
            move_in_square(request.side)

        response.success = True
    except Exception as e:
        response.success = False

    return response

rospy.init_node('service_server') 
service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , my_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
rospy.spin()
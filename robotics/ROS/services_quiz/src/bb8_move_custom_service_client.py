#! /usr/bin/env python

import rospy
from unit_4_services.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest 
import sys

rospy.init_node('service_client')

rospy.wait_for_service('/move_bb8_in_square_custom')

square_service = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage)

small_square_move = BB8CustomServiceMessageRequest()
small_square_move.side = 1
small_square_move.repetitions = 2
result1 = square_service(small_square_move)

big_square_move = BB8CustomServiceMessageRequest()
big_square_move.side = 2
big_square_move.repetitions = 1
result2 = square_service(big_square_move)

print result1 and result2
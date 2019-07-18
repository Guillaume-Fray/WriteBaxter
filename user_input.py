#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rospy import init_node, is_shutdown

inputPub = rospy.Publisher('/face_rec/user_input',String)

if __name__ == '__main__':
  	init_node('user_input')
  	while not is_shutdown():
		print "enter a word: "
      		something = raw_input()
		inputPub.publish(something)

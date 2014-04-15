#! /usr/bin/env python

import roslib
import rospy


import object_search_gui.client



if __name__ == '__main__':
	print
	print
	print "Demo."
	rospy.init_node("demo")

	# Display some content
	displayNo = rospy.get_param("~display", 0)	

	if displayNo == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', displayNo)

	# Setup -- must be done before other marathon_touch_gui calls
	object_search_gui.client.init_gui()

	# Show the main page of the GUI
	object_search_gui.client.display_main_page(displayNo)

	rospy.sleep(2)
	

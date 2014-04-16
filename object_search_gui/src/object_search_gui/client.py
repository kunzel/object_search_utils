import roslib
import rospy
import os

import strands_webserver.client_utils
import strands_webserver.page_utils

def init_gui():
	""" Do setup stuff """
	# switch the server to display relative to 
	root = os.path.join(roslib.packages.get_pkg_dir('object_search_gui'), "web_content") 
	print "Setting root to:", root
	strands_webserver.client_utils.set_http_root(root)

def display_main_page(display_no):
	""" Display the main GUI page on the given display """
	strands_webserver.client_utils.display_relative_page(display_no, 'index.html')


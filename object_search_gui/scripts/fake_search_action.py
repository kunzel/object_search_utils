#!/usr/bin/python
import rospy

import actionlib

from object_search_action.msg import *
from sensor_msgs.msg import PointCloud2

class SearchActor(object):
  # create messages that are used to publish feedback/result
  _feedback = SearchFeedback()
  _result   = SearchResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, SearchAction,
                                            execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    rospy.loginfo("Search action server up: %s"%self._action_name)
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(10)
    success = True
    
    # append the seeds for the fibonacci sequence
    self._feedback.objs=[]
    self._feedback.state="driving"
    
    # publish info to the console for the user
    rospy.loginfo("Searching for '%s'"%goal.obj_desc)

    states = ["pose_selection", "driving", "taking_image" , "image_analysis"]
    
    for i in states:
      self._feedback.state = i
      self._as.publish_feedback(self._feedback)    

      rospy.loginfo(self._feedback.state)
      j = 0
      while j < 10 * 4:
        j += 1
        r.sleep()
        if self._as.is_preempt_requested():
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          success = False
          break
      else:
        continue
      break


    if success:
      self._result.obj_found= (goal.obj_desc == "Mug") #True
      self._result.obj_desc=["Mug"]
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._result.obj_pointcloud=[PointCloud2()]
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('object_search_action')
  SearchActor(rospy.get_name())
  rospy.spin()

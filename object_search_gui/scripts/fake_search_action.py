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
    r = rospy.Rate(1)
    success = True
    
    # append the seeds for the fibonacci sequence
    self._feedback.objs=[]
    self._feedback.state="driving"
    
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, 10, 0, 1))
    
    # start executing the action
    for i in xrange(1, 10):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
      self._feedback.state+="."
      # publish the feedback
      self._as.publish_feedback(self._feedback)
      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep()
      
    if success:
      self._result.obj_found=True
      self._result.obj_desc=["Mug"]
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._result.obj_pointcloud=[PointCloud2()]
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('object_search')
  SearchActor(rospy.get_name())
  rospy.spin()

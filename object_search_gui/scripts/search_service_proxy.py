#!/usr/bin/env python
import roslib; roslib.load_manifest('object_search_gui')

import os
import getopt
import sys
import json
import rospy
from sensor_msgs.msg import *
import tf
import time
import math

# Import opencv
import cv
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError  # to convert sensor_msgs to OpenCV image
from roslib import message
from geometry_msgs.msg import Point32, PoseStamped
from image_geometry import PinholeCameraModel

import actionlib

from object_search_action.msg import *

class ObjectSearchProxy(object):
    """
    An action server that passes on to the object search action server.
    Adds projection of analysed points into an image as feedback.
    """

    def __init__(self):
        rospy.init_node('object_search_proxy')

        self.bridge = CvBridge()
        
        self._action_client =  actionlib.SimpleActionClient("object_search_action",
                                                            SearchAction)
        
        self._action_server = actionlib.SimpleActionServer("object_search_proxy",
                                                           SearchAction,
                                                           auto_start=False)
        self._action_server.register_goal_callback(self._goal_received_cb)
        self._action_server.register_preempt_callback(self._preempt_cb)
        
        self._success = True
        self._tf_listener = tf.TransformListener(190)

        self.camera_image_topic = "/head_xtion/rgb/image_color"
        self.camera_image_info_topic = "/head_xtion/rgb/camera_info"
        self.camera_image_output_topic = "/object_search/image"
        
        self._goal_robot_pose = None
        self._point_clouds = None
            
        self._image = None
        self._image_time = rospy.Time.now()
        self._image_refresh = True # Should new images be kept?
        r = rospy.Rate(1)
        sub = rospy.Subscriber(self.camera_image_topic, Image, self.image_cb)
        #while self._image is None:
            #rospy.loginfo("Waiting for image subscription...")
            #r.sleep()
        rospy.loginfo("Got one.")

        self._image_info = None
        sub = rospy.Subscriber(self.camera_image_info_topic, CameraInfo, self.image_info_cb)
        #while self._image_info is None:
            #rospy.loginfo("Waiting for image calibration subscription...")
            #r.sleep()
        rospy.loginfo("Got it.")

        # Image publisher
        self._image_publisher = rospy.Publisher(self.camera_image_output_topic,
                                                sensor_msgs.msg.Image)
        
        self._current_mode = ""

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_cb)

    def goal_pose_cb(self,goal_pose):
        self._goal_robot_pose=goal_pose


    def image_cb(self,image):
        if self._image_refresh:
            try:
                #self._image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                self._image = self.bridge.imgmsg_to_cv2(image)
                self._image_time = image.header.stamp
            except CvBridgeError, e:
                print e
                
            self._render_image()
        self._image_publisher.publish(self.bridge.cv2_to_imgmsg(self._image,
                                                                encoding="bgr8"))
        
            
    def image_info_cb(self, image_info):
        self._image_info =  PinholeCameraModel()
        self._image_info.fromCameraInfo(image_info)
        isinstance(self._image_info, PinholeCameraModel)

    def _project_pointcloud(self, cloud):
        """
        Projects the given cloud into the current image.
        Assumes robot has not moved between NOW and when the cloud was captured...
        Returns rectangle in image plane
        """

        assert isinstance(cloud, PointCloud2)

        pc1 = PointCloud()
        pc1.header = cloud.header
        # hack the time! dont move the robot :-0
        pc1.header.stamp = rospy.Time.now()

        pc1.points = [Point32(*p) for p in pc2.read_points(cloud)]

        self._tf_listener.waitForTransform(pc1.header.frame_id,
                                           self._image_info.tf_frame, 
                                           rospy.Time(0), 
                                           rospy.Duration(4))

        image_frame_cloud = self._tf_listener.transformPointCloud (
            self._image_info.tf_frame, 
            pc1)
        min_x, max_x, min_y, max_y =  640, 0, 480, 0 # TODO: remove hard coded image size!
        for pt in image_frame_cloud.points:
            u, v = self._image_info.project3dToPixel((pt.x, pt.y, pt.z))
            if v < min_y:
                min_y = int(v)
            if v > max_y:
                max_y = int(v)
            if u < min_x:
                min_x = int(u)
            if u > max_x:
                max_x = int(u)
        location = (((min_x, min_y), (max_x, max_y)))
        rospy.loginfo("Transformed cloud into image plane")
        return location

    def _goal_received_cb(self):
        """
        Pass on the goal to the underlying action server
        """
        rospy.loginfo("[Server] Goal received, passing it on.")
        self._action_client.wait_for_server()
        self._success = True
        goal = self._action_server.accept_new_goal()
        self._action_client.send_goal(goal, self._result_received_cb,
                                      self._active_cb,
                                      self._feedback_received_cb)


    def _active_cb(self):
        rospy.loginfo("[Client] Goal went active.")
        self._image_refresh = True
        self._success = True
        
    def _preempt_cb(self):
        rospy.loginfo("[Server] Goal preempt.")
        self._image_refresh = False
        self._action_client.cancel_goal()
        self._success = False
        self._action_server.set_preempted()

    def _feedback_received_cb(self, feedback):
        """
        Process feedback, add images etc

        feedback struct:
        
        pointcloud[] objs
        pose goal_pose
        string state ( = {pose_selection, driving, taking_image, image_analysis, } )
        need artificial pause after analyse...
        """

        # Lock current image...
        self._image_refresh = False
        
        if feedback.state == "driving":
            # project the feedback goal_pose into image
            self._goal_robot_pose = feedback.goal_pose
            self._current_mode = "Moving to next view point."
        else:
            self._goal_robot_pose = None

        if feedback.state == "image_analysis":
            # don't overwrite current annotated image...
            # project stuff into image....
            self._point_clouds = feedback.objs
            self._current_mode = "Analysing scene."
        else:
            self._point_clouds = None

        if feedback.state == "taking_image":
            self._current_mode = "Aquiring depth image."

        if feedback.state == "pose_selection":
            self._current_mode = "Choosing where to go."
        
        self._action_server.publish_feedback(feedback)
        time.sleep(0)
        self._image_refresh = True
        
        
    def _result_received_cb(self, state, result):
        rospy.loginfo("[Client] Result received.")
        #self._image_refresh = False

        print "Done."
        #print result
        if self._success:
            self._action_server.set_succeeded(result)
            self._image_refresh = False
            self._current_mode = "Object located."
            self._image_refresh = True


    def _render_static_image_annotation(self):
        """
        Render some static annotation on the image - things that go on every
        image always.
        """
        cv2.putText(self._image,self._current_mode, (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)

        cv2.putText(self._image, time.asctime(), (400, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)

    
    def _render_image(self):
        """
        Renders an image with pointclouds highlighted
        """

        self._render_static_image_annotation()

        if self._goal_robot_pose is not None:
            # Render the goal pose as the robot is driving to target...
            self._goal_robot_pose.header.stamp = self._image_time
            self._tf_listener.waitForTransform('/map',
                                               self._image_info.tf_frame, 
                                               self._image_time,
                                               rospy.Duration(4))

            self._goal_robot_pose.pose.position.z=1.5 # force goal point to be 1.5m
            pose = self._tf_listener.transformPose(self._image_info.tf_frame,
                                            self._goal_robot_pose)
            u, v = self._image_info.project3dToPixel((pose.pose.position.x,
                                                      pose.pose.position.y,
                                                      pose.pose.position.z))
            self._goal_robot_pose.pose.position.z=1.45 # force goal point to be 1.5m
            pose = self._tf_listener.transformPose(self._image_info.tf_frame,
                                            self._goal_robot_pose)
            u2, v2 = self._image_info.project3dToPixel((pose.pose.position.x,
                                                      pose.pose.position.y,
                                                      pose.pose.position.z))
            radius = int(math.sqrt((u2-u)**2 + (v2-v)**2))
            cv2.putText(self._image,  "Goal Location", (int(u+radius+1), int(v+radius+1)),
                    cv2.FONT_HERSHEY_SIMPLEX, radius/10.0, 255, radius/200 * 3)
            cv2.circle(self._image, (int(u),int(v)), radius, (0,0,255,127),-1)



        if self._point_clouds is not None:
            cv2.putText(self._image,  "Objects", (400, 400),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 2)
            # Render the bouding boxes of objects...
            # Project each response cluster into image
            box_locations = []
            for cloud in self._point_clouds:
                location = self._project_pointcloud(cloud)
                box_locations.append(location)
                cv.Rectangle(cv.fromarray(self._image),
                             location[0], location[1],
                             (255, 255, 0, ),
                             4)
                cv.PutText(cv.fromarray(self._image,
                                        "Analysing object",
                                        (location[0][0], location[0][1]-10),
                                        text_font,
                                        (255, 0, 255, 255)))
        
                
    def start(self):
        self._action_server.start()
        rospy.loginfo("Object search proxy spinning")
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    proxy = ObjectSearchProxy()
    proxy.start()








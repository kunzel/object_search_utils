/*
	Author : Keerthi Kumar
*/

#include "ros/ros.h"
//#include "view_evaluation/BestViewCone2D.h"
//#include "view_evaluation/BestViewCone3D.h"
#include <cstdlib>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
#include "Vec33.cpp"
#include <tf/transform_datatypes.h>
#include <nav_goals_msgs/NavGoals.h>
//#include <view_evaluation/NavGoals3D.h>
#include "view_evaluation/BestViews.h"
#include "view_evaluation/BestViewsVisualiser.h"


/* Finds the best view pose and moves the robot to that pose.
*/

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose bestPose2D, bestPose3D;
geometry_msgs::PoseArray poseArray2D, poseArray3D;
int num_poses2D, num_poses3D;

// void navigate( geometry_msgs::Pose bestPose)
// {
//   //tell the action client that we want to spin a thread by default
//   MoveBaseClient navigator("move_base", true);

//   //wait for the action server to come up
//   while(!navigator.waitForServer())
//   {
//   	ROS_INFO("Waiting for the move_base action server to come up");
//   }
//   move_base_msgs::MoveBaseGoal goal;

//   goal.target_pose.header.frame_id = "map";
//   goal.target_pose.header.stamp = ros::Time::now();
//   goal.target_pose.pose.position.x = bestPose.position.x;
//   goal.target_pose.pose.position.y = bestPose.position.y;
//   goal.target_pose.pose.position.z = bestPose.position.z;
//   goal.target_pose.pose.orientation.x = bestPose.orientation.x;
//   goal.target_pose.pose.orientation.y = bestPose.orientation.y;
//   goal.target_pose.pose.orientation.z = bestPose.orientation.z;
//   goal.target_pose.pose.orientation.w = bestPose.orientation.w;

//   ROS_INFO("Sending goal");
//   navigator.sendGoal(goal);
//   navigator.waitForResult();

//   if(navigator.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Robot moved to the goal");
//   else
//     ROS_INFO("The base failed to move robot for some reason");
// }


int main(int argc, char **argv)
{
  int bestCone2D = 0;
  int bestCone3D = 0;
  int num_random_poses2D = 15;
  int num_random_poses3D = 3;
  int num_best_poses  = 6;
  float probability2D = 0.0;
  float probability3D = 0.0;
  float inflation_radius2D = 0.3;
  float inflation_radius3D = 0.3;
 

  // Initialize ROS
  ros::init(argc, argv, "Best_View_Cones_Client");

  ros::NodeHandle node;

  ros::Publisher pub_cones2D, pub_bestCones2D, pub_cones3D; 
  ros::Publisher pub_poses2D, pub_poses3D;
  ros::Publisher pub_range3D;

  //pub_frustrum_points = node.advertise<visualization_msgs::MarkerArray>("frustrum", 20);

  // Create a ROS client for the service 'nav_goals' Service file is NavGoals.srv.
  ros::ServiceClient random_poses2D = node.serviceClient<nav_goals_msgs::NavGoals>("nav_goals");
  nav_goals_msgs::NavGoals srv_random_poses2D;

  // Create a ROS client for the service 'nav_goals_3D'. Service file is NavGoals3D.srv
  //ros::ServiceClient random_poses3D = node.serviceClient<view_evaluation::NavGoals3D>("nav_goals_3D");
  //view_evaluation::NavGoals3D srv_random_poses3D;

  // Create a ROS client for the service 'best_view_cone_2D'. Service file is BestViewCone2D.srv
  //ros::ServiceClient client_2D = node.serviceClient<view_evaluation::BestViewCone2D>("best_view_cone_2D");
 //view_evaluation::BestViewCone2D srv_2D;

  //Create a ROS client for the service 'best_view_cone_3D'. Service file is BestViewCone3D.srv
 // ros::ServiceClient client_3D = node.serviceClient<view_evaluation::BestViewCone3D>("best_view_cone_3D");
  //view_evaluation::BestViewCone3D srv_3D;

   // Create a ROS client for the service 'best_view_cone_3D'. Service file is bestViews.srv
  ros::ServiceClient node_3D = node.serviceClient<view_evaluation::BestViews>("bestViews");
  view_evaluation::BestViews srv_node_3D;

 // Get the random 2D poses:

  srv_random_poses2D.request.n = num_random_poses2D;
  srv_random_poses2D.request.inflation_radius = inflation_radius2D;
  //srv_random_poses2D.request.roi = " ";

  if (random_poses2D.call(srv_random_poses2D))
  {
    std::cerr << "service nav_goals received: " << std::endl;
    poseArray2D = (geometry_msgs::PoseArray)srv_random_poses2D.response.goals;
    std::cerr << "poseArray2D size: " << poseArray2D.poses.size()<<std::endl;
    num_poses2D = poseArray2D.poses.size();
  }
  else
  {       
    std::cerr << "service nav_goals not received: " << std::endl;
    return 1;
  }

  std::vector<float> pan(num_random_poses3D);
  pan.push_back (25.0);
  pan.push_back (65.0);
  pan.push_back (85.0);

  std::vector<float> tilt(num_random_poses3D);
  tilt.push_back (90.0);
  tilt.push_back (45.0);
  tilt.push_back (10.0);
  
  srv_node_3D.request.randomPoses = poseArray2D;
  srv_node_3D.request.pan_angles = pan;
  srv_node_3D.request.tilt_angles = tilt;
  srv_node_3D.request.num_best_poses = num_best_poses;

  if (node_3D.call(srv_node_3D))
  {
  	std::cerr << "service bestViews  received: " << std::endl; 
  }
  else
  {
  	std::cerr << "service bestViews not received: " << std::endl;
	return 1;
  }

  // Create a ROS client for the service 'bestViewsVisualiser'. Service file is bestViewsVisualiser.srv
  ros::ServiceClient client = node.serviceClient<view_evaluation::BestViewsVisualiser>("bestViewsVisualiser");
  view_evaluation::BestViewsVisualiser srv;

  srv.request.randomPoses = poseArray2D;
  srv.request.pan_angles = pan;
  srv.request.tilt_angles = tilt;
  srv.request.num_best_poses = num_best_poses ;

  geometry_msgs::PoseArray poses2D, poses3D;
  visualization_msgs::MarkerArray cone2D_markers, bestCone2D_markers, cone3D_markers;

  if (client.call(srv))
  {
    	poses2D = (geometry_msgs::PoseArray)srv.response.poses2D;
	//poses3D = (geometry_msgs::PoseArray)srv.response.poses3D;
    	cone2D_markers = (visualization_msgs::MarkerArray)srv.response.cones2D;
        bestCone2D_markers = (visualization_msgs::MarkerArray)srv.response.bestCones2D;
	cone3D_markers = (visualization_msgs::MarkerArray)srv.response.cones3D;
	num_poses2D = poses2D.poses.size();
	//num_poses3D = poses3D.poses.size();
    	std::cerr << "num_poses2D: " <<num_poses2D<< std::endl;
    	std::cerr << "cone2D_markers: " <<cone2D_markers.markers.size() << std::endl;
	//std::cerr << "num_poses3D: " <<num_poses3D<< std::endl;
    	std::cerr << "cone3D_markers: " <<cone3D_markers.markers.size() << std::endl;
  }
  else
  {
    	ROS_ERROR("Failed to call service bestViewsVisualiser");
    	return 1;
  }
  
  pub_cones2D = node.advertise<visualization_msgs::MarkerArray>("view_cones2D", 1);
  pub_bestCones2D = node.advertise<visualization_msgs::MarkerArray>("best_view_cones2D", 1);
  pub_cones3D = node.advertise<visualization_msgs::MarkerArray>("view_cones3D", 1);
  pub_poses2D = node.advertise<geometry_msgs::PoseArray>("poses2D", 1);
  //pub_poses3D = node.advertise<geometry_msgs::PoseArray>("poses3D", 1);
  //pub_range3D = node.advertise<sensor_msgs::Range>("range_views3D", 50);

  //sensor_msgs::Range cones3D[num_poses3D];
  //tf::Transform tf_poses3D [num_poses3D];
  num_poses3D = 6;
  while(ros::ok())
  {
	pub_cones2D.publish(cone2D_markers);
        pub_bestCones2D.publish(bestCone2D_markers);
	pub_cones3D.publish(cone3D_markers);
	pub_poses2D.publish(poses2D);
 	//pub_poses3D.publish(poses3D);
	
/*
	for( int i = 0; i < num_poses3D; i++) 
  	{
    		std::stringstream temp_string;

		temp_string << "tf_range_cones3D_"  << i;
		std::string frame_id = temp_string.str();
	  	tf_poses3D[i] = tf::Transform::getIdentity();
	  	tf::poseMsgToTF(poses3D.poses[i], tf_poses3D[i]);

		cones3D[i].field_of_view = field_of_view;
		cones3D[i].header.frame_id = frame_id;
		cones3D[i].max_range = max_range;
		cones3D[i].min_range = min_range;
		cones3D[i].radiation_type = radiation_type;
		cones3D[i].range = range;

		br.sendTransform(tf::StampedTransform(tf_poses3D[i], ros::Time::now(), "map", frame_id));
		pub_range3D.publish(cones3D[i]);
  	}

	*/
  }
  
/*
  // Find the best 2D view cone position:
  srv_2D.request.poseArray2D = poseArray2D;
  if (client_2D.call(srv_2D))
  {
	std::cerr << "service best_view_cone_2D received: " << std::endl;
  	bestCone2D = (int)srv_2D.response.bestConeId;
    	probability2D = (float)srv_2D.response.probability;
    	bestPose2D = (geometry_msgs::Pose)srv_2D.response.bestViews;
    	
	std::cerr << "bestCone2D ID: " <<(int)srv_2D.response.bestConeId << std::endl;
    	std::cerr << "bestCone2D Probability: " <<(float)srv_2D.response.probability << std::endl;
	std::cerr << "bestCone2D X: " <<bestPose2D.position.x << std::endl;
	std::cerr << "bestCone2D Y: " <<bestPose2D.position.y << std::endl;
	std::cerr << "bestCone2D Z: " <<bestPose2D.position.z << std::endl;
	std::cerr << "bestCone2D XX: " <<bestPose2D.orientation.x << std::endl;
	std::cerr << "bestCone2D YY: " <<bestPose2D.orientation.y << std::endl;
	std::cerr << "bestCone2D ZZ: " <<bestPose2D.orientation.z << std::endl;
	std::cerr << "bestCone2D WW: " <<bestPose2D.orientation.w << std::endl;
	
        
	// Get the random 3D poses:
        srv_random_poses3D.request.n = num_random_poses3D;
	srv_random_poses3D.request.inflation_radius = inflation_radius3D;
	srv_random_poses3D.request.position = bestPose2D;
	//srv_random_poses3D.request.roi = " ";

	if (random_poses3D.call(srv_random_poses3D))
	{
		std::cerr << "service nav_goals_3D received: " << std::endl;
		poseArray3D = (geometry_msgs::PoseArray)srv_random_poses3D.response.goals;
		std::cerr << "poseArray3D size: " << poseArray3D.poses.size()<<std::endl;
		num_poses3D = poseArray3D.poses.size();


		// Find the best 3D view cone position:
		srv_3D.request.poseArray3D = poseArray3D;
		if (client_3D.call(srv_3D))
	  	{
			std::cerr << "service best_view_cone_3D received: " << std::endl;
		  	bestCone3D = (int)srv_3D.response.bestConeId;
		    	probability3D = (float)srv_3D.response.probability;
		    	bestPose3D = (geometry_msgs::Pose)srv_3D.response.bestViews;
		    	
			std::cerr << "bestCone3D ID: " <<(int)srv_3D.response.bestConeId << std::endl;
		    	std::cerr << "bestCone3D Probability: " <<(float)srv_3D.response.probability << std::endl;
			std::cerr << "bestCone3D X: " <<bestPose3D.position.x << std::endl;
			std::cerr << "bestCone3D Y: " <<bestPose3D.position.y << std::endl;
			std::cerr << "bestCone3D Z: " <<bestPose3D.position.z << std::endl;
			std::cerr << "bestCone3D XX: " <<bestPose3D.orientation.x << std::endl;
			std::cerr << "bestCone3D YY: " <<bestPose3D.orientation.y << std::endl;
			std::cerr << "bestCone3D ZZ: " <<bestPose3D.orientation.z << std::endl;
			std::cerr << "bestCone3D WW: " <<bestPose3D.orientation.w << std::endl;
		
	  	}
	  	else
	  	{
		    	ROS_ERROR("Failed to call service best_view_cone_3D");
		    	return 1;
	  	}
		}
		else
		{
			std::cerr << "service nav_goals_3D not received: " << std::endl;
			return 1;
		}

	
  }
  else
  {
    	ROS_ERROR("Failed to call service best_view_cone_2D");
    	return 1;
  }
 

  // generate a goal to navigate to the best 2Dpose and orient to the best 3D pose.
  //navigate(bestPose2D);
  //navigate(bestPose3D);
*/
/*
  FrustumG frustrum;

//Vec3 p(0,0,5),l(0,0,0),u(0,1,0);
//Vec3 a(i,0,k);
//Plane plane(p, l, a);
  //float nearD = 0.05, farD = 0.3;
  //float angle = 45, ratio = 1;

  // Convert quaternion to RPY.
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(bestPose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
bestPose.position.z = 1.0;
	std::cerr << "roll: " <<roll << std::endl;
	std::cerr << "pitch: " <<pitch << std::endl;
	std::cerr << "yaw: " <<yaw << std::endl;
  //Vec3 p(bestPose.pose.x, bestPose.pose.y, bestPose.pose.z);
  Vec3 p(bestPose.position.x, bestPose.position.y, bestPose.position.z);
  Vec3 l(roll, pitch, yaw);
  Vec3 u(0, 0, 1);
  


  frustrum.setCamInternals(angle, ratio, nearD, farD);
  //frustrum.setCamDef(p, l, u);
  bool result = frustrum.pointInFrustum(a);
  std::cerr << "result: " <<result << std::endl;
  drawPoints(frustrum);
*/
  

  return 0;
}

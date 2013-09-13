#include <iostream>
#include <stdlib.h>  
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <string>
//#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
//#include "view_evaluation/tfCones.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include "Vec33.cpp"
#include <view_evaluation/BestViews.h>
#include "view_evaluation/BestViewsVisualiser.h"

using namespace std;
using namespace octomap;

#define QSR_WEIGHT 100
#define QSR_MEAN_1 0
#define QSR_MEAN_2 0
#define QSR_VAR_1 1
#define QSR_VAR_2 1


double normal_dist_2d(double x, double y, double mean1, double var1, double mean2, double var2)
  {
    return (1.0 / (2.0 * M_PI * sqrt(var1) * sqrt(var2)) * exp(-1 * ( pow(x - mean1,2) / ( 2.0 * var1) +  pow(y - mean2,2) / ( 2.0 * var2) )));
  }


float trapezoidal_shaped_func(float a, float b, float c, float d, float x)
{  
  float min = std::min(std::min((x - a)/(b - a), (float) 1.0), (d - x)/(d - c));
  return std::max(min, (float) 0.0);
}


float r_func(float x)
{
  float a = -0.125;
  float b =  0.125;
  float c =  0.375;
  float d =  0.625;

  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return value;
}

float g_func(float x)
{
  float a =  0.125;
  float b =  0.375;
  float c =  0.625;
  float d =  0.875;

  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return value;
}

float b_func(float x)
{
  float a =  0.375;
  float b =  0.625;
  float c =  0.875;
  float d =  1.125;
  
  x = 1.0 - x;
  
  float value = trapezoidal_shaped_func(a,b,c,d,x);
  
  return value;
}




class Cone
{
  public:
  geometry_msgs:: Point points[4];
  float radius;
  float height;
  float angle;
  float probability;
  int id;

};

  float map_min_x; 
  float map_max_x;
  float map_min_y;
  float map_max_y;
  int map_width;
  int map_height;
  float map_origin_x;
  float map_origin_y;
  float map_resolution;
  int map_received = 0;
  nav_msgs::OccupancyGrid mapData;
  float radius = 0.5; // Radius of cone circle.
  float length = 2.0; // Height of the cone.

bool checkInsideTraingle(float x, float y, Cone cone2D) 
{
  float x1, y1, x2, y2, x3, y3;
  
  x1 = cone2D.points[0].x;
  x2 = cone2D.points[1].x;
  x3 = cone2D.points[3].x;
  y1 = cone2D.points[0].y;
  y2 = cone2D.points[1].y;
  y3 = cone2D.points[3].y;
  
  float A = 0.5 * (-y2 * x3 + y1 * (-x2 + x3) + x1 * (y2 - y3) + x2 * y3);
  float sign;
	
  if (A < 0)
  	sign = -1.0;
  else
  	sign = 1.0;

 float s = (y1 * x3 - x1 * y3 + (y3 - y1) * x + (x1 - x3) * y) * sign;
 float t = (x1 * y2 - y1 * x2 + (y1 - y2) * x + (x2 - x1) * y) * sign;
    
 return s > 0 && t > 0 && (s + t) < 2 * A * sign;
}


bool insideCircle(float x, float y, Cone cone2D)
{
  float radius = cone2D.radius;
  float xmid, ymid;

  xmid = cone2D.points[2].x;
  ymid = cone2D.points[2].y; 
  float distance = sqrt( pow(xmid - x, 2) + pow(ymid - y, 2) );
  //std::cerr << "x:" <<x << " y:" <<y << " c1:" <<cone.x3 << " c2:"<<cone.y3<< " dist:"<<distance<< std::endl;
  if (distance <= radius)
	return true;
  else
	return false;
}


bool checkInsideCone3D(float x, float y, float z, Frustum cone3D)
{ 
  bool result;
  Vec3 point(x, y, z);
  
  result = cone3D.pointInFrustum(point);
  //if (result)
  //std::cerr<< "Inside cone: "<< result<< std::endl;	
  return result;
}


bool checkInsideCone2D(int x, int y, Cone cone2D)
{
  float cell_x, cell_y;
  //std::cerr << "inside checkInsideCone2D " <<x<<y << std::endl;
  cell_x = x * map_resolution + map_origin_x;
  cell_y = y * map_resolution + map_origin_y;
  if (checkInsideTraingle(cell_x, cell_y, cone2D) || insideCircle(cell_x, cell_y, cone2D))
  {
	//std::cerr << "checkInsideCone2D true " <<x<<y << std::endl;
  	return true;
  }
  else
  {
	//std::cerr << "checkInsideCone2D false " <<x<<y << std::endl;
	return false;
  }
}

void findProbabilityOfCones2D(Cone cones2D[], int num_poses2D)
{

  int count = 0;
  int occupied_count = 0;
  int occupied_count_check = 0;
  int free_count = 0;
  int unknown_count = 0;

  std::cerr<< "inside findProbabilityOfCones2D:"<<  std::endl;
  
  for (int i = 0; i < map_height; i++)
    {
      for (int j = 0; j < map_width; j++)
        {
          if (mapData.data[i*map_width+j] > 0)//(mapArray[i][j] > 0)
            {
              occupied_count_check++;
              for (int k = 0; k < num_poses2D; k++)
                {
                  if (checkInsideCone2D(j, i, cones2D[k]))
                    {
                      cones2D[k].probability += 1; 

                      double x_pos = j * map_resolution + map_origin_x;
                      double y_pos = i * map_resolution + map_origin_y;
            
                      cones2D[k].probability += 1 + (QSR_WEIGHT * (normal_dist_2d(x_pos, y_pos , QSR_MEAN_1 , QSR_VAR_1 , QSR_MEAN_2 , QSR_VAR_2))); 


                    }
                }
            }
        }
    }
  for (int k = 0; k < num_poses2D; k++)
    {
      std::cerr<< std::endl<< "cone-"<< k <<" = "<<cones2D[k].probability<< std::endl;
    }
}


//float xx[70000], yy[70000], zz[70000];
void findProbabilityOfCones3D(Frustum frustum[], int num_poses3D)
{
  std::string map = "octomap.bt";
   OcTree* input_tree = new OcTree(map);
  int free = 0;
  int occupied = 0;


  std::cerr<<"Inside findProbabilityOfCones3D"<< std::endl;
    for(OcTree::leaf_iterator it = input_tree->begin_leafs(),
        end=input_tree->end_leafs(); it!= end; ++it)
    {
      if (input_tree->isNodeOccupied(*it))
        {
		double size = it.getSize();
		double x = it.getX();
		double y = it.getY();
		double z = it.getZ();
		//point3d p = it.getCoordinate();
		//xx[occupied] = p.x;
		//yy[occupied] = p.y;
		//zz[occupied] = p.z;
		occupied++;
		//std::cerr<<"X: "<< x<< std::endl;
		//std::cerr<<"Y: "<< y<< std::endl;
		//std::cerr<<"Z: "<< z<< std::endl;
		for (int i = 0; i < num_poses3D; i++)
		{
			if (checkInsideCone3D(x, y, z, frustum[i]))
			{
        
				frustum[i].probability += 1 + (QSR_WEIGHT * (normal_dist_2d(x, y , QSR_MEAN_1 , QSR_VAR_1 , QSR_MEAN_2 , QSR_VAR_2)));
			}
			
		} 

        } 
	else 
        {
          free++;
        }
    }
    std::cerr<<"occupied "<< occupied<< std::endl;
    std::cerr<<"free "<< free<< std::endl;
    for (int i = 0; i < num_poses3D; i++)
    {
 	//std::cerr<<"After Probability 3D Cone "<<i<< ": "<< frustum[i].probability<< std::endl;
    }

}


void generateCones2D(Cone cones[], int num_poses2D, geometry_msgs::PoseArray poseArray2D)
{  
  float left[] = { 2.0, -0.5};
  float right[] = { 2.0, 0.5};
  float middle[] = { 2.0, 0.0};
  tf::Transform tf_poses[num_poses2D];

  std::cerr << "inside generateCones2D" << std::endl;
  for (int i = 0; i < num_poses2D; i++)
  {
  	geometry_msgs::Point conePoints[3];
	tf::Vector3 old_pos[3];
	tf::Vector3 new_pos[3];

	conePoints[0].x = left[0];
	conePoints[0].y = left[1];
	conePoints[1].x = middle[0];
	conePoints[1].y = middle[1];
	conePoints[2].x = right[0];
	conePoints[2].y = right[1];
	tf::poseMsgToTF(poseArray2D.poses[i], tf_poses[i]);
	
	
	for (int l = 0; l < 3; l++)
	{
		old_pos[l].setX(conePoints[l].x);
		old_pos[l].setY(conePoints[l].y);
		old_pos[l].setZ(conePoints[l].z);
		new_pos[l] = tf_poses[i] * old_pos[l];
	}

	cones[i].points[0].x = poseArray2D.poses[i].position.x;
	//std::cerr<<"X: "<<cones[i].points[0].x<<std::endl;
	cones[i].points[0].y = poseArray2D.poses[i].position.y;
	//std::cerr<<"Y: "<<cones[i].points[0].y<<std::endl;
	cones[i].points[0].z = poseArray2D.poses[i].position.z;
	//std::cerr<<"Z: "<<cones[i].points[0].z<<std::endl;

	for (int l = 0; l < 3; l++)
	{
		cones[i].points[l+1].x = new_pos[l].getX();
		cones[i].points[l+1].y = new_pos[l].getY();
		cones[i].points[l+1].z = new_pos[l].getZ();
		//std::cerr<<"X: "<<cones[i].points[l+1].x<<std::endl;
		//std::cerr<<"Y: "<<cones[i].points[l+1].y<<std::endl;
		//std::cerr<<"Z: "<<cones[i].points[l+1].z<<std::endl;
	}

	cones[i].radius = radius;
	cones[i].height = length;
	cones[i].id = i;
	cones[i].probability = 0.0;
  }
}


void generateCones3D(Frustum frustum[], int num_poses3D, geometry_msgs::PoseArray poseArray3D)
{
  
  Vec3 points[8];
/*
ntl: 0.1
ntr: 0.1
nbl: 0.1
nbr: 0.1
ftl: 1.5
ftr: 1.5
fbl: 1.5
fbr: 1.5


ntl: 0.0221695
ntr: -0.0221695
nbl: 0.0221695
nbr: -0.0221695
ftl: 0.332542
ftr: -0.332542
fbl: 0.332542
fbr: -0.332542

ntl: 0.0221695
ntr: 0.0221695
nbl: -0.0221695
nbr: -0.0221695
ftl: 0.332542
ftr: 0.332542
fbl: -0.332542
fbr: -0.332542

*/
  points[0].x = 0.1;//frustum.ntl;
  points[0].y = 0.0221695;//frustum.ntl;
  points[0].z = 0.0221695;//frustum.ntl;
  points[1].x = 0.1;//frustum.ntr;
  points[1].y = -0.0221695;//frustum.ntr;
  points[1].z = 0.0221695;//frustum.ntr;
  points[2].x = 0.1;//frustum.nbl;
  points[2].y = 0.0221695;//frustum.nbl;
  points[2].z = -0.0221695;//frustum.nbl;
  points[3].x = 0.1;//frustum.nbr;
  points[3].y = -0.0221695;//frustum.nbr;
  points[3].z = -0.0221695;//frustum.nbr;

  points[4].x = 1.5;//frustum.ftl;
  points[4].y = 0.332542;//frustum.ftl;
  points[4].z = 0.332542;//frustum.ftl;
  points[5].x = 1.5;//frustum.ftr;
  points[5].y = -0.332542;//frustum.ftr;
  points[5].z = 0.332542;//frustum.ftr;
  points[6].x = 1.5;//frustum.fbl;
  points[6].y = 0.332542;//frustum.fbl;
  points[6].z = -0.332542;//frustum.fbl;
  points[7].x = 1.5;//frustum.fbr;
  points[7].y = -0.332542;//frustum.fbr;
  points[7].z = -0.332542;//frustum.fbr;  

  //std::cerr << "inside generateCone3D" << std::endl;
  for (int main_i = 0; main_i < num_poses3D; main_i++)
  {
	tf::Transform tf_pose;
	Vec3 points_new[8];

	tf::poseMsgToTF(poseArray3D.poses[main_i], tf_pose);
	for (int i = 0; i < 8; i++)
	{

		tf::Vector3 old_pos(points[i].x, points[i].y, points[i].z);

		tf::Vector3 new_pos;
		new_pos = tf_pose * old_pos;
		points_new[i].x = new_pos.getX();
		points_new[i].y = new_pos.getY();
		points_new[i].z = new_pos.getZ();
	}

	/* When considered the frustum parameters:
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
	  frustrum.setCamDef(p, l, u);
	  bool result = frustrum.pointInFrustum(a);
	  std::cerr << "result: " <<result << std::endl;
	  drawPoints(frustrum);

	*/ 

	// When not considered the frustum parameters and hardcoded frustum values:

	frustum[main_i].ntl.x = points_new[0].x;
	frustum[main_i].ntl.y = points_new[0].y;
	frustum[main_i].ntl.z = points_new[0].z;
	frustum[main_i].ntr.x = points_new[1].x;
	frustum[main_i].ntr.y = points_new[1].y;
	frustum[main_i].ntr.z = points_new[1].z;
	frustum[main_i].nbl.x = points_new[2].x;
	frustum[main_i].nbl.y = points_new[2].y;
	frustum[main_i].nbl.z = points_new[2].z;
	frustum[main_i].nbr.x = points_new[3].x;
	frustum[main_i].nbr.y = points_new[3].y;
	frustum[main_i].nbr.z = points_new[3].z;

	frustum[main_i].ftl.x = points_new[4].x;
	frustum[main_i].ftl.y = points_new[4].y;
	frustum[main_i].ftl.z = points_new[4].z;
	frustum[main_i].ftr.x = points_new[5].x;
	frustum[main_i].ftr.y = points_new[5].y;
	frustum[main_i].ftr.z = points_new[5].z;
	frustum[main_i].fbl.x = points_new[6].x;
	frustum[main_i].fbl.y = points_new[6].y;
	frustum[main_i].fbl.z = points_new[6].z;
	frustum[main_i].fbr.x = points_new[7].x;
	frustum[main_i].fbr.y = points_new[7].y;
	frustum[main_i].fbr.z = points_new[7].z;
	
	frustum[main_i].probability = 0.0;
	frustum[main_i].setCamDef();

	/*
	frustum[main_i].ntr(points_new[1].x, points_new[1].y, points_new[1].z);
	frustum[main_i].nbl(points_new[2].x, points_new[2].y, points_new[2].z);
	frustum[main_i].ntr(points_new[3].x, points_new[3].y, points_new[3].z);
	frustum[main_i].ftl(points_new[4].x, points_new[4].y, points_new[4].z);
	frustum[main_i].ftr(points_new[5].x, points_new[5].y, points_new[5].z);
	frustum[main_i].fbl(points_new[6].x, points_new[6].y, points_new[6].z);
	frustum[main_i].fbr(points_new[7].x, points_new[7].y, points_new[7].z);
	*/
	
  }
}


void findBestCone2D(Cone cones2D[], int num_poses2D, int index[])
{
  int swap, swap_index, c, d;
  float index_prob[num_poses2D];
  
  for (int i = 0; i < num_poses2D; i++)
  {
	index[i] = i;
	index_prob[i] = cones2D[i].probability;
  }

  for (c = 0; c < (num_poses2D - 1); c++)
  {
    for (d = 0; d < (num_poses2D - c - 1); d++)
    {
      if (index_prob[d] < index_prob[d + 1]) /* For increasing order use. */
      {
        swap = index_prob[d];
        index_prob[d] = index_prob[d + 1];
        index_prob[d + 1] = swap;
	swap_index = index[d];
        index[d] = index[d + 1];
        index[d + 1] = swap_index;
      }
    }
  }
 
}



void findBestCone3D(float weight[], int num_poses3D, int index[])
{
  int swap, swap_index, c, d;  
 
  for (int i = 0; i < num_poses3D; i++)
  {
	index[i] = i;
  }

  for (c = 0; c < (num_poses3D - 1); c++)
  {
    for (d = 0; d < (num_poses3D - c - 1); d++)
    {
      if (weight[d] < weight[d + 1]) /* For increasing order use. */
      {
        swap = weight[d];
        weight[d] = weight[d + 1];
        weight[d + 1] = swap;
	swap_index = index[d];
        index[d] = index[d + 1];
        index[d + 1] = swap_index;
      }
    }
  }
 
}


void findBestPanTilt(Frustum cones3D[], int num_poses3D, int index[])
{
  int swap, swap_index, c, d;
  float index_prob[num_poses3D];
  
  for (int i = 0; i < num_poses3D; i++)
  {
	index[i] = i;
	index_prob[i] = cones3D[i].probability;
  }

  for (c = 0; c < (num_poses3D - 1); c++)
  {
    for (d = 0; d < (num_poses3D - c - 1); d++)
    {
      if (index_prob[d] < index_prob[d + 1]) /* For increasing order use. */
      {
        swap = index_prob[d];
        index_prob[d] = index_prob[d + 1];
        index_prob[d + 1] = swap;
	swap_index = index[d];
        index[d] = index[d + 1];
        index[d + 1] = swap_index;
      }
    }
  }
 
}



visualization_msgs::MarkerArray drawCones2D(Cone cones2D_visualiser[], int num_poses2D)
{
  
  // Create cone markers.
  visualization_msgs::MarkerArray cone2D_markers;
  cone2D_markers.markers.resize(num_poses2D);

  for (unsigned i = 0; i < num_poses2D; i++)
  {
	geometry_msgs::Point p1, p2, p3;

      	cone2D_markers.markers[i].header.frame_id = "/map";
      	cone2D_markers.markers[i].header.stamp = ros::Time(); // Duration
      	cone2D_markers.markers[i].ns = "cone2D"; // Namespace
	cone2D_markers.markers[i].id = i + 1; // Id
	cone2D_markers.markers[i].type = visualization_msgs::Marker::TRIANGLE_LIST; // Markers are ofTRIANGLE_LIST type.
	p1.x = cones2D_visualiser[i].points[0].x;
	p1.y = cones2D_visualiser[i].points[0].y;
	p1.z = cones2D_visualiser[i].points[0].z;
	p2.x = cones2D_visualiser[i].points[1].x;
	p2.y = cones2D_visualiser[i].points[1].y;
	p2.z = cones2D_visualiser[i].points[1].z;
	p3.x = cones2D_visualiser[i].points[3].x;
	p3.y = cones2D_visualiser[i].points[3].y;
	p3.z = cones2D_visualiser[i].points[3].z;

	cone2D_markers.markers[i].points.push_back(p1);
	cone2D_markers.markers[i].points.push_back(p2);
	cone2D_markers.markers[i].points.push_back(p3);
		
	cone2D_markers.markers[i].scale.x = 1.0;
	cone2D_markers.markers[i].scale.y = 1.0;
	cone2D_markers.markers[i].scale.z = 1.0;
	cone2D_markers.markers[i].color.a = 0.6;
   	cone2D_markers.markers[i].color.b = 0.0;
	cone2D_markers.markers[i].color.r = 1.0;
	cone2D_markers.markers[i].color.g = 0.0;
		
	
	//std::cerr << "size in server: "<< cone2D_markers.markers[i].points.size() << std::endl;
	     
	if (cone2D_markers.markers[i].points.size() > 0)
        	cone2D_markers.markers[i].action = visualization_msgs::Marker::ADD;
      	else
        	cone2D_markers.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  return cone2D_markers;
}


 
visualization_msgs::MarkerArray drawBestCones2D(Cone cones2D_visualiser[], int num_poses2D, int bestPosesIndex[])
{
  
  // Create cone markers.
  visualization_msgs::MarkerArray cone2D_markers;
  cone2D_markers.markers.resize(num_poses2D);

  for (unsigned i = 0; i < num_poses2D; i++)
  {
	geometry_msgs::Point p;

      	cone2D_markers.markers[i].header.frame_id = "/map";
      	cone2D_markers.markers[i].header.stamp = ros::Time(); // Duration
      	cone2D_markers.markers[i].ns = "bestCone2D"; // Namespace
	cone2D_markers.markers[i].id = i + 1; // Id
	cone2D_markers.markers[i].type = visualization_msgs::Marker::POINTS; // Markers are ofTRIANGLE_LIST type.
	p.x = cones2D_visualiser[bestPosesIndex[i]].points[0].x;
	p.y = cones2D_visualiser[bestPosesIndex[i]].points[0].y;
	p.z = cones2D_visualiser[bestPosesIndex[i]].points[0].z;
 
	cone2D_markers.markers[i].points.push_back(p);
		
	cone2D_markers.markers[i].scale.x = 0.3;
	cone2D_markers.markers[i].scale.y = 0.3;
	cone2D_markers.markers[i].scale.z = 0.3;
	cone2D_markers.markers[i].color.a = 1.0;
  cone2D_markers.markers[i].color.b = 1.0;
	cone2D_markers.markers[i].color.r = 0.0;
	cone2D_markers.markers[i].color.g = 0.0;
		
	
	//std::cerr << "size in server: "<< cone2D_markers.markers[i].points.size() << std::endl;
	     
	if (cone2D_markers.markers[i].points.size() > 0)
        	cone2D_markers.markers[i].action = visualization_msgs::Marker::ADD;
      	else
        	cone2D_markers.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  return cone2D_markers;
}


visualization_msgs::MarkerArray drawCones3D(Frustum frustum[], int num_poses3D) 
{
	
	int num_points_frustum = 8;
	int num_lines_frustum = 12;
	int id = 1;
	visualization_msgs::MarkerArray cone3D_markers;

	cone3D_markers.markers.resize(num_poses3D * num_lines_frustum);	
	std::cerr << "cone3D_markers inside function: " <<cone3D_markers.markers.size() << std::endl;

  float max_probability = 0.0000001;
  for (int j = 0; j < num_poses3D; j++)
    {
      if (frustum[j].probability > max_probability)
        max_probability = frustum[j].probability;
    }

	for (int j = 0; j < num_poses3D; j++)
	{

		geometry_msgs::Point p[num_points_frustum];

		p[0].x = frustum[j].ntl.x;
		p[0].y = frustum[j].ntl.y;
		p[0].z = frustum[j].ntl.z;
		p[1].x = frustum[j].ntr.x;
		p[1].y = frustum[j].ntr.y;
		p[1].z = frustum[j].ntr.z;
		p[2].x = frustum[j].nbl.x;
		p[2].y = frustum[j].nbl.y;
		p[2].z = frustum[j].nbl.z;
		p[3].x = frustum[j].nbr.x;
		p[3].y = frustum[j].nbr.y;
		p[3].z = frustum[j].nbr.z;

		p[4].x = frustum[j].ftl.x;
		p[4].y = frustum[j].ftl.y;
		p[4].z = frustum[j].ftl.z;
		p[5].x = frustum[j].ftr.x;
		p[5].y = frustum[j].ftr.y;
		p[5].z = frustum[j].ftr.z;
		p[6].x = frustum[j].fbl.x;
		p[6].y = frustum[j].fbl.y;
		p[6].z = frustum[j].fbl.z;
		p[7].x = frustum[j].fbr.x;
		p[7].y = frustum[j].fbr.y;
		p[7].z = frustum[j].fbr.z;

    
    cone3D_markers.markers[j].header.frame_id = "map";
    cone3D_markers.markers[j].header.stamp = ros::Time(); // Duration
    cone3D_markers.markers[j].ns = "map"; // Namespace
    cone3D_markers.markers[j].id = id; // Id
    //std::cerr<<"id = "<< id<< std::endl;
    cone3D_markers.markers[j].type = visualization_msgs::Marker::LINE_LIST; // Markers are ofTRIANGLE_LIST type.
    cone3D_markers.markers[j].scale.x = 0.01;
    cone3D_markers.markers[j].scale.y = 0.01;
    cone3D_markers.markers[j].scale.z = 0.01;
    cone3D_markers.markers[j].color.a = 1.0;
    cone3D_markers.markers[j].color.b = b_func(frustum[j].probability/max_probability);
    cone3D_markers.markers[j].color.r = r_func(frustum[j].probability/max_probability);
    cone3D_markers.markers[j].color.g = g_func(frustum[j].probability/max_probability);



		for (int i = 0; i < num_lines_frustum; i++)
		{
			switch(i)
			{
				case 0 :
					{
						cone3D_markers.markers[j].points.push_back(p[0]);
						cone3D_markers.markers[j].points.push_back(p[1]);
					}
					break;
				case 1 :
					{
						cone3D_markers.markers[j].points.push_back(p[2]);
						cone3D_markers.markers[j].points.push_back(p[3]);
					}
					break;

				case 2 :
					{
						cone3D_markers.markers[j].points.push_back(p[0]);
						cone3D_markers.markers[j].points.push_back(p[2]);
					}
					break;
				

				case 3 :
					{
						cone3D_markers.markers[j].points.push_back(p[1]);
						cone3D_markers.markers[j].points.push_back(p[3]);
					}
					break;

				case 4 :
					{
						cone3D_markers.markers[j].points.push_back(p[4]);
						cone3D_markers.markers[j].points.push_back(p[5]);
					}
					break;
				case 5 : //comment this case if 3d frustum confuses you to distinguish all sides properly.
					{
						cone3D_markers.markers[j].points.push_back(p[6]);
						cone3D_markers.markers[j].points.push_back(p[7]);
					}
					break;

				case 6 :
					{
						cone3D_markers.markers[j].points.push_back(p[4]);
						cone3D_markers.markers[j].points.push_back(p[6]);
					}
					break;

				case 7 :
					{
						cone3D_markers.markers[j].points.push_back(p[5]);
						cone3D_markers.markers[j].points.push_back(p[7]);
					}
					break;
				case 8 :
					{
						cone3D_markers.markers[j].points.push_back(p[0]);
						cone3D_markers.markers[j].points.push_back(p[4]);
					}
					break;
				case 9 :
					{
						cone3D_markers.markers[j].points.push_back(p[2]);
						cone3D_markers.markers[j].points.push_back(p[6]);
					}
					break;

				case 10 :
					{
						cone3D_markers.markers[j].points.push_back(p[1]);
						cone3D_markers.markers[j].points.push_back(p[5]);
					}
					break;

				case 11 :
					{
						cone3D_markers.markers[j].points.push_back(p[3]);
						cone3D_markers.markers[j].points.push_back(p[7]);
					}
					break;
			}
			id++;
			if (cone3D_markers.markers[j].points.size() >= 0)
				cone3D_markers.markers[j].action = visualization_msgs::Marker::ADD;
	      		else
				cone3D_markers.markers[j].action = visualization_msgs::Marker::DELETE;
		}
	}
 
  return cone3D_markers;
}




// Callback function for the service 'visualsier'.
bool serviceBestViewsVisualiser(view_evaluation::BestViewsVisualiser::Request  &req,
         view_evaluation::BestViewsVisualiser::Response &res)
{
  std::cerr << "inside serviceBestViewsVisualiser: " << std::endl;
  visualization_msgs::MarkerArray cone2D_markers, cone3D_markers, bestCone2D_markers, bestCone3D_markers;
  
  // Fetch the inputs:
  
  geometry_msgs::PoseArray poseArray2D = req.randomPoses; // Random poses.
  int num_poses = poseArray2D.poses.size(); // Number of poses.

  int num_poses2D = num_poses;
  int num_bestPoses2D = req.num_best_poses;
  int bestPosesIndex[num_poses2D];
  //float pan_angles[pan_angles_size]; // Pan angles
  //float tilt_angles[tilt_angles_size]; // Tilt angles
  std::vector<float> pan_angles(req.pan_angles);
  std::vector<float> tilt_angles(req.tilt_angles);
  int pan_angles_size = pan_angles.size();
  int tilt_angles_size = tilt_angles.size();
  int num_poses3D = pan_angles_size;
  float camera_height = 1.0;


  Cone cones2D_visualiser[num_poses2D];

  
  std::cerr << "num_poses2D: " <<num_poses2D << std::endl;
  std::cerr << "pan size: " <<pan_angles_size << std::endl;


  // Generate Cones.
  generateCones2D(cones2D_visualiser, num_poses2D, poseArray2D);

  // Find the probability distribution for each view cones generated.
  findProbabilityOfCones2D(cones2D_visualiser, num_poses2D);

  // Find the best poses index in sorted order based on the probabilities calculated before.
  findBestCone2D(cones2D_visualiser, num_poses2D, bestPosesIndex);
 
  // Draw the 2D cones:
  cone2D_markers = drawCones2D(cones2D_visualiser, num_poses2D);
  bestCone2D_markers = drawBestCones2D(cones2D_visualiser, num_bestPoses2D, bestPosesIndex);

  // Pass the 'n' best 2D poses calculated with different pan and tilt angles to find the best 3D Views.
  geometry_msgs::PoseArray poseArray3D; // Random 3D poses.
  geometry_msgs::Quaternion q[num_bestPoses2D][num_poses3D];


  /*q[0] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(0));
  q[1] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(1));
  q[2] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(2));
  q[3] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(0));
  q[4] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(1));
  q[5] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(2));
  q[6] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(0));
  q[7] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(1));
  q[8] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(2));
  */

  // Fetch the first 'n' best 2D views:
  for (int i = 0; i < num_bestPoses2D; i++)
  {
	poseArray3D.poses.push_back(poseArray2D.poses[bestPosesIndex[i]]);
  }

  float pitch_robot[num_bestPoses2D];
  float yaw_robot[num_bestPoses2D];
  
  for (int i = 0; i < num_bestPoses2D; i++)
    {
      tf::Quaternion qq;
      tf::quaternionMsgToTF (poseArray3D.poses[i].orientation, qq);
      tf::Matrix3x3 m(qq);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      pitch_robot[i] = pitch;
      yaw_robot[i] = yaw;
    }
  
  for (int i = 0; i < num_bestPoses2D; i++)
    {
      for (int j = 0; j < num_poses3D; j++)
        {
          q[i][j] = tf::createQuaternionMsgFromRollPitchYaw(0, pitch_robot[i] + pan_angles.at(j), yaw_robot[i] + tilt_angles.at(j));
        }
    }

  Frustum cones3D_visualiser[num_bestPoses2D][num_poses3D];

  //std::cerr<< " pan size: "<< pan_angles_size<< std::endl;
  //std::cerr<< " tilt size: "<< tilt_angles_size<< std::endl;
  geometry_msgs::PoseArray poseArray3D_tempp;
  poseArray3D_tempp.header.frame_id = "/map";
  for (int i = 0; i < num_bestPoses2D; i++)
  {
	geometry_msgs::PoseArray poseArray3D_temp;
	for (int j = 0; j < num_poses3D; j++)
  	{
		geometry_msgs::Pose pose;
		pose = 	poseArray3D.poses[i];
		pose.position.z = camera_height;
		pose.orientation = q[i][j];
		poseArray3D_temp.poses.push_back(pose);	
		poseArray3D_tempp.poses.push_back(pose);	
	}
	// Generate Cones.
  	generateCones3D(cones3D_visualiser[i], num_poses3D, poseArray3D_temp);
	
	// Find the probability distribution for each view cones generated.
  	findProbabilityOfCones3D(cones3D_visualiser[i], num_poses3D); // cones3D[i].probability
  }
  std::cerr<< " pose size: "<< poseArray3D_tempp.poses.size()<< std::endl;

  float weight[num_bestPoses2D];
  std::vector<float> weight_vector(num_bestPoses2D);
  int bestPosesIndex3D[num_bestPoses2D];

  for (int i = 0; i < num_bestPoses2D; i++)
  { 
	float sum = 0;
	for (int j = 0; j < num_poses3D; j++)
	{
		sum+= cones3D_visualiser[i][j].probability;
	}
	weight[i] = sum;
	std::cerr<< "3d cone sum: "<< sum<< std::endl;
  }
  
  // Find the best poses index in sorted order based on the probabilities calculated before.
  findBestCone3D(weight, num_bestPoses2D, bestPosesIndex3D);

  /*for (int i = 0; i < num_bestPoses2D; i++)
  {
	bestPoseArray.poses.push_back(poseArray3D.poses[bestPosesIndex3D[i]]);
	weight_vector.push_back(weight[i]);
  }
  */
  
  // Draw the 3D cones:
  //std::cerr << "cone3D_markers before function: " <<cone3D_markers.markers.size() << std::endl;
  cone3D_markers = drawCones3D(cones3D_visualiser[bestPosesIndex3D[0]], num_poses3D);
  //std::cerr << "cone3D_markers after function: " <<cone3D_markers.markers.size() << std::endl;


  std::cerr<< "Best 3d pose indices:"<< std::endl;
  for (int i = 0; i < num_bestPoses2D; i++)
  {
	std::cerr<< bestPosesIndex3D[i]<< std::endl;
  }
  // Update the results for the client:
  res.poses2D = poseArray3D_tempp;
  res.cones2D = cone2D_markers;
  res.bestCones2D = bestCone2D_markers;
  //res.poses3D = poseArray3D;
  res.cones3D = cone3D_markers;

  return true;
}








// Callback function for the service 'bestViews.
bool serviceBestViews(view_evaluation::BestViews::Request  &req,
         view_evaluation::BestViews::Response &res)
{
  std::cerr << "inside serviceBestViews: " << std::endl;

  // Fetch the inputs:
  
  geometry_msgs::PoseArray poseArray2D = req.randomPoses; // Random poses.
  int num_poses = poseArray2D.poses.size(); // Number of poses.

  int num_poses2D = num_poses;
  int num_bestPoses2D = req.num_best_poses;
  int bestPosesIndex[num_poses2D];
  //float pan_angles[pan_angles_size]; // Pan angles
  //float tilt_angles[tilt_angles_size]; // Tilt angles
  std::vector<float> pan_angles(req.pan_angles);
  std::vector<float> tilt_angles(req.tilt_angles);
  int pan_angles_size = pan_angles.size();
  int tilt_angles_size = tilt_angles.size();
  int num_poses3D = pan_angles_size;
  float camera_height = 1.0;

  // Outputs
  //float weights[num_poses]; // Weights of random poses.
  geometry_msgs::PoseArray bestPoseArray; // Best poses.

  Cone cones2D[num_poses2D];

  
  std::cerr << "num_poses2D: " <<num_poses2D << std::endl;
  std::cerr << "pan size: " <<pan_angles_size << std::endl;

  // Generate Cones.
  generateCones2D(cones2D, num_poses2D, poseArray2D);

  // Find the probability distribution for each view cones generated.
  findProbabilityOfCones2D(cones2D, num_poses2D);


  // Find the best poses index in sorted order based on the probabilities calculated before.
  findBestCone2D(cones2D, num_poses2D, bestPosesIndex);
  //std::cerr<< "Best pose indices:"<< std::endl;
 // for (int i = 0; i < num_poses2D; i++)
  //{
	//std::cerr<< bestPosesIndex[i]<< std::endl;
  //}

  // Pass the 'n' best 2D poses calculated with different pan and tilt angles to find the best 3D Views.
  geometry_msgs::PoseArray poseArray3D; // Random 3D poses.
  geometry_msgs::Quaternion q[num_bestPoses2D][num_poses3D];
 

  /*q[0] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(0));
  q[1] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(1));
  q[2] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(0), tilt_angles.at(2));
  q[3] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(0));
  q[4] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(1));
  q[5] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(1), tilt_angles.at(2));
  q[6] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(0));
  q[7] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(1));
  q[8] = tf::createQuaternionMsgFromRollPitchYaw(0, pan_angles.at(2), tilt_angles.at(2));
  */
  
  // Fetch the first 'n' best 2D views:
  for (int i = 0; i < num_bestPoses2D; i++)
  {
	poseArray3D.poses.push_back(poseArray2D.poses[bestPosesIndex[i]]);
  }

  float pitch_robot[num_bestPoses2D];
  float yaw_robot[num_bestPoses2D];
  
  for (int i = 0; i < num_bestPoses2D; i++)
    {
      tf::Quaternion qq;
      tf::quaternionMsgToTF (poseArray3D.poses[i].orientation, qq);
      tf::Matrix3x3 m(qq);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      pitch_robot[i] = pitch;
      yaw_robot[i] = yaw;
    }

  for (int i = 0; i < num_bestPoses2D; i++)
    {
      for (int j = 0; j < num_poses3D; j++)
        {
          q[i][j] = tf::createQuaternionMsgFromRollPitchYaw(0, pitch_robot[i] + pan_angles.at(j), yaw_robot[i] + tilt_angles.at(j));
        }
    }

  
  Frustum cones3D[num_bestPoses2D][num_poses3D];

  std::cerr<< " pan size: "<< pan_angles_size<< std::endl;
  std::cerr<< " tilt size: "<< tilt_angles_size<< std::endl;
 
  for (int i = 0; i < num_bestPoses2D; i++)
  {
	geometry_msgs::PoseArray poseArray3D_temp;
	for (int j = 0; j < num_poses3D; j++)
  	{
		geometry_msgs::Pose pose;
		pose = 	poseArray3D.poses[i];
		pose.position.z = camera_height;
		pose.orientation = q[i][j];
		poseArray3D_temp.poses.push_back(pose);		
	}

	// Generate Cones.
  	generateCones3D(cones3D[i], num_poses3D, poseArray3D_temp);
	
	// Find the probability distribution for each view cones generated.
  	findProbabilityOfCones3D(cones3D[i], num_poses3D); // cones3D[i].probability
  }

  float weight[num_bestPoses2D];
  std::vector<float> weight_vector(num_bestPoses2D);
  std::vector<float> bestPanTiltWeights(num_poses3D);
  int bestPosesIndex3D[num_bestPoses2D];
  int bestPanTiltIndex[num_poses3D];

  for (int i = 0; i < num_bestPoses2D; i++)
  { 
	float sum = 0;
	for (int j = 0; j < num_poses3D; j++)
	{
		sum+= cones3D[i][j].probability;
	}
	weight[i] = sum;
  }
  
  // Find the best poses index in sorted order based on the probabilities calculated before.
  findBestCone3D(weight, num_bestPoses2D, bestPosesIndex3D);

  for (int i = 0; i < num_bestPoses2D; i++)
  {
	bestPoseArray.poses.push_back(poseArray3D.poses[bestPosesIndex3D[i]]);
	weight_vector.push_back(weight[i]);
  }
  findBestPanTilt(cones3D[bestPosesIndex3D[0]], num_poses3D, bestPanTiltIndex);

  float pan[num_poses3D];
  float tilt[num_poses3D];
  std::vector<float> pan_sorted(num_poses3D);
  std::vector<float> tilt_sorted(num_poses3D);
  
  for (int i = 0; i < num_poses3D; i++)
  {
	tf::Quaternion qq;
	//tf::quaternionMsgToTF (q[i], qq);
	tf::quaternionMsgToTF (q[bestPosesIndex3D[0]][i], qq);

  tf::Matrix3x3 m(qq);
  	double roll, pitch, yaw;
  	m.getRPY(roll, pitch, yaw);
	pan[i] = pitch;
	tilt[i] = yaw;
	//bestPanTiltWeights.push_back(cones3D[bestPosesIndex3D[0][bestPanTiltIndex[i]].probability);
  }
  for (int i = 0; i < num_poses3D; i++)
  {
	pan_sorted.push_back(pan[bestPanTiltIndex[i]]);
	tilt_sorted.push_back(tilt[bestPanTiltIndex[i]]);
	//bestPanTiltWeights.push_back(cones3D[bestPosesIndex3D[0][bestPanTiltIndex[i]].probability);
  }
  
  // Update the results:
  res.bestPoses = bestPoseArray;
  res.bestPosesWeights = weight_vector;
  res.pan_sorted = pan_sorted;
  res.tilt_sorted = tilt_sorted;
  //res.bestPanTiltWeights = bestPanTiltWeights;
  return true;
}

// Call back function for the subscriber to map data.
void process_map (const nav_msgs::OccupancyGrid &map)
{
	/* map info for tum kitchen
	map width: 148
	map height: 238
	map_min_x: -2.55
	map_min_x: -6.25
	map_max_x: 4.85
	map_max_x: 5.65
	map_origin_x: -2.55
	map_origin_x: -6.25

	occupied_count: 1964
	free_count: 28610
	unknown_count: 4650
	*/

	if (map_received == 0)
	{
		std::cerr << "map topic called: " << std::endl;
	 	mapData = map;
		std::cerr << "map width: " <<mapData.info.width << std::endl;
		std::cerr << "map height: " <<mapData.info.height << std::endl;

		map_origin_x = mapData.info.origin.position.x;
		map_origin_y = mapData.info.origin.position.y;
		map_width = mapData.info.width;
		map_height = mapData.info.height;
		map_min_x = mapData.info.origin.position.x;
	   	map_max_x = mapData.info.origin.position.x + mapData.info.width * mapData.info.resolution;
	   	map_min_y = mapData.info.origin.position.y;
	  	map_max_y = mapData.info.origin.position.y + mapData.info.height * mapData.info.resolution;
		map_resolution = mapData.info.resolution;
		/*std::cerr << "map_min_x: " <<map_min_x << std::endl;
		std::cerr << "map_min_y: " <<map_min_y<< std::endl;
		std::cerr << "map_max_x: " <<map_max_x << std::endl;
		std::cerr << "map_max_y: " <<map_max_y<< std::endl;
		std::cerr << "map_origin_x: " <<map_origin_x << std::endl;
		std::cerr << "map_origin_y: " <<map_origin_y<< std::endl;
		*/
		map_received = 1;
	}
	//std::cerr << "map_received: " << std::endl;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Best_Views");
  ros::NodeHandle node; // Ros node handler.

  // Create a ROS subscriber for map data.
  ros::Subscriber sub_map = node.subscribe ("map", 1, process_map);

  // Create a ROS server for the service 'bestViews'. Service file is BestViews.srv
  ros::ServiceServer service_BestViews = node.advertiseService("bestViews", serviceBestViews);

  //Create a ROS server for the service 'visualiser'. Service file is Visualiser.srv
  ros::ServiceServer service_BestViewsVisualiser = node.advertiseService("bestViewsVisualiser", serviceBestViewsVisualiser);
 
  ros::spin ();
}

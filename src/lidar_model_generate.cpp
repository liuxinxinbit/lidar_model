
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <sstream>
#include <string>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <cstdlib>
#include <ctime>

using namespace std;
pcl::PointXYZ initial_point;
double direction_flag_x;
double direction_flag_y;
double direction ;

float step_size;
void test_model_parameter_initial()
{
  unsigned seed;
  seed = time(0);
  srand(seed);
  direction= rand()%180*3.1415926/180;
	initial_point.x=rand()%400-200;
  cout<<direction<<endl;
	initial_point.y=-180;
	initial_point.z=0;
	direction_flag_x=cos(direction);
	direction_flag_y=sin(direction);
	step_size=0.7+(rand()%10000/10000)*100/50;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_generation(pcl::PointXYZ center_point,float radius)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ pointt;

  for (int x = -3; x < 3; x++)
  {
    for (int y = -2; y < 2; y++)
    {
      for (int z = -1; z < 3; z++)
      {
        if (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) < 3)
        {

          pointt.x = center_point.x + double(x);
          pointt.y = center_point.y + double(y);
          pointt.z = center_point.z + double(z);
          if (rand() % (rand() % 5 + 1) > 1)
            cloud->points.push_back(pointt);
        }
      }
    }
  }

  cloud->width = cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr get_nextcloud()
{
	if(abs(initial_point.x)>200) direction_flag_x*=-1;
	if(abs(initial_point.y)>200) direction_flag_y*=-1;
	initial_point.x+=step_size*direction_flag_x;
	initial_point.y+=step_size*direction_flag_y;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new  pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_generation(initial_point,5);
	return cloud;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_model_talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("pandar_lidar_model", 1);
	ros::Rate loop_rate(10);
	test_model_parameter_initial();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	int count=0;
  double timestam = ros::Time::now().toSec(); 
	while ( ros::ok() ) {
		cloud = get_nextcloud();
		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg (*cloud,msg);
    msg.header.frame_id = "lxxp";
    // pcl_conversions::fromPCL(*cloud,msg);
		count++;
		// cout<<count<<endl;
		chatter_pub.publish(msg);
    double timeNow = ros::Time::now().toSec();
    if(timeNow-timestam>10){
      timestam = timeNow;
      test_model_parameter_initial();
    }
		ros::spinOnce();
		loop_rate.sleep();
	}
	
  return 0;
}

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "lidar_model/stereo_vision_msg.h"
#include <vector>
#include <tf2/LinearMath/Quaternion.h>



using namespace std;



class lidar_marker{
public:
        visualization_msgs::Marker marker;
        lidar_marker()
        {      
                marker.header.frame_id = "lxxp";
                marker.header.stamp = ros::Time::now();
                marker.ns = "detection_visualization";
                marker.id = 0;
                marker.type = 0;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 0;marker.pose.position.y = 0;marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;marker.pose.orientation.y = 0.0;marker.pose.orientation.z = 0.0;marker.pose.orientation.w = 1.0;
                marker.scale.x = 1.0;marker.scale.y = 1.0;marker.scale.z = 1.0;
                marker.color.r = 0.0f;marker.color.g = 1.0f;marker.color.b = 0.0f;marker.color.a = 0.7f;
                marker.lifetime = ros::Duration();
                }
                visualization_msgs::Marker get_marker(){
                        return marker;
                }
                void set_marker_id(int id){
                        marker.id = id;
                }
                void set_marker_shape(uint32_t shape)
                {
                        marker.type = shape;
                }
                void set_marker_position(float x, float y, float z)
                {
                        marker.pose.position.x = x;
                        marker.pose.position.y = y;
                        marker.pose.position.z = z;
                }
                void set_marker_orientation(float x, float y, float z, float w)
                {
                        marker.pose.orientation.x = x;
                        marker.pose.orientation.y = y;
                        marker.pose.orientation.z = z;
                        marker.pose.orientation.w = w;
                }
                void set_marker_scale(float x, float y, float z)
                {
                        marker.scale.x = x;
                        marker.scale.y = y;
                        marker.scale.z = z;
                }
                void set_marker_color(float red, float green, float blue, float alpha)
                {
                        marker.color.r = red;
                        marker.color.g = green;
                        marker.color.b = blue;
                        marker.color.a = alpha;
                }
                void set_marker_action(bool flag)
                {
                        if(flag){
                                marker.action = visualization_msgs::Marker::ADD;
                                
                        }
                        else{
                                marker.action = visualization_msgs::Marker::DELETE;
                                set_marker_scale(0, 0, 0);
                        }
                }
};
class lidar_marker_list{
        public :
                vector<lidar_marker> lml;
                lidar_marker lidar_marker_1;
                lidar_marker lidar_marker_2;
                lidar_marker lidar_marker_3;
                lidar_marker lidar_marker_4;
                lidar_marker lidar_marker_5;
                lidar_marker lidar_marker_6;
                lidar_marker lidar_marker_7;
                lidar_marker lidar_marker_8;
                lidar_marker_list(uint32_t shape)
                {
                        lidar_marker_1.set_marker_id(0);
                        lidar_marker_1.set_marker_shape(shape);

                        lidar_marker_2.set_marker_id(1);
                        lidar_marker_2.set_marker_shape(shape);

                        lidar_marker_3.set_marker_id(2);
                        lidar_marker_3.set_marker_shape(shape);

                        lidar_marker_4.set_marker_id(3);
                        lidar_marker_4.set_marker_shape(shape);

                        lidar_marker_5.set_marker_id(4);
                        lidar_marker_5.set_marker_shape(shape);

                        lidar_marker_6.set_marker_id(5);
                        lidar_marker_6.set_marker_shape(shape);

                        lidar_marker_7.set_marker_id(6);
                        lidar_marker_7.set_marker_shape(shape);

                        lidar_marker_8.set_marker_id(7);
                        lidar_marker_8.set_marker_shape(shape);
                }
};

class lidar_marker_arrow_list{
        public :
                vector<lidar_marker> lml;
                lidar_marker lidar_marker_1;
                lidar_marker lidar_marker_2;
                lidar_marker lidar_marker_3;
                lidar_marker lidar_marker_4;
                lidar_marker lidar_marker_5;
                lidar_marker lidar_marker_6;
                lidar_marker lidar_marker_7;
                lidar_marker lidar_marker_8;
                lidar_marker_arrow_list(uint32_t shape)
                {
                        lidar_marker_1.set_marker_id(10);
                        lidar_marker_1.set_marker_shape(shape);

                        lidar_marker_2.set_marker_id(11);
                        lidar_marker_2.set_marker_shape(shape);

                        lidar_marker_3.set_marker_id(12);
                        lidar_marker_3.set_marker_shape(shape);

                        lidar_marker_4.set_marker_id(13);
                        lidar_marker_4.set_marker_shape(shape);

                        lidar_marker_5.set_marker_id(14);
                        lidar_marker_5.set_marker_shape(shape);

                        lidar_marker_6.set_marker_id(15);
                        lidar_marker_6.set_marker_shape(shape);

                        lidar_marker_7.set_marker_id(16);
                        lidar_marker_7.set_marker_shape(shape);

                        lidar_marker_8.set_marker_id(17);
                        lidar_marker_8.set_marker_shape(shape);
                }
};

lidar_marker marker_value_set(lidar_model::stereo_vision_msg obstacal_msg, lidar_marker lidar_marker_index, int i, bool flag)
{
        if (flag)
        {
                lidar_marker_index.set_marker_position(-obstacal_msg.obstacals[i].position.x,
                                                       -obstacal_msg.obstacals[i].position.y,
                                                       obstacal_msg.obstacals[i].position.z);
                lidar_marker_index.set_marker_scale(obstacal_msg.obstacals[i].radius * 2,
                                                    obstacal_msg.obstacals[i].radius * 2,
                                                    obstacal_msg.obstacals[i].radius * 2);
                tf2::Quaternion Pose_Quaternion;
                double yaw = atan2(-obstacal_msg.obstacals[i].velocity.y, -obstacal_msg.obstacals[i].velocity.x);
                Pose_Quaternion.setRPY(0, 0, yaw);
                Pose_Quaternion.normalize();
                lidar_marker_index.set_marker_orientation(Pose_Quaternion[0],
                                                          Pose_Quaternion[1],
                                                          Pose_Quaternion[2],
                                                          Pose_Quaternion[3]);
                lidar_marker_index.set_marker_action(flag);
        }
        else
        {
                lidar_marker_index.set_marker_scale(0, 0, 0);
                lidar_marker_index.set_marker_action(flag);
        }

        return lidar_marker_index;
}

lidar_marker arrow_marker_value_set(lidar_model::stereo_vision_msg obstacal_msg, lidar_marker lidar_marker_index, int i, bool flag)
{
        if (flag)
        {
                lidar_marker_index.set_marker_position(-obstacal_msg.obstacals[i].position.x,
                                                       -obstacal_msg.obstacals[i].position.y,
                                                       obstacal_msg.obstacals[i].position.z);
                lidar_marker_index.set_marker_color(1.0f, 0.0f, 0.0f, 1.0f);
                lidar_marker_index.set_marker_scale(obstacal_msg.obstacals[i].radius * 5,
                                                    obstacal_msg.obstacals[i].radius,
                                                    obstacal_msg.obstacals[i].radius);
                // cout << obstacal_msg.obstacals[i].velocity.x << " " << obstacal_msg.obstacals[i].velocity.y << endl;
                tf2::Quaternion Pose_Quaternion;
                double yaw = atan2(-obstacal_msg.obstacals[i].velocity.y, -obstacal_msg.obstacals[i].velocity.x);
                Pose_Quaternion.setRPY(0, 0, yaw);
                Pose_Quaternion.normalize();
                lidar_marker_index.set_marker_orientation(Pose_Quaternion[0],
                                                          Pose_Quaternion[1],
                                                          Pose_Quaternion[2],
                                                          Pose_Quaternion[3]);
                lidar_marker_index.set_marker_action(flag);
        }
        else
        {
                lidar_marker_index.set_marker_scale(0, 0, 0);
                lidar_marker_index.set_marker_action(flag);
        }

        return lidar_marker_index;
}
class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
        {
                sub_detection_result = nh.subscribe("/pointcloud_tracking", 1, &SubscribeAndPublish::getdetectionresultcallback, this);
                marker_pub = nh.advertise<visualization_msgs::Marker>("detection_visualization", 1);
        }
        void getdetectionresultcallback(const lidar_model::stereo_vision_msg::ConstPtr &msg)
        {
                lidar_model::stereo_vision_msg obstacal_msg=*msg;
                 for (size_t i = 0; i <8 ; i++)
                {
                switch (i)
                {
                case 0:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_1 = marker_value_set(obstacal_msg,  lml.lidar_marker_1,  i, true);
                                marker_pub.publish(lml.lidar_marker_1.get_marker());

                                mal.lidar_marker_1 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_1,  i, true);
                                marker_pub.publish(mal.lidar_marker_1.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_1 = marker_value_set(obstacal_msg,  lml.lidar_marker_1,  i, false);
                                marker_pub.publish(lml.lidar_marker_1.get_marker());

                                mal.lidar_marker_1 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_1,  i, false);
                                marker_pub.publish(mal.lidar_marker_1.get_marker());
                        }
                        break;
                case 1:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_2 = marker_value_set(obstacal_msg,  lml.lidar_marker_2,  i, true);
                                marker_pub.publish(lml.lidar_marker_2.get_marker());

                                mal.lidar_marker_2 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_2,  i, true);
                                marker_pub.publish(mal.lidar_marker_2.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_2 = marker_value_set(obstacal_msg,  lml.lidar_marker_2,  i, false);
                                marker_pub.publish(lml.lidar_marker_2.get_marker());

                                mal.lidar_marker_2 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_2,  i, false);
                                marker_pub.publish(mal.lidar_marker_2.get_marker());
                        }
                        break;
                case 2:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_3 = marker_value_set(obstacal_msg,  lml.lidar_marker_3,  i, true);
                                marker_pub.publish(lml.lidar_marker_3.get_marker());

                                mal.lidar_marker_3 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_3,  i, true);
                                marker_pub.publish(mal.lidar_marker_3.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_3= marker_value_set(obstacal_msg,  lml.lidar_marker_3,  i, false);
                                marker_pub.publish(lml.lidar_marker_3.get_marker());

                                mal.lidar_marker_3 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_3,  i, false);
                                marker_pub.publish(mal.lidar_marker_3.get_marker());
                        }
                        break;
                case 3:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_4 = marker_value_set(obstacal_msg,  lml.lidar_marker_4,  i, true);
                                marker_pub.publish(lml.lidar_marker_4.get_marker());

                                mal.lidar_marker_4 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_4,  i, true);
                                marker_pub.publish(mal.lidar_marker_4.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_4 = marker_value_set(obstacal_msg,  lml.lidar_marker_4,  i, false);
                                marker_pub.publish(lml.lidar_marker_4.get_marker());

                                mal.lidar_marker_4 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_4,  i, false);
                                marker_pub.publish(mal.lidar_marker_4.get_marker());
                        }
                        break;
                case 4:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_5 = marker_value_set(obstacal_msg,  lml.lidar_marker_5,  i, true);
                                marker_pub.publish(lml.lidar_marker_5.get_marker());

                                mal.lidar_marker_5 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_5,  i, true);
                                marker_pub.publish(mal.lidar_marker_5.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_5 = marker_value_set(obstacal_msg,  lml.lidar_marker_5,  i, false);
                                marker_pub.publish(lml.lidar_marker_5.get_marker());

                                mal.lidar_marker_5= arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_5,  i, false);
                                marker_pub.publish(mal.lidar_marker_5.get_marker());
                        }
                        break;
                case 5:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_6 = marker_value_set(obstacal_msg,  lml.lidar_marker_6,  i, true);
                                marker_pub.publish(lml.lidar_marker_6.get_marker());

                                mal.lidar_marker_6 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_6,  i, true);
                                marker_pub.publish(mal.lidar_marker_6.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_6 = marker_value_set(obstacal_msg,  lml.lidar_marker_6,  i, false);
                                marker_pub.publish(lml.lidar_marker_6.get_marker());

                                mal.lidar_marker_6 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_6,  i, false);
                                marker_pub.publish(mal.lidar_marker_6.get_marker());
                        }
                        break;
                case 6:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_7 = marker_value_set(obstacal_msg,  lml.lidar_marker_7,  i, true);
                                marker_pub.publish(lml.lidar_marker_7.get_marker());

                                mal.lidar_marker_7 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_7,  i, true);
                                marker_pub.publish(mal.lidar_marker_7.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_7 = marker_value_set(obstacal_msg,  lml.lidar_marker_7,  i, false);
                                marker_pub.publish(lml.lidar_marker_7.get_marker());

                                mal.lidar_marker_7 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_7,  i, false);
                                marker_pub.publish(mal.lidar_marker_7.get_marker());
                        }
                        break;
                case 7:
                        if(i<obstacal_msg.obstacals.size())
                        {
                                lml.lidar_marker_8 = marker_value_set(obstacal_msg,  lml.lidar_marker_8,  i, true);
                                marker_pub.publish(lml.lidar_marker_8.get_marker());

                                mal.lidar_marker_8 =  arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_8,  i, true);
                                marker_pub.publish(mal.lidar_marker_8.get_marker());
                        }
                        else
                        {
                                lml.lidar_marker_8 = marker_value_set(obstacal_msg,  lml.lidar_marker_8,  i, false);
                                marker_pub.publish(lml.lidar_marker_8.get_marker());

                                mal.lidar_marker_8 = arrow_marker_value_set(obstacal_msg,  mal.lidar_marker_8,  i, false);
                                marker_pub.publish(mal.lidar_marker_8.get_marker());
                        }
                        break;
                default:
                        break;
                }
                }

        }

private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;      //目标检测和跟踪结果发布器
        ros::Subscriber sub_detection_result; //激光雷达点云订阅器
        uint32_t cube_shape = visualization_msgs::Marker::CUBE;
        uint32_t arrow_shape = visualization_msgs::Marker::ARROW;
        lidar_marker_list lml = lidar_marker_list(cube_shape);
        lidar_marker_arrow_list mal = lidar_marker_arrow_list(arrow_shape);
        int index=0;
};

int main(int argc, char **argv)
{
        ros::init(argc, argv, "detection_result_show");
        SubscribeAndPublish sap;
        ros::spin();
        return 0;
}

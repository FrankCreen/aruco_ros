#include <iostream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace aruco;


bool useRectifiedImages;
cv::Mat inImage;
MarkerDetector mDetector;
vector<Marker> markers;
aruco::CameraParameters camParam;
double marker_size;
int marker_id;
bool rotate_marker_axis_;

ros::Subscriber cam_info_sub;
bool cam_info_received;


void chatCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("hell!");
    ROS_INFO("position %f %f %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    std::cout<<"\n\n"<<std::endl;
}




int main(int argc,char** argv)
{
    ros::init(argc,argv,"listener");

    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/aruco_single/pose",10,chatCallback);



    ros::spin();

    return 0;

}

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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace aruco;

using namespace message_filters;

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


/*
  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    useRectifiedImages = true;
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

void chatCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("hell!");
    //ROS_INFO("position %f %f %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    std::cout<<"\n\n"<<std::endl;
    if(cam_info_received){
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            inImage = cv_ptr->image;
            //detection results will go into "markers"
            markers.clear();
            //Ok, let's detect
            mDetector.detect(inImage, markers, camParam, marker_size, false);
            //for each marker, draw info and its boundaries in the image
            for(size_t i=0; i<markers.size(); ++i)
            {
            // only publishing the selected marker
            if(markers[i].id == marker_id)
            {
                std::cout<<"in try"<<std::endl; 
                //显示检测道德marker中心坐标
                //!!!!!!!!!!!!!!!!!
                //markers[i].write(inImage,cv::Scalar(0,0,0),2,msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
            }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }


}
*/


int main(int argc,char** argv)
{
    ros::init(argc,argv,"listener");

    ros::NodeHandle nh;
    
    
    //cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

    //ros::Subscriber sub = nh.subscribe("/image",10,chatCallback);
    ros::Subscriber sub = nh.subscribe("/aruco_single/pose",10,chatCallback);

    // message_filters::Subscriber<Image> img_sub(nh, "/image", 10);
    // message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/aruco_single/pose", 10);
    // message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), info_sub, pose_sub);
    // //TimeSynchronizer<CameraInfo, PoseStamped> sync(info_sub, pose_sub, 10);
    // sync.registerCallback(boost::bind(&chatCallback, _1, _2));


    ros::spin();

    return 0;

}

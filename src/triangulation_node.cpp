#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "webcam_lidar/webcam_lidarConfig.h"

using namespace cv;

tf::TransformListener* pListener = NULL;
webcam_lidar::webcam_lidarConfig param;
sensor_msgs::PointCloud cloud;
ros::Publisher cloud_pub;

void reconfigureCallBack(webcam_lidar::webcam_lidarConfig &config, uint32_t level)
{
  param = config;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("exception: %s", e.what());
    return;
  }

  cloud.points.clear();
  cloud.points.resize(cv_ptr->image.cols);

  for (int img_pt_x = 0; img_pt_x < cv_ptr->image.cols; img_pt_x++)
  {
    for (int img_pt_y=0; img_pt_y < cv_ptr->image.rows; img_pt_y++)
    {
      Vec3b pixel = cv_ptr->image.at<Vec3b>(Point(img_pt_x/3,img_pt_y)); // single channel
       if (pixel[0] > param.pixel_threshold && pixel[1] > param.pixel_threshold && pixel[2] > param.pixel_threshold)
       {
        // triangulation
        double pt_h    = -(img_pt_y-cv_ptr->image.rows/2)*(param.ccd_vertical_measure/cv_ptr->image.rows); // projected point height on CCD in meters
        double pt_x    = -(img_pt_x-cv_ptr->image.cols/2)*(param.ccd_horizontal_measure/cv_ptr->image.cols); // projected point horizontal translation in meters
        double com_div = param.cam_focal_length * (1.0/tan(param.cam_laser_angle)) - pt_h;
        // distance
        cloud.points[img_pt_x].x = (param.cam_laser_distance * param.cam_focal_length) / com_div;
        // horizontal
        cloud.points[img_pt_x].y = (param.cam_laser_distance * pt_x) / com_div;
        // vertical
        cloud.points[img_pt_x].z = 0.0; // at webcam_lidar_frame
      }
    }
  }

  cloud.header.stamp = ros::Time::now();
  cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webcam_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  dynamic_reconfigure::Server<webcam_lidar::webcam_lidarConfig> server;
  dynamic_reconfigure::Server<webcam_lidar::webcam_lidarConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallBack, _1, _2);
  server.setCallback(f);

  std::string camera_topic;
  nhLocal.param("camera_topic", camera_topic, std::string("/webcam_lidar/image_filtered"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(camera_topic, 1, imageCallback);

  std::string webcam_lidar_frame, camera_frame, laser_frame;
  nhLocal.param("camera_frame", camera_frame, std::string("head_camera"));
  nhLocal.param("laser_frame", laser_frame, std::string("laser_frame"));
  nhLocal.param("webcam_lidar_frame", webcam_lidar_frame, std::string("lidar_link"));

  cloud.header.frame_id = webcam_lidar_frame;
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/webcam_lidar/pointcloud", 50);

  bool use_tf;
  nhLocal.param("use_tf", use_tf, true);
  if (use_tf)
  {
    pListener = new (tf::TransformListener);
    bool initialized = false;
    ROS_INFO("Waiting for camera to laser transformation...");
    while (nh.ok() && !initialized)
    {
      try
      {
          tf::StampedTransform cam_to_laser_transform;
          ros::Time now = ros::Time::now();
          pListener->waitForTransform(camera_frame, laser_frame, now, ros::Duration(0.1) );
          pListener->lookupTransform( camera_frame, laser_frame, now, cam_to_laser_transform);
          tf::Vector3 origin = cam_to_laser_transform.getOrigin();
          nhLocal.setParam("cam_laser_distance", fabs(origin.z()));
          tf::Quaternion q = cam_to_laser_transform.getRotation();
          tf::Matrix3x3 laser_rotation(q);
          double frame_roll, frame_pitch, frame_yaw;
          laser_rotation.getRPY(frame_roll, frame_pitch, frame_yaw);
          nhLocal.setParam("cam_laser_angle", fabs(frame_pitch));
          initialized = true;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
      ros::spinOnce();
    }
  }
  ROS_INFO("Triangulation node initialized.");
  ros::spin();
  return 0;
}

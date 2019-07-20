#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/bgsegm.hpp>

#include "webcam_lidar/color_filterConfig.h"

bool headless_mode = false;
bool strobed_mode = true;

unsigned char key;
using namespace cv;
cv::Mat img_laser;
cv::Mat img_even_cf;
cv::Mat img_odd_cf;
sensor_msgs::ImagePtr out;
image_transport::Publisher pub;
webcam_lidar::color_filterConfig param;
static const std::string FILTER_WINDOW = "Filtered";
static const std::string VIDEO_WINDOW =  "Feed";

void reconfigureCallBack(webcam_lidar::color_filterConfig &config, uint32_t level) {
  param = config;
}

bool even    = false;
bool got_odd = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    even = !even;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("exception: %s", e.what());
    return;
  }

  // in strobed mode every second frame is expected to be illuminated
  if (strobed_mode){
    if (!even) {
      cv_ptr->image.copyTo(img_odd_cf);
      cv::cvtColor(img_odd_cf, img_odd_cf, cv::COLOR_BGR2HSV);
      cv::inRange( img_odd_cf,  cv::Scalar(param.hue_lo, param.sat_lo, param.val_lo), cv::Scalar(param.hue_hi, param.sat_hi, param.val_hi), img_odd_cf);
      got_odd = true; // second image is copied
    }
    else {
      cv_ptr->image.copyTo(img_even_cf);
      cv::cvtColor(img_even_cf, img_even_cf, cv::COLOR_BGR2HSV);
      cv::inRange( img_even_cf,  cv::Scalar(param.hue_lo, param.sat_lo, param.val_lo), cv::Scalar(param.hue_hi, param.sat_hi, param.val_hi), img_even_cf);
    }
    if (got_odd) { // got both odd and even frames, calculate difference
      cv::absdiff(  img_odd_cf, img_even_cf, img_laser);
      cv::threshold(img_laser,  img_laser, param.trs_lo, param.trs_hi, THRESH_BINARY);
      if (!headless_mode)
      {
        cv::imshow(VIDEO_WINDOW,  cv_ptr->image);
        cv::imshow(FILTER_WINDOW, img_laser);
      }
    }
  }

  // otherwise apply color filter on all frames
  else {
    cv::cvtColor( cv_ptr->image, img_laser, cv::COLOR_BGR2HSV);
    cv::inRange(  img_laser,  cv::Scalar(param.hue_lo, param.sat_lo, param.val_lo), cv::Scalar(param.hue_hi, param.sat_hi, param.val_hi), img_laser);
    cv::threshold(img_laser, img_laser, param.trs_lo, param.trs_hi, THRESH_BINARY);
    if (!headless_mode)
    {
      cv::imshow(VIDEO_WINDOW,  cv_ptr->image);
      cv::imshow(FILTER_WINDOW, img_laser);
    }
  }

  key = cv::waitKey(1);
  out = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_laser).toImageMsg();
  pub.publish(out);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  dynamic_reconfigure::Server<webcam_lidar::color_filterConfig> server;
  dynamic_reconfigure::Server<webcam_lidar::color_filterConfig>::CallbackType f;
  f = boost::bind(&reconfigureCallBack, _1, _2);
  server.setCallback(f);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/head_camera/image_raw", 1, imageCallback);
  pub = it.advertise("/head_camera/image_color_filtered", 1);

  ROS_INFO_STREAM("Using OpenCV" << CV_MAJOR_VERSION);

  nhLocal.param("headless", headless_mode, false);
  if (!headless_mode)
  {
    cv::namedWindow(FILTER_WINDOW,1);
    cv::namedWindow(VIDEO_WINDOW,1);
  }
  nhLocal.param("strobed_mode", strobed_mode, false);

  ros::spin();

  if (!headless_mode)
  {
    cv::destroyWindow(FILTER_WINDOW);
    cv::destroyWindow(VIDEO_WINDOW);
  }

  return 0;
}

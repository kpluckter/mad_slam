#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
// #include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include <tictoc_profiler/profiler.hpp>
// #include <opencv2/cudafeatures2d.hpp>

// #include <vector>
#include <utility.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
// #include "opencv2/core/eigen.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// #include <Eigen/SVD>
#include <Eigen/Dense>
// #include <Eigen/Sparse>
#include <Eigen/Geometry>
//#include <sensor_msgs/image_encodings.h>
// #include <cmath>

using namespace cv;
using namespace std;

int iter = 0;
double curr_range = 0;
double prev_range = 0;
double x_pos;
double y_pos;
tf::Quaternion q;
geometry_msgs::Pose del_pose;
float f = 360.92322105846165;
float K_data[9] = {398.08506106571883, 0.0, 544.1835601549292, 0.0, 398.61412649616574, 527.0113871539716, 0.0, 0.0, 1.0};
float D_data[4] = {-0.044870407883172814, 0.0006096639792439277, -0.0052786650027411, 0.0011179813973823697};
float K_pin_data[9] = {360.92322105846165, 0.0, 1008.1053038457412, 0.0, 361.40289743414974, 866.738969242332, 0.0, 0.0, 1.0};
Mat K = Mat(3,3,CV_32F, K_data);
Mat D = Mat(4,1,CV_32F, D_data);
Mat K_pin = Mat(3,3,CV_32F, K_pin_data);

bool init = false;
Point2f point;
vector<Point2f> points[2];
vector<Point2f> points_rot;
vector<Point2f> points_prev;
vector<Point2f> points_prev_new;

Mat map1, map2, prev_im, curr_im;
vector<Point2f> pts_to_track;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize = Size(10,10);
Size winSize = Size(31,31);

const int MAX_COUNT = 500;
bool needToInit = false;
bool nightMode = false;
bool addRemovePt = false;

class LKTracker
{
public:
  LKTracker(ros::NodeHandle &private_node_handle);
  ~LKTracker(){};

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void rangeCallback(const sensor_msgs::LaserScanConstPtr& msg);

  void posEstCallback(const nav_msgs::OdometryPtr& msg);
  // void posEstCallback(const geometry_msgs::PosePtr& msg);

  Point2f get_dist(Point2f point, double z_img, double z_world, const tf::Quaternion& q);

  void rotate_vec(float x, float y, float z, const tf::Quaternion& q, tf::Vector3& vprime);

  // void extract_pose();
  void track_and_extract();
  // void track();

  void im_align(IplImage* im1, CvRect omega, IplImage* im2);

 
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  ros::Subscriber sub_3;
};

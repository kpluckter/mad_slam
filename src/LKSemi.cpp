#include <LKSemi.h>

LKTracker::LKTracker(ros::NodeHandle &private_node_handle)
{
  n_ = private_node_handle;
  pub_ = n_.advertise<geometry_msgs::Twist>("/local_pose_est", 1);

  sub_1 = n_.subscribe("/pinhole/image_raw", 1, &LKTracker::imageCallback, this);
  sub_2 = n_.subscribe("/sf30/range", 1, &LKTracker::rangeCallback, this);
  sub_3 = n_.subscribe("/at_drone_interface/odometry", 1, &LKTracker::posEstCallback, this);
  // sub_4 = n_.subscribe("/at_drone_interface/gps_full_data", 1, &LKTracker::gpsCallback, this);
}

void LKTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  curr_im = cv_ptr->image;
  // cout << "header = " << msg->header.seq << endl;
  iter = msg->header.seq;
  // if (iter % 10 == 0) {
  // char buffer2[18] = {};
  // sprintf(buffer2, "iter%d.jpg", iter);
  // cv::imwrite(buffer2, curr_im);
  // }
  cvtColor(curr_im, curr_im, CV_BGR2GRAY);
  // remap(curr_im, curr_im, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
  return;
}

void LKTracker::rangeCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  curr_range = msg->ranges[0];
  return;
}

// void LKTracker::gpsCallback(const at_drone_interface::GpsDataPtr& msg)
// {
//   if (gt_prev.x == 0 && gt_prev.y == 0) {
//     gt.x = 0;
//     gt.y = 0;
//     gt.z = 0;
//   } else {
//     gt.x = msg->latitude - gt_prev.x;
//     gt.y = msg->longitude - gt_prev.y;
//     gt.z = msg->altitude - gt_prev.z;
//   }
//   gt_prev.x = msg->latitude;
//   gt_prev.y = msg->longitude;
//   gt_prev.z = msg->altitude;
// }


void LKTracker::posEstCallback(const nav_msgs::OdometryPtr& msg)
{
  x_pos = msg->pose.pose.position.x;
  y_pos = msg->pose.pose.position.y;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,q);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);
  cout << "(roll,pitch,yaw) = (" << roll << "," << pitch << "," << yaw << ")" << endl;
  q.setRPY(roll,pitch,0.0);
  // q.setRPY(roll,pitch,0.0);
  return;
}

void LKTracker::track_and_extract()
{
  cout << curr_range << endl;
  curr_range = curr_range*cos(roll)*cos(pitch);
  cout << curr_range << endl;
  Mat image;
  curr_im.copyTo(image);
  int count = 0;
  if (!init) {
    init = true;
    goodFeaturesToTrack(curr_im, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.4);
    cornerSubPix(curr_im, points[1], subPixWinSize, Size(-1,-1), termcrit);
    addRemovePt = false;
  } else {
    cout << "looking for new points" << endl;
    points_rot.clear();
    points_prev_new.clear();
    vector<uchar> status;
    vector<float> err;
    if (prev_im.empty())
      curr_im.copyTo(prev_im);
    calcOpticalFlowPyrLK(prev_im, curr_im, points[0], points[1], status, err);//, winSize, 3, termcrit, 0, 0.001);
    cout << "got flow" << endl;
    size_t i, k;
    for( i = k = 0; i < points[1].size(); i++ )
    {
      if( addRemovePt )
      {
        if( norm(point - points[1][i]) <= 5 )
        {
          addRemovePt = false;
          continue;
        }
      }

      if( !status[i] ){
        continue;

      }

      points[1][k] = points[1][i];
      points_rot.push_back(get_dist(points[1][i], f, curr_range, q));
      if (points[0].size() > 0) points[0][k] = points[0][i];
      if (points_prev.size() > 0)  points_prev_new.push_back(points_prev[i]);
      
      k++;
      circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
    }
    points[1].resize(k);
    if (points[0].size() > 0) points[0].resize(k);
  }

  // if (points_prev_new.size() == points_rot.size() && points_rot.size() > 0 && !new_pts) {
  if (points[1].size() == points[0].size() && points[1].size() > 0 && !new_pts) {
    cout << "estimateRigidTransform" << endl;
    // Mat rigid_transform =  estimateRigidTransform(points_prev_new, points_rot, false);
    // Mat rigid_transform = getAffineTransform(points_prev_new, points_rot);
    double tx, ty, theta;
    // ransac(points[1], points[0], points[1].size(), &tx, &ty, curr_range, prev_range, q, q_prev);
    ransac_rot(points[1], points[0], points[1].size(), &tx, &ty, &theta, curr_range, prev_range, q, q_prev);
    // del_pose.linear.x = rigid_transform.at<double>(0,2);
    // del_pose.linear.y = rigid_transform.at<double>(1,2);
    del_pose.linear.x = ty;
    del_pose.linear.y = -tx;
    del_pose.linear.x = (del_pose.linear.x * cos(-0.5) - del_pose.linear.y * sin(-0.5));
    del_pose.linear.y = (del_pose.linear.x * sin(-0.5) + del_pose.linear.y * cos(-0.5));
    del_pose.linear.z = curr_range - prev_range;
    del_pose.angular.z = theta;
    cout << "got it" << endl;
  } else {
    del_pose.linear.x = 0;
    del_pose.linear.y = 0;
    del_pose.linear.z = 0;
    new_pts = false;
  }

  q_prev = q;
  rp = roll;
  pp = pitch;
  yp = yaw;

  if (iter % 20 == 0) {
    cout << " getting new features " << endl;
    goodFeaturesToTrack(curr_im, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.4);
    cornerSubPix(curr_im, points[1], subPixWinSize, Size(-1,-1), termcrit);
    addRemovePt = false;
    points[0].clear();
    new_pts = true;
  }

  cout << "del_pose = " << del_pose << endl;
  prev_range = curr_range;
  std::swap(points[1], points[0]);
  points_prev = points_rot;
  cv::swap(prev_im, curr_im);

  // drawCircle(640, 512, 20, q, image);
  // arrowedLine(image, Point(750,750), Point(750+(int)(del_pose.linear.x*100.0),750+(int)(del_pose.linear.y*100.0)), Scalar(0,255,0,255), 3);
  // char buffer[18] = {};
  // sprintf(buffer, "tracked%d.jpg", iter);
  // cv::imwrite(buffer, image);
  return;
}

void LKTracker::rotate_vec(float x, float y, float z, const tf::Quaternion& q, tf::Vector3& vprime)
{
  tf::Vector3 v(-y, x, z);
  tf::Vector3 temp_v = tf::quatRotate(q, v);
  vprime.setX(temp_v.getY());
  vprime.setY(-temp_v.getX());
  vprime.setZ(temp_v.getZ());

  // vprime.setX(x);
  // vprime.setY(y);
  // vprime.setZ(z);

  return;
}

Point2f LKTracker::angular_correction(Point2f point, Point2f del)
{
  double dx,dy;
  double Tz = prev_range - curr_range;
  // cout << "del.x = " << del.x << ",  del.y = " << del.y << endl;
  // cout << "point.x = " << point.x << ",  point.y = " << point.y << endl;
  // cout << "calc1.x = " << (delz*point.x-del.x*f)/curr_range << endl;
  // cout << "calc2.x = " << -(pitch-pp)*f-(yaw-yp)*f << endl;
  // cout << "calc3.x = " << (-(roll-rp)*point.x*point.y-(pitch-pp)*pow(point.x,2.0))/f << endl;
  // dx = (delz*point.x-del.x*f)/curr_range-(pitch-pp)*f-(yaw-yp)*f+(-(roll-rp)*point.x*point.y-(pitch-pp)*pow(point.x,2.0))/f;
  // dy = (delz*point.y-del.y*f)/curr_range-(roll-rp)*f+(yaw-yp)*f+(-(roll-rp)*pow(point.y,2.0)-(pitch-pp)*point.x*point.y)/f;
  
  // dx = -del.x*curr_range/f-(pitch-pp)*curr_range;//-(yaw-yp)*point.y;
  // dy = -del.y*curr_range/f-(roll-rp)*curr_range;//+(yaw-yp)*point.x;
  // dx = del.y - (pitch-pp)*f/2 + (yaw-yp)*(point.x-640);// + (-(roll-rp)*point.x*point.y - (pitch-pp)*pow(point.y,2))/f;
  // dy = -del.x - (roll-rp)*f/2 - (yaw-yp)*(point.y-520); // + ((roll-rp)*pow(point.x,2) + (pitch-pp)*point.x*point.y)/f;
  // dx = (Tz*(-point.y)-Tx*f)/curr_range - (pitch-pp)*f + (-(roll-rp)*point.x*point.y - (pitch-pp)*pow(point.x,2))/f;
  // dy = (Tz*(point.x)-Ty*f)/curr_range - (roll-rp)*f +((roll-rp)*pow(point.y,2) + (pitch-pp)*point.x*point.y)/f;
  // dx = del.x*curr_range/f-(roll-rp)*curr_range;//-(yaw-yp)*point.y;
  // dy = del.y*curr_range/f-(pitch-pp)*curr_range;//+(yaw-yp)*point.x;
  // cout << "dx = " << dx << endl;
  // cout << "dy = " << dy << endl;
  // cout << "roll = " << roll << endl;
  // cout << "rp = " << rp << endl;
  // cout << "pitch = " << pitch << endl;
  // cout << "pp = " << pp << endl;


  dx = del.y - (pitch-pp)*f/2 + (yaw-yp)*(point.x-625);
  dy = -del.x - (roll-rp)*f/2 - (yaw-yp)*(point.y-530);
  return Point2f(-0.5*dy*curr_range/f,0.5*dx*curr_range/f);
  // return del;
}

Point2f LKTracker::get_dist(Point2f point, double z_img, double z_world, const tf::Quaternion& q)
{
  tf::Vector3 new_point;
  rotate_vec(point.x - 625, point.y - 530, z_img, q, new_point);
  double x_prime = new_point.getX();
  double y_prime = new_point.getY();

  // double x_prime = point.x-625;
  // double y_prime = point.y-530;
  return Point2f(x_prime*z_world/z_img*0.5, y_prime*z_world/z_img*0.5);
}

// Point2f LKTracker::project_points(Point2f p1, Point2f p2, double h1, double h2, const tf::Quaternion& q1, const tf::Quaternion& q2)
// {
//   tf::Vector3 new_point;
//   rotate_vec(point.x - 625, point.y - 530, z_img, q, new_point);
//   double x_prime = new_point.getX();
//   double y_prime = new_point.getY();
// }

void LKTracker::drawCircle(int x, int y, int size, const tf::Quaternion& q, Mat image)
{
  tf::Vector3 vprime;
  cout << "(x,y) = (" << x <<"," << y << ")" << endl;
  rotate_vec(x-630, y-530, f, tf::inverse(q), vprime);
  // rotate_vec(x, y, f, q, vprime);
  double x_prime = vprime.getX();
  double y_prime = vprime.getY();
  // double x_prime = vprime.getY();
  // double y_prime = -vprime.getX();
  cout << "(x,y) = (" << x_prime <<"," << y_prime << ")" << endl;
  circle(image, Point(x,y), size, Scalar(0,255,0,255), 3);
  circle(image, Point(x_prime,y_prime), size, Scalar(255,0,0,255), 3);
  return;
}

void LKTracker::ransac(vector<Point2f> src_pts, vector<Point2f> dst_pts, int best_match_count, double *tx, double *ty, double r1, double r2, tf::Quaternion q1, tf::Quaternion q2)
{
  double thresh = 2;
  Point2f p1, p2;
  if (best_match_count > 4)
  {
    int inlier_count, curr_inliers, r;
    curr_inliers = 0.5;
    for (int i = 0; i < 10; i++)
    {
      inlier_count = 0;
      double x_diff, y_diff, x_avg, y_avg, xd, yd;
      for (int j = 0; j < 4; j++)
      {
        r = rand() % best_match_count;
        // p1 = get_dist(src_pts[r], f, r1, q1);
        // p2 = get_dist(dst_pts[r], f, r2, q2);
        // x_diff += (p1.x - p2.x);
        // y_diff += (p1.y - p2.y);

        // p1 = get_dist(src_pts[r], f, r1, q1);
        // p2 = get_dist(dst_pts[r], f, r2, q2);
        // tf::Vector3 p1prime;
        // rotate_vec(p1.x, p1.y, r1, q1*q2.inverse(), p1prime);
        // x_diff += (p1prime.getX() - p2.x);
        // y_diff += (p1prime.getY() - p2.y);
        p1 = angular_correction(src_pts[r], (src_pts[r]-dst_pts[r]));
        x_diff += p1.x;
        y_diff += p1.y;
      }

      x_avg = x_diff/4;
      y_avg = y_diff/4;
      
      x_diff = 0;
      y_diff = 0;
      double avg_err = 0;
      for (int k = 0; k < best_match_count; k++)
      {
        // p1 = get_dist(src_pts[k], f, r1, q1);
        // p2 = get_dist(dst_pts[k], f, r2, q2);
        // xd = p1.x-p2.x;
        // yd = p1.y-p2.y;
        p1 = angular_correction(src_pts[r], (src_pts[r]-dst_pts[r]));
        xd = p1.x;
        yd = p1.y;
        avg_err += sqrt(pow((x_avg-xd),2) + pow((y_avg-yd),2));
        if (sqrt(pow((x_avg-xd),2) + pow((y_avg-yd),2)) < thresh)
        {
          inlier_count ++;
          x_diff += xd;
          y_diff += yd;
        }
      }
      if (inlier_count > curr_inliers)
      {
        *tx = x_diff/inlier_count;
        *ty = y_diff/inlier_count;
        curr_inliers = inlier_count;
      }
    }
  } else
  {
    double src_x, src_y, dst_x, dst_y;
    for (int i = 0; i < best_match_count; i++)
    {
      p1 = get_dist(src_pts[i], f, r1, q1);
      p2 = get_dist(dst_pts[i], f, r2, q2);
      
      src_x += p1.x;
      src_y += p1.y;
      dst_x += p2.x;
      dst_y += p2.y;
    }
    src_x = src_x/best_match_count;
    src_y = src_y/best_match_count;
    dst_x = dst_x/best_match_count;
    dst_y = dst_y/best_match_count;
    
    *tx = (src_x - dst_x);
    *ty = (src_y - dst_y);
  }
  return;
}


void LKTracker::ransac_rot(vector<Point2f> src_pts, vector<Point2f> dst_pts, int best_match_count, double *tx, double *ty, double *theta, double r1, double r2, tf::Quaternion q1, tf::Quaternion q2)
{
  vector<char> mask;
  if (best_match_count < 6) {
    *tx = 0;
    *ty = 0;
    *theta = 0;
    landing_condition = false;
    return;
  }
  landing_condition = false;
  Point2f p2d1, p2d2;
  Point3f p1, p2;
  vector<Point2f> points1, points2;
  vector<Point3f> dst, src;
  for (int i = 0; i < best_match_count; i++)
  {
    p2d1 = get_dist(src_pts[i], f, r1, q1);
    p2d2 = get_dist(dst_pts[i], f, r2, q2);
    // get_3dp(src_pts[i], f, r1, q1, &p1, &p2d1);
    // get_3dp(dst_pts[i], f, r2, q2, &p2, &p2d2);
    points2.push_back(p2d1);
    points1.push_back(p2d2);
    // src.push_back(p1);
    // dst.push_back(p2);
  }
 Mat affine = estimateRigidTransform(points1, points2, false);
//  Mat affine = estimateAffine2D(points1, points2);
  //cout << "affine = " << affine << endl;
 if (affine.rows > 0) {
   *theta = (acos(affine.at<double>(0,0)) + asin(affine.at<double>(0,1)) + -asin(affine.at<double>(1,0)) + acos(affine.at<double>(1,1)))/4.0;
   *tx = affine.at<double>(0,2);
   *ty = affine.at<double>(1,2);
   if ((*theta) != (*theta)) {
     *theta = 0;
   }
   if ((*tx) != (*tx)) {
     *tx = 0;
   }
   if ((*ty) != (*ty)) {
     *ty = 0;
   }
 } else {
   *theta = 0;
   *tx = 0;
   *ty = 0;
    landing_condition = true;
  
 }
 return;
}
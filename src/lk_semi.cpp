#include <LKSemi.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lk_tracker");

  ros::NodeHandle private_node_handle("~");
  LKTracker tracker(private_node_handle);

  ros::Rate r(10);

  // ca::Profiler::enable();

  fisheye::initUndistortRectifyMap(K, D, Mat::eye(3,3, CV_32F), (K_scaled)  ,Size(1500,1500), CV_32F, map1, map2);
  // iter++;
  ofstream myfile;
  gt_prev.x = 0;
  gt_prev.y = 0;
  gt_prev.z = 0;
  bool start = false;
  myfile.open("results.txt");
  while (ros::ok())
  {
    cout << "iter = " << iter << endl;
    ros::spinOnce();
    if (start) {
      if (curr_im.rows > 0) {
        tracker.track_and_extract();
        del_pose.angular.x = iter;
        tracker.pub_.publish(del_pose);
        // iter += 1;
        myfile << del_pose.linear.x;
        myfile << ",";
        myfile << del_pose.linear.y;
        myfile << ",";
        myfile << del_pose.linear.z;
        myfile << ",";
        myfile << q.getX();
        myfile << ",";
        myfile << q.getY();
        myfile << ",";
        myfile << q.getZ();
        myfile << ",";
        myfile << q.getW();
        myfile << ",";
        myfile << gt.x;
        myfile << ",";
        myfile << gt.y;
        myfile << ",";
        myfile << gt.z;
        myfile << ",";
        myfile << iter;
        myfile << ",";
        myfile << roll;
        myfile << ",";
        myfile << rp;
        myfile << ",";
        myfile << pitch;
        myfile << ",";
        myfile << pp;
        myfile << "\n";
      }
    }
    if (!start && curr_range > 0.3) start = true;
    if (start && curr_range < 0.2) start = false;    
    
    r.sleep(); 
  }
  myfile.close();
  return 0;
}

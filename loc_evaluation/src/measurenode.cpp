#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

float gt_x;
float gt_y;
float gt_x_old;
float gt_y_old;
float standstill_x;
float standstill_y;
uint standstill_counter(0);
bool init(false);
bool robot_standstill(false);
uint min_standstill_count(25);
ros::Time stamp;
bool use_standstill;
bool failure_tracking;
double pos_failure_thres;
double ang_failure_thres;
size_t fails = 0;


void compare(const nav_msgs::Odometry::ConstPtr& msg)
{
  gt_x_old = gt_x;
  gt_y_old = gt_y;
  float div_x = msg->pose.pose.position.x;
  float div_y = msg->pose.pose.position.y;
  ROS_INFO_STREAM("Div x: " << div_x << " Div y: " << div_y);
  gt_x = div_x;
  gt_y = div_y;
  stamp = msg->header.stamp;

  if (fabs(gt_x_old - gt_x) < 0.1 && fabs(gt_y_old - gt_y) < 0.1)
  {
    if (!init)
    {
     standstill_counter = 0;
     standstill_x = gt_x;
     standstill_y = gt_y;
     init = true;
    }
    else
    {
      if ((fabs(standstill_x - gt_x) < 0.1 && fabs(standstill_y - gt_y) < 0.1))
        standstill_counter++;
      else
        init = false;
    }
  }
  else
    init = false;

  if (use_standstill)
  {
      if (min_standstill_count < standstill_counter)
        robot_standstill = true;
      ROS_INFO_STREAM("Standstill counter:" <<standstill_counter);
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "loc_evaluation");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub = n.subscribe("base_pose_ground_truth", 1, compare);

  ros::Duration t(1);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string map("/map");
  std::string name;

  ros::Time failure_time = ros::Time(0);
  ros::Duration failure_timeout = ros::Duration(2.);

  std::string path;

  int count_max;

  if (!pn.getParam("tf_prefix", name))
  {
    name = "";
    std::cout << "it failed" << std::endl;
  }

  if (!pn.getParam("path", path))
    path = "";
  std::string file_name, mean_file_name, failure_file_name;
  if (!pn.getParam("file_name", file_name))
    file_name = "measure.txt";

  mean_file_name = std::string("mean_" + file_name);
  failure_file_name = std::string("failures_" + file_name);

  if (!pn.getParam("count_max", count_max))
    count_max = 30;

 if (!pn.getParam("use_standstill", use_standstill))
    use_standstill = false;

 if (!pn.getParam("track_failures", failure_tracking))
    failure_tracking = false;
 if (!pn.getParam("pos_failure_thres", pos_failure_thres))
    pos_failure_thres = 2.0;
 if (!pn.getParam("ang_failure_thres", ang_failure_thres))
    ang_failure_thres = 1.57;

  mean_file_name = path + "/" + mean_file_name;
  file_name = path + "/" + file_name;
  failure_file_name = path + "/" + failure_file_name;
  std::ofstream log, mean_log, failure_log;
  log.open(file_name.c_str(), std::ofstream::out);
  mean_log.open(mean_file_name.c_str(), std::ofstream::out);
  if (failure_tracking)
    failure_log.open(failure_file_name.c_str(), std::ofstream::out);

  int count = 0;
  float sum_r = 0;
  std::vector<float> r_vec;
  while (ros::ok())
  {

    t.sleep();

    ros::spinOnce();

    try
    {
      listener.waitForTransform(map, "/base_footprint", stamp, ros::Duration(0.1));
      /* @todo maybe use ros::Time(0) */
      listener.lookupTransform(map, "/base_footprint", stamp, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      continue;
    }
    float trans_x = transform.getOrigin().getX();
    float trans_y = transform.getOrigin().getY();
    ROS_INFO_STREAM("trans_x: " << trans_x << " trans_y: " << trans_y);

    float r = std::sqrt((trans_x - gt_x)*(trans_x - gt_x)+(trans_y - gt_y)*(trans_y - gt_y));
    if (std::isinf(r))
        continue;
    sum_r += r;
    float err_y = fabs(trans_y-gt_y);
    ROS_INFO_STREAM("R: " << r );
    ROS_INFO_STREAM("Err_y:"<<err_y);

    //log << err_y << std::endl;
    log << r << std::endl;
    ++count;
    r_vec.push_back(r);
    ROS_INFO_STREAM(count);

    if (r > pos_failure_thres && (ros::Time::now() - failure_time) > failure_timeout)
    {
      fails++;
      failure_time = ros::Time::now();
    }

    if (count_max == count)
    {
      ROS_INFO_STREAM("Ended Recording due to max measurement count");
      ROS_INFO_STREAM(path);
      mean_log <<sum_r/count<<std::endl;
      failure_log << fails <<std::endl;
      log.close();
      mean_log.close();
      failure_log.close();
      break;
    }
    else if (robot_standstill)
    {
      ROS_INFO_STREAM("Ended Recording due to robot stand still");
      ROS_INFO_STREAM(path);
      mean_log <<sum_r/count<<std::endl;
      failure_log << fails <<std::endl;
      log.close();
      mean_log.close();
      failure_log.close();
      break;
    }
  }

  mean_log <<"mean: "<<sum_r/count<<std::endl;
  float max_element = *(std::max_element(r_vec.begin(),(r_vec.end()-1)));
  float min_element = *(std::min_element(r_vec.begin(),(r_vec.end()-1)));
  mean_log <<"max_element: "<<max_element<<std::endl;
  mean_log <<"min_element: "<<min_element<<std::endl;
  mean_log.close();
  failure_log << fails <<std::endl;
  failure_log.close();
  return 0;
}

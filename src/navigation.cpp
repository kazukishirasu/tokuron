#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_hello");
  ros::NodeHandle n;
  ros::Rate r(10);
  while (ros::ok())
  {
    ROS_INFO("Hello world");
    r.sleep();
  }
  return 0;
}

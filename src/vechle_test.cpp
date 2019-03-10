#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <path_gen_srv/path_gen_srv.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot1");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<path_gen_srv::path_gen_srv>("/path_server");
  client.waitForExistence();
  ROS_INFO("Server detected.");
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("robot1_path", 10);
  ros::Rate rate(1);
  path_gen_srv::path_gen_srv path_req;
  path_req.request.goal.position.x = 4.5;
  path_req.request.goal.position.y = 4.5;
  path_req.request.start_point.position.x = 0;
  path_req.request.start_point.position.y = 0;
  nav_msgs::Path path;
  ROS_INFO("Initialization complete.");
  while (ros::ok())
  {
    if (client.call(path_req))
    {
      path = path_req.response.Path;
      path_pub.publish(path);
    }
    else
    {
      ROS_ERROR("Failed to call service,is it running?");
    }
    rate.sleep();
  }
  return 0;
}
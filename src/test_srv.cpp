#include <ros/ros.h>
#include <path_gen_srv/path_gen_srv.h>
#include <nav_msgs/GetMap.h>
#include <iostream>

using namespace std;

uint8_t map_arr[4000][4000];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rubbish");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap get_map;
  while (!client.call(get_map))
  {
    ROS_INFO("service call failed,call again.");
  }
  uint16_t counter = 0;
  for (int row = 0; row < 4000; row++)
  {
    for (int column = 0; column < 4000; column++)
    {
      map_arr[row][column] = get_map.response.map.data[counter];
    }
  }
  ROS_INFO("%f", get_map.response.map.info.resolution);
  return 0;
}
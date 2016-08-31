/*
  useage: rosrun save_load_mesh save <filename.ply>
  .ply will save in ($ROS_HOME)/<filename.ply>

  or just run:
  rosservice call /Chisel/SaveMesh "file_name: '<filename.ply>'"
*/
#include <ros/ros.h>
#include <chisel_msgs/SaveMeshService.h>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<chisel_msgs::SaveMeshService>("/Chisel/SaveMesh");
    chisel_msgs::SaveMeshService srv;

    string file_name = argv[1];
    cout << "file_name: " << file_name << endl;

    srv.request.file_name = file_name;

    if (client.call(srv))
    {
        ROS_INFO(".ply save done.");
    }
    else
    {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}

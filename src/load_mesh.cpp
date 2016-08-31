// rosrun save_load_mesh load ($ROS_HOME)/<filename.ply>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

FILE *file;
char str[101];
visualization_msgs::Marker marker;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"load");
    ros::NodeHandle nh("~");

    ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("full_mesh",1000);
    sleep(1); // wait 1 second for init publisher

    file = fopen(argv[1],"r");
    if (!file)
    {
        ROS_ERROR("File open error!");
        return 1;
    }
    // ignore rubbish
    for (int i=0;i<3;i++)
        fgets(str,100,file);
    // read cnt
    int cnt;
    sscanf(str,"element vertex %d",&cnt);
    ROS_INFO("cnt = %d",cnt);
    if (cnt==0)
    {
        ROS_ERROR("No mesh!");
        return 1;
    }
    // ignore rubbish
    for (int i=0;i<4;i++)
        fgets(str,100,file);
    // has color ?
    bool has_color;
    if (str[0]=='p') has_color = true;
    else has_color = false;
    if (has_color)
    {
        ROS_INFO("has_color = true");
        for (int i=0;i<3;i++)
            fgets(str,100,file);
    }
    else
        ROS_INFO("has_color = false");
    // ignore rubbish
    for (int i=0;i<2;i++)
        fgets(str,100,file);

    // begin to read mesh -> marker
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "world";
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    for (int i=0;i<cnt;i++)
    {
        fgets(str,100,file);
        geometry_msgs::Point pt;
        if (has_color)
        {
            int r,g,b;
            sscanf(str,"%lf %lf %lf %d %d %d",&pt.x,&pt.y,&pt.z,&r,&g,&b);
            std_msgs::ColorRGBA color;
            color.r = r/255.0;
            color.g = g/255.0;
            color.b = b/255.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }
        else
        {
            sscanf(str,"%lf %lf %lf",&pt.x,&pt.y,&pt.z);
        }
        marker.points.push_back(pt);
    }
    // ignore indices info
    fclose(file);

    ROS_INFO("marker.points.size() = %lu",marker.points.size());
    ROS_INFO("marker.colors.size() = %lu",marker.colors.size());
    pub_marker.publish(marker);
    ROS_INFO("marker publish done.");
    ros::shutdown();
    return 0;
}

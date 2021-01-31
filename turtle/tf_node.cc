#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_node");
    ros::NodeHandle nl;

    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = nl.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    srv.request.x = 4.0;
    srv.request.y = 4.0;
    srv.request.name = "turtle2";
    
    ROS_INFO("x:%f, y:%f, name: %s",srv.request.x, srv.request.y, srv.request.name.c_str());
    add_turtle.call(srv);

    tf::TransformListener listener;
    
    ros::Rate rate(10.0);
    while(nl.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/turtle2", "turtle1", ros::Time(0), transform);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            continue;
        }
        rate.sleep();
    }
    
    return 0;
}
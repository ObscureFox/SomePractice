/*
 * This file is used to publish navigation message
 *
 * It has some mistake
 *
 *
 */
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <turtlesim/Pose.h>
#include <cmath>

bool flag = true;
int nav_count = 0;
//订阅turtle当前的位姿信息，计算是否到达导航点，发布导航信息
class Nav_node
{
    public:
        Nav_node()
        {
            sub_ = n_.subscribe("turtle1/pose", 1, &Nav_node::callback, this);
            pub_ = n_.advertise<turtlesim::Pose>("/nav_goal", 1);
            ROS_INFO("hello2");
        }
    void callback(const turtlesim::Pose& msg)
    {
        double nav_points[6][3] = {
            {8, 6, 0},
            {3, 2, 3},
            {5, 8, 0},
            {7, 2, 2},
            {2, 6, 0},
            {8, 6, 0}
        };
        ros::Rate loop_rate(10);
        double dist = 0 ;
        while (ros::ok() && nav_count <= 5)
        {
            turtlesim::Pose nav_goal;
            nav_goal.x = nav_points[nav_count][0];
            nav_goal.y = nav_points[nav_count][1];
            //判断是否到达了导航点，欧式距离
            dist = sqrt(pow((msg.x - nav_goal.x),2) + pow((msg.y - nav_goal.y),2));
            if(dist < 0.5)
            {
                ROS_INFO_ONCE("arrive goal: (x:%f, y: %f)", nav_goal.x, nav_goal.y);
                nav_count++;
                continue;
            }
            else
            {
                pub_.publish(nav_goal);
                loop_rate.sleep();
            }
        }
    }
    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_node");

    Nav_node nav_node_sample;
    ROS_INFO("start navigation !"); 
    ros::spin();
    return 0;     
}
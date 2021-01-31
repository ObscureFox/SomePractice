#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <Eigen/Core>
#include <vector>

#define PI 3.141592653
class ControlNode
{
public:
    ControlNode()
    {   
        n_.param("/vel_resolution", v_resolution, 0.1);
        n_.param("/w_resolution", w_resolution, 0.1);
        n_.param("/vel_max", robotModel[0], 1.0);
        n_.param("/acceleration_max", robotModel[1], 0.2);
        n_.param("/angular_velocity_max", robotModel[2], 0.4);
        n_.param("/angular_acceleration_max", robotModel[3], 0.4);
        n_.param("/heading_evaluation_coeff", eval_param[0], 1.0);
        n_.param("/dist_evaluation_coeff", eval_param[1], 1.0);
        n_.param("/vel_evaluation_coeff", eval_param[2], 1.0);
        n_.param("/prediction_time", predection_time, 1.0);

        pub_ = n_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
        sub_ = n_.subscribe("/nav_goal", 1, &ControlNode::NavGoalCallback, this);
        sub2_ = n_.subscribe("turtle1/pose", 1, &ControlNode::TurtlePoseCallback, this);
    }
    
    void NavGoalCallback(const turtlesim::Pose& nav_msg)
    {
        nav_goal[0] = nav_msg.x;
        nav_goal[1] = nav_msg.y;
    }

    void TurtlePoseCallback(const turtlesim::Pose& msg)
    {
        double pose[3] = {msg.x, msg.y, msg.theta};
        geometry_msgs::Twist vel;
        DynamicWindowApproach(nav_goal, pose, robotModel);
        vel.linear.x = best_vw[0];
        vel.angular.z = best_vw[1];
        pub_.publish(vel);
    }

    void DynamicWindowApproach(double robotGoal[2], double robotPose[3], double Model[4])
    {
        robot_state[0] = robotPose[0];
        robot_state[1] = robotPose[1];
        robot_state[2] = robotPose[2];
        GenerateWindow();
        for (v  =  window_vw[0]; v <= window_vw[1]; v += v_resolution)
        {
            for (w = window_vw[2]; w <= window_vw[3]; w += w_resolution)
            {
                GenerateTraj(v, w, robot_state);
                FindDistance(window_vw, robotModel);
                ComputeBrakeDistance(window_vw);
                if (dist > brake_distance)
                {
                    //heading = HeadingCost(robotPose[2], robotGoal[2], window_vw);
                    //clearance = (dist - brake_distance)/(dmax - brake_distance);
                    ComputeCost(v, w);
                    if (cost_sum > optimal_cost)
                    {
                        best_vw[0] = v;
                        best_vw[1] = w;
                        optimal_cost = cost_sum;
                    }
                }
            }
        }
        robot_state[3] = best_vw[0];
        robot_state[4] = best_vw[1]; 
    }

    void GenerateWindow()
    {
        window_vw[0] = std::max(robotModel[0], (robot_state[3] - dt*robotModel[2]));
        window_vw[1] = std::min(robotModel[1], (robot_state[3] + dt*robotModel[2]));
        window_vw[2] = std::max(robotModel[2], (robot_state[4] - dt*robotModel[2]));
        window_vw[3] = std::min(robotModel[3], (robot_state[4] + dt*robotModel[2]));
    }

    void FindDistance(double window_vw_temp[4], double robotModel_temp[5])
    {
        dist = 1;
    }

    void ComputeBrakeDistance(double window_vw_temp[4])
    {
        brake_distance = 0.1;
    }

    void ComputeCost(double v_temp, double w_temp)
    {
        //heading cost function
        goalTheta = atan2((nav_goal[1] - robot_state[1]), (nav_goal[0] - robot_state[0]));
        if(fabs(goalTheta - robot_state[2]) > PI)
        {
            heading = fabs(PI/2 - (2*PI - fabs(goalTheta - robot_state[2])));
        }
        else
        {
            heading = fabs(PI/2 - fabs(goalTheta - robot_state[2]));
        }
        //velocity cost function
        v_temp = fabs(v_temp);
        //clearance cost function
        clearance = 0.1;
        //cost sum 

        cost_sum = heading * eval_param[0] + clearance * eval_param[1] +  v_temp * eval_param[2];
    }

    void GenerateTraj(double param_v, double param_w, double robot_state_temp[5])
    {
        for(double time = 0; time < predection_time; time += dt)
        {        
            robot_state[0] += dt * cos(param_v);
            robot_state[1] += dt * sin(param_v);
            robot_state[2] += dt * param_w;
            //traj.push_back(robot_state);
        }
        //trajectories.push_back(traj);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;

    double nav_goal[2] = {8,6};
    
    //机器人状态：x(m), y(m), yaw(rad), v(m/s), w(rad/s)
    double robot_state[5]={5.544445, 5.544445, 0.000000, 0, 0};
    double robotModel[4]={1, 0.2, 0.4, 0.4};//机器人模型参数：最大速度，最大加速度，最大角速度，最大角加速度
    double v_resolution;//速度分辨率,角度分辨率
    double w_resolution;
    double window_vw[4] = {0, 0, 0, 0};//速度的动态窗口 最小速度 最大速度 最小角速度 最大角速度
    double v = 1.;
    double w = 1.;
    double best_vw[2] ={0,0};//最佳线速度， 角速度
    double brake_distance;//制动距离
    double dist;//实时距离
    double dmax;//所有解中最大距离
    double clearance = 1;
    //总评价函数 
    double cost_sum;
    double optimal_cost = 0;
    double heading;
    std::vector<double> headings;
    double eval_param[3] = {1,1,1};
    //目标点方位
    double goalTheta;  
    //预测轨迹
    //std::vector<state> traj;
    //std::vector<std::vector<state>> trajectories;
    //预测轨迹时长
    double predection_time = 1;
    double dt = 0.1;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ControlNode ControlSample;
    ros::spin();
}
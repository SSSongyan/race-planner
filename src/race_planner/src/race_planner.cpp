#include "race_planner/race_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(race_planner::RacePlanner, nav_core::BaseLocalPlanner)

namespace race_planner
{

    RacePlanner::RacePlanner()
    {
        setlocale(LC_ALL, "");
    }
    
    RacePlanner::~RacePlanner()
    {}

    void RacePlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("RacePlanner initialized with name: %s", name.c_str());
    }
    
    bool RacePlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        return true;
    }
    
    bool RacePlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        return true;
    }
    
    bool RacePlanner::isGoalReached()
    {
        return false;
    }
}

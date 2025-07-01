#ifndef _RACE_PLANNER_H_
#define _RACE_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace race_planner
{
    class RacePlanner : public nav_core::BaseLocalPlanner
    {
        public:
            RacePlanner();
            ~RacePlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();

        private:
            double L = 0.3; // 小车轴距
            double Lfw = 0.0; // 动态前视距离
            double alpha = 0.5; // 舵机打角调节参数
            double beta = 1.5; // 速度控制比例系数
            double min_Lfw = 1.0; // 最小前视距离
            double max_Lfw = 3.0; // 最大前视距离
    };
}

#endif
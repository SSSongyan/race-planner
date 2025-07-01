#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "race_planner/race_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(race_planner::RacePlanner, nav_core::BaseLocalPlanner)

namespace race_planner
{
    int target_index = 0;
    bool pose_judge = false; 
    bool goal_reached = false; 
    tf::TransformListener* tf_listener = nullptr;
    std::vector<geometry_msgs::PoseStamped> global_plan;

    RacePlanner::RacePlanner() {}
    RacePlanner::~RacePlanner() {}

    void RacePlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("Race-Planner Start!");
        tf_listener = new tf::TransformListener();
    }

    bool RacePlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index = 0;
        global_plan = plan;
        pose_judge = false;
        goal_reached = false;

        // 计算路径平均余弦值 
        double sum_cos = 0.0;
        int count = 0;

        for (int i = 1; i < plan.size() - 1; i++)
        {
            auto& p0 = plan[i - 1].pose.position;
            auto& p1 = plan[i].pose.position;
            auto& p2 = plan[i + 1].pose.position;

            double v1_x = p1.x - p0.x, v1_y = p1.y - p0.y;
            double v2_x = p2.x - p1.x, v2_y = p2.y - p1.y;

            double dot = v1_x * v2_x + v1_y * v2_y;
            double norm1 = hypot(v1_x, v1_y);
            double norm2 = hypot(v2_x, v2_y);

            if (norm1 > 1e-3 && norm2 > 1e-3)
            {
                double cos_angle = dot / (norm1 * norm2);
                sum_cos += cos_angle;
                count++;
            }
        }

        double avg_cos = (count > 0) ? (sum_cos / count) : 1.0;

        // 根据平均余弦值调整前视距离 
        Lfw = alpha * max_Lfw / (2.0 - avg_cos);
        Lfw = std::min(std::max(Lfw, min_Lfw), max_Lfw);
        ROS_INFO("New plan set, avg_cos: %.3f, Lfw: %.3f", avg_cos, Lfw);
        
        return true;
    }

    bool RacePlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if (global_plan.empty()) return false;

        // 检查是否到达目标点
        int final_index = global_plan.size() - 1;
        geometry_msgs::PoseStamped pose_final;
        tf_listener->transformPose("tianracer/base_link", global_plan[final_index], pose_final);
        if(pose_judge == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < 1.0){ pose_judge = true;}
        }
        if(pose_judge == true)
        {
            goal_reached = true;
            ROS_WARN("Goal reached!");
            return true;
        }

        // 选取前视目标点
        geometry_msgs::PoseStamped target_pose;
        for (int i = target_index; i < global_plan.size(); i++)
        {
            geometry_msgs::PoseStamped current_pose;
            tf_listener->transformPose("tianracer/base_link", global_plan[i], current_pose);
            double dx = current_pose.pose.position.x;
            double dy = current_pose.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= Lfw)
            {
                target_pose = current_pose;
                target_index = i;
                ROS_INFO("Target index updated to %d, distance: %.2f", target_index, dist);
                break;
            }

            if(i == global_plan.size() - 1)
            {
                target_pose = current_pose;
                ROS_INFO("Reached end of plan, using last point as target.");
            }
        }

        // 纯追踪控制算法 
        double dx = target_pose.pose.position.x;
        double dy = target_pose.pose.position.y;
        double eta = atan2(dy, dx);
        double steering_angle = atan2( (2 * L * sin(eta)) , Lfw ); // 舵机打角计算公式

        cmd_vel.linear.x = beta * std::sqrt(dx * dx + dy * dy); // 比例控制线速度
        cmd_vel.angular.z = cmd_vel.linear.x * tan(steering_angle) / L; // 角速度计算公式

        return true;
    }

    bool RacePlanner::isGoalReached()
    {
        return goal_reached;
    }

} // namespace race_planner

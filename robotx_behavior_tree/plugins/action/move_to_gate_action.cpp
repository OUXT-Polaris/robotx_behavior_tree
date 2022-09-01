#include "rclcpp/rclcpp.hpp"
#include "robotx_behavior_tree/action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "robotx_behavior_msgs/msg/task_object.hpp"
#include <memory>
#include <vector>
#include <string>
#include <optional>

namespace robotx_behavior_tree
{
class MoveToGateAction : public ActionROS2Node
{   
    public:
        MoveToGateAction(const std::string & name, const BT::NodeConfiguration & config)
        : ActionROS2Node(name, config), buffer_(get_clock()), listener_(buffer_)
        {
            declare_parameter("goal_tolerance", 1.0);
            get_parameter("goal_tolerance", goal_tolerance_);
            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);

        }

    protected:
        BT::NodeStatus onStart() override
        {
            std::vector<robotx_behavior_msgs::msg::TaskObject> green_buoy;
            std::vector<robotx_behavior_msgs::msg::TaskObject> red_buoy;
            const auto task_objects_array = getTaskObjects();
            try
            {
                for(size_t i=0;i<static_cast<int>(task_objects_array->task_objects.size());i++){
                    if(task_objects_array->task_objects[i].object_kind == 1){
                        green_buoy.push_back(task_objects_array->task_objects[i]);
                    } else if (task_objects_array->task_objects[i].object_kind == 2)
                    {
                        red_buoy.push_back(task_objects_array->task_objects[i]);
                    }else{
                    }
                }

                buoy1_x = green_buoy[0].x;
                buoy1_y = green_buoy[0].y;
                buoy2_x = red_buoy[0].x;
                buoy2_y = red_buoy[0].y;
                
                goal_x = (buoy1_x + buoy2_x) / 2;
                goal_y = (buoy1_y + buoy2_y) / 2;
                goal_theta = 0.0;

                if(goal_x && goal_y && goal_theta){
                    goal.header.frame_id = "map";
                    goal.pose.position.x = goal_x;
                    goal.pose.position.y = goal_y;
                    goal.pose.position.z = 0.0;

                    goal.pose.orientation.w = 0.0;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;

                    goal_pub_->publish(goal);

                    RCLCPP_INFO(get_logger(), "Published! goal to through the gate");
                    return BT::NodeStatus::RUNNING;
                }else{
                    return BT::NodeStatus::FAILURE;
                }

                return BT::NodeStatus::FAILURE;

            }
            catch(rclcpp::exceptions::InvalidTopicNameError & error)
            {
                std::cerr << error.what() << '\n';
                return BT::NodeStatus::FAILURE;
            }
            
        }
        BT::NodeStatus onRunning() override
        {
            auto pose = getCurrentPose();
            get_parameter("goal_tolerance", goal_tolerance_);
            if(pose){
                distance = getDistance(pose.value(), goal.pose);
            }
            if(distance < goal_tolerance_){
                RCLCPP_INFO(get_logger(), "Throgh Goal : SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        double getDistance(const geometry_msgs::msg::Pose pose1, const geometry_msgs::msg::Pose pose2){
            auto dx = pose1.position.x - pose2.position.x;
            auto dy = pose1.position.y - pose2.position.y;
            auto dz = pose1.position.z - pose2.position.z;

            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        const std::optional<geometry_msgs::msg::Pose> getCurrentPose()
        {
            try{
                auto transform_stamped = buffer_.lookupTransform("map", "base_link", rclcpp::Time(0), tf2::durationFromSec(1.0));
                geometry_msgs::msg::Pose pose;

                pose.position.x = transform_stamped.transform.translation.x;
                pose.position.y = transform_stamped.transform.translation.y;
                pose.position.z = transform_stamped.transform.translation.z;
                pose.orientation = transform_stamped.transform.rotation;
                return pose;
            } catch (tf2::ExtrapolationException & ex) {
                RCLCPP_ERROR(get_logger(), ex.what());
                return std::nullopt;
            }
            return std::nullopt;
        }

        float buoy1_x;
        float buoy1_y;
        float buoy2_x;
        float buoy2_y;
        float goal_x;
        float goal_y;
        float goal_theta;

        float distance;
        double goal_tolerance_;

        geometry_msgs::msg::PoseStamped goal;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

};
}

#include "behavior_tree_action_builder/register_nodes.hpp"  // NOLINT

REGISTER_NODES(robotx_behavior_tree, MoveToGateAction)
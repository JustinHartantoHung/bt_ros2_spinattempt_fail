#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/spin.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "behaviortree_cpp_v3/action_node.h"

struct float32
{
	float value;
};

namespace BT
{
template <> inline
float32 convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 1)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        float32 output;
        output     = convertFromString<float32>(parts[0]);
	return output;
    }
}
} // end namespace BT

class Spinbot : public BT::AsyncActionNode
{
public:
    Spinbot(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<float32>("spin_dis")};
    }

    virtual BT::NodeStatus tick() override
    {
        node_ = rclcpp::Node::make_shared("spin_client");
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::Spin>(node_, "spin");
        // if no server is present, fail after 5 seconds
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            // RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }
        // Take the goal from the InputPort of the Node
        float32 spin_dis;
        if (!getInput<float32>("spin_dis", spin_dis)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [goal] gay");
        }

        _aborted = false;
        
        RCLCPP_INFO(node_->get_logger(), "Sending goal %f", spin_dis);
        

        nav2_msgs::action::Spin::Goal goal_msg;
        
        goal_msg.target_yaw = 10;

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(node_->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                return BT::NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                return BT::NodeStatus::FAILURE;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                return BT::NodeStatus::FAILURE;
        }

        if (_aborted) {
            // this happens only if method halt() was invoked
            //_client.cancelAllGoals();
            RCLCPP_INFO(node_->get_logger(), "MoveBase aborted");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "result received");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override {
        _aborted = true;
    }
private:
    bool _aborted;
    // auto node_ = std::make_shared<rclcpp::Node>("nav2_client");
    rclcpp::Node::SharedPtr node_;
};

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_to_pos.hpp"
#include "my_robot_interfaces/msg/trigger.hpp"

using MoveToPos = my_robot_interfaces::action::MoveToPos;
using MoveToPosGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPos>;
using namespace std::placeholders;

class MoveToPosClientNode : public rclcpp::Node
{
public:
    MoveToPosClientNode() : Node("move_to_pos_client")
    {
        move_to_pos_client_ = rclcpp_action::create_client<MoveToPos>(this, "move_to_pos");

        cancel_sub_ = this->create_subscription<my_robot_interfaces::msg::Trigger>("cancel_move_to_pos_goal", 10, std::bind(&MoveToPosClientNode::cancel_goal, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "move_to_pos_client has been started");
    }

    void send_goal(int target_position, int velocity)
    {
        // wait for the action server
        move_to_pos_client_->wait_for_action_server();

        // create gaol
        auto goal = MoveToPos::Goal();
        goal.target_position = target_position;
        goal.velocity = velocity;

        // Add callbacks
        auto options = rclcpp_action::Client<MoveToPos>::SendGoalOptions();
        options.result_callback = std::bind(&MoveToPosClientNode::get_result_callback, this, _1);
        options.goal_response_callback = std::bind(&MoveToPosClientNode::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&MoveToPosClientNode::goal_feedback_callback, this, _1, _2);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        move_to_pos_client_->async_send_goal(goal, options);

    }

    void cancel_goal(const my_robot_interfaces::msg::Trigger msg)
    {
        (void)msg;
        cancel_callback();
    }

private:
    void cancel_callback()
    {
        if (this->goal_handle_)
        {

            RCLCPP_INFO(this->get_logger(), "Cancel the goal");
            move_to_pos_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
            
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "goal cancle requested but there is no goal to cancel");
            
        }
        
    }

    // callback to know if the gaol is accepted or rejected
    void goal_response_callback(const MoveToPosGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            // rejected
            RCLCPP_INFO(this->get_logger(), "goal got rejected");
        }
        else
        {
            // accepted
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "goal got accepted");
        }
    }

    // callback ro recieve feedback during goal execution
    void goal_feedback_callback(const MoveToPosGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const MoveToPos::Feedback> feedback)
    {
        (void)goal_handle;
        int current_position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d", current_position);
    }

    // calback to recive the result once the goal is done
    void get_result_callback(const MoveToPosGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }

        int position = result.result->position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Result: position: %d, message: %s", position, message.c_str());
    }

    rclcpp_action::Client<MoveToPos>::SharedPtr move_to_pos_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    MoveToPosGoalHandle::SharedPtr goal_handle_;
    rclcpp::Subscription<my_robot_interfaces::msg::Trigger>::SharedPtr cancel_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPosClientNode>();
    node->send_goal(76, 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
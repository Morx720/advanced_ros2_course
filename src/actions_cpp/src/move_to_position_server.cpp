#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_to_pos.hpp"

using MoveToPos = my_robot_interfaces::action::MoveToPos;
using MoveToPosGoalHandle = rclcpp_action::ServerGoalHandle<MoveToPos>;
using namespace std::placeholders;

class MoveToPosServerNode : public rclcpp::Node
{
public:
    MoveToPosServerNode() : Node("move_to_pos_server")
    {
        this->declare_parameter("min_pos", 0);
        min_pos_ = this->get_parameter("min_pos").as_int();

        this->declare_parameter("max_pos", 100);
        max_pos_ = this->get_parameter("max_pos").as_int();

        this->declare_parameter("start_pos", 50);
        position_ = this->get_parameter("start_pos").as_int();

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_to_pos_server_ = rclcpp_action::create_server<MoveToPos>(
            this,
            "move_to_pos",
            std::bind(&MoveToPosServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveToPosServerNode::cancel_callback, this, _1),
            std::bind(&MoveToPosServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_);

        RCLCPP_INFO(this->get_logger(), "move_to_pos_server has been started");
    }

private:
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPos::Goal> goal)
    {
        // prevent unused warning
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "Recieved a goal");

        // // policy: refuse new goal if one goal being executed
        // {
        //     std::lock_guard<std::mutex> lock(mutex_);
        //     if (goal_handle_)
        //     {
        //         if (goal_handle_->is_active())
        //         {
        //             RCLCPP_INFO(this->get_logger(), "A goal is still active, reject new goal");
        //             return rclcpp_action::GoalResponse::REJECT;
        //         }
        //     }
        // }

        if (min_pos_ > goal->target_position or goal->target_position > max_pos_ or goal->velocity <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "rejecting the goal");
            RCLCPP_DEBUG(this->get_logger(), "target position: %ld", goal->target_position);
            RCLCPP_DEBUG(this->get_logger(), "velocity: %ld", goal->velocity);

            return rclcpp_action::GoalResponse::REJECT;
        }

        // policy: preempt existing goal when resiving a new valid goal
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_)
            {
                if (goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(), "Abort current goal and accept new goal");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveToPosGoalHandle> goal_handle)
    {

        RCLCPP_INFO(this->get_logger(), "Recieved cancel request");

        // prevent unused warning
        (void)goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<MoveToPosGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal..");

        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveToPosGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        // get request from the goal
        int target_position = goal_handle->get_goal()->target_position;
        int velocity = goal_handle->get_goal()->velocity;

        // Execute the action
        auto result = std::make_shared<MoveToPos::Result>();
        auto feedback = std::make_shared<MoveToPos::Feedback>();
        rclcpp::Rate loop_rate(1.0 / 1);
        while (position_ != target_position)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_)
                {
                    result->position = position_;
                    result->message = "Preempted by another goal";
                    goal_handle->abort(result);
                    return;
                }
            }

            if (goal_handle->is_canceling())
            {
                result->position = position_;
                result->message = "goal canceled or aborted";
                goal_handle->canceled(result);
                return;
            }

            int rem_dis_ = abs(target_position - position_);

            if (target_position > position_)
            {
                // move to plus
                if (rem_dis_ >= velocity)
                {
                    position_ += velocity;
                }
                else
                {
                    position_ += rem_dis_;
                }
            }
            else if (target_position < position_)
            {
                // minus
                if (rem_dis_ >= velocity)
                {
                    position_ += velocity;
                }
                else
                {
                    position_ += rem_dis_;
                }
            }

            RCLCPP_INFO(this->get_logger(), "%d", position_);
            feedback->current_position = position_;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        // Set the final state and return the result
        result->position = position_;
        result->message = "position reatched";
        goal_handle->succeed(result);
    }

    int min_pos_;
    int max_pos_;
    int position_;
    rclcpp_action::Server<MoveToPos>::SharedPtr move_to_pos_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::shared_ptr<MoveToPosGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPosServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

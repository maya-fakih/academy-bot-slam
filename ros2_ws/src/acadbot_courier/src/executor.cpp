// executor.cpp — see executor.hpp for the contract this implements.
#include "executor.hpp"
#include <tf2/LinearMath/Quaternion.h>

Executor::Executor(rclcpp::Node * node)
: node_(node)
{
  // Same action name Nav2 always exposes — same one patrol_commander uses.
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

void Executor::driveTo(double x, double y, double yaw,
                        const std::string & frame_id,
                        FeedbackCb on_feedback, DoneCb on_done)
{
  if (!client_->action_server_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "Executor: Nav2 action server not ready.");
    goal_active_ = false;
    on_done(false, "nav2 not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

  // Fires once, immediately: did Nav2 accept the goal at all?
  opts.goal_response_callback =
    [this, on_done](GoalHandle::SharedPtr gh) {
      if (!gh) {
        RCLCPP_WARN(node_->get_logger(), "Executor: goal rejected by Nav2.");
        goal_active_ = false;
        on_done(false, "goal rejected by nav2");
        return;
      }
      current_goal_handle_ = gh;
    };

  // Fires repeatedly while driving.
  opts.feedback_callback =
    [on_feedback](GoalHandle::SharedPtr,
                  const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      on_feedback(fb->distance_remaining);
    };

  // Fires exactly once when the goal ends, however it ends.
  opts.result_callback =
    [this, on_done](const GoalHandle::WrappedResult & result) {
      goal_active_ = false;
      current_goal_handle_.reset();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          on_done(true, "reached");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          on_done(false, "nav2 aborted (recoveries exhausted)");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          on_done(false, "canceled");
          break;
        default:
          on_done(false, "unknown result code");
          break;
      }
    };

  goal_active_ = true;
  client_->async_send_goal(goal, opts);
}

void Executor::cancel()
{
  if (current_goal_handle_) {
    client_->async_cancel_goal(current_goal_handle_);
  }
}
#pragma once
// executor.hpp
// ---------------------------------------------------------------------------
// Executor: a plain C++ class (NOT a node, NOT an action server) that owns a
// Nav2 navigate_to_pose action client and drives to exactly one pose at a
// time. It knows nothing about jobs, legs, retries, or job IDs — that logic
// belongs to the dispatcher (architecture.md decision 2). This class only
// answers: "drive here, tell me how it's going, tell me when it's done."
// ---------------------------------------------------------------------------
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class Executor
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Fired repeatedly while driving, with Nav2's own distance_remaining.
  using FeedbackCb = std::function<void(double distance_remaining)>;
  // Fired exactly once when the goal ends. success=true only on SUCCEEDED.
  // reason is human-readable, useful for ABORTED/CANCELED/REJECTED cases.
  using DoneCb = std::function<void(bool success, const std::string & reason)>;

  // node: the owning node (dispatcher). We piggyback on its logger, clock,
  // and executor — we do NOT spin our own node, we're not a node at all.
  explicit Executor(rclcpp::Node * node);

  // Send a single navigate_to_pose goal. Any previous in-flight goal from
  // this Executor is NOT auto-cancelled — call cancel() yourself first if
  // you need that (dispatcher decides that policy, not us).
  void driveTo(double x, double y, double yaw,
               const std::string & frame_id,
               FeedbackCb on_feedback, DoneCb on_done);

  // Cancel whatever goal is currently in flight. Safe to call if nothing
  // is running (no-op). done_cb from the original driveTo() call will still
  // fire with success=false, reason="canceled" once Nav2 confirms.
  void cancel();

  // True if a goal is currently active (sent, not yet finished/canceled).
  bool isBusy() const { return goal_active_; }

private:
  rclcpp::Node * node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandle::SharedPtr current_goal_handle_;
  bool goal_active_{false};
};
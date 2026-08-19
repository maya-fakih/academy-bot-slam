// dispatcher.cpp
// ---------------------------------------------------------------------------
// Dispatcher node — STAGE 3: adds the ExecuteDelivery action server, real
// two-leg driving (pickup then dropoff), per-leg retries, and honest
// success/failure reporting. This covers requirements 1-8; requirement 9
// (single launch) is separate, not in this file.
// ---------------------------------------------------------------------------
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"
#include "acadbot_courier_msgs/action/execute_delivery.hpp"
#include "executor.hpp"

using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
using ExecuteDelivery = acadbot_courier_msgs::action::ExecuteDelivery;
using GoalHandleExecuteDelivery = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;

struct LocationPose { double x, y, yaw; };

// A queued/running job. goal_handle is null until the requester actually
// calls the ExecuteDelivery action for this job_id — feedback/result only
// get published once that happens; the job still runs either way.
struct Job
{
  std::string job_id;
  std::string pickup, dropoff;
  std::shared_ptr<GoalHandleExecuteDelivery> goal_handle;
};

class Dispatcher : public rclcpp::Node
{
public:
  Dispatcher() : Node("dispatcher")
  {
    loadLocations();
    retry_limit_ = declare_parameter<int>("retry_limit", 2);
    executor_ = std::make_unique<Executor>(this);

    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&Dispatcher::handleRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<ExecuteDelivery>(
      this, "execute_delivery",
      std::bind(&Dispatcher::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&Dispatcher::handleCancel, this, std::placeholders::_1),
      std::bind(&Dispatcher::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dispatcher ready — %zu known locations, retry_limit=%d.",
      locations_.size(), retry_limit_);
  }

private:
  // ---- location config (decision 5) --------------------------------------
  void loadLocations()
  {
    declare_parameter<std::vector<std::string>>("location_names",
                                                  std::vector<std::string>{});
    auto names = get_parameter("location_names").as_string_array();

    for (const auto & name : names) {
      declare_parameter<double>("locations." + name + ".x", 0.0);
      declare_parameter<double>("locations." + name + ".y", 0.0);
      declare_parameter<double>("locations." + name + ".yaw", 0.0);

      LocationPose pose;
      get_parameter("locations." + name + ".x", pose.x);
      get_parameter("locations." + name + ".y", pose.y);
      get_parameter("locations." + name + ".yaw", pose.yaw);
      locations_[name] = pose;

      RCLCPP_INFO(get_logger(), "  location '%s' = (%.2f, %.2f, yaw=%.2f)",
                  name.c_str(), pose.x, pose.y, pose.yaw);
    }
  }

  // ---- RequestDelivery service (requirements 1, 2) -----------------------
  void handleRequest(
    const std::shared_ptr<RequestDelivery::Request> req,
    std::shared_ptr<RequestDelivery::Response> res)
  {
    if (!locations_.count(req->pickup)) {
      res->accepted = false;
      res->reason = "unknown pickup location: " + req->pickup;
      res->job_id = "";
      RCLCPP_WARN(get_logger(), "Rejected: %s", res->reason.c_str());
      return;
    }
    if (!locations_.count(req->dropoff)) {
      res->accepted = false;
      res->reason = "unknown dropoff location: " + req->dropoff;
      res->job_id = "";
      RCLCPP_WARN(get_logger(), "Rejected: %s", res->reason.c_str());
      return;
    }

    std::string job_id = "job-" + std::to_string(next_job_id_++);

    res->accepted = true;
    res->reason = "accepted";
    res->job_id = job_id;

    RCLCPP_INFO(get_logger(), "Accepted %s: %s -> %s",
                job_id.c_str(), req->pickup.c_str(), req->dropoff.c_str());

    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back({job_id, req->pickup, req->dropoff, nullptr});
    tryStartNext();
  }

  // ---- ExecuteDelivery action server (requirements 3, 4, 6) --------------
  // Accept only goals whose job_id was actually handed out by the service.
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteDelivery::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & job : queue_) {
      if (job.job_id == goal->job_id) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    }
    if (running_job_ && running_job_->job_id == goal->job_id) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    RCLCPP_WARN(get_logger(), "ExecuteDelivery goal for unknown job_id '%s'",
                goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteDelivery>)
  {
    // Cancel means stop (requirement 6) — always allow it, cancel the
    // in-flight Nav2 goal immediately. The executor's done callback fires
    // afterwards and finishes the action as CANCELED.
    executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteDelivery> gh)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string & job_id = gh->get_goal()->job_id;
    for (auto & job : queue_) {
      if (job.job_id == job_id) { job.goal_handle = gh; return; }
    }
    if (running_job_ && running_job_->job_id == job_id) {
      running_job_->goal_handle = gh;
    }
  }

  // ---- driving (requirements 3, 4, 5, 6, 7) -------------------------------
  void tryStartNext()
  {
    if (running_job_ || queue_.empty()) return;

    running_job_ = std::make_shared<Job>(queue_.front());
    queue_.pop_front();

    RCLCPP_INFO(get_logger(), "[%s] starting: %s -> %s",
                running_job_->job_id.c_str(),
                running_job_->pickup.c_str(), running_job_->dropoff.c_str());

    driveLeg("pickup", running_job_->pickup, 0);
  }

  // leg is "pickup" or "dropoff". attempt counts retries for THIS leg only
  // (architecture.md decision 4 — retries are per leg).
  void driveLeg(const std::string & leg, const std::string & location_name,
                int attempt)
  {
    auto job = running_job_;  // keep alive across the async callback
    auto p = locations_.at(location_name);

    publishFeedback(job, leg, location_name, p, -1.0);

    executor_->driveTo(p.x, p.y, p.yaw, "map",
      [this, job, leg, location_name, p](double dist) {
        publishFeedback(job, leg, location_name, p, dist);
      },
      [this, job, leg, location_name, attempt](bool success,
                                                 const std::string & reason) {
        onLegDone(job, leg, location_name, attempt, success, reason);
      });
  }

  void publishFeedback(const std::shared_ptr<Job> & job, const std::string & leg,
                        const std::string & location_name, const LocationPose & p,
                        double dist)
  {
    if (!job->goal_handle) return;
    auto fb = std::make_shared<ExecuteDelivery::Feedback>();
    fb->current_leg = leg;
    fb->heading_to.name = location_name;
    fb->heading_to.x = p.x;
    fb->heading_to.y = p.y;
    fb->heading_to.yaw = p.yaw;
    fb->distance_remaining = dist;  // -1.0 until first real Nav2 feedback
    job->goal_handle->publish_feedback(fb);
  }

  void onLegDone(const std::shared_ptr<Job> & job, const std::string & leg,
                 const std::string & location_name, int attempt,
                 bool success, const std::string & reason)
  {
    if (success) {
      RCLCPP_INFO(get_logger(), "[%s] %s leg OK.", job->job_id.c_str(), leg.c_str());
      if (leg == "pickup") {
        driveLeg("dropoff", job->dropoff, 0);
      } else {
        finish(job, true, "delivered", "");
      }
      return;
    }

    if (reason == "canceled") {
      finish(job, false, "canceled by requester", leg);
      return;
    }

    if (attempt + 1 < retry_limit_) {
      RCLCPP_WARN(get_logger(), "[%s] %s leg failed (%s) — retry %d/%d",
                  job->job_id.c_str(), leg.c_str(), reason.c_str(),
                  attempt + 2, retry_limit_);
      driveLeg(leg, location_name, attempt + 1);
      return;
    }

    RCLCPP_ERROR(get_logger(), "[%s] %s leg failed after %d attempts — giving up.",
                 job->job_id.c_str(), leg.c_str(), retry_limit_);
    finish(job, false, leg + " leg failed: " + reason, leg);
  }

  // success=true -> SUCCEEDED. success=false + failed_leg empty -> this only
  // happens on the cancel path, reported as CANCELED. success=false with a
  // failed_leg -> retries exhausted, reported as ABORTED (requirement 7).
  void finish(const std::shared_ptr<Job> & job, bool success,
              const std::string & message, const std::string & failed_leg)
  {
    if (job->goal_handle) {
      auto result = std::make_shared<ExecuteDelivery::Result>();
      result->success = success;
      result->result_message = message;
      result->failed_leg = failed_leg;
      if (success) {
        job->goal_handle->succeed(result);
      } else if (message == "canceled by requester") {
        job->goal_handle->canceled(result);
      } else {
        job->goal_handle->abort(result);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    running_job_.reset();
    tryStartNext();
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  std::map<std::string, LocationPose> locations_;
  std::unique_ptr<Executor> executor_;

  std::mutex mutex_;
  std::deque<Job> queue_;
  std::shared_ptr<Job> running_job_;

  int retry_limit_{2};
  int next_job_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dispatcher>());
  rclcpp::shutdown();
  return 0;
}
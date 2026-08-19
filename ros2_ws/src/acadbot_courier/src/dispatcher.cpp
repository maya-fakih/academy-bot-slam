// dispatcher.cpp
// ---------------------------------------------------------------------------
// Dispatcher node — STAGE 2: node skeleton + named-location config +
// RequestDelivery service + job queue + Executor wiring (pickup leg only).
//
// ExecuteDelivery action server, dropoff leg chaining, retries, and
// reporting feedback/result back to a requester come in stage 3.
// ---------------------------------------------------------------------------
#include <deque>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"
#include "executor.hpp"

using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;

struct LocationPose { double x, y, yaw; };

struct Job
{
  std::string job_id;
  std::string pickup, dropoff;
};

class Dispatcher : public rclcpp::Node
{
public:
  Dispatcher() : Node("dispatcher")
  {
    loadLocations();
    executor_ = std::make_unique<Executor>(this);

    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&Dispatcher::handleRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "dispatcher ready — %zu known locations.",
                locations_.size());
  }

private:
  // Locations arrive as nested params: locations.<name>.x / .y / .yaw
  // (architecture.md decision 5). Declared here with an empty default so a
  // missing YAML just means zero known locations, not a crash.
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

    // Job ID generation — simple incrementing counter for now, good enough
    // to be unique within one dispatcher run.
    std::string job_id = "job-" + std::to_string(next_job_id_++);

    res->accepted = true;
    res->reason = "accepted";
    res->job_id = job_id;

    RCLCPP_INFO(get_logger(), "Accepted %s: %s -> %s",
                job_id.c_str(), req->pickup.c_str(), req->dropoff.c_str());

    queue_.push_back({job_id, req->pickup, req->dropoff});
    tryStartNext();
  }

  // Starts the next queued job's pickup leg, only if nothing is running.
  // Stage 2: drives the pickup leg and logs the result — does not yet chain
  // into the dropoff leg or report anything back to a requester. That
  // wiring (and the retry logic from architecture.md decision 4) is stage 3,
  // once ExecuteDelivery exists to report through.
  void tryStartNext()
  {
    if (job_running_ || queue_.empty()) return;

    Job job = queue_.front();
    queue_.pop_front();
    job_running_ = true;

    auto p = locations_.at(job.pickup);
    RCLCPP_INFO(get_logger(), "[%s] driving to pickup '%s'",
                job.job_id.c_str(), job.pickup.c_str());

    executor_->driveTo(p.x, p.y, p.yaw, "map",
      [this, job](double dist) {
        RCLCPP_DEBUG(get_logger(), "[%s] distance remaining: %.2f",
                     job.job_id.c_str(), dist);
      },
      [this, job](bool success, const std::string & reason) {
        RCLCPP_INFO(get_logger(), "[%s] pickup leg finished: %s (%s)",
                    job.job_id.c_str(), success ? "OK" : "FAILED",
                    reason.c_str());
        job_running_ = false;
        tryStartNext();  // pick up the next queued job
      });
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  std::map<std::string, LocationPose> locations_;
  std::unique_ptr<Executor> executor_;
  std::deque<Job> queue_;
  bool job_running_{false};
  int next_job_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dispatcher>());
  rclcpp::shutdown();
  return 0;
}
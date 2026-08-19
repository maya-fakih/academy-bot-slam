// dispatcher.cpp
// ---------------------------------------------------------------------------
// Dispatcher node — STAGE 1: node skeleton + named-location config +
// RequestDelivery service (accept/reject only).
//
// ExecuteDelivery action server, job queue, and Executor wiring come in
// later stages, once this stage builds and runs cleanly.
// ---------------------------------------------------------------------------
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"

using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;

struct LocationPose { double x, y, yaw; };

class Dispatcher : public rclcpp::Node
{
public:
  Dispatcher() : Node("dispatcher")
  {
    loadLocations();

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

    // TODO(stage 2): push {job_id, pickup, dropoff} onto a queue instead of
    // just accepting and forgetting — nothing drives yet at this stage.
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  std::map<std::string, LocationPose> locations_;
  int next_job_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dispatcher>());
  rclcpp::shutdown();
  return 0;
}
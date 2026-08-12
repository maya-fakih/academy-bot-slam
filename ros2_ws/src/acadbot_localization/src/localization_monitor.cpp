#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class LocalizationMonitor : public rclcpp::Node
{
    public:
        LocalizationMonitor() : Node("localization_monitor")
        {
            // ---- Parameters (override from yaml/cli) ----
            // seconds between lines, default 1.o
            report_period_ = declare_parameter<double>("report_period", 1.0);
            // it is considered converged if the stddev is less than this many meters
            converged_sigma_ = declare_parameter<double>("converged_sigma", 0.25);
        }

    private:
        double report_period_;
        double converged_sigma_;
};

// 2. The C++ node
// localization_monitor answers a question RViz only answers by eye: is AMCL actually localised, or is it just running?

// It subscribes to /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped) and, on a timer, logs one line: the robot's x, y, yaw, and how uncertain AMCL is about that position.

// The uncertainty is in the message's covariance — a row-major 6×6 array. Index 0 is the variance in x, index 7 the variance in y. Report the standard deviation, sqrt of the larger of the two, in metres.
// Two parameters, in the config YAML: report_period (seconds between lines, default 1.0) and converged_sigma (metres, default 0.25). Print CONVERGED or SEARCHING by comparing against the threshold.
// Before an initial pose is set, AMCL publishes nothing at all. Handle that: log a warning saying so rather than printing zeros. That warning is the most useful line your node will produce.
// Add the node to your launch file so one command starts everything.
// Keep it small. It is a subscriber, a timer, two parameters and a log line — about eighty lines including comments.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <algorithm>
#include <string>

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

            pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            10,
            std::bind(&LocalizationMonitor::pose_callback, this, std::placeholders::_1));

            // timer to log results each report_period
            timer_ = create_wall_timer(
            std::chrono::duration<double>(report_period_),
            std::bind(&LocalizationMonitor::timer_callback, this));
        }

    private:
        double report_period_;
        double converged_sigma_;
        rclcpp::TimerBase::SharedPtr timer_;
        // latest AMCL estimate we've received, if any
        geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
        bool has_pose_ = false;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            latest_pose_ = *msg;
            has_pose_ = true;
        }

        void timer_callback()
        {
            // AMCL publishes nothing on /amcl_pose until it's given an initial pose
            // estimate (2D Pose Estimate in RViz) - has_pose_ lets us tell that
            // apart from a real pose sitting at the origin.
            if (!has_pose_)
            {
                RCLCPP_WARN(get_logger(), "No pose received yet - has AMCL been given an initial pose estimate?");
                return;
            }
            double x = latest_pose_.pose.pose.position.x;
            double y = latest_pose_.pose.pose.position.y;
            double yaw = tf2::getYaw(latest_pose_.pose.pose.orientation);
            
            // covariance is a flat 6x6 matrix (x,y,z,roll,pitch,yaw), row-major.
            // diagonal entries are each dimension's own variance: index = row*6 + col.
            // x-x is (0,0) -> index 0. y-y is (1,1) -> index 7.
            // AMCL only meaningfully varies in x/y/yaw on a 2D map (no z/roll/pitch
            // for a ground robot) - using x and y here as the "am I localized" signal.
            double var_x = latest_pose_.pose.covariance[0];
            double var_y = latest_pose_.pose.covariance[7];
            
            // sqrt of the larger variance = a cheap, conservative stand-in for the
            // true x-y uncertainty ellipse's longest axis. Not exact, but never
            // overstates confidence - errs toward SEARCHING over false CONVERGED.
            double sigma = std::sqrt(std::max(var_x, var_y));

            std::string status = (sigma < converged_sigma_) ? "CONVERGED" : "SEARCHING";
            RCLCPP_INFO(get_logger(), "x=%.2f y=%.2f yaw=%.2f sigma=%.3f [%s]",
                        x, y, yaw, sigma, status.c_str());
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationMonitor>());
    rclcpp::shutdown();
    return 0;
}

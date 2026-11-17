// Path node: walks the dog round a list of waypoints, again and again. It steers towards the
// next waypoint by sending cmd_vel to the gait node, using the body's position from /odom
// (Gazebo's true position of the body).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const double PI = 3.14159265358979323846;

class PathNode : public rclcpp::Node {
public:
    PathNode() : Node("path_node") {
        waypoints_ = declare_parameter<std::vector<double>>("waypoints");  // x0, y0, x1, y1, ...
        speed_ = declare_parameter("speed", 0.06);         // m/s when facing the waypoint
        max_turn_ = declare_parameter("max_turn", 0.4);    // rad/s
        turn_gain_ = declare_parameter("turn_gain", 1.5);  // rad/s of turn per rad of heading error
        reach_ = declare_parameter("reach", 0.06);         // m, this close counts as reached
        if (waypoints_.size() < 2 || waypoints_.size() % 2 != 0) {
            throw std::invalid_argument("waypoints must be a list of x, y pairs");
        }
        if (max_turn_ <= 0.0 || reach_ <= 0.0) {
            throw std::invalid_argument("max_turn and reach must be above 0");
        }

        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](const nav_msgs::msg::Odometry& msg) { steer(msg); });

        // The gait node keeps the last cmd_vel, so if /odom stops, tell it to stop too.
        last_odom_ = now();
        timer_ = create_timer(100ms, [this] {
            if ((now() - last_odom_).seconds() > 0.5) pub_->publish(geometry_msgs::msg::Twist());
        });
    }

private:
    void steer(const nav_msgs::msg::Odometry& odom) {
        last_odom_ = now();
        const auto& p = odom.pose.pose.position;
        const auto& q = odom.pose.pose.orientation;
        double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

        double dx = waypoints_[2 * next_] - p.x;
        double dy = waypoints_[2 * next_ + 1] - p.y;
        if (std::hypot(dx, dy) < reach_) {
            next_ = (next_ + 1) % (waypoints_.size() / 2);  // after the last, the first again
            return;
        }

        // turn towards the waypoint, and walk slower the more the dog faces away from it
        double error = std::remainder(std::atan2(dy, dx) - yaw, 2 * PI);
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = std::clamp(turn_gain_ * error, -max_turn_, max_turn_);
        cmd.linear.x = speed_ * std::max(0.0, std::cos(error));
        pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> waypoints_;
    double speed_, max_turn_, turn_gain_, reach_;
    size_t next_ = 0;         // index of the waypoint the dog walks to
    rclcpp::Time last_odom_;  // when the last /odom arrived
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathNode>());
    rclcpp::shutdown();
    return 0;
}

// Gait node for the robot dog. Every 20 ms it works out where each foot should be, turns that
// into hip and knee angles with leg.hpp and sends the 8 angles to the leg controller.
// The dog stands up, then walks forward at `speed` until a cmd_vel arrives. While no turn is
// asked for, it holds its heading with the IMU. Like the course robot, it can also stand and
// sit: send "walk", "stand" or "sit" on the mode topic.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_dog_sim/leg.hpp"

using namespace std::chrono_literals;

const Leg LEG{0.067, 0.075};  // femur and tibia of the course robot, in metres
const double TRACK = 0.115;   // distance between the left and right feet, in metres (estimate)

// Sitting: every leg bends more than when standing, the rear legs most, so the body tilts back.
const Point SIT_FRONT{0.0, 0.09};  // foot under the hip, 9 cm down
const Point SIT_REAR{0.0, 0.04};
const double MOVE_TIME = 1.0;      // s, how long a change of pose takes

class GaitNode : public rclcpp::Node {
public:
    GaitNode() : Node("gait_node") {
        std::string name = declare_parameter<std::string>("gait", "walk");
        period_ = declare_parameter("period", 0.8);              // s, one gait cycle
        stand_time_ = declare_parameter("stand_time", 2.0);      // s, standing before walking
        target_speed_ = declare_parameter("speed", 0.06);        // m/s, until a cmd_vel arrives
        max_stride_ = declare_parameter("max_stride", 0.06);     // m
        heading_gain_ = declare_parameter("heading_gain", 1.0);  // rad/s of turn per rad of error
        gait_.height = declare_parameter("height", 0.10);        // m, hip above the foot
        gait_.lift = declare_parameter("lift", 0.02);            // m

        // A leg is in the air while (phase + its offset) mod 1 is between the duty factor and 1.
        if (name == "walk") {
            gait_.duty = 0.75;
            offset_ = {0.5, 0.0, 0.75, 0.25};  // swing order RL, FL, RR, FR
        } else if (name == "trot") {
            gait_.duty = 0.5;
            offset_ = {0.0, 0.5, 0.5, 0.0};    // FL with RR, then FR with RL
        } else {
            throw std::invalid_argument("gait must be walk or trot");
        }

        // The foot is farthest from the hip at the ends of the longest stride, and nearest at
        // the top of the swing. If both are in reach, the whole foot path is.
        double hip, knee;
        if (period_ <= 0.0 || !ik(LEG, {max_stride_ / 2, gait_.height}, hip, knee) ||
            !ik(LEG, {0.0, gait_.height - gait_.lift}, hip, knee)) {
            throw std::invalid_argument("period must be above 0 and the whole foot path in reach");
        }

        // the legs start straight, as the dog is spawned
        feet_.fill({0.0, LEG.thigh + LEG.shin});
        from_ = feet_;

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_controller/commands", 10);
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
                target_speed_ = msg.linear.x;
                target_turn_ = msg.angular.z;
            });
        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "mode", 10, [this](const std_msgs::msg::String& msg) {
                if (msg.data == "walk" || msg.data == "stand" || msg.data == "sit") {
                    requested_ = msg.data;
                } else {
                    RCLCPP_WARN(get_logger(), "mode must be walk, stand or sit");
                }
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, [this](const sensor_msgs::msg::Imu& msg) {
                // yaw (the heading) from the orientation quaternion
                const auto& q = msg.orientation;
                yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            });
        timer_ = create_timer(20ms, [this] { step(); });  // 50 Hz on the node's clock
    }

private:
    void step() {
        if (!started_) {
            start_ = now();
            started_ = true;
        }
        double t = (now() - start_).seconds();

        // a new mode: remember where the feet are, and start again from a stop
        if (requested_ != mode_) {
            mode_ = requested_;
            change_time_ = t;
            from_ = feet_;
            speed_ = 0.0;
            turn_ = 0.0;
            heading_ = yaw_;
        }
        double since = t - change_time_;  // s in this mode

        std::array<Point, 4> feet;  // FL, FR, RL, RR
        if (mode_ != "walk" || since < stand_time_) {
            // Move every foot in a straight line from where it was to the stand or sit pose,
            // in MOVE_TIME seconds. A walk starts with stand_time seconds of standing.
            double a = std::min(since / MOVE_TIME, 1.0);
            a = (1 - std::cos(PI * a)) / 2;  // 0 to 1, starting and ending slowly
            for (int k = 0; k < 4; k++) {
                Point goal{0.0, gait_.height};  // stand: foot under the hip
                if (mode_ == "sit") goal = (k < 2) ? SIT_FRONT : SIT_REAR;  // FL, FR in front
                const Point& from = from_[k];
                feet[k] = {from.x + a * (goal.x - from.x), from.z + a * (goal.z - from.z)};
            }
        } else {
            // follow the commanded speed and turn rate smoothly (time constant 1 s)
            speed_ += 0.02 * (target_speed_ - speed_);
            turn_ += 0.02 * (target_turn_ - turn_);

            // heading hold: while no turn is asked for, steer back to the heading the dog had
            // when the last turn ended. remainder() wraps the error into -pi..pi. Right after a
            // turn, turn_ is still dying away, so the dog swings a little past that heading first.
            double turn = turn_;
            if (target_turn_ == 0.0) {
                turn += heading_gain_ * std::remainder(heading_ - yaw_, 2 * PI);
            } else {
                heading_ = yaw_;
            }

            // body speed = stride / time on the ground (for feet that do not slip), so
            // stride = speed * duty * period. To turn left, the right feet push back further.
            double ground_time = gait_.duty * period_;
            double left = (speed_ - turn * TRACK / 2) * ground_time;
            double right = (speed_ + turn * TRACK / 2) * ground_time;
            // a stride longer than max_stride shrinks both sides by the same factor,
            // so the dog still turns on the same curve
            double scale = std::max({1.0, std::abs(left) / max_stride_,
                                     std::abs(right) / max_stride_});
            left /= scale;
            right /= scale;

            double phase = (since - stand_time_) / period_;
            for (int k = 0; k < 4; k++) {
                Gait g = gait_;
                g.stride = (k % 2 == 0) ? left : right;  // FL and RL are the left legs
                feet[k] = foot_at(g, std::fmod(phase + offset_[k], 1.0));
            }
        }

        std_msgs::msg::Float64MultiArray cmd;  // hip and knee of FL, FR, RL, RR
        for (const Point& foot : feet) {
            double hip, knee;
            ik(LEG, foot, hip, knee);  // always in reach: foot path checked above, poses fixed
            cmd.data.push_back(hip);
            cmd.data.push_back(knee);
        }
        pub_->publish(cmd);
        feet_ = feet;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    Gait gait_;
    std::array<double, 4> offset_;  // phase offset of each leg, order FL, FR, RL, RR
    double period_, stand_time_, max_stride_, heading_gain_;
    double target_speed_;           // m/s, the speed parameter until a cmd_vel arrives
    double target_turn_ = 0.0;      // rad/s, from cmd_vel
    double speed_ = 0.0;            // smoothed speed, m/s
    double turn_ = 0.0;             // smoothed turn rate, rad/s
    double yaw_ = 0.0;              // from the IMU, rad
    double heading_ = 0.0;          // heading to hold, rad
    std::string mode_ = "walk";     // walk, stand or sit
    std::string requested_ = "walk";
    double change_time_ = 0.0;      // s, when the mode last changed
    std::array<Point, 4> from_;     // the feet when the mode last changed
    std::array<Point, 4> feet_;     // the feet sent last
    rclcpp::Time start_;
    bool started_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitNode>());
    rclcpp::shutdown();
    return 0;
}

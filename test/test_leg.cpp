#include <gtest/gtest.h>

#include <cmath>

#include "robot_dog_sim/leg.hpp"

const Leg LEG{0.067, 0.075};  // femur and tibia of the robot dog, in metres

// ik() must accept exactly the points within reach, and fk(ik(p)) must give back p
TEST(Leg, InverseThenForwardGivesBackTheFoot) {
    for (int i = -6; i <= 6; i++) {
        for (int j = 3; j <= 14; j++) {
            Point target{0.01 * i, 0.01 * j};
            double d = std::hypot(target.x, target.z);
            bool reachable = d >= LEG.shin - LEG.thigh && d <= LEG.thigh + LEG.shin;
            double hip, knee;
            ASSERT_EQ(ik(LEG, target, hip, knee), reachable);
            if (!reachable) continue;
            Point p = fk(LEG, hip, knee);
            EXPECT_NEAR(p.x, target.x, 1e-9);
            EXPECT_NEAR(p.z, target.z, 1e-9);
        }
    }
}

// the pose the gait node stands in: every foot 10 cm under its hip
TEST(Leg, StandingPose) {
    double hip, knee;
    ASSERT_TRUE(ik(LEG, {0.0, 0.10}, hip, knee));
    EXPECT_NEAR(knee * 180 / PI, 90.65, 0.01);
    EXPECT_NEAR(hip * 180 / PI, -48.59, 0.01);
}

TEST(Leg, OutOfReach) {
    double hip, knee;
    EXPECT_FALSE(ik(LEG, {0.0, 0.143}, hip, knee));  // farther than 67 + 75 mm
    EXPECT_FALSE(ik(LEG, {0.0, 0.007}, hip, knee));  // closer than 75 - 67 mm
}

TEST(Gait, FootStaysOnTheGroundInStance) {
    Gait g;
    for (int i = 0; i < 50; i++) EXPECT_DOUBLE_EQ(foot_at(g, 0.01 * i).z, g.height);
}

// the swing leaves and meets the ground at stance height and peaks `lift` higher
TEST(Gait, SwingStartsAndEndsOnTheGround) {
    Gait g;
    EXPECT_DOUBLE_EQ(foot_at(g, g.duty).z, g.height);
    EXPECT_NEAR(foot_at(g, (1 + g.duty) / 2).z, g.height - g.lift, 1e-12);
    EXPECT_NEAR(foot_at(g, 1.0 - 1e-9).z, g.height, 1e-9);
    EXPECT_NEAR(foot_at(g, 1.0 - 1e-9).x, foot_at(g, 0.0).x, 1e-9);  // the loop closes
}

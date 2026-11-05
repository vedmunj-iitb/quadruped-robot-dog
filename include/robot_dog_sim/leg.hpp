#pragma once
#include <algorithm>
#include <cmath>

// The leg maths from my leg-ik-gait repo, in metres and with a smoother swing.
// A planar leg with two joints, hip and knee.
// Frame: origin at the hip, x forward, z down. Lengths in metres, angles in radians.
//   hip  = angle of the thigh from straight down, positive forward
//   knee = angle of the shin relative to the thigh, 0 = straight leg

const double PI = 3.14159265358979323846;

struct Leg {
    double thigh;  // hip to knee
    double shin;   // knee to foot
};

struct Point {
    double x, z;
};

// joint angles -> foot position
inline Point fk(const Leg& leg, double hip, double knee) {
    return {leg.thigh * std::sin(hip) + leg.shin * std::sin(hip + knee),
            leg.thigh * std::cos(hip) + leg.shin * std::cos(hip + knee)};
}

// foot position -> joint angles. Returns false if the point is out of reach.
// The knee always bends the same way (knee >= 0, the knee points backward).
inline bool ik(const Leg& leg, Point foot, double& hip, double& knee) {
    double d2 = foot.x * foot.x + foot.z * foot.z;
    // law of cosines on the hip-knee-foot triangle
    double c = (d2 - leg.thigh * leg.thigh - leg.shin * leg.shin) / (2 * leg.thigh * leg.shin);
    if (c < -1.0 - 1e-9 || c > 1.0 + 1e-9) return false;
    knee = std::acos(std::clamp(c, -1.0, 1.0));
    // direction to the foot, minus the angle the bent knee adds
    hip = std::atan2(foot.x, foot.z) -
          std::atan2(leg.shin * std::sin(knee), leg.thigh + leg.shin * std::cos(knee));
    return true;
}

// One foot's path over a gait cycle, phase p from 0 to 1.
// Stance: foot on the ground, moving back by one stride (this pushes the body forward).
// Swing:  foot moves forward, up and down again on cosine curves. They start and end
//         with zero slope, so the foot lifts off and touches down with no vertical speed.
struct Gait {
    double height = 0.10;  // hip height above the foot
    double stride = 0.04;
    double lift = 0.02;
    double duty = 0.5;     // fraction of the cycle the foot is on the ground
};

inline Point foot_at(const Gait& g, double p) {
    if (p < g.duty) {
        double u = p / g.duty;
        return {g.stride / 2 - g.stride * u, g.height};
    }
    double u = (p - g.duty) / (1 - g.duty);
    double s = (1 - std::cos(PI * u)) / 2;  // 0 to 1, starting and ending at zero speed
    return {-g.stride / 2 + g.stride * s, g.height - g.lift * (1 - std::cos(2 * PI * u)) / 2};
}

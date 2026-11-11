// Task-space pose math for the A0509 GUI MoveL panel. No Qt / DRFL dependency, so it is unit-tested
// offline by pose_math_test.cpp.
//
//   Doosan posx (controller, TP, monitoring): [x, y, z, A, B, C]  mm + ZYZ Euler [deg]
//       R = Rz(A) * Ry(B) * Rz(C)
//   GUI input:                                [x, y, z, roll, pitch, yaw]  mm + RPY [deg]
//       R = Rz(yaw) * Ry(pitch) * Rx(roll)   (fixed axes X -> Y -> Z, same as ROS/URDF rpy)
//
// Relative MoveL targets are computed here from the current TCP and sent as an ABSOLUTE base-frame
// amovel, so the controller never has to interpret a relative orientation:
//   base: p' = p + d,      R' = dR * R   (translate along base axes, rotate about base axes at the TCP)
//   tool: p' = p + R * d,  R' = R * dR   (translate/rotate along the current tool axes)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace pm {

using Vec3 = std::array<double, 3>;
using Vec6 = std::array<double, 6>;
using Mat3 = std::array<std::array<double, 3>, 3>;  // row-major: m[row][col]

struct Frame {
    Vec3 p;  // [mm]
    Mat3 R;  // tool orientation in base
};

enum class LinMode { Absolute, RelBase, RelTool };

constexpr double kPi = 3.14159265358979323846;
// |sin B| (ZYZ) or |cos pitch| (RPY) below this is treated as the Euler singularity. The resulting
// orientation error is at most asin(1e-9) ~ 6e-8 deg, while above it the double-precision
// atan2 split is still exact enough (matrices here are built from double products).
constexpr double kSingularEps = 1e-9;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Wrap to (-180, 180].
inline double wrap180(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w <= -180.0) w += 360.0;
    if (w > 180.0) w -= 360.0;
    return w;
}

inline Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) r[i][j] += a[i][k] * b[k][j];
    return r;
}

inline Vec3 mul(const Mat3& a, const Vec3& v) {
    Vec3 r{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) r[i] += a[i][k] * v[k];
    return r;
}

inline Mat3 transpose(const Mat3& a) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i][j] = a[j][i];
    return r;
}

inline Mat3 rotx(double deg) {
    const double c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
    return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}
inline Mat3 roty(double deg) {
    const double c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
    return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
inline Mat3 rotz(double deg) {
    const double c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
    return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

inline Mat3 zyz_to_R(double A, double B, double C) { return mul(mul(rotz(A), roty(B)), rotz(C)); }
inline Mat3 rpy_to_R(double roll, double pitch, double yaw) { return mul(mul(rotz(yaw), roty(pitch)), rotx(roll)); }

// ZYZ [deg] with B in [0, 180], A and C in (-180, 180]. At B = 0 / 180 only A+C / A-C is defined:
// A is then taken from hintA (pass the current A so the posx stays close to what the TP shows).
inline Vec3 R_to_zyz(const Mat3& R, double hintA = 0.0) {
    const double sB = std::hypot(R[0][2], R[1][2]);
    const double B = std::atan2(sB, R[2][2]);
    double A, C;
    if (sB > kSingularEps) {
        A = std::atan2(R[1][2], R[0][2]);
        C = std::atan2(R[2][1], -R[2][0]);
    } else if (R[2][2] > 0) {  // B = 0:   R = Rz(A + C)
        A = deg2rad(wrap180(hintA));
        C = std::atan2(R[1][0], R[0][0]) - A;
    } else {                   // B = 180: R = Rz(A - C) * Ry(180)
        A = deg2rad(wrap180(hintA));
        C = A - std::atan2(-R[1][0], -R[0][0]);
    }
    return {wrap180(rad2deg(A)), rad2deg(B), wrap180(rad2deg(C))};
}

// RPY [deg] with pitch in [-90, 90], roll and yaw in (-180, 180]. At pitch = +-90 only roll -+ yaw
// is defined: yaw is then taken from hintYaw.
inline Vec3 R_to_rpy(const Mat3& R, double hintYaw = 0.0) {
    const double cp = std::hypot(R[0][0], R[1][0]);
    const double pitch = std::atan2(-R[2][0], cp);
    double roll, yaw;
    if (cp > kSingularEps) {
        roll = std::atan2(R[2][1], R[2][2]);
        yaw = std::atan2(R[1][0], R[0][0]);
    } else if (R[2][0] < 0) {  // pitch = +90: roll - yaw = atan2(R01, R11)
        yaw = deg2rad(wrap180(hintYaw));
        roll = std::atan2(R[0][1], R[1][1]) + yaw;
    } else {                   // pitch = -90: roll + yaw = atan2(-R01, R11)
        yaw = deg2rad(wrap180(hintYaw));
        roll = std::atan2(-R[0][1], R[1][1]) - yaw;
    }
    return {wrap180(rad2deg(roll)), rad2deg(pitch), wrap180(rad2deg(yaw))};
}

// Rotation angle [deg] between two orientations (angle of a^T * b), robust near 0 and 180.
inline double rot_angle_deg(const Mat3& a, const Mat3& b) {
    const Mat3 d = mul(transpose(a), b);
    const double tr = d[0][0] + d[1][1] + d[2][2];
    const double s = 0.5 * std::sqrt((d[2][1] - d[1][2]) * (d[2][1] - d[1][2]) +
                                     (d[0][2] - d[2][0]) * (d[0][2] - d[2][0]) +
                                     (d[1][0] - d[0][1]) * (d[1][0] - d[0][1]));
    return rad2deg(std::atan2(s, 0.5 * (tr - 1.0)));
}

inline double dist(const Vec3& a, const Vec3& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// ---- conversions between the 6-value forms and Frame ----
template <typename T>
inline Frame frame_from_posx(const std::array<T, 6>& x) {
    return {{double(x[0]), double(x[1]), double(x[2])}, zyz_to_R(double(x[3]), double(x[4]), double(x[5]))};
}

inline Frame frame_from_xyzrpy(const Vec6& v) {
    return {{v[0], v[1], v[2]}, rpy_to_R(v[3], v[4], v[5])};
}

inline Vec6 posx_from_frame(const Frame& f, double hintA = 0.0) {
    const Vec3 e = R_to_zyz(f.R, hintA);
    return {f.p[0], f.p[1], f.p[2], e[0], e[1], e[2]};
}

inline Vec6 xyzrpy_from_frame(const Frame& f, double hintYaw = 0.0) {
    const Vec3 e = R_to_rpy(f.R, hintYaw);
    return {f.p[0], f.p[1], f.p[2], e[0], e[1], e[2]};
}

// ---- MoveL target ----
// input: Absolute -> [x, y, z, roll, pitch, yaw] target in base.
//        RelBase / RelTool -> [dx, dy, dz, droll, dpitch, dyaw] increment in base / tool axes.
inline Frame movel_target(const Frame& cur, LinMode mode, const Vec6& input) {
    if (mode == LinMode::Absolute) return frame_from_xyzrpy(input);
    const Vec3 d{input[0], input[1], input[2]};
    const Mat3 dR = rpy_to_R(input[3], input[4], input[5]);
    Frame t;
    if (mode == LinMode::RelBase) {
        t.p = {cur.p[0] + d[0], cur.p[1] + d[1], cur.p[2] + d[2]};
        t.R = mul(dR, cur.R);
    } else {
        const Vec3 dp = mul(cur.R, d);
        t.p = {cur.p[0] + dp[0], cur.p[1] + dp[1], cur.p[2] + dp[2]};
        t.R = mul(cur.R, dR);
    }
    return t;
}

// Time estimate [s] for a trapezoidal profile (used only for the GUI watchdog).
inline double trapezoid_time(double distance, double vel, double acc) {
    if (distance <= 0 || vel <= 0 || acc <= 0) return 0;
    if (distance >= vel * vel / acc) return distance / vel + vel / acc;
    return 2.0 * std::sqrt(distance / acc);
}

}  // namespace pm

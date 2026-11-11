// Offline unit test for pose_math.h (RPY <-> Doosan ZYZ, MoveL target). No robot / network needed.
// Built by a0509_gui/CMakeLists.txt; run: a0509_gui/build/pose_math_test
#include "pose_math.h"

#include <cstdio>
#include <random>
#include <string>

namespace {
int g_checks = 0, g_fail = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_fail;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

void near(double a, double b, double tol, const std::string& what) {
    check(std::fabs(a - b) <= tol, what + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

double mat_err(const pm::Mat3& a, const pm::Mat3& b) {
    double e = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) e = std::max(e, std::fabs(a[i][j] - b[i][j]));
    return e;
}

void mat_near(const pm::Mat3& a, const pm::Mat3& b, double tol, const std::string& what) {
    check(mat_err(a, b) <= tol, what + " (max err " + std::to_string(mat_err(a, b)) + ")");
}

void test_basic_rotations() {
    // Rz(90) maps x -> y, Ry(90) maps z -> x, Rx(90) maps y -> z (right-handed)
    const pm::Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    auto v = pm::mul(pm::rotz(90), x);
    near(v[1], 1, 1e-12, "Rz(90) x -> y");
    v = pm::mul(pm::roty(90), z);
    near(v[0], 1, 1e-12, "Ry(90) z -> x");
    v = pm::mul(pm::rotx(90), y);
    near(v[2], 1, 1e-12, "Rx(90) y -> z");
}

void test_known_values() {
    // Tool pointing straight down (typical A0509 home TCP): ZYZ (0, 180, 0) == RPY (180, 0, 180)
    const pm::Mat3 down = pm::zyz_to_R(0, 180, 0);
    mat_near(down, pm::rpy_to_R(180, 0, 180), 1e-12, "ZYZ(0,180,0) == RPY(180,0,180)");
    const pm::Vec3 rpy = pm::R_to_rpy(down);
    near(std::fabs(rpy[0]), 180, 1e-9, "down: |roll| = 180");
    near(rpy[1], 0, 1e-9, "down: pitch = 0");
    near(std::fabs(rpy[2]), 180, 1e-9, "down: |yaw| = 180");

    // ZYZ (90, 90, 0): tool z -> base +y... R = Rz(90) Ry(90): z axis = Rz(90)*(1,0,0) = (0,1,0)
    const pm::Mat3 R = pm::zyz_to_R(90, 90, 0);
    near(R[1][2], 1, 1e-12, "ZYZ(90,90,0) tool z = base +y");

    // Pure yaw is the same rotation in both conventions: ZYZ (30, 0, 0) == RPY (0, 0, 30)
    mat_near(pm::zyz_to_R(30, 0, 0), pm::rpy_to_R(0, 0, 30), 1e-12, "ZYZ(30,0,0) == RPY(0,0,30)");

    // Rotation about x as ZYZ: Rx(t) = Rz(-90) Ry(t) Rz(90)
    mat_near(pm::zyz_to_R(-90, 25, 90), pm::rpy_to_R(25, 0, 0), 1e-12, "ZYZ(-90,25,90) == Rx(25)");
}

void test_round_trips() {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> ang(-180.0, 180.0), half(0.0, 180.0), pitch(-90.0, 90.0);
    for (int i = 0; i < 20000; ++i) {
        // ZYZ -> R -> ZYZ -> R
        const double A = ang(rng), B = half(rng), C = ang(rng);
        const pm::Mat3 R = pm::zyz_to_R(A, B, C);
        const pm::Vec3 e = pm::R_to_zyz(R, A);
        mat_near(pm::zyz_to_R(e[0], e[1], e[2]), R, 1e-9, "ZYZ round trip " + std::to_string(i));
        check(e[1] >= 0 && e[1] <= 180 && e[0] > -180 && e[0] <= 180 && e[2] > -180 && e[2] <= 180,
              "ZYZ canonical range " + std::to_string(i));
        if (B > 1e-3 && B < 180 - 1e-3) {  // unique away from the singularity
            near(pm::wrap180(e[0] - A), 0, 1e-6, "ZYZ A unique");
            near(e[1], B, 1e-6, "ZYZ B unique");
            near(pm::wrap180(e[2] - C), 0, 1e-6, "ZYZ C unique");
        }
        // RPY -> R -> RPY -> R
        const double r = ang(rng), p = pitch(rng), y = ang(rng);
        const pm::Mat3 Q = pm::rpy_to_R(r, p, y);
        const pm::Vec3 q = pm::R_to_rpy(Q, y);
        mat_near(pm::rpy_to_R(q[0], q[1], q[2]), Q, 1e-9, "RPY round trip " + std::to_string(i));
        // Cross conversion: ZYZ -> RPY -> ZYZ keeps the orientation
        const pm::Vec3 viaRpy = pm::R_to_rpy(R);
        const pm::Vec3 back = pm::R_to_zyz(pm::rpy_to_R(viaRpy[0], viaRpy[1], viaRpy[2]), A);
        mat_near(pm::zyz_to_R(back[0], back[1], back[2]), R, 1e-9, "ZYZ->RPY->ZYZ " + std::to_string(i));
    }
}

void test_singularities() {
    for (double hint : {-170.0, -45.0, 0.0, 33.3, 179.0}) {
        for (double sum : {-120.0, 0.0, 77.0}) {
            // B = 0: only A + C defined
            const pm::Mat3 R0 = pm::zyz_to_R(10, 0, sum - 10);
            pm::Vec3 e = pm::R_to_zyz(R0, hint);
            near(e[0], hint, 1e-9, "B=0 keeps hint A");
            near(e[1], 0, 1e-9, "B=0");
            mat_near(pm::zyz_to_R(e[0], e[1], e[2]), R0, 1e-12, "B=0 orientation");
            // B = 180: only A - C defined (tool pointing down, the usual working pose)
            const pm::Mat3 R1 = pm::zyz_to_R(sum, 180, 20);
            e = pm::R_to_zyz(R1, hint);
            near(e[0], hint, 1e-9, "B=180 keeps hint A");
            near(e[1], 180, 1e-9, "B=180");
            mat_near(pm::zyz_to_R(e[0], e[1], e[2]), R1, 1e-12, "B=180 orientation");
            // pitch = +-90: only roll -+ yaw defined
            for (double p : {90.0, -90.0}) {
                const pm::Mat3 Q = pm::rpy_to_R(sum, p, 15);
                const pm::Vec3 q = pm::R_to_rpy(Q, hint);
                near(q[2], hint, 1e-9, "pitch=+-90 keeps hint yaw");
                near(q[1], p, 1e-9, "pitch=+-90");
                mat_near(pm::rpy_to_R(q[0], q[1], q[2]), Q, 1e-12, "pitch=+-90 orientation");
            }
        }
    }
    // Monitoring sends float: B = 180.0f exactly -> must not produce a random A/C split
    const std::array<float, 6> posx{367.5f, 6.5f, 422.0f, 12.0f, 180.0f, -40.0f};
    const pm::Frame f = pm::frame_from_posx(posx);
    const pm::Vec6 x = pm::posx_from_frame(f, posx[3]);
    near(x[3], 12.0, 1e-6, "float B=180 keeps A");
    near(x[5], -40.0, 1e-6, "float B=180 keeps C");
}

void test_movel_targets() {
    // Current TCP: tool pointing down at (400, 0, 300)
    const pm::Frame cur = pm::frame_from_posx(std::array<double, 6>{400, 0, 300, 0, 180, 0});

    // Absolute with the current pose loaded as XYZ/RPY -> same pose
    const pm::Vec6 loaded = pm::xyzrpy_from_frame(cur);
    pm::Frame t = pm::movel_target(cur, pm::LinMode::Absolute, loaded);
    near(pm::dist(t.p, cur.p), 0, 1e-9, "abs reload: position");
    near(pm::rot_angle_deg(t.R, cur.R), 0, 1e-6, "abs reload: orientation");

    // Relative base: Z +50 -> up 50 mm, orientation unchanged
    t = pm::movel_target(cur, pm::LinMode::RelBase, {0, 0, 50, 0, 0, 0});
    near(t.p[2], 350, 1e-9, "rel base Z+50");
    near(pm::rot_angle_deg(t.R, cur.R), 0, 1e-9, "rel base Z+50 keeps orientation");

    // Relative tool: Z +10 with tool pointing down -> base Z -10 (approach)
    t = pm::movel_target(cur, pm::LinMode::RelTool, {0, 0, 10, 0, 0, 0});
    near(t.p[0], 400, 1e-9, "rel tool Z+10: x");
    near(t.p[2], 290, 1e-9, "rel tool Z+10: base z -10");

    // Relative tool: X +10 with ZYZ(0,180,0): tool x = base -x
    t = pm::movel_target(cur, pm::LinMode::RelTool, {10, 0, 0, 0, 0, 0});
    near(t.p[0], 390, 1e-9, "rel tool X+10 -> base x -10");

    // Relative base: yaw +90 keeps position, tool x axis turns from -x to -y
    t = pm::movel_target(cur, pm::LinMode::RelBase, {0, 0, 0, 0, 0, 90});
    near(pm::dist(t.p, cur.p), 0, 1e-9, "rel base yaw keeps position");
    near(t.R[1][0], -1, 1e-12, "rel base yaw+90: tool x -> base -y");
    near(pm::rot_angle_deg(t.R, cur.R), 90, 1e-9, "rel base yaw+90 angle");

    // Relative tool: rotate about tool z (spin the gripper) keeps tool z axis
    t = pm::movel_target(cur, pm::LinMode::RelTool, {0, 0, 0, 0, 0, 30});
    near(t.R[2][2], -1, 1e-12, "rel tool yaw keeps tool z down");
    near(pm::rot_angle_deg(t.R, cur.R), 30, 1e-9, "rel tool yaw 30 angle");

    // Relative tool roll 10: tool z tilts but TCP point stays
    t = pm::movel_target(cur, pm::LinMode::RelTool, {0, 0, 0, 10, 0, 0});
    near(pm::dist(t.p, cur.p), 0, 1e-9, "rel tool roll keeps TCP point");
    near(pm::rot_angle_deg(t.R, cur.R), 10, 1e-9, "rel tool roll angle");

    // Base vs tool differ for a tilted tool
    const pm::Frame tilted = pm::frame_from_posx(std::array<double, 6>{300, 100, 250, 30, 135, -60});
    const pm::Frame tb = pm::movel_target(tilted, pm::LinMode::RelBase, {0, 0, 20, 0, 0, 0});
    const pm::Frame tt = pm::movel_target(tilted, pm::LinMode::RelTool, {0, 0, 20, 0, 0, 0});
    near(pm::dist(tb.p, tilted.p), 20, 1e-9, "tilted base step length");
    near(pm::dist(tt.p, tilted.p), 20, 1e-9, "tilted tool step length");
    check(pm::dist(tb.p, tt.p) > 1, "tilted: base and tool steps differ");
    // tool step goes along the tool z axis (third column of R)
    near(tt.p[0] - tilted.p[0], 20 * tilted.R[0][2], 1e-9, "tool step along tool z (x)");
    near(tt.p[2] - tilted.p[2], 20 * tilted.R[2][2], 1e-9, "tool step along tool z (z)");

    // posx produced for the controller reproduces the target orientation
    const pm::Vec6 px = pm::posx_from_frame(tt, 30);
    mat_near(pm::zyz_to_R(px[3], px[4], px[5]), tt.R, 1e-9, "posx_from_frame orientation");
}

void test_helpers() {
    near(pm::wrap180(190), -170, 1e-12, "wrap 190");
    near(pm::wrap180(-180), 180, 1e-12, "wrap -180");
    near(pm::wrap180(540), 180, 1e-12, "wrap 540");
    near(pm::rot_angle_deg(pm::rotz(0), pm::rotz(179.5)), 179.5, 1e-9, "angle near 180");
    near(pm::rot_angle_deg(pm::rotx(0), pm::rotx(1e-5)), 1e-5, 1e-9, "angle near 0");
    near(pm::trapezoid_time(100, 50, 100), 2.5, 1e-12, "trapezoid");
    near(pm::trapezoid_time(10, 50, 100), 2 * std::sqrt(0.1), 1e-12, "triangle");
    near(pm::trapezoid_time(0, 50, 100), 0, 0, "zero distance");
}
}  // namespace

int main() {
    test_basic_rotations();
    test_known_values();
    test_round_trips();
    test_singularities();
    test_movel_targets();
    test_helpers();
    std::printf("pose_math_test: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}

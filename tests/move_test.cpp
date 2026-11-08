// A0509 real-robot minimal motion test with software stop layers.
//   - Rotates J6 by +STEP_DEG and back at very low speed.
//   - Any key / Ctrl+C during motion -> MoveStop(QUICK)
//   - Watchdog timeout, E-stop/protective-stop state, alarm, or disconnect -> stop & exit
// Hardware E-stop on the teach pendant is still the primary safety device.
#ifndef DRCF_VERSION
#define DRCF_VERSION 2  // controller GF02.x -> 2, GF03.x -> 3
#endif
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
using namespace DRAFramework;
using Clock = std::chrono::steady_clock;

constexpr float STEP_DEG = 5.0f;   // J6 rotation
constexpr float VEL = 3.0f;        // deg/s
constexpr float ACC = 3.0f;        // deg/s^2
constexpr float OP_SPEED = 100.0f;  // operation speed override [%]

static CDRFLEx robot;
static std::atomic<bool> g_access{false};
static std::atomic<bool> g_standby{false};
static std::atomic<bool> g_abort{false};
static std::atomic<bool> g_sigint{false};
static termios g_tty_saved;
static bool g_tty_raw = false;

static void tty_raw(bool on) {
    if (on && !g_tty_raw && tcgetattr(STDIN_FILENO, &g_tty_saved) == 0) {
        termios t = g_tty_saved;
        t.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
        g_tty_raw = true;
    } else if (!on && g_tty_raw) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_saved);
        g_tty_raw = false;
    }
}

static bool key_pressed() {
    pollfd p{STDIN_FILENO, POLLIN, 0};
    if (poll(&p, 1, 0) > 0) { char c; return read(STDIN_FILENO, &c, 1) > 0; }
    return false;
}

static void shutdown_and_exit(int code) {
    robot.MoveStop(STOP_TYPE_QUICK);
    robot.set_robot_mode(ROBOT_MODE_MANUAL);
    robot.close_connection();
    tty_raw(false);
    std::exit(code);
}

// Runs one async joint move; returns false if it had to be stopped.
static bool guarded_movej(float target[NUM_JOINT], float max_sec) {
    if (!robot.amovej(target, VEL, ACC)) {
        std::cerr << "[ERR] amovej rejected" << std::endl;
        return false;
    }
    auto t0 = Clock::now();
    auto last_log = t0;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int cm;
    while ((cm = robot.check_motion()) != 0) {
        double el = std::chrono::duration<double>(Clock::now() - t0).count();
        if (Clock::now() - last_log > std::chrono::milliseconds(500)) {
            last_log = Clock::now();
            std::cout << "  t=" << el << "s check_motion=" << cm
                      << " J6=" << robot.get_current_posj()->_fPosition[5] << std::endl;
        }
        const char* why = nullptr;
        if (key_pressed())     why = "key pressed";
        else if (g_sigint)     why = "Ctrl+C";
        else if (g_abort)      why = "robot fault / disconnect";
        else if (el > max_sec) why = "watchdog timeout";
        if (why) {
            robot.MoveStop(STOP_TYPE_QUICK);
            std::cerr << "\n[STOP] " << why << " -> MoveStop(QUICK)" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "  reached: J6=" << robot.get_current_posj()->_fPosition[5] << " (target "
              << target[5] << ")" << std::endl;
    return true;
}

int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "192.168.137.101";

    std::signal(SIGINT, [](int) { g_sigint = true; });
    robot.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL a) {
        std::cout << "[access] " << a << std::endl;
        if (a == MONITORING_ACCESS_CONTROL_GRANT) g_access = true;
        if (a == MONITORING_ACCESS_CONTROL_LOSS) { g_access = false; g_abort = true; }
    });
    robot.set_on_monitoring_state([](const ROBOT_STATE s) {
        std::cout << "[state] " << s << std::endl;
        g_standby = (s == STATE_STANDBY);
        if (s == STATE_EMERGENCY_STOP || s == STATE_SAFE_STOP || s == STATE_SAFE_STOP2)
            g_abort = true;
    });
    robot.set_on_log_alarm([](LPLOG_ALARM a) {
        std::cerr << "[alarm] level=" << (int)a->_iLevel << " group=" << (int)a->_iGroup
                  << " index=" << a->_iIndex << " " << a->_szParam[0] << std::endl;
    });
    robot.set_on_disconnected([]() {
        std::cerr << "[ERR] disconnected from controller" << std::endl;
        g_abort = true;
    });

    if (!robot.open_connection(ip)) {
        std::cerr << "[ERR] open_connection failed: " << ip << std::endl;
        return 1;
    }
    robot.setup_monitoring_version(1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    g_standby = (robot.get_robot_state() == STATE_STANDBY);

    for (int i = 0; i < 15 && !(g_access && g_standby) && !g_sigint; ++i) {
        if (!g_access)       robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        else if (!g_standby) robot.set_robot_control(CONTROL_SERVO_ON);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!(g_access && g_standby)) {
        std::cerr << "[ERR] control access/servo-on failed (state="
                  << robot.get_robot_state() << ")" << std::endl;
        shutdown_and_exit(1);
    }
    g_abort = false;  // ignore transient states seen before servo-on

    if (!robot.set_robot_system(ROBOT_SYSTEM_REAL) || !robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
        std::cerr << "[ERR] set_robot_system/mode failed" << std::endl;
        shutdown_and_exit(1);
    }
    std::cout << "change_operation_speed(" << OP_SPEED << "%) -> "
              << robot.change_operation_speed(OP_SPEED) << std::endl;

    LPROBOT_POSE cur = robot.get_current_posj();
    float home[NUM_JOINT], target[NUM_JOINT];
    for (int i = 0; i < NUM_JOINT; ++i) home[i] = target[i] = cur->_fPosition[i];
    target[5] += STEP_DEG;
    std::cout << "start J6=" << home[5] << "  target J6=" << target[5] << std::endl;
    // generous upper bound: full-speed move time / operation speed + margin
    float max_sec = (STEP_DEG / VEL + 2 * VEL / ACC) * (100.0f / OP_SPEED) + 10.0f;

    std::cout << "\nJ6 will rotate +" << STEP_DEG << " deg and back (vel " << VEL
              << " deg/s, op speed " << OP_SPEED << "%).\n"
              << "During motion: press ANY KEY or Ctrl+C to stop.\n"
              << "Press Enter to start (Ctrl+C to quit)..." << std::flush;
    std::cin.get();
    if (g_sigint) shutdown_and_exit(0);

    tty_raw(true);
    bool ok = guarded_movej(target, max_sec) && guarded_movej(home, max_sec);
    tty_raw(false);
    std::cout << (ok ? "done" : "aborted") << std::endl;
    shutdown_and_exit(ok ? 0 : 2);
}

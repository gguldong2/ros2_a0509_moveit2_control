// A0509 per-joint movej test: J1..J6 each rotate +STEP_DEG[i] and back, one at a time.
//   - Before each joint: Enter = run, s = skip, q = quit
//   - Any key / Ctrl+C during motion -> MoveStop(QUICK)
//   - Watchdog timeout, E-stop/protective-stop state, access loss, or disconnect -> stop & exit
// Hardware E-stop on the teach pendant is still the primary safety device.
#ifndef DRCF_VERSION
#define DRCF_VERSION 2  // controller GF02.x -> 2, GF03.x -> 3
#endif
#include "DRFLEx.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
using namespace DRAFramework;
using Clock = std::chrono::steady_clock;

// Per-joint step [deg]; sign selects direction.
constexpr float STEP_DEG[NUM_JOINT] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
// Conservative software limits [deg] (A0509 spec: J1,J2,J4,J6 +-360, J3 +-150, J5 +-135)
constexpr float LIMIT_DEG[NUM_JOINT] = {350.0f, 350.0f, 145.0f, 350.0f, 130.0f, 350.0f};
constexpr float VEL = 2.0f;         // deg/s
constexpr float ACC = 2.0f;         // deg/s^2
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
        tcflush(STDIN_FILENO, TCIFLUSH);
        g_tty_raw = true;
    } else if (!on && g_tty_raw) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_saved);
        tcflush(STDIN_FILENO, TCIFLUSH);
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

static void print_posj(const char* label) {
    LPROBOT_POSE p = robot.get_current_posj();
    std::cout << label;
    for (int i = 0; i < NUM_JOINT; ++i) std::cout << " " << p->_fPosition[i];
    std::cout << std::endl;
}

// Runs one async joint move; returns false if it had to be stopped.
static bool guarded_movej(float target[NUM_JOINT], int joint) {
    float dist = 0;
    LPROBOT_POSE cur = robot.get_current_posj();
    for (int i = 0; i < NUM_JOINT; ++i) dist = std::max(dist, std::fabs(target[i] - cur->_fPosition[i]));
    float max_sec = (dist / VEL + 2 * VEL / ACC) * (100.0f / OP_SPEED) + 10.0f;

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
            std::cout << "  t=" << el << "s J" << joint + 1 << "="
                      << robot.get_current_posj()->_fPosition[joint] << std::endl;
        }
        const char* why = nullptr;
        if (key_pressed())     why = "key pressed";
        else if (g_sigint)     why = "Ctrl+C";
        else if (g_abort)      why = "robot fault / access loss / disconnect";
        else if (el > max_sec) why = "watchdog timeout";
        if (why) {
            robot.MoveStop(STOP_TYPE_QUICK);
            std::cerr << "\n[STOP] " << why << " -> MoveStop(QUICK)" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "  reached J" << joint + 1 << "=" << robot.get_current_posj()->_fPosition[joint]
              << " (target " << target[joint] << ")" << std::endl;
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
        g_standby = (s == STATE_STANDBY);
        if (s == STATE_EMERGENCY_STOP || s == STATE_SAFE_STOP || s == STATE_SAFE_STOP2) {
            std::cerr << "[state] " << s << " (E-stop / protective stop)" << std::endl;
            g_abort = true;
        }
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
    robot.change_operation_speed(OP_SPEED);

    float home[NUM_JOINT];
    LPROBOT_POSE cur = robot.get_current_posj();
    for (int i = 0; i < NUM_JOINT; ++i) home[i] = cur->_fPosition[i];
    print_posj("home posj [deg]:");
    std::cout << "vel " << VEL << " deg/s, acc " << ACC << " deg/s^2\n"
              << "During motion: press ANY KEY or Ctrl+C to stop.\n";

    for (int j = 0; j < NUM_JOINT; ++j) {
        float target[NUM_JOINT];
        std::copy(home, home + NUM_JOINT, target);
        target[j] += STEP_DEG[j];
        if (std::fabs(target[j]) > LIMIT_DEG[j]) {
            std::cout << "\nJ" << j + 1 << ": target " << target[j] << " exceeds limit +-"
                      << LIMIT_DEG[j] << " -> skipped" << std::endl;
            continue;
        }

        std::cout << "\nJ" << j + 1 << ": " << home[j] << " -> " << target[j] << " -> " << home[j]
                  << " deg.  [Enter]=run  s=skip  q=quit : " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line) || g_sigint || line == "q") shutdown_and_exit(0);
        if (line == "s") continue;

        tty_raw(true);
        bool ok = guarded_movej(target, j) && guarded_movej(home, j);
        tty_raw(false);
        if (!ok) {
            print_posj("stopped at posj [deg]:");
            shutdown_and_exit(2);
        }
    }

    print_posj("\nfinal posj [deg]:");
    std::cout << "all joints done" << std::endl;
    shutdown_and_exit(0);
}

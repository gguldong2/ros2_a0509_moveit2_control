// A0509 read-only connection test: no control request, no servo-on, no motion.
#ifndef DRCF_VERSION
#define DRCF_VERSION 2  // controller GF02.x -> 2, GF03.x -> 3
#endif
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
using namespace DRAFramework;

static std::mutex g_mtx;
static float g_posx[NUM_TASK] = {0};
static std::atomic<bool> g_got_data{false};

static const char* state_str(ROBOT_STATE s) {
    static const char* n[] = {"INITIALIZING", "STANDBY", "MOVING", "SAFE_OFF", "TEACHING",
                              "SAFE_STOP", "EMERGENCY_STOP", "HOMMING", "RECOVERY",
                              "SAFE_STOP2", "SAFE_OFF2"};
    return (s >= 0 && s <= STATE_SAFE_OFF2) ? n[s] : "OTHER";
}

int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "192.168.137.101";
    CDRFLEx robot;
    robot.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX d) {
        std::lock_guard<std::mutex> lk(g_mtx);
        for (int i = 0; i < NUM_TASK; ++i) g_posx[i] = d->_tCtrl._tTask._fActualPos[0][i];
        g_got_data = true;
    });

    if (!robot.open_connection(ip)) {
        std::cerr << "[ERR] open_connection failed: " << ip << std::endl;
        return 1;
    }
    robot.setup_monitoring_version(1);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    SYSTEM_VERSION v = {'\0'};
    robot.get_system_version(&v);
    ROBOT_STATE st = robot.get_robot_state();
    std::cout << "Library version : " << robot.get_library_version() << "\n"
              << "Controller      : " << v._szController << "\n"
              << "Robot model     : " << v._szRobotModel << "\n"
              << "Robot serial    : " << v._szRobotSerial << "\n"
              << "Robot state     : " << st << " (" << state_str(st) << ")\n"
              << "Robot mode      : " << robot.get_robot_mode() << " (0=MANUAL, 1=AUTONOMOUS)\n"
              << "Robot system    : " << robot.get_robot_system() << " (0=REAL, 1=VIRTUAL)\n";

    LPROBOT_POSE j = robot.get_current_posj();
    std::cout << "posj [deg]      :";
    for (int i = 0; i < NUM_JOINT; ++i) std::cout << " " << j->_fPosition[i];
    std::cout << "\nposx [mm,deg]   :";
    if (g_got_data) {
        std::lock_guard<std::mutex> lk(g_mtx);
        for (int i = 0; i < NUM_TASK; ++i) std::cout << " " << g_posx[i];
    } else {
        std::cout << " (no monitoring data yet)";
    }
    std::cout << std::endl;
    robot.close_connection();
    return 0;
}

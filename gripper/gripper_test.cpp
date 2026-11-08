// Robotiq 2F-85 test over the A0509 flange RS-485 (DRFL flange_serial_*), Modbus RTU.
// The arm never moves: no servo ON, no motion commands. Only control access is requested.
//   Build: ./build.sh gripper/gripper_test.cpp 2   (저장소 루트에서)
//   Run:   ./gripper_test [--ip 192.168.137.101] [--force]
// Robotiq defaults: slave 0x09, 115200 8N1. Output regs 0x03E8.., input regs 0x07D0..
#ifndef DRCF_VERSION
#define DRCF_VERSION 2
#endif
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
using namespace DRAFramework;
using namespace std::chrono_literals;

constexpr int PORT = 1;
constexpr uint8_t SLAVE = 0x09;

static CDRFLEx robot;
static std::atomic<bool> g_access{false}, g_denied{false};

static uint16_t crc16(const std::vector<uint8_t>& d) {
    uint16_t crc = 0xFFFF;
    for (uint8_t b : d) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}
static void append_crc(std::vector<uint8_t>& d) {
    uint16_t c = crc16(d);
    d.push_back(c & 0xFF);
    d.push_back(c >> 8);
}
static std::string hex(const uint8_t* p, int n) {
    std::ostringstream s;
    char buf[4];
    for (int i = 0; i < n; ++i) { std::snprintf(buf, sizeof buf, "%02X ", p[i]); s << buf; }
    return s.str();
}

// Send one Modbus frame and read the reply (accumulates partial reads until expected length).
static bool transact(std::vector<uint8_t> req, std::vector<uint8_t>& rsp, size_t expect) {
    append_crc(req);
    std::printf("  TX: %s\n", hex(req.data(), req.size()).c_str());
    if (!robot.flange_serial_write(static_cast<int>(req.size()), reinterpret_cast<char*>(req.data()), PORT)) {
        std::printf("  flange_serial_write 실패\n");
        return false;
    }
    rsp.clear();
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (rsp.size() < expect && std::chrono::steady_clock::now() < deadline) {
        LPFLANGE_SER_RXD_INFO_EX r = robot.flange_serial_read(0.5f, PORT);
        if (r && r->_iSize > 0) rsp.insert(rsp.end(), r->_cRxd, r->_cRxd + r->_iSize);
    }
    std::printf("  RX: %s\n", rsp.empty() ? "(응답 없음)" : hex(rsp.data(), rsp.size()).c_str());
    if (rsp.size() < expect) return false;
    std::vector<uint8_t> body(rsp.begin(), rsp.begin() + expect - 2);
    uint16_t c = crc16(body);
    if (rsp[expect - 2] != (c & 0xFF) || rsp[expect - 1] != (c >> 8)) {
        std::printf("  CRC 불일치\n");
        return false;
    }
    if (rsp[0] != SLAVE || (rsp[1] & 0x80)) {
        std::printf("  Modbus 예외 응답\n");
        return false;
    }
    return true;
}

// Write action request + position/speed/force (FC16, 3 registers from 0x03E8).
static bool write_cmd(uint8_t action, uint8_t pos, uint8_t speed, uint8_t force) {
    std::vector<uint8_t> req = {SLAVE, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                                action, 0x00, 0x00, pos, speed, force};
    std::vector<uint8_t> rsp;
    return transact(req, rsp, 8);
}

// Read gripper status (FC03, 3 registers from 0x07D0).
static bool read_status(bool verbose = true, uint8_t* sta_out = nullptr, uint8_t* pos_out = nullptr) {
    std::vector<uint8_t> rsp;
    if (!transact({SLAVE, 0x03, 0x07, 0xD0, 0x00, 0x03}, rsp, 11)) return false;
    const uint8_t st = rsp[3], fault = rsp[5], req_pos = rsp[6], pos = rsp[7], cur = rsp[8];
    const int gACT = st & 1, gGTO = (st >> 3) & 1, gSTA = (st >> 4) & 3, gOBJ = (st >> 6) & 3;
    if (verbose) {
        static const char* sta[] = {"리셋/미활성", "활성화 중", "(미사용)", "활성화 완료"};
        static const char* obj[] = {"이동 중", "열다가 물체 감지", "닫다가 물체 감지", "목표 도달(물체 없음)"};
        std::printf("  gACT=%d gGTO=%d gSTA=%d(%s) gOBJ=%d(%s)\n", gACT, gGTO, gSTA, sta[gSTA], gOBJ, obj[gOBJ]);
        std::printf("  fault=0x%02X  요청위치=%u  현재위치=%u (0=열림, 255=닫힘)  전류=%u\n", fault, req_pos, pos, cur);
    }
    if (sta_out) *sta_out = static_cast<uint8_t>(gSTA);
    if (pos_out) *pos_out = pos;
    return true;
}

static int ask_int(const std::string& prompt, int def, int lo, int hi) {
    std::printf("%s [%d]: ", prompt.c_str(), def);
    std::fflush(stdout);
    std::string line;
    if (!std::getline(std::cin, line) || line.empty()) return def;
    try {
        int v = std::stoi(line);
        if (v >= lo && v <= hi) return v;
    } catch (...) {}
    std::printf("  범위 밖 -> 기본값 %d 사용\n", def);
    return def;
}

int main(int argc, char** argv) {
    std::string ip = "192.168.137.101";
    bool force = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (a == "--force") force = true;
    }

    robot.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL a) {
        if (a == MONITORING_ACCESS_CONTROL_GRANT) g_access = true;
        if (a == MONITORING_ACCESS_CONTROL_LOSS) g_access = false;
        if (a == MONITORING_ACCESS_CONTROL_DENY) g_denied = true;
    });
    if (!robot.open_connection(ip)) { std::printf("연결 실패: %s\n", ip.c_str()); return 1; }
    robot.setup_monitoring_version(1);
    std::this_thread::sleep_for(1s);

    robot.ManageAccessControl(force ? MANAGE_ACCESS_CONTROL_FORCE_REQUEST : MANAGE_ACCESS_CONTROL_REQUEST);
    std::printf(force ? "제어권 강제 요청...\n" : "제어권 요청... TP 팝업에서 승인하세요.\n");
    for (int i = 0; i < 150 && !g_access && !g_denied; ++i) std::this_thread::sleep_for(100ms);
    if (!g_access) { std::printf("제어권 획득 실패 (--force 옵션 고려)\n"); robot.close_connection(); return 1; }
    std::printf("제어권 획득. (로봇 팔은 서보 ON/이동하지 않습니다)\n");

    if (!robot.flange_serial_open(PORT, 115200, BYTE_SIZE_EIGHTBITS, PARITY_CHECK_NONE, STOPBITS_ONE)) {
        std::printf("flange_serial_open 실패: 이 제어기/플랜지에서 플랜지 시리얼을 지원하지 않을 수 있음\n");
        robot.close_connection();
        return 1;
    }
    std::printf("플랜지 시리얼 열림 (115200 8N1)\n\n[1] 통신 확인: 상태 읽기\n");
    if (!read_status()) {
        std::printf("\n그리퍼 응답 없음. 확인할 것: TP 플랜지 24V 전원 설정, 배선(RS-485 A/B), 그리퍼 LED\n");
        robot.flange_serial_close(PORT);
        robot.close_connection();
        return 1;
    }

    std::printf("\n명령: a=활성화  o=열기  c=닫기  p=위치 지정  s=상태  r=리셋  q=종료\n"
                "※ 활성화(a) 시 그리퍼가 스스로 완전히 열고 닫으며 보정합니다. 손가락 사이를 비우세요.\n");
    int speed = 64, force_v = 32;  // 0~255, 낮게 시작
    std::string cmd;
    while (true) {
        std::printf("\n그리퍼 (speed=%d force=%d) > ", speed, force_v);
        std::fflush(stdout);
        if (!std::getline(std::cin, cmd) || cmd == "q") break;
        if (cmd == "s") read_status();
        else if (cmd == "r") { write_cmd(0x00, 0, 0, 0); read_status(); }
        else if (cmd == "a") {
            std::printf("리셋 후 활성화...\n");
            write_cmd(0x00, 0, 0, 0);
            std::this_thread::sleep_for(200ms);
            write_cmd(0x01, 0, 0, 0);
            uint8_t sta = 0;
            for (int i = 0; i < 50; ++i) {
                std::this_thread::sleep_for(200ms);
                if (read_status(false, &sta) && sta == 3) break;
            }
            read_status();
            std::printf(sta == 3 ? "활성화 완료\n" : "활성화 확인 실패\n");
        } else if (cmd == "o" || cmd == "c" || cmd == "p") {
            int pos = cmd == "o" ? 0 : cmd == "c" ? 255 : ask_int("위치 (0=열림~255=닫힘)", 128, 0, 255);
            if (cmd == "p" || cmd == "c") {
                speed = ask_int("속도 (0~255)", speed, 0, 255);
                force_v = ask_int("힘 (0~255)", force_v, 0, 255);
            }
            // rACT=1, rGTO=1
            write_cmd(0x09, static_cast<uint8_t>(pos), static_cast<uint8_t>(speed), static_cast<uint8_t>(force_v));
            std::this_thread::sleep_for(1500ms);
            read_status();
        } else std::printf("알 수 없는 명령\n");
    }

    robot.flange_serial_close(PORT);
    robot.close_connection();
    std::printf("종료\n");
    return 0;
}

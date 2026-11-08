// Read-only diagnosis for Robotiq 2F-85 on the A0509 flange RS-485.
// Sends only status-read frames (FC03) for several port/baud/slave combinations. Arm and gripper do not move.
//   ./gripper_scan [--ip 192.168.137.101] [--force]
#ifndef DRCF_VERSION
#define DRCF_VERSION 2
#endif
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
using namespace DRAFramework;
using namespace std::chrono_literals;

static CDRFLEx robot;
static std::atomic<bool> g_access{false}, g_denied{false};

static void append_crc(std::vector<uint8_t>& d) {
    uint16_t c = 0xFFFF;
    for (uint8_t b : d) { c ^= b; for (int i = 0; i < 8; ++i) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1; }
    d.push_back(c & 0xFF); d.push_back(c >> 8);
}

static int probe(int port, int baud, uint8_t slave) {
    if (!robot.flange_serial_open(port, baud, BYTE_SIZE_EIGHTBITS, PARITY_CHECK_NONE, STOPBITS_ONE)) {
        std::printf("port=%d baud=%-6d : open 실패\n", port, baud);
        return -1;
    }
    std::this_thread::sleep_for(100ms);
    // drain anything pending
    for (int i = 0; i < 3; ++i) { auto r = robot.flange_serial_read(0.05f, port); if (!r || r->_iSize <= 0) break; }
    std::vector<uint8_t> req = {slave, 0x03, 0x07, 0xD0, 0x00, 0x03};
    append_crc(req);
    bool w = robot.flange_serial_write((int)req.size(), reinterpret_cast<char*>(req.data()), port);
    int total = 0, reads = 0, nulls = 0;
    std::vector<uint8_t> rx;
    auto deadline = std::chrono::steady_clock::now() + 800ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto r = robot.flange_serial_read(0.2f, port);
        ++reads;
        if (!r) { ++nulls; continue; }
        if (r->_iSize > 0) { rx.insert(rx.end(), r->_cRxd, r->_cRxd + r->_iSize); total += r->_iSize; }
    }
    std::printf("port=%d baud=%-6d slave=0x%02X write=%d reads=%d null=%d rx=%d :", port, baud, slave, w, reads, nulls, total);
    for (auto b : rx) std::printf(" %02X", b);
    std::printf("\n");
    robot.flange_serial_close(port);
    std::this_thread::sleep_for(100ms);
    return total;
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
    if (!robot.open_connection(ip)) { std::printf("연결 실패\n"); return 1; }
    robot.setup_monitoring_version(1);
    std::this_thread::sleep_for(1s);
    robot.ManageAccessControl(force ? MANAGE_ACCESS_CONTROL_FORCE_REQUEST : MANAGE_ACCESS_CONTROL_REQUEST);
    std::printf(force ? "제어권 강제 요청...\n" : "제어권 요청... TP 팝업에서 승인하세요.\n");
    for (int i = 0; i < 150 && !g_access && !g_denied; ++i) std::this_thread::sleep_for(100ms);
    if (!g_access) { std::printf("제어권 획득 실패\n"); robot.close_connection(); return 1; }

    std::printf("상태 읽기 프레임만 전송 (그리퍼/로봇 이동 없음)\n\n");
    bool found = false;
    for (int port : {1, 2, 0})
        for (int baud : {115200, 57600, 38400, 19200, 9600})
            if (probe(port, baud, 0x09) > 0) found = true;
    if (!found) {
        std::printf("\n기본 slave 0x09 응답 없음 -> 115200, port 1에서 slave 0x01~0x0F 확인\n");
        for (uint8_t s = 1; s <= 15; ++s)
            if (s != 9 && probe(1, 115200, s) > 0) found = true;
    }
    std::printf(found ? "\n응답 있음: 위 rx가 있는 조합 확인\n" : "\n모든 조합 응답 없음\n");
    robot.close_connection();
    return 0;
}

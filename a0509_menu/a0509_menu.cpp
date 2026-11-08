// A0509 / 공식 DRFL SDK / ROS 없는 단일 C++ 소스 (a0509_menu (1).cpp 기반 수정판)
// 초기화: 1 연결 -> 2 제어권 -> 3 준비(자동 서보 ON, 자동 모드, 원위치 저장) -> 관절 메뉴
// 빌드 (Ubuntu 22.04, 제어기 GF02.x -> DRCF_VERSION=2 / GF03.x -> 3):
//   ./build.sh a0509_menu/a0509_menu.cpp 2   (저장소 루트에서)
// 연습: ./a0509_menu
// 실기: ./a0509_menu --ip 192.168.137.101 --execute [--force] [--step 0.5]
// 검사: ./a0509_menu --self-test
// --step: 1회 이동량 (기본 1도, 최대 10도). --force: 펜던트 승인 없이 제어권 강제 획득.
// 원위치: 연결 후 처음 3번에서 저장한 자세 (정지 후 3번을 다시 해도 유지, o로 재저장).
// 3번: SAFE_OFF면 자동 서보 ON. 비상정지/보호정지 상태면 서보 ON하지 않음.
// 종료/연결 해제 시 3번 이전의 로봇 모드로 복원합니다.
// 이동 중 아무 키: 정지 후 메뉴 복귀 / 이동 중 Ctrl+C: 정지 후 프로그램 종료.
// 소프트웨어 정지는 비상정지를 대체하지 않습니다.
//
// 원본 대비 수정 사항:
//  - J6 감소 키 추가 (e/f = J6 +/-), 원위치 복귀 키 f -> h
//  - 속도 5도/s, 10도/s^2 -> 2도/s, 2도/s^2
//  - 3번에서 SAFE_OFF면 자동 서보 ON
//  - 종료 시 이전 로봇 모드 복원
//  - 이동 중 아무 키로 정지 (기존: Ctrl+C만 가능, 프로그램 종료)
//  - 정지/오류 후 3번 재실행 시 원위치를 덮어쓰지 않음 (o 키로 명시적 재저장)
//  - 관절 소프트 한계 검사, 이동 거리 기반 워치독, 목표 미도달 정지 감지
//  - 메뉴 대기 중 Ctrl+C 즉시 종료 (SA_RESTART 미사용)
//  - 연결 직후 모니터링 수신 전 관절값이 0으로 읽히는 문제 방지 (수신 대기 + 신선도 검사)
//  - 0번에 TCP 좌표 표시 (get_current_posx()가 GF02.08에서 멈추므로 모니터링 콜백 사용)
#include "DRFLEx.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

using namespace DRAFramework;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Pose = std::array<float, 6>;

constexpr float JOINT_VEL = 2.0f;  // deg/s
constexpr float JOINT_ACC = 2.0f;  // deg/s^2
// 보수적 소프트 한계 [deg] (A0509 사양: J1,J2,J4,J6 +-360, J3 +-150, J5 +-135)
constexpr float LIMIT_DEG[6] = {350.0f, 350.0f, 145.0f, 350.0f, 130.0f, 350.0f};

int main_menu(void);
// 0: 미연결 / 1: 연결됨 / 2: 제어권 획득 / 3: 조작 준비 완료
static unsigned short flag = 0;
static std::atomic<bool> access_granted{false}, access_denied{false}, disconnected{false};
static volatile std::sig_atomic_t interrupted = 0;
static std::mutex tcp_mutex;
static Pose tcp_pose{};
static Clock::time_point tcp_stamp{};

void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void on_signal(int) { interrupted = 1; }
void on_access(MONITORING_ACCESS_CONTROL value) {
    if (value == MONITORING_ACCESS_CONTROL_GRANT) access_granted = true;
    if (value == MONITORING_ACCESS_CONTROL_LOSS) access_granted = false;
    if (value == MONITORING_ACCESS_CONTROL_DENY) access_denied = true;
}
void on_disconnect() { disconnected = true; access_granted = false; }
void on_monitoring(const LPMONITORING_DATA_EX data) {
    std::lock_guard<std::mutex> lock(tcp_mutex);
    std::copy_n(data->_tCtrl._tTask._fActualPos[0], 6, tcp_pose.begin());
    tcp_stamp = Clock::now();
}
bool monitoring_fresh() {
    std::lock_guard<std::mutex> lock(tcp_mutex);
    return tcp_stamp != Clock::time_point{} && Clock::now() - tcp_stamp < 500ms;
}
float number(const std::string& text) {
    size_t used = 0;
    float value = 0;
    try { value = std::stof(text, &used); } catch (const std::exception&) { used = 0; }
    require(used == text.size() && std::isfinite(value), "유한한 숫자를 입력하세요.");
    return value;
}
bool ipv4(const std::string& text) {
    std::istringstream stream(text);
    std::string part;
    int count = 0;
    while (std::getline(stream, part, '.')) {
        if (part.empty() || part.size() > 3) return false;
        int value = 0;
        for (char c : part) { if (c < '0' || c > '9') return false; value = value * 10 + c - '0'; }
        if (value > 255) return false;
        ++count;
    }
    return count == 4 && !text.empty() && text.back() != '.';
}
const char* state_name(int s) {
    static const char* n[] = {"INITIALIZING", "STANDBY", "MOVING", "SAFE_OFF", "TEACHING",
                              "SAFE_STOP", "EMERGENCY_STOP", "HOMMING", "RECOVERY",
                              "SAFE_STOP2", "SAFE_OFF2"};
    return (s >= 0 && s <= STATE_SAFE_OFF2) ? n[s] : "OTHER";
}
void print_pose(const char* label, const Pose& values, const char* unit = "deg") {
    std::printf("%s [", label);
    for (size_t i = 0; i < 6; ++i) std::printf("%s%.3f", i ? ", " : "", values[i]);
    std::printf("] %s\n", unit);
}
bool near(const Pose& current, const Pose& target, float tolerance) {
    for (size_t i = 0; i < 6; ++i)
        if (!std::isfinite(current[i]) || !std::isfinite(target[i]) ||
            std::abs(current[i] - target[i]) > tolerance) return false;
    return true;
}

// 이동 중 키 입력을 즉시 읽기 위한 터미널 raw 모드 (소멸 시 복원)
class RawTerminal {
    termios saved{};
    bool active = false;
public:
    RawTerminal() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved) != 0) return;
        termios raw = saved;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        tcflush(STDIN_FILENO, TCIFLUSH);
        active = true;
    }
    ~RawTerminal() {
        if (!active) return;
        tcflush(STDIN_FILENO, TCIFLUSH);  // 정지용 키가 다음 메뉴 입력으로 넘어가지 않게
        tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    }
    bool key_pressed() const {
        if (!active) return false;
        pollfd p{STDIN_FILENO, POLLIN, 0};
        if (poll(&p, 1, 0) <= 0) return false;
        char c;
        return read(STDIN_FILENO, &c, 1) > 0;
    }
};

class Control {
    CDRFLEx robot;
    bool live, force, connected = false, moving = false, origin_valid = false;
    bool mode_changed = false;
    ROBOT_MODE previous_mode = ROBOT_MODE_MANUAL;
    std::string ip;
    float step;
    Pose origin{}, simulated{10, 20, 30, 40, 50, 60};

    void ready() {
        refresh();
        require(flag == 3 && origin_valid, "먼저 1 -> 2 -> 3 초기화를 완료하세요.");
        if (live) {
            require(!interrupted && access_granted && !disconnected, "제어권 또는 연결을 확인하세요.");
            require(robot.get_robot_state() == STATE_STANDBY && robot.check_motion() == 0,
                    "STANDBY/정지 상태가 아닙니다. 3번을 다시 실행하거나 펜던트를 확인하세요.");
            require(robot.get_robot_mode() == ROBOT_MODE_AUTONOMOUS, "자동 모드가 해제됐습니다.");
        }
    }
    void servo_on() {
        const auto state = robot.get_robot_state();
        if (state == STATE_STANDBY) return;
        require(state == STATE_SAFE_OFF || state == STATE_SAFE_OFF2,
                std::string("서보 ON 불가: 상태 ") + state_name(state) +
                    ". 펜던트에서 비상정지/보호정지를 먼저 해제하세요.");
        std::printf("서보 ON 요청 (%s -> STANDBY)...\n", state_name(state));
        require(robot.set_robot_control(CONTROL_SERVO_ON), "서보 ON 요청이 거부됐습니다.");
        const auto deadline = Clock::now() + 10s;
        while (Clock::now() < deadline) {
            require(!interrupted && access_granted && !disconnected, "중단/연결 끊김/제어권 상실");
            if (robot.get_robot_state() == STATE_STANDBY) { std::printf("서보 ON 완료\n"); return; }
            std::this_thread::sleep_for(100ms);
        }
        throw std::runtime_error(std::string("서보 ON 시간 초과: 상태 ") + state_name(robot.get_robot_state()));
    }
    void move_to(Pose target) {
        ready();
        for (int i = 0; i < 6; ++i)
            require(std::abs(target[i]) <= LIMIT_DEG[i],
                    "J" + std::to_string(i + 1) + " 목표 " + std::to_string(target[i]) +
                        "도가 소프트 한계 +-" + std::to_string(static_cast<int>(LIMIT_DEG[i])) + "도를 넘습니다.");
        print_pose("목표 관절", target);
        if (!live) { simulated = target; std::printf("[연습] 실제 연결/이동 없음\n"); return; }
        const Pose current = position();
        if (near(current, target, 0.001f)) { std::printf("이미 목표 위치입니다.\n"); return; }
        float dist = 0;
        for (int i = 0; i < 6; ++i) dist = std::max(dist, std::abs(target[i] - current[i]));
        const auto limit = std::chrono::duration<double>(dist / JOINT_VEL + 2.0 * JOINT_VEL / JOINT_ACC + 10.0);

        RawTerminal terminal;
        std::printf("이동 중... (아무 키: 정지 / Ctrl+C: 정지 후 종료)\n");
        moving = true;  // 응답 불명확/실패 시에도 정지를 요청할 수 있도록 먼저 표시
        try {
            require(robot.amovej(target.data(), JOINT_VEL, JOINT_ACC, 0, MOVE_MODE_ABSOLUTE),
                    "관절 이동 명령이 거부됐습니다.");
            const auto start = Clock::now();
            Clock::time_point idle_since{};
            while (true) {
                require(!interrupted, "Ctrl+C 정지 요청");
                require(!terminal.key_pressed(), "키 입력 정지 요청");
                require(!disconnected && access_granted, "연결 끊김/제어권 상실");
                require(Clock::now() - start < limit, "이동 시간 초과 (워치독)");
                const auto state = robot.get_robot_state();
                require(state == STATE_STANDBY || state == STATE_MOVING,
                        std::string("동작 중 로봇 상태 이상: ") + state_name(state));
                const int motion = robot.check_motion();
                require(motion >= 0, "동작 상태 조회 실패");
                if (motion == 0 && state == STATE_STANDBY && Clock::now() - start > 200ms) {
                    const Pose now = position();
                    if (near(now, target, 0.1f)) {
                        moving = false;
                        std::printf("이동 완료\n");
                        print_pose("현재 관절", now);
                        return;
                    }
                    if (idle_since == Clock::time_point{}) idle_since = Clock::now();
                    require(Clock::now() - idle_since < 1500ms, "목표 도달 전에 로봇이 멈췄습니다.");
                } else {
                    idle_since = {};
                }
                std::this_thread::sleep_for(20ms);
            }
        } catch (...) {
            const bool stopped = robot.stop(STOP_TYPE_QUICK);
            moving = !stopped;
            flag = 2;  // 새 조작 전 3번 준비 절차 필요 (원위치는 유지)
            std::printf("정지 요청: %s. 실제 정지를 확인하세요.\n", stopped ? "전달됨" : "실패");
            throw;
        }
    }
public:
    Control(bool execute, bool force_access, std::string address, float increment)
        : live(execute), force(force_access), ip(address), step(increment) {
        require(std::isfinite(step) && step > 0 && step <= 10, "이동량은 0 초과 10도 이하입니다.");
    }
    ~Control() { close(); }
    void refresh() {
        if (!live || !connected) return;
        if (disconnected) { flag = 0; }
        else if (flag >= 2 && !access_granted) { flag = 1; }
    }
    Pose position() {
        require(connected, "먼저 1번으로 연결하세요.");
        if (!live) return simulated;
        require(!disconnected, "제어기 연결이 끊겼습니다.");
        require(monitoring_fresh(), "모니터링 데이터가 최신이 아닙니다 (관절값을 신뢰할 수 없음).");
        auto p = robot.get_current_posj();
        require(p != nullptr, "현재 관절 조회 실패");
        Pose result{};
        std::copy_n(p->_fPosition, 6, result.begin());
        for (float v : result) require(std::isfinite(v), "관절 피드백 오류");
        return result;
    }
    void show() {
        print_pose("현재 관절", position());
        if (!live) return;
        std::printf("상태 %s / 모드 %s / 제어권 %s\n", state_name(robot.get_robot_state()),
                    robot.get_robot_mode() == ROBOT_MODE_AUTONOMOUS ? "AUTONOMOUS" : "MANUAL",
                    access_granted ? "있음" : "없음");
        std::lock_guard<std::mutex> lock(tcp_mutex);
        if (tcp_stamp != Clock::time_point{} && Clock::now() - tcp_stamp < 500ms)
            print_pose("현재 TCP (BASE)", tcp_pose, "mm,deg");
        else
            std::printf("현재 TCP: 모니터링 데이터 없음\n");
        if (origin_valid) print_pose("저장된 원위치", origin);
    }
    void connect() {
        require(flag == 0, "이미 연결됐습니다. 순서대로 다음 단계를 선택하세요.");
        if (connected) close();
        access_granted = false; access_denied = false; disconnected = false;
        origin_valid = false; mode_changed = false;
        if (live) {
            robot.set_on_monitoring_access_control(on_access);
            robot.set_on_disconnected(on_disconnect);
            robot.set_on_monitoring_data_ex(on_monitoring);
            {
                std::lock_guard<std::mutex> lock(tcp_mutex);
                tcp_stamp = {};
            }
            require(robot.open_connection(ip, 12345), "제어기 연결 실패: IP/포트 확인");
            connected = true;
            try {
                require(robot.setup_monitoring_version(1), "모니터링 설정 실패");
                // 첫 모니터링 데이터 전에는 get_current_posj()가 0을 반환함 -> 수신까지 대기
                const auto deadline = Clock::now() + 5s;
                while (!monitoring_fresh() && Clock::now() < deadline) std::this_thread::sleep_for(50ms);
                require(monitoring_fresh(), "모니터링 데이터 수신 실패");
                SYSTEM_VERSION version{};
                require(robot.get_system_version(&version), "버전 조회 실패");
                std::printf("Model=%s, Controller=%s, SDK=%s, DRCF_VERSION=%d\n",
                            version._szRobotModel, version._szController, robot.get_library_version(), DRCF_VERSION);
                require(robot.get_robot_system() == ROBOT_SYSTEM_REAL, "실제 로봇 시스템이 아닙니다.");
            } catch (...) { close(); throw; }
        } else connected = true;
        flag = 1;
        std::printf("1단계 완료: %s\n", live ? "제어기 연결됨" : "연습용 연결 (네트워크 사용 안 함)");
    }
    void acquire() {
        refresh();
        require(flag == 1, "먼저 1번 연결을 완료하세요. 제어권 획득 후에는 3번을 선택하세요.");
        if (live) {
            access_denied = false;
            require(robot.manage_access_control(force ? MANAGE_ACCESS_CONTROL_FORCE_REQUEST
                                                      : MANAGE_ACCESS_CONTROL_REQUEST),
                    "제어권 요청 실패");
            std::printf(force ? "제어권 강제 요청...\n" : "제어권 요청... 펜던트 팝업에서 승인하세요.\n");
            auto deadline = Clock::now() + 15s;
            while (!access_granted && !access_denied && !disconnected && !interrupted &&
                   Clock::now() < deadline) std::this_thread::sleep_for(50ms);
            require(access_granted && !disconnected && !interrupted,
                    "제어권 획득 실패: 펜던트 승인/다른 제어 프로그램 확인 (--force 옵션 고려)");
        }
        flag = 2;
        std::printf("2단계 완료: 제어권 획득%s\n", live ? "" : " (연습)");
    }
    void prepare() {
        refresh();
        require(flag == 2, "먼저 1 -> 2 순서로 실행하세요. 이미 준비 완료 상태입니다.");
        if (live) {
            require(!interrupted && access_granted && !disconnected, "제어권/연결 확인 필요");
            servo_on();
            require(robot.check_motion() == 0, "로봇이 아직 움직이는 중입니다.");
            if (!mode_changed) { previous_mode = robot.get_robot_mode(); mode_changed = true; }
            require(robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS), "자동 모드 설정 실패");
            require(robot.get_robot_mode() == ROBOT_MODE_AUTONOMOUS, "자동 모드 확인 실패");
            require(access_granted && !disconnected && robot.get_robot_state() == STATE_STANDBY,
                    "초기화 도중 로봇 상태가 변경됐습니다.");
        }
        if (!origin_valid) {
            origin = position();
            origin_valid = true;
            print_pose("저장한 원위치", origin);
        } else {
            print_pose("기존 원위치 유지", origin);
        }
        flag = 3;
        std::printf("3단계 완료: 관절 조작 가능%s\n", live ? "" : " (연습)");
    }
    void save_origin() {
        ready();
        origin = position();
        print_pose("원위치 재저장", origin);
    }
    void joint(int index, int sign) {
        ready();
        require(index >= 1 && index <= 6 && (sign == 1 || sign == -1), "관절 인수 오류");
        auto target = position();
        target[index - 1] += sign * step;
        move_to(target);
    }
    void home() { ready(); move_to(origin); }
    void stop() {
        refresh();
        require(connected && flag >= 2, "정지 명령에는 연결과 제어권이 필요합니다.");
        if (live) require(robot.stop(STOP_TYPE_QUICK), "정지 요청 실패: 실제 정지를 확인하세요.");
        moving = false;
        flag = 2;
        std::printf("정지 요청%s. 조작 재개 전 3번을 실행하세요.\n", live ? " 전달됨" : " (연습)");
    }
    void close() {
        if (connected && live) {
            if (moving && !robot.stop(STOP_TYPE_QUICK))
                std::printf("정지 요청 실패. 필요 시 펜던트 비상정지를 사용하세요.\n");
            if (mode_changed && !disconnected && robot.get_robot_mode() != previous_mode) {
                const bool ok = robot.set_robot_mode(previous_mode);
                std::printf("로봇 모드 복원 (%s)%s\n",
                            previous_mode == ROBOT_MODE_MANUAL ? "MANUAL" : "AUTONOMOUS", ok ? "" : " 실패");
            }
            robot.close_connection();
        }
        moving = false; connected = false; origin_valid = false; mode_changed = false; flag = 0;
    }
};

int self_test() {
    int checks = 0;
    auto check = [&](bool ok) { require(ok, "자체 검사 실패"); ++checks; };
    auto reject = [&](auto fn) { bool failed = false; try { fn(); } catch (const std::exception&) { failed = true; } check(failed); };
    Control test(false, false, "", 1);
    reject([&] { test.joint(1, 1); });
    reject([&] { test.home(); });
    reject([&] { test.acquire(); });
    reject([&] { test.prepare(); });
    test.connect(); check(flag == 1);
    reject([&] { test.connect(); });
    reject([&] { test.prepare(); });
    reject([&] { test.joint(1, 1); });
    test.acquire(); check(flag == 2);
    reject([&] { test.joint(1, 1); });
    reject([&] { test.home(); });
    test.prepare(); check(flag == 3);
    auto start = test.position();
    for (int axis = 1; axis <= 6; ++axis) {
        test.joint(axis, 1); auto target = start; target[axis - 1] += 1;
        check(near(test.position(), target, 0.0001f));
        test.home(); check(near(test.position(), start, 0.0001f));
        test.joint(axis, -1); target[axis - 1] -= 2;
        check(near(test.position(), target, 0.0001f));
        test.home(); check(near(test.position(), start, 0.0001f));
    }
    reject([&] { test.prepare(); });
    // 정지 후 3번 재실행: 원위치는 유지되어야 함
    test.joint(6, 1);
    test.stop(); check(flag == 2);
    reject([&] { test.joint(1, 1); });
    test.prepare(); check(flag == 3);
    test.home(); check(near(test.position(), start, 0.0001f));
    // 소프트 한계: J5 시작 50도, 한계 130도 -> 10도씩 8번(130도)까지 허용, 9번째 거부
    {
        Control limit(false, false, "", 10);
        flag = 0;
        limit.connect(); limit.acquire(); limit.prepare();
        for (int i = 0; i < 8; ++i) limit.joint(5, 1);
        reject([&] { limit.joint(5, 1); });
        limit.close();
    }
    test.close(); check(flag == 0);
    reject([&] { test.joint(1, 1); });
    std::printf("PASS: %d offline initialization/motion checks\n", checks);
    return 0;
}

int main(int argc, char** argv) {
    try {
        std::string ip;
        bool execute = false, force = false;
        float step = 1;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--self-test") return self_test();
            if (arg == "--execute") execute = true;
            else if (arg == "--force") force = true;
            else if (arg == "--ip" || arg == "--step") {
                require(i + 1 < argc, "실행 옵션의 값이 없습니다.");
                if (arg == "--ip") ip = argv[++i]; else step = number(argv[++i]);
            } else if (arg == "--help") {
                std::printf("a0509_menu [--ip IPv4] [--step DEG] [--execute] [--force] | --self-test\n"); return 0;
            } else throw std::runtime_error("알 수 없는 실행 옵션");
        }
        require(!execute || ipv4(ip), "실기 조작은 --ip 실제_IP --execute를 지정하세요.");
        // SA_RESTART 없이 등록: 메뉴 입력 대기 중 Ctrl+C가 즉시 반영되도록
        struct sigaction action{};
        action.sa_handler = on_signal;
        sigemptyset(&action.sa_mask);
        sigaction(SIGINT, &action, nullptr);
        sigaction(SIGTERM, &action, nullptr);
        Control control(execute, force, ip, step);
        std::printf("%s / 1회 %.2f도 / 속도 %.1f도/s / 가속도 %.1f도/s^2\n",
                    execute ? "[실제 로봇 제어]" : "[연습: 실제 연결/이동 없음]", step, JOINT_VEL, JOINT_ACC);
        int key;
        while (!interrupted) {
            control.refresh();
            if ((key = main_menu()) == 0 || interrupted) break;
            try {
                switch (key) {
                    case '0': std::printf("No.0 현재 상태\n"); control.show(); break;
                    case '1': std::printf("No.1 연결\n"); control.connect(); break;
                    case '2': std::printf("No.2 제어권 획득\n"); control.acquire(); break;
                    case '3': std::printf("No.3 서보 ON / 자동 모드 / 원위치 저장\n"); control.prepare(); break;
                    case '4': std::printf("J1 증가\n"); control.joint(1, 1); break;
                    case '5': std::printf("J1 감소\n"); control.joint(1, -1); break;
                    case '6': std::printf("J2 증가\n"); control.joint(2, 1); break;
                    case '7': std::printf("J2 감소\n"); control.joint(2, -1); break;
                    case '8': std::printf("J3 증가\n"); control.joint(3, 1); break;
                    case '9': std::printf("J3 감소\n"); control.joint(3, -1); break;
                    case 'a': std::printf("J4 증가\n"); control.joint(4, 1); break;
                    case 'b': std::printf("J4 감소\n"); control.joint(4, -1); break;
                    case 'c': std::printf("J5 증가\n"); control.joint(5, 1); break;
                    case 'd': std::printf("J5 감소\n"); control.joint(5, -1); break;
                    case 'e': std::printf("J6 증가\n"); control.joint(6, 1); break;
                    case 'f': std::printf("J6 감소\n"); control.joint(6, -1); break;
                    case 'h': std::printf("원위치 복귀\n"); control.home(); break;
                    case 'o': std::printf("원위치 재저장\n"); control.save_origin(); break;
                    case 's': control.stop(); break;
                    case 'x': control.close(); std::printf("연결 종료. 다시 1번부터 시작하세요.\n"); break;
                    case 'q': std::printf("exit\n"); control.close(); return 0;
                    default: std::printf("메뉴의 키 한 글자만 입력하세요.\n"); break;
                }
            } catch (const std::exception& error) {
                std::printf("오류: %s\n", error.what());
                control.refresh();
            }
        }
        std::printf("\n종료합니다.\n");
        return 0;  // Control 소멸자: 이동 중이면 정지, 모드 복원, 연결 해제
    } catch (const std::exception& error) { std::printf("오류: %s\n", error.what()); return 1; }
}

int main_menu(void) {
    std::printf("\n-------------------------------------------------\n");
    std::printf(" MAIN MENU (초기화 단계 flag=%hu/3)\n", flag);
    std::printf("-------------------------------------------------\n");
    std::printf(" 1. 제어기 연결\n");
    std::printf(" 2. 제어권 획득 (1번 완료 후)\n");
    std::printf(" 3. 서보 ON / 자동 모드 / 원위치 저장 (2번 완료 후)\n");
    std::printf("-------------------------------------------------\n");
    std::printf(" 4. J1 증가       5. J1 감소\n");
    std::printf(" 6. J2 증가       7. J2 감소\n");
    std::printf(" 8. J3 증가       9. J3 감소\n");
    std::printf(" a. J4 증가       b. J4 감소\n");
    std::printf(" c. J5 증가       d. J5 감소\n");
    std::printf(" e. J6 증가       f. J6 감소\n");
    std::printf(" h. 원위치 복귀    o. 현재 자세를 원위치로 재저장\n");
    std::printf("-------------------------------------------------\n");
    std::printf(" 0. 현재 상태 조회    s. 정지 요청 (메뉴 대기 중)\n");
    std::printf(" x. 연결 종료        q. 프로그램 종료\n");
    std::printf(" 이동 중 아무 키: 정지 / Ctrl+C: 정지 후 종료\n");
    std::printf("-------------------------------------------------\n");
    std::printf("SELECT THE COMMAND NUMBER : "); std::fflush(stdout);
    std::string input;
    if (!std::getline(std::cin, input)) return 0;
    if (input.size() != 1) return -1;  // Enter 잔여 문자/여러 글자 오입력 방지
    return static_cast<unsigned char>(input[0]);
}

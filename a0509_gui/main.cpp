// A0509 Qt jog GUI on DRFL (no ROS). Same control flow as a0509_menu:
//   connect -> control access -> auto servo ON (SAFE_OFF only) -> AUTONOMOUS -> jog
//   -> on exit: stop if moving, restore previous robot mode, disconnect.
// STOP button / Esc / Space -> MoveStop(QUICK). Software stop does NOT replace the E-stop.
// Joint feedback is used only while the monitoring stream is fresh (< 500 ms); before the first
// sample get_current_posj() returns all zeros, which would otherwise be saved as "home".
// Build: cd a0509_gui && mkdir build && cd build && cmake .. && make   (controller GF02.x -> DRCF_VERSION=2)
#include "DRFLEx.h"

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace DRAFramework;
using Clock = std::chrono::steady_clock;
using Pose = std::array<float, 6>;

namespace {
// Conservative software limits [deg] (A0509 spec: J1,J2,J4,J6 +-360, J3 +-150, J5 +-135)
constexpr float LIMIT_DEG[6] = {350.0f, 350.0f, 145.0f, 350.0f, 130.0f, 350.0f};
constexpr float JOINT_TOL_DEG = 0.1f;

// ---- state shared with DRFL callback threads ----
std::atomic<bool> g_access{false}, g_denied{false}, g_disconnected{false}, g_fault{false};
std::mutex g_mtx;
Pose g_tcp{};
Clock::time_point g_tcp_stamp{};
std::vector<std::string> g_logs;

void push_log(const std::string& s) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_logs.push_back(s);
}

const char* state_name(int s) {
    static const char* n[] = {"INITIALIZING", "STANDBY", "MOVING", "SAFE_OFF", "TEACHING",
                              "SAFE_STOP", "EMERGENCY_STOP", "HOMMING", "RECOVERY",
                              "SAFE_STOP2", "SAFE_OFF2"};
    return (s >= 0 && s <= STATE_SAFE_OFF2) ? n[s] : "OTHER";
}

bool is_fault(ROBOT_STATE s) {
    return s == STATE_EMERGENCY_STOP || s == STATE_SAFE_STOP || s == STATE_SAFE_STOP2;
}

void on_access(const MONITORING_ACCESS_CONTROL a) {
    if (a == MONITORING_ACCESS_CONTROL_GRANT) { g_access = true; push_log("제어권 획득"); }
    if (a == MONITORING_ACCESS_CONTROL_LOSS)  { g_access = false; push_log("제어권 상실"); }
    if (a == MONITORING_ACCESS_CONTROL_DENY)  { g_denied = true; push_log("제어권 요청 거부됨"); }
}
void on_state(const ROBOT_STATE s) {
    if (is_fault(s)) g_fault = true;
    push_log(std::string("상태 -> ") + state_name(s));
}
void on_alarm(LPLOG_ALARM a) {
    push_log("[알람] level=" + std::to_string(a->_iLevel) + " group=" + std::to_string(a->_iGroup) +
             " index=" + std::to_string(a->_iIndex) + " " + a->_szParam[0]);
}
void on_disconnected() {
    g_disconnected = true;
    g_access = false;
    push_log("컨트롤러 연결 끊김");
}
void on_monitoring(const LPMONITORING_DATA_EX d) {
    std::lock_guard<std::mutex> lk(g_mtx);
    for (int i = 0; i < 6; ++i) g_tcp[i] = d->_tCtrl._tTask._fActualPos[0][i];
    g_tcp_stamp = Clock::now();
}
// Before the first monitoring sample get_current_posj() returns all zeros -> never trust stale data.
bool monitoring_fresh() {
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_tcp_stamp != Clock::time_point{} && Clock::now() - g_tcp_stamp < std::chrono::milliseconds(500);
}
}  // namespace

class MainWindow : public QWidget {
public:
    MainWindow() {
        setWindowTitle("A0509 Jog (DRFL)");
        auto* root = new QVBoxLayout(this);

        // --- connection row ---
        auto* connRow = new QHBoxLayout;
        connRow->addWidget(new QLabel("IP"));
        ip_ = new QLineEdit("192.168.137.101");
        ip_->setMaximumWidth(150);
        connRow->addWidget(ip_);
        force_ = new QCheckBox("강제 제어권 요청 (TP 승인 없이)");
        connRow->addWidget(force_);
        connectBtn_ = makeButton("연결");
        connRow->addWidget(connectBtn_);
        connRow->addStretch();
        root->addLayout(connRow);

        // --- status row ---
        auto* statusBox = new QGroupBox("상태");
        auto* sl = new QHBoxLayout(statusBox);
        connLbl_ = new QLabel; stateLbl_ = new QLabel; modeLbl_ = new QLabel; accessLbl_ = new QLabel;
        for (auto* l : {connLbl_, stateLbl_, modeLbl_, accessLbl_}) sl->addWidget(l);
        sl->addStretch();
        root->addWidget(statusBox);

        // --- joints ---
        auto* jointBox = new QGroupBox("관절 조그");
        auto* grid = new QGridLayout(jointBox);
        QFont mono("Monospace");
        mono.setStyleHint(QFont::TypeWriter);
        mono.setPointSize(12);
        for (int j = 0; j < 6; ++j) {
            grid->addWidget(new QLabel(QString("J%1").arg(j + 1)), j, 0);
            minus_[j] = makeButton("−");
            plus_[j] = makeButton("+");
            minus_[j]->setMinimumSize(60, 36);
            plus_[j]->setMinimumSize(60, 36);
            jointLbl_[j] = new QLabel("-");
            jointLbl_[j]->setFont(mono);
            jointLbl_[j]->setMinimumWidth(120);
            jointLbl_[j]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            grid->addWidget(minus_[j], j, 1);
            grid->addWidget(plus_[j], j, 2);
            grid->addWidget(jointLbl_[j], j, 3);
            QObject::connect(minus_[j], &QPushButton::clicked, [this, j] { jog(j, -1); });
            QObject::connect(plus_[j], &QPushButton::clicked, [this, j] { jog(j, +1); });
        }
        root->addWidget(jointBox);

        // --- settings + TCP ---
        auto* setRow = new QHBoxLayout;
        setRow->addWidget(new QLabel("Step [deg]"));
        step_ = new QDoubleSpinBox;
        step_->setRange(0.1, 10.0); step_->setSingleStep(0.5); step_->setValue(1.0);
        setRow->addWidget(step_);
        setRow->addWidget(new QLabel("속도 [deg/s] (가속도 동일)"));
        vel_ = new QDoubleSpinBox;
        vel_->setRange(0.5, 10.0); vel_->setSingleStep(0.5); vel_->setValue(2.0);
        setRow->addWidget(vel_);
        setRow->addStretch();
        root->addLayout(setRow);

        tcpLbl_ = new QLabel("TCP: -");
        tcpLbl_->setFont(mono);
        root->addWidget(tcpLbl_);

        // --- actions ---
        auto* actRow = new QHBoxLayout;
        servoBtn_ = makeButton("Servo ON");
        homeBtn_ = makeButton("원위치로");
        setHomeBtn_ = makeButton("현재 자세를 원위치로 저장");
        stopBtn_ = makeButton("■ 정지 (Esc / Space)");
        stopBtn_->setMinimumHeight(50);
        stopBtn_->setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold;font-size:16px;}"
                                "QPushButton:disabled{background:#9e9e9e;}");
        actRow->addWidget(servoBtn_);
        actRow->addWidget(homeBtn_);
        actRow->addWidget(setHomeBtn_);
        actRow->addStretch();
        actRow->addWidget(stopBtn_, 1);
        root->addLayout(actRow);

        log_ = new QPlainTextEdit;
        log_->setReadOnly(true);
        log_->setMaximumBlockCount(500);
        log_->setFocusPolicy(Qt::NoFocus);
        root->addWidget(log_, 1);

        QObject::connect(connectBtn_, &QPushButton::clicked, [this] { robot_ ? disconnectRobot() : connectRobot(); });
        QObject::connect(servoBtn_, &QPushButton::clicked, [this] { beginServo(); });
        QObject::connect(homeBtn_, &QPushButton::clicked, [this] { if (haveHome_) startMove(home_, "원위치"); });
        QObject::connect(setHomeBtn_, &QPushButton::clicked, [this] {
            Pose p;
            if (!readJoints(p)) { log("원위치 저장 실패: 관절 피드백이 최신이 아닙니다"); return; }
            home_ = p; haveHome_ = true; log("원위치 저장: " + poseStr(home_));
        });
        QObject::connect(stopBtn_, &QPushButton::clicked, [this] { stopNow("정지 버튼"); });
        for (auto key : {Qt::Key_Escape, Qt::Key_Space}) {
            auto* sc = new QShortcut(QKeySequence(key), this);
            sc->setContext(Qt::ApplicationShortcut);
            QObject::connect(sc, &QShortcut::activated, [this] { stopNow("키보드 정지"); });
        }

        QObject::connect(&timer_, &QTimer::timeout, [this] { tick(); });
        timer_.start(100);
        refreshUi();
        resize(640, 720);
    }

protected:
    void closeEvent(QCloseEvent* e) override {
        disconnectRobot();
        e->accept();
    }

private:
    enum class Phase { Off, WaitAccess, WaitServo, Ready, Moving };

    QPushButton* makeButton(const QString& text) {
        auto* b = new QPushButton(text);
        b->setFocusPolicy(Qt::NoFocus);  // Space must never "press" a focused motion button
        return b;
    }

    void log(const QString& s) {
        log_->appendPlainText(QDateTime::currentDateTime().toString("hh:mm:ss.zzz ") + s);
    }

    static QString poseStr(const Pose& p, int prec = 2) {
        QStringList l;
        for (float v : p) l << QString::number(v, 'f', prec);
        return "[" + l.join(", ") + "]";
    }

    // Returns false (and leaves p untouched) when joint feedback cannot be trusted.
    bool readJoints(Pose& p) {
        if (!robot_ || !monitoring_fresh()) return false;
        LPROBOT_POSE j = robot_->get_current_posj();
        if (!j) return false;
        Pose r{};
        for (int i = 0; i < 6; ++i) {
            r[i] = j->_fPosition[i];
            if (!std::isfinite(r[i])) return false;
        }
        p = r;
        return true;
    }

    // ---------- connection sequence ----------
    void connectRobot() {
        g_access = g_denied = g_disconnected = g_fault = false;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_tcp_stamp = {};
        }
        robot_ = std::make_unique<CDRFLEx>();
        robot_->set_on_monitoring_access_control(on_access);
        robot_->set_on_monitoring_state(on_state);
        robot_->set_on_log_alarm(on_alarm);
        robot_->set_on_disconnected(on_disconnected);
        robot_->set_on_monitoring_data_ex(on_monitoring);

        const std::string ip = ip_->text().trimmed().toStdString();
        log(QString("연결 시도: %1").arg(ip.c_str()));
        if (!robot_->open_connection(ip, 12345)) {
            log("연결 실패");
            robot_.reset();
            refreshUi();
            return;
        }
        robot_->setup_monitoring_version(1);
        const auto monDeadline = Clock::now() + std::chrono::seconds(5);
        while (!monitoring_fresh() && Clock::now() < monDeadline) {
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!monitoring_fresh()) {
            log("모니터링 데이터 수신 실패 (관절값을 신뢰할 수 없음). 연결 해제");
            disconnectRobot();
            return;
        }
        SYSTEM_VERSION v{};
        robot_->get_system_version(&v);
        log(QString("연결됨: model=%1 controller=%2 SDK=%3")
                .arg(v._szRobotModel).arg(v._szController).arg(robot_->get_library_version()));
        if (robot_->get_robot_system() != ROBOT_SYSTEM_REAL) {
            log("컨트롤러가 REAL 시스템이 아닙니다. 연결 해제");
            disconnectRobot();
            return;
        }
        modeChanged_ = false;
        haveHome_ = false;
        const bool force = force_->isChecked();
        robot_->ManageAccessControl(force ? MANAGE_ACCESS_CONTROL_FORCE_REQUEST : MANAGE_ACCESS_CONTROL_REQUEST);
        log(force ? "제어권 강제 요청" : "제어권 요청 - TP 팝업에서 승인하세요");
        setPhase(Phase::WaitAccess, 15);
    }

    void beginServo() {
        if (!robot_ || !g_access) return;
        const ROBOT_STATE s = robot_->get_robot_state();
        if (s == STATE_STANDBY) { enterReady(); return; }
        if (s != STATE_SAFE_OFF && s != STATE_SAFE_OFF2) {
            log(QString("Servo ON 거부: 상태 %1. TP에서 비상정지/보호정지를 먼저 해제하세요").arg(state_name(s)));
            setPhase(Phase::Ready);
            return;
        }
        g_fault = false;
        log("Servo ON 요청");
        if (!robot_->set_robot_control(CONTROL_SERVO_ON)) {
            log("Servo ON 요청이 거부되었습니다");
            setPhase(Phase::Ready);
            return;
        }
        setPhase(Phase::WaitServo, 10);
    }

    void enterReady() {
        if (!modeChanged_) {
            prevMode_ = robot_->get_robot_mode();
            modeChanged_ = true;
        }
        if (robot_->get_robot_mode() != ROBOT_MODE_AUTONOMOUS && !robot_->set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
            log("AUTONOMOUS 모드 전환 실패");
        }
        if (!haveHome_) {
            Pose p;
            if (readJoints(p)) {
                home_ = p;
                haveHome_ = true;
                log("원위치 저장: " + poseStr(home_));
            } else {
                log("원위치 자동 저장 실패: 관절 피드백이 최신이 아닙니다. [현재 자세를 원위치로 저장]을 눌러주세요");
            }
        }
        log("준비 완료 (STANDBY, AUTONOMOUS)");
        setPhase(Phase::Ready);
    }

    void disconnectRobot() {
        if (!robot_) return;
        if (!g_disconnected) {
            if (phase_ == Phase::Moving) robot_->MoveStop(STOP_TYPE_QUICK);
            if (modeChanged_ && robot_->get_robot_mode() != prevMode_) {
                const bool ok = robot_->set_robot_mode(prevMode_);
                log(QString("로봇 모드 복원 -> %1%2").arg(prevMode_).arg(ok ? "" : " (실패)"));
            }
            robot_->close_connection();
        }
        robot_.reset();
        log("연결 해제");
        setPhase(Phase::Off);
    }

    void setPhase(Phase p, int timeoutSec = 0) {
        phase_ = p;
        deadline_ = Clock::now() + std::chrono::seconds(timeoutSec);
        refreshUi();
    }

    // ---------- motion ----------
    bool canMove(QString* why) {
        if (phase_ != Phase::Ready) { *why = "준비 상태가 아닙니다"; return false; }
        if (!g_access) { *why = "제어권이 없습니다"; return false; }
        if (robot_->get_robot_state() != STATE_STANDBY) { *why = "STANDBY 상태가 아닙니다"; return false; }
        if (robot_->check_motion() != 0) { *why = "로봇이 이미 움직이는 중입니다"; return false; }
        if (robot_->get_robot_mode() != ROBOT_MODE_AUTONOMOUS) { *why = "AUTONOMOUS 모드가 아닙니다"; return false; }
        return true;
    }

    void jog(int j, int sign) {
        if (!robot_) return;
        Pose target;
        if (!readJoints(target)) { log("이동 불가: 관절 피드백이 최신이 아닙니다"); return; }
        target[j] += sign * static_cast<float>(step_->value());
        if (std::fabs(target[j]) > LIMIT_DEG[j]) {
            log(QString("J%1 목표 %2도가 한계 ±%3도를 넘습니다").arg(j + 1).arg(target[j]).arg(LIMIT_DEG[j]));
            return;
        }
        startMove(target, QString("J%1 %2%3도").arg(j + 1).arg(sign > 0 ? "+" : "−").arg(step_->value()));
    }

    void startMove(Pose target, const QString& what) {
        QString why;
        if (!robot_ || !canMove(&why)) { log("이동 불가: " + why); return; }
        Pose cur;
        if (!readJoints(cur)) { log("이동 불가: 관절 피드백이 최신이 아닙니다"); return; }
        float dist = 0;
        for (int i = 0; i < 6; ++i) dist = std::max(dist, std::fabs(target[i] - cur[i]));
        if (dist < 0.001f) { log("이미 목표 위치입니다"); return; }

        const float vel = static_cast<float>(vel_->value()), acc = vel;
        target_ = target;
        maxSec_ = dist / vel + 2.0 * vel / acc + 10.0;
        g_fault = false;
        if (!robot_->amovej(target_.data(), vel, acc, 0.f, MOVE_MODE_ABSOLUTE)) {
            log("컨트롤러가 이동 명령을 거부했습니다");
            return;
        }
        moveStart_ = Clock::now();
        idleSince_ = {};
        log(QString("이동 시작: %1 (%2 deg/s) -> %3").arg(what).arg(vel).arg(poseStr(target_)));
        setPhase(Phase::Moving);
    }

    void stopNow(const QString& reason) {
        if (!robot_) return;
        robot_->MoveStop(STOP_TYPE_QUICK);
        log("[정지] " + reason + " -> MoveStop(QUICK)");
        if (phase_ == Phase::Moving) setPhase(Phase::Ready);
    }

    void pollMotion(ROBOT_STATE st, const Pose& cur) {
        const double el = std::chrono::duration<double>(Clock::now() - moveStart_).count();
        if (g_fault || is_fault(st)) { stopNow(QString("로봇 이상 상태 %1").arg(state_name(st))); return; }
        if (!g_access) { stopNow("제어권 상실"); return; }
        if (el > maxSec_) { stopNow(QString("워치독 시간 초과 (%1s)").arg(maxSec_, 0, 'f', 1)); return; }
        if (el < 0.2) return;

        if (robot_->check_motion() == 0 && st == STATE_STANDBY) {
            bool reached = true;
            for (int i = 0; i < 6; ++i) reached &= std::fabs(cur[i] - target_[i]) <= JOINT_TOL_DEG;
            if (reached) {
                log(QString("이동 완료 (%1s): %2").arg(el, 0, 'f', 1).arg(poseStr(cur)));
                setPhase(Phase::Ready);
                return;
            }
            if (idleSince_ == Clock::time_point{}) idleSince_ = Clock::now();
            if (Clock::now() - idleSince_ > std::chrono::milliseconds(1500)) {
                log("로봇이 목표 도달 전에 멈췄습니다: " + poseStr(cur));
                setPhase(Phase::Ready);
            }
        } else {
            idleSince_ = {};
        }
    }

    // ---------- periodic update ----------
    void tick() {
        std::vector<std::string> logs;
        Pose tcp;
        bool tcpFresh;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            logs.swap(g_logs);
            tcp = g_tcp;
            tcpFresh = g_tcp_stamp != Clock::time_point{} && Clock::now() - g_tcp_stamp < std::chrono::milliseconds(500);
        }
        for (auto& s : logs) log(QString::fromStdString(s));
        if (!robot_) return;
        if (g_disconnected) { disconnectRobot(); return; }

        const ROBOT_STATE st = robot_->get_robot_state();
        const ROBOT_MODE mode = robot_->get_robot_mode();
        Pose cur{};
        const bool jointsFresh = readJoints(cur);
        for (int j = 0; j < 6; ++j)
            jointLbl_[j]->setText(jointsFresh ? QString("%1°").arg(cur[j], 9, 'f', 3) : QString("(데이터 없음)"));
        if (phase_ == Phase::Moving && !jointsFresh) {
            stopNow("관절 피드백 끊김");
            refreshButtons(st);
            return;
        }
        tcpLbl_->setText(tcpFresh ? QString("TCP [mm,deg]: X %1  Y %2  Z %3  A %4  B %5  C %6")
                                        .arg(tcp[0], 0, 'f', 1).arg(tcp[1], 0, 'f', 1).arg(tcp[2], 0, 'f', 1)
                                        .arg(tcp[3], 0, 'f', 1).arg(tcp[4], 0, 'f', 1).arg(tcp[5], 0, 'f', 1)
                                  : QString("TCP: (모니터링 데이터 없음)"));
        stateLbl_->setText(QString("상태: <b>%1</b>").arg(state_name(st)));
        stateLbl_->setStyleSheet(is_fault(st) ? "color:#c62828" : st == STATE_STANDBY ? "color:#2e7d32" : "");
        modeLbl_->setText(QString("모드: <b>%1</b>").arg(mode == ROBOT_MODE_AUTONOMOUS ? "AUTONOMOUS" : mode == ROBOT_MODE_MANUAL ? "MANUAL" : "기타"));
        accessLbl_->setText(QString("제어권: <b>%1</b>").arg(g_access ? "있음" : "없음"));

        switch (phase_) {
        case Phase::WaitAccess:
            if (g_access) beginServo();
            else if (g_denied || Clock::now() > deadline_) {
                log(g_denied ? "제어권 요청이 거부되었습니다" : "제어권 대기 시간 초과");
                disconnectRobot();
            }
            break;
        case Phase::WaitServo:
            if (st == STATE_STANDBY) enterReady();
            else if (Clock::now() > deadline_) {
                log(QString("Servo ON 시간 초과 (상태 %1)").arg(state_name(st)));
                setPhase(Phase::Ready);
            }
            break;
        case Phase::Moving:
            pollMotion(st, cur);
            break;
        default:
            break;
        }
        refreshButtons(st);
    }

    void refreshUi() {
        const bool on = robot_ != nullptr;
        connectBtn_->setText(on ? "연결 해제" : "연결");
        ip_->setEnabled(!on);
        force_->setEnabled(!on);
        connLbl_->setText(QString("연결: <b>%1</b>").arg(on ? "ON" : "OFF"));
        if (!on) {
            stateLbl_->setText("상태: -"); stateLbl_->setStyleSheet("");
            modeLbl_->setText("모드: -"); accessLbl_->setText("제어권: -");
            tcpLbl_->setText("TCP: -");
            for (auto* l : jointLbl_) l->setText("-");
        }
        refreshButtons(on ? robot_->get_robot_state() : STATE_NOT_READY);
    }

    void refreshButtons(ROBOT_STATE st) {
        const bool ready = robot_ && phase_ == Phase::Ready && g_access && st == STATE_STANDBY && monitoring_fresh();
        for (int j = 0; j < 6; ++j) { minus_[j]->setEnabled(ready); plus_[j]->setEnabled(ready); }
        homeBtn_->setEnabled(ready && haveHome_);
        setHomeBtn_->setEnabled(ready);
        servoBtn_->setEnabled(robot_ && phase_ == Phase::Ready && g_access &&
                              (st == STATE_SAFE_OFF || st == STATE_SAFE_OFF2));
        stopBtn_->setEnabled(robot_ != nullptr);
        connectBtn_->setEnabled(phase_ != Phase::Moving);
    }

    // widgets
    QLineEdit* ip_;
    QCheckBox* force_;
    QPushButton *connectBtn_, *servoBtn_, *homeBtn_, *setHomeBtn_, *stopBtn_;
    QPushButton *minus_[6], *plus_[6];
    QLabel *connLbl_, *stateLbl_, *modeLbl_, *accessLbl_, *tcpLbl_;
    QLabel* jointLbl_[6];
    QDoubleSpinBox *step_, *vel_;
    QPlainTextEdit* log_;
    QTimer timer_;

    // robot
    std::unique_ptr<CDRFLEx> robot_;
    Phase phase_ = Phase::Off;
    Clock::time_point deadline_, moveStart_, idleSince_;
    Pose home_{}, target_{};
    bool haveHome_ = false, modeChanged_ = false;
    ROBOT_MODE prevMode_ = ROBOT_MODE_MANUAL;
    double maxSec_ = 0;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}

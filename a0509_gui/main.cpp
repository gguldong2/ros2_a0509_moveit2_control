// A0509 Qt jog GUI on DRFL (no ROS). Same control flow as a0509_menu:
//   connect -> control access -> auto servo ON (SAFE_OFF only) -> AUTONOMOUS -> jog / MoveL
//   -> on exit: stop if moving, restore previous robot mode, disconnect.
// STOP button / Esc / Space -> MoveStop(QUICK). Software stop does NOT replace the E-stop.
// Joint feedback is used only while the monitoring stream is fresh (< 500 ms); before the first
// sample get_current_posj() returns all zeros, which would otherwise be saved as "home".
// MoveL: X/Y/Z [mm] + Roll/Pitch/Yaw [deg] input (absolute / relative base / relative tool). The target is
// computed in pose_math.h from the monitored TCP (get_current_posx() hangs on controller 2.8) and sent as an
// absolute base-frame amovel in the controller's ZYZ [A, B, C].
// Build: cd a0509_gui && mkdir build && cd build && cmake .. && make   (controller GF02.x -> DRCF_VERSION=2)
#include "DRFLEx.h"
#include "pose_math.h"

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
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
// MoveL: "reached" tolerance, and the size above which a confirmation dialog is shown
constexpr double LIN_TOL_MM = 1.0, LIN_TOL_DEG = 0.5;
constexpr double LIN_CONFIRM_MM = 100.0, LIN_CONFIRM_DEG = 30.0;

// ---- state shared with DRFL callback threads ----
std::atomic<bool> g_access{false}, g_denied{false}, g_disconnected{false}, g_fault{false};
std::mutex g_mtx;
Pose g_tcp{};                  // TCP in base [mm, ZYZ deg]
float g_rotm[3][3]{};          // TCP rotation matrix from the same sample (ZYZ convention self-check)
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
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) g_rotm[r][c] = d->_tCtrl._tTask._fRotationMatrix[r][c];
    g_tcp_stamp = Clock::now();
}
// Before the first monitoring sample get_current_posj() returns all zeros -> never trust stale data.
bool fresh_locked() {  // caller holds g_mtx
    return g_tcp_stamp != Clock::time_point{} && Clock::now() - g_tcp_stamp < std::chrono::milliseconds(500);
}
bool monitoring_fresh() {
    std::lock_guard<std::mutex> lk(g_mtx);
    return fresh_locked();
}
// TCP [mm, ZYZ deg] from the monitoring stream; false (p untouched) when stale or invalid.
bool read_tcp(Pose& p) {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (!fresh_locked()) return false;
    for (float v : g_tcp)
        if (!std::isfinite(v)) return false;
    p = g_tcp;
    return true;
}
}  // namespace

class MainWindow : public QWidget {
public:
    MainWindow() {
        setWindowTitle("A0509 Jog / MoveL (DRFL)");
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
        // joint jog (left) | MoveL (right)
        auto* motionRow = new QHBoxLayout;
        motionRow->addWidget(jointBox);
        motionRow->addWidget(buildMoveLBox(), 1);
        root->addLayout(motionRow);

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
        // A focused text field (MoveL inputs, step/speed boxes) would otherwise claim Space/Esc and the
        // stop shortcut would not fire while the cursor is in it -> see eventFilter().
        QCoreApplication::instance()->installEventFilter(this);

        QObject::connect(&timer_, &QTimer::timeout, [this] { tick(); });
        timer_.start(100);
        refreshUi();
        resize(1080, 860);
    }
    ~MainWindow() override { QCoreApplication::instance()->removeEventFilter(this); }

protected:
    void closeEvent(QCloseEvent* e) override {
        disconnectRobot();
        e->accept();
    }

    bool eventFilter(QObject* obj, QEvent* e) override {
        if (e->type() == QEvent::ShortcutOverride) {
            const auto* k = static_cast<QKeyEvent*>(e);
            // Leave the override un-accepted for every receiver: no widget can take the key, so the
            // Esc / Space STOP shortcut always fires (and nothing is typed into the field).
            if ((k->key() == Qt::Key_Escape || k->key() == Qt::Key_Space) && k->modifiers() == Qt::NoModifier)
                return true;
        } else if (e->type() == QEvent::Wheel) {
            // A mouse wheel passing over a MoveL target field must not silently change the target.
            for (auto* s : lin_)
                if (s && (obj == s || obj->parent() == s)) return true;
        }
        return QWidget::eventFilter(obj, e);
    }

private:
    enum class Phase { Off, WaitAccess, WaitServo, Ready, Moving };
    enum class MoveKind { Joint, Linear };

    QPushButton* makeButton(const QString& text) {
        auto* b = new QPushButton(text);
        b->setFocusPolicy(Qt::NoFocus);  // Space must never "press" a focused motion button
        return b;
    }

    static QDoubleSpinBox* makeSpin(double lo, double hi, double value, int decimals, double step = 1.0) {
        auto* s = new QDoubleSpinBox;
        s->setRange(lo, hi);
        s->setDecimals(decimals);
        s->setSingleStep(step);
        s->setValue(value);
        s->setButtonSymbols(QAbstractSpinBox::NoButtons);
        s->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        s->setMinimumWidth(110);
        return s;
    }

    // ---------- MoveL panel ----------
    QGroupBox* buildMoveLBox() {
        auto* box = new QGroupBox("MoveL  (직선, base 좌표계 / TCP)");
        auto* g = new QGridLayout(box);
        static const char* const kLabels[6] = {"X [mm]", "Y [mm]", "Z [mm]",
                                               "Roll (Rx) [°]", "Pitch (Ry) [°]", "Yaw (Rz) [°]"};
        static const char* const kTips[6] = {
            "절대: 목표 X / 상대: X 방향 이동량", "절대: 목표 Y / 상대: Y 방향 이동량",
            "절대: 목표 Z / 상대: Z 방향 이동량 (base 기준 + = 위쪽, tool 기준 + = 툴이 바라보는 방향)",
            "X축 기준 회전 (RPY: R = Rz(yaw)·Ry(pitch)·Rx(roll), 고정축 X→Y→Z, ROS rpy와 동일)",
            "Y축 기준 회전", "Z축 기준 회전"};
        for (int i = 0; i < 6; ++i) {
            g->addWidget(new QLabel(kLabels[i]), i, 0);
            lin_[i] = i < 3 ? makeSpin(-2000, 2000, 0, 2) : makeSpin(-360, 360, 0, 3);
            lin_[i]->setToolTip(kTips[i]);
            g->addWidget(lin_[i], i, 1, 1, 3);
        }

        auto* modeRow = new QHBoxLayout;
        linAbs_ = new QRadioButton("절대");
        linRelBase_ = new QRadioButton("상대(base)");
        linRelTool_ = new QRadioButton("상대(tool)");
        linAbs_->setToolTip("입력한 X/Y/Z, Roll/Pitch/Yaw 자체가 목표 자세 (base 좌표계)");
        linRelBase_->setToolTip("현재 TCP에서 base 좌표축 방향으로 이동 / TCP 위치를 고정한 채 base 축 기준 회전");
        linRelTool_->setToolTip("현재 TCP에서 툴 좌표축 방향으로 이동/회전. 예) Z에 10 → 툴이 바라보는 방향으로 10mm");
        auto* group = new QButtonGroup(box);
        for (auto* r : {linAbs_, linRelBase_, linRelTool_}) {
            r->setFocusPolicy(Qt::NoFocus);
            group->addButton(r);
            modeRow->addWidget(r);
        }
        linRelBase_->setChecked(true);  // all-zero inputs + relative = no motion (safe default)
        modeRow->addStretch();
        g->addLayout(modeRow, 6, 0, 1, 4);

        linVel_ = makeSpin(1, 250, 50, 1);
        linAcc_ = makeSpin(10, 1000, 100, 0, 10);
        linRotVel_ = makeSpin(1, 60, 15, 1);
        linRotAcc_ = makeSpin(1, 180, 30, 1);
        linVel_->setToolTip("TCP 직선 속도 (amovel vel[0])");
        linAcc_->setToolTip("TCP 직선 가속도 (amovel acc[0])");
        linRotVel_->setToolTip("TCP 회전 속도 (amovel vel[1]). 자세만 바꿀 때는 이 값이 이동 시간을 정합니다");
        linRotAcc_->setToolTip("TCP 회전 가속도 (amovel acc[1])");
        g->addWidget(new QLabel("속도 v [mm/s]"), 7, 0);
        g->addWidget(linVel_, 7, 1);
        g->addWidget(new QLabel("회전 [°/s]"), 7, 2);
        g->addWidget(linRotVel_, 7, 3);
        g->addWidget(new QLabel("가속 a [mm/s²]"), 8, 0);
        g->addWidget(linAcc_, 8, 1);
        g->addWidget(new QLabel("회전 [°/s²]"), 8, 2);
        g->addWidget(linRotAcc_, 8, 3);

        auto* btnRow = new QHBoxLayout;
        linLoadBtn_ = makeButton("현재값 불러오기");
        linZeroBtn_ = makeButton("0으로");
        linGoBtn_ = makeButton("MoveL 실행");
        linLoadBtn_->setToolTip("현재 TCP를 X/Y/Z, Roll/Pitch/Yaw로 채우고 [절대]로 전환. 여기서 조금만 고쳐서 실행하면 안전합니다");
        linZeroBtn_->setToolTip("입력칸을 모두 0으로 ([절대]였으면 [상대(base)]로 전환). 상대 이동할 때 움직일 축만 입력하세요");
        linGoBtn_->setToolTip(QString("amovel: TCP가 직선으로 이동합니다. 특이점 근처에서는 실패할 수 있으니 짧은 거리부터 시험하세요.\n"
                                      "%1mm 또는 %2° 보다 큰 이동은 확인 창이 뜹니다. 정지: Esc / Space / 정지 버튼")
                                  .arg(LIN_CONFIRM_MM).arg(LIN_CONFIRM_DEG));
        linGoBtn_->setMinimumHeight(36);
        linGoBtn_->setStyleSheet("QPushButton{font-weight:bold;}");
        btnRow->addWidget(linLoadBtn_);
        btnRow->addWidget(linZeroBtn_);
        btnRow->addStretch();
        btnRow->addWidget(linGoBtn_);
        g->addLayout(btnRow, 9, 0, 1, 4);
        g->setRowStretch(10, 1);

        QObject::connect(linLoadBtn_, &QPushButton::clicked, [this] { loadCurrentTcp(); });
        QObject::connect(linZeroBtn_, &QPushButton::clicked, [this] {
            for (auto* s : lin_) s->setValue(0);
            if (linAbs_->isChecked()) {
                linRelBase_->setChecked(true);
                log("MoveL 입력 0으로 (모드: 절대 -> 상대(base))");
            }
        });
        QObject::connect(linGoBtn_, &QPushButton::clicked, [this] { doMoveL(); });
        return box;
    }

    pm::LinMode linMode() const {
        if (linAbs_->isChecked()) return pm::LinMode::Absolute;
        return linRelTool_->isChecked() ? pm::LinMode::RelTool : pm::LinMode::RelBase;
    }

    static QString modeName(pm::LinMode m) {
        return m == pm::LinMode::Absolute ? "절대" : m == pm::LinMode::RelBase ? "상대(base)" : "상대(tool)";
    }

    // "[x, y, z | roll, pitch, yaw]" (or ZYZ A, B, C)
    static QString vecStr(const pm::Vec6& v) {
        return QString("[%1, %2, %3 | %4, %5, %6]")
            .arg(v[0], 0, 'f', 2).arg(v[1], 0, 'f', 2).arg(v[2], 0, 'f', 2)
            .arg(v[3], 0, 'f', 3).arg(v[4], 0, 'f', 3).arg(v[5], 0, 'f', 3);
    }

    void loadCurrentTcp() {
        Pose tcp;
        if (!read_tcp(tcp)) { log("현재값 불러오기 실패: TCP 피드백이 최신이 아닙니다"); return; }
        const pm::Vec6 v = pm::xyzrpy_from_frame(pm::frame_from_posx(tcp));
        for (int i = 0; i < 6; ++i) lin_[i]->setValue(v[i]);
        linAbs_->setChecked(true);
        log("MoveL 입력 <- 현재 TCP [X,Y,Z | R,P,Y] " + vecStr(v));
    }

    void doMoveL() {
        if (!robot_) return;
        QString why;
        if (!canMove(&why)) { log("MoveL 불가: " + why); return; }
        Pose tcp;
        if (!read_tcp(tcp)) { log("MoveL 불가: TCP 피드백이 최신이 아닙니다"); return; }

        // A half-typed field ("", "-") keeps its previous value() -> never move to a number not on screen.
        // (check the raw text: interpretText() would silently revert it to the old value first)
        for (auto* s : {lin_[0], lin_[1], lin_[2], lin_[3], lin_[4], lin_[5], linVel_, linAcc_, linRotVel_, linRotAcc_})
            if (!s->hasAcceptableInput()) { log(QString("MoveL 불가: 입력값을 확인하세요 ('%1')").arg(s->text())); return; }
        const pm::LinMode mode = linMode();
        pm::Vec6 in;
        for (int i = 0; i < 6; ++i) in[i] = lin_[i]->value();
        const pm::Frame cur = pm::frame_from_posx(tcp);
        const pm::Frame tgt = pm::movel_target(cur, mode, in);
        const double dist = pm::dist(cur.p, tgt.p), ang = pm::rot_angle_deg(cur.R, tgt.R);
        if (dist < 0.01 && ang < 0.01) { log("이미 목표 위치입니다"); return; }

        const pm::Vec6 curRpy = pm::xyzrpy_from_frame(cur), tgtRpy = pm::xyzrpy_from_frame(tgt);
        if (dist > LIN_CONFIRM_MM || ang > LIN_CONFIRM_DEG) {
            const auto ans = QMessageBox::question(
                this, "MoveL 확인",
                QString("큰 이동입니다: 거리 %1 mm, 회전 %2°  (모드: %3)\n\n"
                        "현재 [X,Y,Z | R,P,Y]: %4\n목표 [X,Y,Z | R,P,Y]: %5\n\n"
                        "경로에 사람/장애물이 없는지 확인하세요. 실행할까요?")
                    .arg(dist, 0, 'f', 1).arg(ang, 0, 'f', 1).arg(modeName(mode))
                    .arg(vecStr(curRpy)).arg(vecStr(tgtRpy)),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (ans != QMessageBox::Yes) { log("MoveL 취소"); return; }
            // The dialog ran the event loop: the connection or robot state may have changed meanwhile.
            if (!robot_) return;
            if (!canMove(&why)) { log("MoveL 불가: " + why); return; }
            Pose now;
            if (!read_tcp(now)) { log("MoveL 불가: TCP 피드백이 최신이 아닙니다"); return; }
            const pm::Frame n = pm::frame_from_posx(now);
            if (pm::dist(n.p, cur.p) > 0.5 || pm::rot_angle_deg(n.R, cur.R) > 0.2) {
                log("MoveL 취소: 확인하는 동안 로봇 자세가 바뀌었습니다. 다시 실행하세요");
                return;
            }
        }

        // Controller posx = ZYZ; at B = 0/180 keep the current A so the numbers stay close to the TP.
        const pm::Vec6 px = pm::posx_from_frame(tgt, tcp[3]);
        Pose posx;
        for (int i = 0; i < 6; ++i) posx[i] = static_cast<float>(px[i]);
        const double v = linVel_->value(), a = linAcc_->value(), w = linRotVel_->value(), aw = linRotAcc_->value();
        float vel[2] = {static_cast<float>(v), static_cast<float>(w)};
        float acc[2] = {static_cast<float>(a), static_cast<float>(aw)};

        g_fault = false;
        if (!robot_->amovel(posx.data(), vel, acc, 0.f, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE)) {
            log("컨트롤러가 MoveL 명령을 거부했습니다");
            return;
        }
        moveKind_ = MoveKind::Linear;
        targetL_ = pm::frame_from_posx(posx);  // exactly what the controller received (float)
        maxSec_ = pm::trapezoid_time(dist, v, a) + pm::trapezoid_time(ang, w, aw) + 10.0;
        moveStart_ = Clock::now();
        idleSince_ = {};
        log(QString("이동 시작: MoveL %1 (%2 mm/s, %3°/s) 거리 %4 mm, 회전 %5° -> [X,Y,Z | R,P,Y] %6")
                .arg(modeName(mode)).arg(v).arg(w).arg(dist, 0, 'f', 1).arg(ang, 0, 'f', 1).arg(vecStr(tgtRpy)));
        log(QString("  amovel(posx %1, vel [%2, %3], acc [%4, %5], ABS, BASE)")
                .arg(vecStr(px)).arg(v).arg(w).arg(a).arg(aw));
        setPhase(Phase::Moving);
    }

    // Once per connection: the monitoring stream also carries the TCP rotation matrix. If it equals
    // Rz(A)*Ry(B)*Rz(C) of the reported posx, the ZYZ convention used for MoveL is confirmed on this
    // controller. Informational only (some firmware may leave the matrix empty).
    void checkRotationConvention() {
        Pose tcp;
        pm::Mat3 M{};
        double norm = 0;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            tcp = g_tcp;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) { M[r][c] = g_rotm[r][c]; norm += M[r][c] * M[r][c]; }
        }
        if (!(norm > 0.5)) return;  // not provided
        const pm::Mat3 R = pm::zyz_to_R(tcp[3], tcp[4], tcp[5]), Mt = pm::transpose(M);
        double e = 0, et = 0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                e = std::max(e, std::fabs(R[r][c] - M[r][c]));
                et = std::max(et, std::fabs(R[r][c] - Mt[r][c]));
            }
        if (std::min(e, et) < 1e-3)
            log("TCP 자세 규약 확인: 모니터링 회전행렬 = ZYZ(A,B,C) (MoveL RPY 변환 정상)");
        else
            log(QString("[주의] 모니터링 회전행렬이 ZYZ(A,B,C) 계산과 다릅니다 (오차 %1). "
                        "MoveL 회전은 작은 값으로 먼저 확인하세요").arg(std::min(e, et), 0, 'f', 4));
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
        checkRotationConvention();
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
        targetJ_ = target;
        maxSec_ = dist / vel + 2.0 * vel / acc + 10.0;
        g_fault = false;
        if (!robot_->amovej(targetJ_.data(), vel, acc, 0.f, MOVE_MODE_ABSOLUTE)) {
            log("컨트롤러가 이동 명령을 거부했습니다");
            return;
        }
        moveKind_ = MoveKind::Joint;
        moveStart_ = Clock::now();
        idleSince_ = {};
        log(QString("이동 시작: %1 (%2 deg/s) -> %3").arg(what).arg(vel).arg(poseStr(targetJ_)));
        setPhase(Phase::Moving);
    }

    void stopNow(const QString& reason) {
        if (!robot_) return;
        robot_->MoveStop(STOP_TYPE_QUICK);
        log("[정지] " + reason + " -> MoveStop(QUICK)");
        if (phase_ == Phase::Moving) setPhase(Phase::Ready);
    }

    // cur = joints [deg], tcp = TCP [mm, ZYZ deg]; both fresh (checked by tick()).
    void pollMotion(ROBOT_STATE st, const Pose& cur, const Pose& tcp) {
        const double el = std::chrono::duration<double>(Clock::now() - moveStart_).count();
        if (g_fault || is_fault(st)) { stopNow(QString("로봇 이상 상태 %1").arg(state_name(st))); return; }
        if (!g_access) { stopNow("제어권 상실"); return; }
        if (el > maxSec_) { stopNow(QString("워치독 시간 초과 (%1s)").arg(maxSec_, 0, 'f', 1)); return; }
        if (el < 0.2) return;

        if (robot_->check_motion() == 0 && st == STATE_STANDBY) {
            bool reached = true;
            QString where;
            if (moveKind_ == MoveKind::Joint) {
                for (int i = 0; i < 6; ++i) reached &= std::fabs(cur[i] - targetJ_[i]) <= JOINT_TOL_DEG;
                where = poseStr(cur);
            } else {
                const pm::Frame c = pm::frame_from_posx(tcp);
                const double pe = pm::dist(c.p, targetL_.p), re = pm::rot_angle_deg(c.R, targetL_.R);
                reached = pe <= LIN_TOL_MM && re <= LIN_TOL_DEG;
                where = QString("[X,Y,Z | R,P,Y] %1 (목표와 차이 %2 mm, %3°)")
                            .arg(vecStr(pm::xyzrpy_from_frame(c))).arg(pe, 0, 'f', 2).arg(re, 0, 'f', 2);
            }
            if (reached) {
                log(QString("이동 완료 (%1s): %2").arg(el, 0, 'f', 1).arg(where));
                setPhase(Phase::Ready);
                return;
            }
            if (idleSince_ == Clock::time_point{}) idleSince_ = Clock::now();
            if (Clock::now() - idleSince_ > std::chrono::milliseconds(1500)) {
                log("로봇이 목표 도달 전에 멈췄습니다: " + where);
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
            tcpFresh = fresh_locked();
            for (float v : tcp) tcpFresh = tcpFresh && std::isfinite(v);
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
        if (phase_ == Phase::Moving && (!jointsFresh || !tcpFresh)) {
            stopNow(jointsFresh ? "TCP 피드백 끊김" : "관절 피드백 끊김");
            refreshButtons(st);
            return;
        }
        if (tcpFresh) {
            const pm::Vec6 r = pm::xyzrpy_from_frame(pm::frame_from_posx(tcp));
            tcpLbl_->setText(QString("TCP  X %1  Y %2  Z %3 [mm]   Roll %4  Pitch %5  Yaw %6 [°]\n"
                                     "     (TP 표시 ZYZ  A %7  B %8  C %9 [°])")
                                 .arg(r[0], 8, 'f', 2).arg(r[1], 8, 'f', 2).arg(r[2], 8, 'f', 2)
                                 .arg(r[3], 8, 'f', 2).arg(r[4], 7, 'f', 2).arg(r[5], 8, 'f', 2)
                                 .arg(tcp[3], 0, 'f', 2).arg(tcp[4], 0, 'f', 2).arg(tcp[5], 0, 'f', 2));
        } else {
            tcpLbl_->setText("TCP: (모니터링 데이터 없음)");
        }
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
            pollMotion(st, cur, tcp);
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
        const bool fresh = robot_ && monitoring_fresh();
        const bool ready = fresh && phase_ == Phase::Ready && g_access && st == STATE_STANDBY;
        for (int j = 0; j < 6; ++j) { minus_[j]->setEnabled(ready); plus_[j]->setEnabled(ready); }
        homeBtn_->setEnabled(ready && haveHome_);
        setHomeBtn_->setEnabled(ready);
        linGoBtn_->setEnabled(ready);
        linLoadBtn_->setEnabled(fresh);
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
    // MoveL panel
    QDoubleSpinBox* lin_[6] = {};  // X, Y, Z [mm], Roll, Pitch, Yaw [deg]
    QDoubleSpinBox *linVel_, *linAcc_, *linRotVel_, *linRotAcc_;
    QRadioButton *linAbs_, *linRelBase_, *linRelTool_;
    QPushButton *linLoadBtn_, *linZeroBtn_, *linGoBtn_;
    QPlainTextEdit* log_;
    QTimer timer_;

    // robot
    std::unique_ptr<CDRFLEx> robot_;
    Phase phase_ = Phase::Off;
    MoveKind moveKind_ = MoveKind::Joint;
    Clock::time_point deadline_, moveStart_, idleSince_;
    Pose home_{}, targetJ_{};
    pm::Frame targetL_{};
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

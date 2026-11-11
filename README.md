# doosan-a0509-drfl

두산로보틱스 **A0509** 실물 로봇을 **ROS 없이** 공식 [API-DRFL](https://github.com/DoosanRobotics/API-DRFL)(C++)로 제어하는 예제 모음입니다.
Ubuntu 22.04 PC에서 테스트 프로그램, CLI 조그 메뉴, Qt GUI, Robotiq 2F-85 그리퍼 테스트를 제공합니다.

> ⚠️ **안전**: 실물 로봇을 움직이는 코드입니다. 항상 티치펜던트의 **비상정지 버튼을 손에 들고** 실행하세요.
> 프로그램의 소프트웨어 정지는 통신이 살아 있을 때만 동작하며 비상정지를 대체하지 않습니다.

## 검증 환경

| 항목 | 값 |
|---|---|
| PC | Ubuntu 22.04.5 LTS, x86_64 |
| 로봇 | A0509 |
| 컨트롤러 | GF02080100 (v2.8) → `DRCF_VERSION=2` |
| DRFL | GL013304 |
| 네트워크 | PC 192.168.137.2 / 공유기 192.168.137.1 / 컨트롤러 192.168.137.101:12345 |

## 구성

| 경로 | 내용 | 상태 |
|---|---|---|
| `tests/connect_test.cpp` | 연결 + 버전/상태/관절/TCP 읽기 (제어권·이동 없음) | ✅ 실기 확인 |
| `tests/move_test.cpp` | 제어권 → 자동 Servo ON → J6 +5° 왕복 | ✅ 실기 확인 |
| `tests/joint_test.cpp` | J1~J6를 하나씩 +5° 왕복 (관절마다 Enter 확인, 2°/s) | ✅ 실기 확인 |
| `a0509_menu/a0509_menu.cpp` | CLI 조그 메뉴 (1 연결 → 2 제어권 → 3 준비 → 관절 조작) | ✅ 실기 확인 |
| `a0509_gui/` | Qt5 조그 GUI (CMake) — 관절 조그 | ✅ 실기 확인 |
| `a0509_gui/` (MoveL 패널) | X/Y/Z + Roll/Pitch/Yaw 직선 이동 (절대 / 상대(base) / 상대(tool)) | ⚠️ 실기 미검증 (오프라인 테스트만 통과) |
| `gripper/gripper_test.cpp` | Robotiq 2F-85 (플랜지 RS-485, Modbus RTU) 활성화/열기/닫기 | ⚠️ 미검증 (그리퍼 응답 없음 조사 중) |
| `gripper/gripper_scan.cpp` | 그리퍼 통신 진단 (port/baud/slave 스캔, 상태 읽기만) | ⚠️ 미검증 |

## 설치

```bash
sudo apt install -y build-essential cmake qtbase5-dev   # qtbase5-dev는 GUI용 (ROS Humble 설치 시 이미 있음)
git clone https://github.com/srbae7115/doosan-a0509-drfl.git
cd doosan-a0509-drfl
scripts/setup_drfl.sh          # 공식 API-DRFL을 third_party/에 클론
scripts/build_all.sh 2         # 전체 빌드 (컨트롤러 3.x면 3)
```

- DRFL 저장소에 Ubuntu 버전별 `libDRFL.a`와 Poco 공유 라이브러리가 들어 있어 `libpoco-dev` 설치 없이 rpath로 링크합니다.
- 다른 위치의 API-DRFL을 쓰려면 `DRFL_ROOT=/path/to/API-DRFL scripts/build_all.sh`.
- 파일 하나만 빌드: `./build.sh tests/connect_test.cpp 2`

## 사용법

먼저 읽기 전용 연결 테스트로 통신을 확인합니다 (로봇이 움직이지 않음).

```bash
./tests/connect_test 192.168.137.101
```

### CLI 메뉴

```bash
./a0509_menu/a0509_menu                                       # 연습 모드 (네트워크 사용 안 함)
./a0509_menu/a0509_menu --ip 192.168.137.101 --execute        # 실기 (TP 팝업 승인)
./a0509_menu/a0509_menu --ip 192.168.137.101 --execute --force  # 실기 (제어권 강제 획득)
./a0509_menu/a0509_menu --self-test                           # 오프라인 자체 검사
```

메뉴에서 `1 → 2 → 3` 순서로 초기화한 뒤 조작합니다.

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| `4`/`5` | J1 +/− | `a`/`b` | J4 +/− |
| `6`/`7` | J2 +/− | `c`/`d` | J5 +/− |
| `8`/`9` | J3 +/− | `e`/`f` | J6 +/− |
| `h` | 원위치 복귀 | `o` | 현재 자세를 원위치로 재저장 |
| `0` | 상태 조회 (관절, TCP) | `s` | 정지 요청 |
| `x` | 연결 종료 | `q` | 프로그램 종료 |

- 3번: SAFE_OFF면 자동 Servo ON (비상정지/보호정지 상태면 거부), AUTONOMOUS 전환, 원위치 저장
- 이동 중 **아무 키** → 정지 후 메뉴 복귀, **Ctrl+C** → 정지 후 종료
- 속도 2°/s, 가속도 2°/s², 1회 이동량 기본 1° (`--step`, 최대 10°)
- 종료 시 이전 로봇 모드(MANUAL)로 복원

### Qt GUI

```bash
./a0509_gui/build/a0509_gui
```

- [연결] → TP 승인(또는 "강제 제어권 요청" 체크) → 자동 Servo ON → J1~J6 [−]/[+]
- **정지 버튼 / Esc / Space** → `MoveStop(QUICK)` (모든 버튼이 포커스를 받지 않아 Space가 이동 버튼을 누르지 않음.
  입력칸에 커서가 있어도 Esc / Space는 항상 정지로 동작)
- 비상정지·보호정지, 제어권 상실, 연결 끊김, 워치독, 관절(TCP) 피드백 끊김 시 자동 정지

#### MoveL (직선 이동, 오른쪽 패널)

| 입력 | 단위 | 설명 |
|---|---|---|
| X / Y / Z | mm | base 좌표계 TCP 위치 |
| Roll (Rx) / Pitch (Ry) / Yaw (Rz) | ° | RPY, `R = Rz(yaw)·Ry(pitch)·Rx(roll)` (고정축 X→Y→Z, ROS rpy와 동일) |
| 속도 v / 회전 | mm/s, °/s | `amovel` vel[0], vel[1] (기본 50 mm/s, 15 °/s) |
| 가속 a / 회전 | mm/s², °/s² | `amovel` acc[0], acc[1] (기본 100 mm/s², 30 °/s²) |

- **절대**: 입력값이 목표 자세 · **상대(base)**: base 축 방향으로 이동 / TCP 위치를 고정한 채 base 축 기준 회전 ·
  **상대(tool)**: 툴 축 기준 이동/회전 (예: 툴이 아래를 볼 때 Z +10 → 10 mm 하강)
- [현재값 불러오기] → 현재 TCP를 채우고 [절대]로 전환 → 일부만 고쳐서 [MoveL 실행]하는 방식을 권장
- [0으로] → 입력 0 (절대였으면 상대(base)로 전환). 기본 모드는 상대(base) + 0이라 그냥 눌러도 움직이지 않음
- 100 mm 또는 30°보다 큰 이동은 확인 창, 입력이 덜 끝난 칸(`-` 등)이 있으면 실행 거부, MoveL 입력칸은 마우스 휠로 바뀌지 않음
- TP와 컨트롤러는 자세를 **ZYZ 오일러 (A, B, C)** 로 표시합니다. GUI는 RPY를 입력받아 현재 TCP 기준으로 목표를 계산한 뒤
  `amovel(posx, ABS, BASE)`로 보냅니다. 상태 줄에 RPY와 TP 표시값(ZYZ)을 함께 보여 줍니다.
- 처음에는 **짧은 상대 이동(5~10 mm)**과 낮은 속도로 방향을 확인하세요. TP의 TCP(툴) 설정이 바뀌면 같은 관절 자세에서도 TCP 값이 달라집니다.
- 변환 계산은 로봇 없이 `a0509_gui/build/pose_math_test`로 검사합니다 (`scripts/build_all.sh`가 빌드 후 자동 실행).

### 그리퍼 (Robotiq 2F-85, 미검증)

```bash
./gripper/gripper_scan --force   # 통신 진단: 상태 읽기 프레임만 전송
./gripper/gripper_test --force   # a=활성화 o=열기 c=닫기 p=위치 s=상태
```

로봇 팔은 Servo ON/이동하지 않습니다. 활성화(`a`) 시 그리퍼가 스스로 완전히 열고 닫으니 손가락 사이를 비우세요.

## 알려진 문제 / 주의 사항

자세한 내용은 [docs/notes.md](docs/notes.md).

- 컨트롤러 **2.8에서 `get_current_posx()`가 응답 없이 멈춤** → 모니터링 콜백(`set_on_monitoring_data_ex`)으로 TCP를 읽음
- **연결 직후 `get_current_posj()`가 0을 반환** (첫 모니터링 데이터 수신 전) → 수신 대기 + 0.5초 신선도 검사
- 공식 예제는 `ROBOT_SYSTEM_VIRTUAL`, IP 127.0.0.1로 되어 있어 실물에서는 수정 필요
- 공식 `doosan-robot2` + MoveIt2는 **컨트롤러 2.12 이상** 필요 → 2.8에서는 사용 불가

## 라이선스

API-DRFL은 Doosan Robotics의 BSD 3-Clause 라이선스이며 이 저장소에 포함하지 않습니다 (`scripts/setup_drfl.sh`로 받음).

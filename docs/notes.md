# 실험 노트 / 트러블슈팅

컨트롤러 GF02080100 (v2.8), DRFL GL013304, Ubuntu 22.04 기준.

## 1. `DRCF_VERSION`

컨트롤러 버전 문자열이 `GF02...`이면 2, `GF03...`이면 3으로 빌드합니다. `connect_test`로 확인할 수 있습니다.

## 2. `get_current_posx()`가 멈춤

v2.8 컨트롤러에서 `get_current_posx()`가 반환되지 않고 무한 대기합니다 (`DRCF_VERSION` 2/3 모두).
TCP 좌표는 모니터링 콜백에서 읽습니다.

```cpp
robot.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX d) {
    // d->_tCtrl._tTask._fActualPos[0][0..5]  (0: tool, 1: flange) [mm, deg]
});
robot.open_connection(ip);
robot.setup_monitoring_version(1);
```

## 3. 연결 직후 관절값이 0

첫 모니터링 데이터가 도착하기 전에는 `get_current_posj()`가 `[0,0,0,0,0,0]`을 반환합니다.
이 값을 "원위치"로 저장하면 원위치 복귀 시 로봇이 모든 관절 0° 자세로 크게 움직일 수 있어 **위험**합니다.
→ 연결 후 모니터링 데이터 수신을 기다리고, 마지막 수신이 0.5초 이내일 때만 관절값을 사용합니다.

## 4. 공식 예제를 실물에 그대로 쓰지 말 것

`example/Linux_64/1_minimal_motion_sample.cpp` 등은 `set_robot_system(ROBOT_SYSTEM_VIRTUAL)`, IP `127.0.0.1`로 되어 있습니다.
실물에서는 `ROBOT_SYSTEM_REAL`과 컨트롤러 IP를 사용합니다.

## 5. 속도가 너무 느려 안 움직이는 것처럼 보임

`change_operation_speed(10)`(전체 속도 10%) × `movej` 3°/s = 실제 0.3°/s → 눈으로 거의 안 보입니다.
`change_operation_speed()`는 API 모션에도 적용되고 컨트롤러에 유지됩니다. 테스트 프로그램은 100%로 설정합니다.

## 6. 제어권

- `MANAGE_ACCESS_CONTROL_REQUEST`: TP에 승인 팝업
- `MANAGE_ACCESS_CONTROL_FORCE_REQUEST`: TP 승인 없이 획득 (`--force`, GUI 체크박스)

## 7. Servo ON

`set_robot_control(CONTROL_SERVO_ON)`은 SAFE_OFF 상태에서만 요청합니다.
비상정지(EMERGENCY_STOP)/보호정지(SAFE_STOP) 상태에서는 TP에서 원인을 먼저 해제해야 합니다.

## 8. MoveIt

공식 `doosan-robot2`(humble) README: *MoveIt2 requires Controller Version 2.12 or higher.*
v2.8에서 MoveIt을 쓰려면 A0509 URDF/MoveIt 설정만 사용하고, `FollowJointTrajectory`를 받아 DRFL `movesj`로 실행하는 브리지 노드를 직접 만들어야 합니다 (미구현).

DRFL만으로도 `movel`, `movejx`, `movec`, `movesj`, `movesx`, `moveb`, `ikin`/`fkin` 등을 사용할 수 있습니다 (v2.8 지원 여부는 개별 확인 필요).

## 9. Robotiq 2F-85 (플랜지 RS-485) — 조사 중

- `flange_serial_open(1, 115200)` 성공, 상태 읽기 프레임 `09 03 07 D0 00 03 04 0E` 송신 성공, **응답 없음**
- 확인할 것: 그리퍼 LED, TP의 플랜지 툴 전원(24V) 설정, TP Robotiq 플러그인이 버스를 점유하는지, A0509 플랜지/v2.8의 플랜지 시리얼 지원 여부
- `gripper/gripper_scan`으로 port/baud/slave 조합 진단

## 10. TCP 값 변화

관절은 거의 그대로인데 TCP Z가 284 → 448 mm로 바뀐 적이 있음 → TP의 TCP(툴) 설정 변경으로 추정.
`movel` 등 좌표 기반 모션 전에 TP에서 TCP 설정을 확인하세요.

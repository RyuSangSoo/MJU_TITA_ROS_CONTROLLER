# Tita UI

ROS2 환경에서 TiTa IMU와 Joint State를 모니터링하는 Qt 기반 UI입니다.

## 기능
- IMU 및 JointState 토픽 구독
- IMU orientation에서 roll/pitch/yaw 계산
- 모든 값을 `TitaState`에 저장
- IMU(roll/pitch/yaw), Joint 1/2/3(Left/Right) 그래프 표시
- Left/Right (Pos/Vel/Eff) + RPY 테이블 표시
- Control 섹션에서 Left/Right J1/J2/J3 목표 각도, 이동 시간, Kp/Kd(조인트별) 입력 후 PD 제어
- `std_msgs/msg/Float64MultiArray` 500Hz 퍼블리시
- ESC 키로 창 종료
- CSV 로깅 (세미콜론 `;` 구분자)

## 의존성
Qt5가 필요합니다. Ubuntu 기준:
```bash
sudo apt-get update
sudo apt-get install -y qtbase5-dev
```

## 토픽
기본값:
- IMU: `/imu_sensor_broadcaster/imu`
- Joint States: `/joint_states`
- Control: `/tita_hw/effort_controller/command`

파라미터로 변경 가능:
- `imu_topic`
- `joint_states_topic`
- `control_topic`

## 빌드
```bash
colcon build --packages-select tita_ui
```

## 실행
```bash
source install/setup.bash
ros2 run tita_ui tita_ui_node
```

### 토픽 변경 실행
```bash
ros2 run tita_ui tita_ui_node --ros-args \
  -p imu_topic:=/imu_sensor_broadcaster/imu \
  -p joint_states_topic:=/joint_states \
  -p control_topic:=/tita_hw/effort_controller/command
```

## 로깅
- `R` 키 또는 **Start Logging** 버튼으로 시작/중지 토글
- `;`(세미콜론) 구분 CSV
- 기본 저장 위치: 실행 경로의 `data/`
- 쓰기 불가 시: `~/tita_logs/`로 자동 저장
- UI에 실제 저장 경로 표시

## 조인트 매핑
다음 이름들이 `TitaState`에 매핑됩니다:
- `joint_left_leg_1` -> Left.J1
- `joint_left_leg_2` -> Left.J2
- `joint_left_leg_3` -> Left.J3
- `joint_left_leg_4` -> Left.Wheel
- `joint_right_leg_1` -> Right.J1
- `joint_right_leg_2` -> Right.J2
- `joint_right_leg_3` -> Right.J3
- `joint_right_leg_4` -> Right.Wheel

## Control 퍼블리시
- 토픽 타입: `std_msgs/msg/Float64MultiArray`
- 주기: 500Hz
- 데이터 순서: `[Left.J1, Left.J2, Left.J3, Right.J1, Right.J2, Right.J3]` (Effort)
- `Send` 버튼을 누르면 현재 위치에서 목표 각도로 이동하도록 PD 계산
- `Stop` 버튼을 누르면 effort가 0으로 출력됨
- Command 목표 각도/속도는 `Command.Pos`/`Command.Vel`에 저장되며 그래프에서 실측값과 비교 표시됨

## 참고
- UI 업데이트는 약 30Hz로 제한되어 부드럽게 표시됩니다.
- 기본 창 모드는 최대화(Fullscreen 아님)입니다.

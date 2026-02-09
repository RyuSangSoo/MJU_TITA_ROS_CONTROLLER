# Tita UI

ROS2 환경에서 TiTa IMU와 Joint State를 모니터링하는 Qt 기반 UI입니다.

## 기능
- IMU 및 JointState 토픽 구독
- IMU orientation에서 roll/pitch/yaw 계산
- 모든 값을 `TitaState`에 저장
- IMU(roll/pitch/yaw), Joint 1/2/3(Left/Right) 그래프 표시
- Left/Right (Pos/Vel/Eff) + RPY 테이블 표시
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

파라미터로 변경 가능:
- `imu_topic`
- `joint_states_topic`

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
  -p joint_states_topic:=/joint_states
```

## 로깅
- `R` 키 또는 **Start Logging** 버튼으로 시작/중지 토글
- `;`(세미콜론) 구분 CSV
- IMU 각속도(`angular_velocity_x/y/z`) 포함
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

## 참고
- UI 업데이트는 약 30Hz로 제한되어 부드럽게 표시됩니다.
- 기본 창 모드는 최대화(Fullscreen 아님)입니다.

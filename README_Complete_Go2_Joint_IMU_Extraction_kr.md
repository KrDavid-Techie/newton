# Newton Physics Engine용 Go2 4족 보행 로봇 관절 데이터 및 IMU 추출

이 저장소는 Newton Physics Engine의 4족 보행 로봇 시뮬레이션에서 관절 값과 IMU 데이터를 추출하기 위한 포괄적인 유틸리티와 예제를 제공하며, 특히 Go2 로봇(대표 예제로 ANYmal 사용)을 위해 설계되었습니다.

## 📋 개요

Newton Physics Engine은 복잡한 로봇 시스템을 지원하는 강력한 물리 시뮬레이션 프레임워크입니다. 이 솔루션은 세 가지 주요 구성 요소로 이루어집니다:

1. **`joint_data_extractor.py`** - 관절 데이터 및 IMU 센서 데이터 추출을 위한 핵심 유틸리티
2. **`go2_simulation_example.py`** - 실시간 데이터 추출이 포함된 완전한 시뮬레이션 예제
3. **생성된 JSON 파일들** - 제어 시스템을 위한 구조화된 관절 및 IMU 데이터

이 툴킷을 사용하면 다음을 수행할 수 있습니다:

  - **관절 데이터 추출**: 시뮬레이션에서 실시간 관절 위치 및 속도 가져오기
  - **IMU 데이터 수집**: 3축 가속도, 각속도, 방향 정보 추출
  - **JSON으로 변환**: 분석을 위해 관절 및 IMU 데이터를 구조화된 JSON 형식으로 변환
  - **로우레벨(Low-Level) 제어**: 로봇 제어 시스템 및 모션 계획을 위한 데이터 준비
  - **궤적 분석**: 전체 모션 시퀀스 기록 및 분석
  - **센서 융합**: IMU 데이터를 활용한 상태 추정 및 균형 제어

## 🚀 빠른 시작

### 사전 요구 사항

- Newton Physics Engine 설치
- Python 3.8+
- 필수 종속성: `warp`, `mujoco`, `numpy`

### 기본 사용법

```bash
# 10초 시뮬레이션 실행 및 관절 + IMU 데이터 추출
python go2_simulation_example.py --duration 10 --output_dir ./go2_data

# 성능을 위한 헤드리스 모드 실행
python go2_simulation_example.py --headless --duration 30 --fps 100

# 짧은 테스트 실행
python go2_simulation_example.py --duration 1 --headless --output_dir ./test_output
```

## 특징

### 관절 데이터 추출
  - ✅ **실시간 관절 위치 및 속도** 추출
  - ✅ 쉬운 데이터 교환을 위한 **JSON 직렬화**
  - ✅ **4족 보행 로봇 특화** 관절 매핑 (다리, 베이스)
  - ✅ **궤적 기록** 및 분석
  - ✅ **관절 한계** 추출
  - ✅ **유연한 관절 매핑** (다양한 로봇 구성에 적응)

### IMU 데이터 수집
  - ✅ **선형 가속도**: 중력 보상이 포함된 3축 가속도
  - ✅ **각속도**: 3축 자이로스코프 데이터 (rad/s)
  - ✅ **방향**: 쿼터니언 표현 및 오일러 각도
  - ✅ **고주파 데이터**: 시뮬레이션 타임스텝 주파수로 제공
  - ✅ **중력 보상**: 현실적인 가속도 측정
  - ✅ **수치 미분**: 가속도 계산

### 성능 특징
  - ✅ **CUDA 가속**: 사용 가능한 경우 자동 GPU 가속 지원
  - ✅ **메모리 관리**: 메모리 오버플로우 방지를 위한 구성 가능한 히스토리 크기
  - ✅ **효율적인 직렬화**: Warp의 효율적인 배열 변환 사용
  - ✅ **배치 처리**: 여러 시뮬레이션 프레임에서 데이터 추출 가능

## 📊 생성된 데이터 구조

### 1. IMU가 포함된 관절 궤적 (`go2_joint_trajectory.json`)

시간에 따른 관절 위치, 속도 및 IMU 데이터가 포함된 완전한 시뮬레이션 데이터:

```json
{
  "metadata": {
    "robot_type": "Go2_quadruped",
    "frame_count": 50,
    "duration": 1.0,
    "fps": 50,
    "joint_names": ["floating_base", "LF_HAA", "LF_HFE", "LF_KFE", ...],
    "includes_imu_data": true,
    "simulation_parameters": {
      "timestep": 0.005,
      "substeps": 4
    }
  },
  "trajectory": [
    {
      "timestamp": 0.02,
      "joints": {
        "positions": [0.0, 0.0, 0.62, 0.0, 0.0, 0.0, 1.0, ...],
        "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...],
        "names": ["floating_base", "LF_HAA", "LF_HFE", ...]
      },
      "joints_by_name": {
        "floating_base": {
          "position": {
            "translation": [0.0, 0.0, 0.62],
            "rotation_quaternion": [0.0, 0.0, 0.0, 1.0]
          },
          "velocity": {
            "linear": [0.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0]
          }
        },
        "LF_HAA": {"position": 0.0, "velocity": 0.0},
        "LF_HFE": {"position": 0.4, "velocity": 0.0},
        "LF_KFE": {"position": -0.8, "velocity": 0.0}
      },
      "imu": {
        "timestamp": 0.02,
        "linear_acceleration": [0.0, 0.0, -9.81],
        "angular_velocity": [-0.0001, -0.0106, 0.0479],
        "orientation": {
          "quaternion": [-1.54e-07, -8.66e-05, 0.000378, 0.999999],
          "euler_angles": [-3.74e-07, -0.000173, 0.000757]
        }
      }
    }
  ]
}
```

### 2. IMU 데이터만 (`go2_imu_data.json`)

센서 융합 및 상태 추정을 위한 전용 IMU 센서 데이터:

```json
{
  "metadata": { 
    "robot_type": "Go2_quadruped",
    "includes_imu_data": true,
    "frame_count": 50,
    "duration": 1.0
  },
  "imu_data": [
    {
      "timestamp": 0.02,
      "imu": {
        "linear_acceleration": [0.12, -0.05, 9.75],
        "angular_velocity": [0.001, -0.010, 0.048],
        "orientation": {
          "quaternion": [0.0, 0.0, 0.0, 1.0],
          "euler_angles": [0.0, 0.0, 0.0]
        }
      }
    }
  ]
}
```

### 3. 다리 각도 (`go2_leg_angles.json`)

더 쉬운 4족 보행 로봇 제어를 위해 다리별로 구성된 관절 데이터:

```json
{
  "metadata": { ... },
  "leg_data": [
    {
      "timestamp": 0.02,
      "legs": {
        "front_left": {
          "LF_HAA": {"position": 0.0, "velocity": 0.0},
          "LF_HFE": {"position": 0.4, "velocity": 0.0},
          "LF_KFE": {"position": -0.8, "velocity": 0.0}
        },
        "front_right": {
          "RF_HAA": {"position": 0.0, "velocity": 0.0},
          "RF_HFE": {"position": 0.4, "velocity": 0.0},
          "RF_KFE": {"position": -0.8, "velocity": 0.0}
        },
        "rear_left": {
          "LH_HAA": {"position": 0.0, "velocity": 0.0},
          "LH_HFE": {"position": -0.4, "velocity": 0.0},
          "LH_KFE": {"position": 0.8, "velocity": 0.0}
        },
        "rear_right": {
          "RH_HAA": {"position": 0.0, "velocity": 0.0},
          "RH_HFE": {"position": -0.4, "velocity": 0.0},
          "RH_KFE": {"position": 0.8, "velocity": 0.0}
        },
        "floating_base": {
          "position": {
            "translation": [0.0, 0.0, 0.62],
            "rotation_quaternion": [0.0, 0.0, 0.0, 1.0]
          },
          "velocity": {
            "linear": [0.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0]
          }
        }
      }
    }
  ]
}
```

### 4. 관절 한계 (`go2_joint_limits.json`)

제어 시스템을 위한 관절 제약:

```json
{
  "joint_names": ["floating_base", "LF_HAA", "LF_HFE", ...],
  "limits": {
    "lower": [-1000000.0, -1000000.0, ..., -0.72, ...],
    "upper": [1000000.0, 1000000.0, ..., 0.49, ...]
  }
}
```

## 🔧 통합 예제

### IMU가 포함된 관절 데이터 추출기 사용

```python
from joint_data_extractor import JointDataExtractor

# Newton 모델로 추출기 초기화 (메인 바디 IMU를 위해 base_body_index=0)
extractor = JointDataExtractor(model, base_body_index=0)

# IMU 데이터가 포함된 현재 관절 상태 추출
joint_data = extractor.extract_joint_state(current_state, sim_time, include_imu=True)

# IMU 데이터만 추출
imu_data = extractor.extract_imu_data(current_state, sim_time)

# JSON으로 저장
extractor.save_to_json(joint_data, "robot_data.json")

# 특정 관절 값 접근
joints_by_name = joint_data["joints_by_name"]
for joint_name, joint_info in joints_by_name.items():
    position = joint_info.get("position")
    velocity = joint_info.get("velocity")
    print(f"{joint_name}: pos={position}, vel={velocity}")
```

### IMU 데이터 사용

```python
# 추출된 상태에서 IMU 데이터 접근
imu = joint_data["imu"]

# 선형 가속도 (m/s²) - 중력 보상 포함
linear_accel = imu["linear_acceleration"]  # [ax, ay, az]

# 각속도 (rad/s)
angular_vel = imu["angular_velocity"]      # [wx, wy, wz]

# 방향
quaternion = imu["orientation"]["quaternion"]      # [qx, qy, qz, qw]
euler_angles = imu["orientation"]["euler_angles"]  # [roll, pitch, yaw] in radians
```

### IMU가 포함된 ROS 통합

```python
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3, Quaternion

def convert_to_ros_messages(joint_data):
    # 관절 상태 메시지
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_data["joints"]["names"]
    joint_state.position = joint_data["joints"]["positions"]
    joint_state.velocity = joint_data["joints"]["velocities"]
    
    # IMU 메시지
    imu_msg = Imu()
    imu_msg.header.stamp = joint_state.header.stamp
    imu_msg.header.frame_id = "base_link"
    
    imu_data = joint_data["imu"]
    
    # 선형 가속도
    imu_msg.linear_acceleration = Vector3(*imu_data["linear_acceleration"])
    
    # 각속도
    imu_msg.angular_velocity = Vector3(*imu_data["angular_velocity"])
    
    # 방향
    quat = imu_data["orientation"]["quaternion"]
    imu_msg.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    
    return joint_state, imu_msg
```

### IMU를 활용한 실시간 제어

```python
# 실시간 관절 및 IMU 모니터링
simulation = Go2SimulationExample(viewer=None, enable_visualization=False)

for step in range(1000):
    simulation.step(extract_data=True)
    
    # 현재 데이터 가져오기
    joint_data = simulation.get_current_joint_data(include_imu=True)
    imu_data = simulation.get_current_imu_data()
    leg_angles = simulation.get_leg_angles()
    
    # IMU 기반 상태 추정
    orientation = imu_data["orientation"]["euler_angles"]
    angular_velocity = imu_data["angular_velocity"]
    linear_acceleration = imu_data["linear_acceleration"]
    
    # 여기에 제어 알고리즘 적용
    control_commands = your_controller(leg_angles, imu_data)
    
    # 명령 적용
    apply_joint_commands(control_commands)
```

### Go2 시뮬레이션 예제 실행

```bash
# 기본 시뮬레이션 (10초, 시각화 포함)
python go2_simulation_example.py

# 사용자 정의 지속 시간 및 출력 디렉토리
python go2_simulation_example.py --duration 20 --output_dir ./my_robot_data

# 헤드리스 모드 (시각화 없음)
python go2_simulation_example.py --headless --duration 30

# 사용자 정의 프레임 속도
python go2_simulation_example.py --fps 60 --duration 15
```

## 🎛️ 설정 옵션

### 시뮬레이션 매개변수

  - `--duration SECONDS` - 시뮬레이션 지속 시간 (기본값: 10초)
  - `--output_dir DIR` - JSON 파일을 위한 출력 디렉토리 (기본값: ./go2\_data)
  - `--headless` - 시각화 없이 실행
  - `--fps FPS` - 시뮬레이션 프레임 속도 (기본값: 50 FPS)

### IMU 설정

```python
# IMU를 위한 특정 베이스 바디로 초기화
extractor = JointDataExtractor(model, base_body_index=0)  # 0 = 메인 바디

# IMU 데이터 포함 또는 제외하여 추출
joint_data = extractor.extract_joint_state(state, timestamp, include_imu=True)
```

### 솔버 설정

MuJoCo 솔버는 다음과 같이 구성됩니다:
- `njmax=600`: 오버플로우 경고 방지를 위한 제약 버퍼 증가
- `iterations=100`: 정확도를 위한 솔버 반복
- `cone=mjCONE_ELLIPTIC`: 접촉 원뿔 모델

## 📝 데이터 형식 세부사항

### 관절 명명 규칙

시스템은 ANYmal 명명 규칙을 사용합니다 (다른 로봇에도 적응 가능):

- **플로팅 베이스**: `floating_base` (6-DOF: 3 병진 + 3 회전)
- **앞 왼쪽**: `LF_HAA`, `LF_HFE`, `LF_KFE` (고관절 외전, 고관절 굴곡, 무릎)
- **앞 오른쪽**: `RF_HAA`, `RF_HFE`, `RF_KFE`
- **뒤 왼쪽**: `LH_HAA`, `LH_HFE`, `LH_KFE`
- **뒤 오른쪽**: `RH_HAA`, `RH_HFE`, `RH_KFE`

### IMU 좌표계

- **X축**: 전진 (로봇 앞쪽)
- **Y축**: 좌측 (로봇 왼쪽)  
- **Z축**: 위쪽 (로봇 상단)
- **중력**: -Z 방향으로 적용 (-9.81 m/s²)

### 쿼터니언 규칙

- 형식: [x, y, z, w] (스칼라-마지막)
- 월드에서 바디 프레임으로의 회전을 나타내는 단위 쿼터니언

### 오일러 각도 규칙

- 순서: 롤 (X), 피치 (Y), 요 (Z)
- 범위: [-π, π] 라디안
- ZYX 내재적 회전 순서

## 🔍 문제 해결

### 일반적인 문제

1. **"RuntimeError: Item indexing is not supported"**: 적절한 warp 배열 처리로 수정됨
2. **"nefc overflow" 경고**: 솔버 구성에서 `njmax` 매개변수 증가
3. **IMU 데이터 누락**: 관절 상태를 추출할 때 `include_imu=True` 확인
4. **CUDA 메모리 오류**: 시뮬레이션 지속 시간 줄이기 또는 CUDA 그래프 비활성화
5. **가져오기 오류**: 환경에 모든 종속성이 설치되어 있는지 확인

### 성능 팁

- 배치 처리를 위해 `--headless` 모드 사용
- 더 긴 시뮬레이션을 위해 `--fps` 줄이기
- IMU 데이터는 최소한의 계산 오버헤드 추가
- 메모리 효율성을 위해 궤적 히스토리 크기 제한

## 💡 고급 사용법

### 사용자 정의 관절 매핑

```python
# 로봇을 위한 사용자 정의 관절 매핑 정의
custom_mapping = {
    "front_left": ["FL_hip", "FL_thigh", "FL_calf"],
    "front_right": ["FR_hip", "FR_thigh", "FR_calf"],
    "rear_left": ["RL_hip", "RL_thigh", "RL_calf"],
    "rear_right": ["RR_hip", "RR_thigh", "RR_calf"],
    "floating_base": ["floating_base"]
}

# 추출기와 함께 사용
leg_angles = extract_quadruped_leg_angles(joint_data, custom_mapping)
```

### 궤적 분석

```python
# 시뮬레이션 중 여러 상태 수집
states = []
timestamps = []

for frame in range(num_frames):
    simulation.step()
    states.append(simulation.state_0)
    timestamps.append(simulation.sim_time)

# 전체 궤적 추출
extractor = JointDataExtractor(model)
trajectory_data = extractor.extract_trajectory(states, timestamps)

# 궤적 저장
extractor.save_to_json(trajectory_data, "robot_trajectory.json")
```

### 실시간 관절 모니터링

```python
# 시뮬레이션 중 특정 관절 모니터링
def monitor_joints(simulation):
    joint_data = simulation.get_current_joint_data()
    leg_angles = simulation.get_leg_angles()
    
    # 앞 왼쪽 다리 관절 각도 출력
    if "front_left" in leg_angles:
        fl_leg = leg_angles["front_left"]
        print(f"앞 왼쪽 - 고관절: {fl_leg.get('LF_HAA', {}).get('position', 0):.3f}")
        print(f"앞 왼쪽 - 무릎: {fl_leg.get('LF_KFE', {}).get('position', 0):.3f}")

# 시뮬레이션 루프에서 사용
for frame in range(total_frames):
    simulation.step()
    if frame % 50 == 0:  # 50 FPS에서 매초
        monitor_joints(simulation)
```

## 🌟 적용 분야

이 툴킷은 다양한 적용을 가능하게 합니다:

- **로봇 제어**: 실시간 제어 알고리즘을 위한 관절 상태 추출
- **모션 분석**: 보행 패턴 및 관절 궤적 분석
- **머신 러닝**: RL/IL 알고리즘을 위한 훈련 데이터 생성
- **시스템 통합**: Newton 시뮬레이션을 외부 제어 시스템과 연동
- **연구**: 4족 보행 로봇의 이동 및 동역학 연구
- **상태 추정**: 센서 융합 및 자세 추정을 위한 IMU 데이터 사용
- **균형 제어**: IMU 피드백을 사용한 균형 및 안정성 제어기 구현

## 🔮 향후 개선 사항

- 다중 IMU 센서 지원
- 자력계 시뮬레이션
- 접촉력 센싱 통합
- 제어 시스템으로의 실시간 스트리밍
- 센서 노이즈 모델링
- 추가 로봇 형식 지원 (MJCF, SDF)
- 인기 로보틱스 프레임워크와의 통합
- 힘/토크 데이터 추출

## 🤝 기여하기

기여를 환영합니다! 버그 및 기능 요청에 대해 풀 리퀘스트를 제출하거나 이슈를 열어주세요.

이 툴킷을 확장하려면:

1. **새 로봇 모델 추가**: `_setup_robot()`의 로봇 로딩 로직 확장
2. **사용자 정의 관절 매핑**: 새 로봇을 위한 관절 매핑 함수 수정
3. **추가 데이터**: 힘, 토크, 접촉을 포함하도록 추출 확장
4. **내보내기 형식**: 다른 데이터 형식 (CSV, HDF5 등) 지원 추가

## 📄 라이선스

이 프로젝트는 Apache License 2.0에 따라 라이선스가 부여됩니다 - 자세한 내용은 LICENSE 파일을 참조하세요.

---

**참고**: 이 구현은 ANYmal 로봇을 대표적인 4족 보행 로봇으로 사용합니다. 실제 Go2 로봇 시뮬레이션을 위해서는 적절한 Go2 URDF/USD 파일이 사용 가능할 때 에셋 로딩 섹션을 교체하세요.

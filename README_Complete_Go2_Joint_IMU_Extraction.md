# Go2 Quadruped Robot Joint Data and IMU Extraction with Newton Physics Engine

This repository contains comprehensive utilities and examples for extracting joint values and IMU data from quadruped robot simulations using the Newton Physics Engine and converting them to JSON format for low-level control systems.

## 📋 Overview

The solution consists of three main components:

1. **`joint_data_extractor.py`** - Core utility for extracting joint data and IMU sensor data
2. **`go2_simulation_example.py`** - Complete simulation example with real-time data extraction
3. **Generated JSON files** - Structured joint and IMU data for control systems

## 🚀 Quick Start

### Prerequisites

- Newton Physics Engine installed
- Python 3.8+
- Required dependencies: `warp`, `mujoco`, `numpy`

### Basic Usage

```bash
# Run a 10-second simulation and extract joint + IMU data
python go2_simulation_example.py --duration 10 --output_dir ./go2_data

# Run in headless mode for performance
python go2_simulation_example.py --headless --duration 30 --fps 100

# Short test run
python go2_simulation_example.py --duration 1 --headless --output_dir ./test_output
```

## 📊 Generated Data Structure

### 1. Joint Trajectory with IMU (`go2_joint_trajectory.json`)

Complete simulation data with joint positions, velocities, and IMU data over time:

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

### 2. IMU Data Only (`go2_imu_data.json`)

Dedicated IMU sensor data for sensor fusion and state estimation:

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

### 3. Leg Angles (`go2_leg_angles.json`)

Joint data organized by leg for easier quadruped control:

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

### 4. Joint Limits (`go2_joint_limits.json`)

Joint constraints for control systems:

```json
{
  "joint_names": ["floating_base", "LF_HAA", "LF_HFE", ...],
  "limits": {
    "lower": [-1000000.0, -1000000.0, ..., -0.72, ...],
    "upper": [1000000.0, 1000000.0, ..., 0.49, ...]
  }
}
```

## 🔧 Integration Examples

### Using the Joint Data Extractor with IMU

```python
from joint_data_extractor import JointDataExtractor, create_quadruped_joint_mapping

# Initialize with your Newton model (base_body_index=0 for main body IMU)
extractor = JointDataExtractor(model, base_body_index=0)

# Extract current joint state with IMU data
joint_data = extractor.extract_joint_state(current_state, sim_time, include_imu=True)

# Extract IMU data only
imu_data = extractor.extract_imu_data(current_state, sim_time)

# Save to JSON
extractor.save_to_json(joint_data, "robot_data.json")

# Get quadruped-specific leg angles
mapping = create_quadruped_joint_mapping()
leg_angles = extract_quadruped_leg_angles(joint_data, mapping)
```

### IMU Data Usage

```python
# Access IMU data from extracted state
imu = joint_data["imu"]

# Linear acceleration (m/s²) - includes gravity compensation
linear_accel = imu["linear_acceleration"]  # [ax, ay, az]

# Angular velocity (rad/s)
angular_vel = imu["angular_velocity"]      # [wx, wy, wz]

# Orientation
quaternion = imu["orientation"]["quaternion"]      # [qx, qy, qz, qw]
euler_angles = imu["orientation"]["euler_angles"]  # [roll, pitch, yaw] in radians
```

### ROS Integration with IMU

```python
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3, Quaternion

def convert_to_ros_messages(joint_data):
    # Joint state message
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_data["joints"]["names"]
    joint_state.position = joint_data["joints"]["positions"]
    joint_state.velocity = joint_data["joints"]["velocities"]
    
    # IMU message
    imu_msg = Imu()
    imu_msg.header.stamp = joint_state.header.stamp
    imu_msg.header.frame_id = "base_link"
    
    imu_data = joint_data["imu"]
    
    # Linear acceleration
    imu_msg.linear_acceleration = Vector3(*imu_data["linear_acceleration"])
    
    # Angular velocity  
    imu_msg.angular_velocity = Vector3(*imu_data["angular_velocity"])
    
    # Orientation
    quat = imu_data["orientation"]["quaternion"]
    imu_msg.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    
    return joint_state, imu_msg
```

### Real-time Control with IMU

```python
# Real-time joint and IMU monitoring
simulation = Go2SimulationExample(viewer=None, enable_visualization=False)

for step in range(1000):
    simulation.step(extract_data=True)
    
    # Get current data
    joint_data = simulation.get_current_joint_data(include_imu=True)
    imu_data = simulation.get_current_imu_data()
    leg_angles = simulation.get_leg_angles()
    
    # IMU-based state estimation
    orientation = imu_data["orientation"]["euler_angles"]
    angular_velocity = imu_data["angular_velocity"]
    linear_acceleration = imu_data["linear_acceleration"]
    
    # Your control algorithm here
    control_commands = your_controller(leg_angles, imu_data)
    
    # Apply commands
    apply_joint_commands(control_commands)
```

## 📈 Key Features

### Joint Data Extraction
- ✅ **Real-time joint positions and velocities** extraction
- ✅ **JSON serialization** for easy data exchange with control systems
- ✅ **Quadruped-specific organization** by legs and joint types
- ✅ **Trajectory recording** for motion analysis
- ✅ **Joint limits extraction** for control constraints
- ✅ **Flexible joint mapping** that adapts to different robot configurations

### IMU Data Collection
- ✅ **Linear Acceleration**: 3-axis acceleration with gravity compensation
- ✅ **Angular Velocity**: 3-axis gyroscope data (rad/s)
- ✅ **Orientation**: Quaternion representation and Euler angles
- ✅ **High-frequency data** available at simulation timestep frequency
- ✅ **Gravity compensation** for realistic acceleration measurements
- ✅ **Numerical differentiation** for acceleration calculation

### Performance Features
- ✅ **CUDA Acceleration**: Automatic GPU acceleration when available
- ✅ **Memory Management**: Configurable history size to prevent memory overflow
- ✅ **Efficient Serialization**: Uses Warp's efficient array conversion
- ✅ **Batch Processing**: Can extract data from multiple simulation frames

## 🎛️ Configuration Options

### Simulation Parameters

- `--duration`: Simulation duration in seconds (default: 10.0)
- `--fps`: Simulation framerate (default: 50)
- `--headless`: Run without visualization for performance
- `--output_dir`: Directory for generated JSON files

### IMU Configuration

```python
# Initialize with specific base body for IMU
extractor = JointDataExtractor(model, base_body_index=0)  # 0 = main body

# Extract with or without IMU data
joint_data = extractor.extract_joint_state(state, timestamp, include_imu=True)
```

### Solver Configuration

The MuJoCo solver is configured with:
- `njmax=600`: Increased constraint buffer to prevent overflow warnings
- `iterations=100`: Solver iterations for accuracy
- `cone=mjCONE_ELLIPTIC`: Contact cone model

## 🔍 Troubleshooting

### Common Issues

1. **"RuntimeError: Item indexing is not supported"**: Fixed with proper warp array handling
2. **"nefc overflow" warnings**: Increase `njmax` parameter in solver configuration
3. **Missing IMU data**: Ensure `include_imu=True` when extracting joint state
4. **CUDA memory errors**: Reduce simulation duration or disable CUDA graph
5. **Import errors**: Ensure all dependencies are installed in your environment

### Performance Tips

- Use `--headless` mode for batch processing
- Reduce `--fps` for longer simulations
- IMU data adds minimal computational overhead
- Limit trajectory history size for memory efficiency

## 📝 Data Format Details

### Joint Naming Convention

The system uses ANYmal naming convention (adaptable to other robots):

- **Floating Base**: `floating_base` (6-DOF: 3 translation + 3 rotation)
- **Front Left**: `LF_HAA`, `LF_HFE`, `LF_KFE` (Hip Abduction, Hip Flexion, Knee)
- **Front Right**: `RF_HAA`, `RF_HFE`, `RF_KFE`
- **Rear Left**: `LH_HAA`, `LH_HFE`, `LH_KFE`
- **Rear Right**: `RH_HAA`, `RH_HFE`, `RH_KFE`

### IMU Coordinate System

- **X-axis**: Forward (robot front)
- **Y-axis**: Left (robot left side)  
- **Z-axis**: Up (robot top)
- **Gravity**: Applied in -Z direction (-9.81 m/s²)

### Quaternion Convention

- Format: [x, y, z, w] (scalar-last)
- Unit quaternion representing rotation from world to body frame

### Euler Angles Convention

- Order: Roll (X), Pitch (Y), Yaw (Z)
- Range: [-π, π] radians
- ZYX intrinsic rotation order

## 🤖 Supported Robot Types

While designed for Go2, the system works with any quadruped robot by:

1. Adapting the joint mapping in `_adapt_joint_mapping()`
2. Modifying initial pose in `_set_initial_pose()`
3. Updating the URDF/USD asset loading
4. Configuring the base body index for IMU

## 💡 Advanced Usage

### Custom Joint Mapping

```python
# Define custom joint mapping for your robot
custom_mapping = {
    "front_left": ["FL_hip", "FL_thigh", "FL_calf"],
    "front_right": ["FR_hip", "FR_thigh", "FR_calf"],
    "rear_left": ["RL_hip", "RL_thigh", "RL_calf"],
    "rear_right": ["RR_hip", "RR_thigh", "RR_calf"],
    "floating_base": ["floating_base"]
}

# Use with extractor
leg_angles = extract_quadruped_leg_angles(joint_data, custom_mapping)
```

### Trajectory Analysis

```python
# Collect multiple states during simulation
states = []
timestamps = []

for frame in range(num_frames):
    simulation.step()
    states.append(simulation.state_0)
    timestamps.append(simulation.sim_time)

# Extract complete trajectory
extractor = JointDataExtractor(model)
trajectory_data = extractor.extract_trajectory(states, timestamps)

# Save trajectory
extractor.save_to_json(trajectory_data, "robot_trajectory.json")
```

### Real-time Joint Monitoring

```python
# Monitor specific joints during simulation
def monitor_joints(simulation):
    joint_data = simulation.get_current_joint_data()
    leg_angles = simulation.get_leg_angles()
    
    # Print front left leg joint angles
    if "front_left" in leg_angles:
        fl_leg = leg_angles["front_left"]
        print(f"Front Left - Hip: {fl_leg.get('LF_HAA', {}).get('position', 0):.3f}")
        print(f"Front Left - Knee: {fl_leg.get('LF_KFE', {}).get('position', 0):.3f}")

# Use in simulation loop
for frame in range(total_frames):
    simulation.step()
    if frame % 50 == 0:  # Every second at 50 FPS
        monitor_joints(simulation)
```

## 🌟 Applications

This toolkit enables various applications:

- **Robot Control**: Extract joint states for real-time control algorithms
- **Motion Analysis**: Analyze gait patterns and joint trajectories
- **Machine Learning**: Generate training data for RL/IL algorithms
- **System Integration**: Bridge Newton simulations with external control systems
- **Research**: Study locomotion and dynamics of quadruped robots
- **State Estimation**: Use IMU data for sensor fusion and pose estimation
- **Balance Control**: Implement balance and stability controllers using IMU feedback

## 🔮 Future Enhancements

- Support for multiple IMU sensors
- Magnetometer simulation
- Contact force sensing integration
- Real-time streaming to control systems
- Sensor noise modeling
- Support for additional robot formats (MJCF, SDF)
- Integration with popular robotics frameworks
- Force/torque data extraction

## 📄 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

To extend this toolkit:

1. **Add new robot models**: Extend robot loading logic in `_setup_robot()`
2. **Custom joint mappings**: Modify joint mapping functions for new robots
3. **Additional data**: Extend extraction to include forces, torques, contacts
4. **Export formats**: Add support for other data formats (CSV, HDF5, etc.)

---

**Note**: This implementation uses ANYmal robot as a representative quadruped. For actual Go2 robot simulation, replace the asset loading section with appropriate Go2 URDF/USD files when available.
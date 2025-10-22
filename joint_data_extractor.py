#!/usr/bin/env python3
"""
Joint Data Extractor for Newton Physics Engine

This module provides utilities to extract joint values (positions, velocities, names)
and IMU data from Newton Physics Engine simulations and convert them to JSON format.

The extracted data can be used for:
- Low-level robot control 
- Motion analysis
- Data logging and replay
- Integration with other control systems
- IMU sensor fusion and state estimation

Author: Newton Physics Research
"""

import json
import numpy as np
import warp as wp
from typing import Dict, List, Optional, Union
from pathlib import Path
import newton


class JointDataExtractor:
    """
    Utility class to extract and convert joint data and IMU data from Newton simulations to JSON format.
    
    This class provides methods to:
    - Extract joint positions and velocities from simulation states
    - Extract IMU data (acceleration, angular velocity, orientation) from base body
    - Map joint indices to joint names
    - Convert data to JSON format with timestamps
    - Save/load joint trajectories and IMU data
    """
    
    def __init__(self, model: newton.Model, base_body_index: int = 0):
        """
        Initialize the joint data extractor.
        
        Args:
            model (newton.Model): The Newton physics model containing joint information
            base_body_index (int): Index of the base body for IMU data extraction (default: 0)
        """
        self.model = model
        self.joint_names = model.joint_key if hasattr(model, 'joint_key') else []
        self.joint_count = len(self.joint_names)
        self.joint_dof_count = model.joint_dof_count if hasattr(model, 'joint_dof_count') else 0
        self.base_body_index = base_body_index
        
        # Create mapping from joint names to indices
        self.joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}
        
        # IMU data storage for calculating acceleration
        self.previous_state = None
        self.previous_timestamp = None
        
        print(f"Initialized JointDataExtractor for model with {self.joint_count} joints")
        print(f"Joint names: {self.joint_names}")
        print(f"Base body index for IMU: {self.base_body_index}")
    
    def extract_joint_state(self, state: newton.State, timestamp: float = None, include_imu: bool = True) -> Dict:
        """
        Extract current joint positions and velocities from a simulation state.
        
        Args:
            state (newton.State): Current simulation state
            timestamp (float, optional): Timestamp for this state snapshot
            include_imu (bool): Whether to include IMU data extraction
            
        Returns:
            Dict: Joint data containing positions, velocities, names, metadata, and optionally IMU data
        """
        # Extract joint positions and velocities
        joint_positions = []
        joint_velocities = []
        
        if state.joint_q is not None:
            # Convert warp array to numpy and then to list for JSON serialization
            joint_q_np = state.joint_q.numpy()
            joint_positions = joint_q_np.tolist()
        
        if state.joint_qd is not None:
            # Convert warp array to numpy and then to list for JSON serialization  
            joint_qd_np = state.joint_qd.numpy()
            joint_velocities = joint_qd_np.tolist()
        
        # Create structured joint data
        joint_data = {
            "timestamp": timestamp,
            "metadata": {
                "joint_count": self.joint_count,
                "joint_coord_count": len(joint_positions),
                "joint_dof_count": len(joint_velocities),
                "joint_names": self.joint_names,
                "includes_imu": include_imu
            },
            "joints": {
                "positions": joint_positions,
                "velocities": joint_velocities,
                "names": self.joint_names
            }
        }
        
        # Add IMU data if requested
        if include_imu:
            imu_data = self.extract_imu_data(state, timestamp)
            joint_data["imu"] = imu_data
        
        # Add individual joint data for easier access
        joint_data["joints_by_name"] = {}
        
        # Map positions by joint name (handling floating base separately)
        for i, joint_name in enumerate(self.joint_names):
            joint_info = {"name": joint_name}
            
            # For floating base joints, positions include translation + quaternion (7 values)
            # For regular joints, typically 1 value per joint
            if joint_name == "floating_base" and len(joint_positions) >= 7:
                joint_info["position"] = {
                    "translation": joint_positions[0:3],  # x, y, z
                    "rotation_quaternion": joint_positions[3:7]  # qx, qy, qz, qw
                }
                joint_info["velocity"] = {
                    "linear": joint_velocities[0:3] if len(joint_velocities) >= 6 else [0, 0, 0],
                    "angular": joint_velocities[3:6] if len(joint_velocities) >= 6 else [0, 0, 0]
                }
            else:
                # For regular joints, find the corresponding position/velocity
                # This is simplified - in practice you'd need the joint's DOF mapping
                if i < len(joint_positions):
                    joint_info["position"] = joint_positions[i] if i < len(joint_positions) else 0.0
                if i < len(joint_velocities):
                    joint_info["velocity"] = joint_velocities[i] if i < len(joint_velocities) else 0.0
            
            joint_data["joints_by_name"][joint_name] = joint_info
        
        return joint_data
    
    def extract_imu_data(self, state: newton.State, timestamp: float = None) -> Dict:
        """
        Extract IMU data from the base body (acceleration, angular velocity, orientation).
        
        Args:
            state (newton.State): Current simulation state
            timestamp (float, optional): Timestamp for this state snapshot
            
        Returns:
            Dict: IMU data containing linear acceleration, angular velocity, and orientation
        """
        imu_data = {
            "timestamp": timestamp,
            "linear_acceleration": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "orientation": {
                "quaternion": [0.0, 0.0, 0.0, 1.0],  # x, y, z, w
                "euler_angles": [0.0, 0.0, 0.0]      # roll, pitch, yaw (radians)
            }
        }
        
        if state.body_q is not None and self.base_body_index < len(state.body_q):
            try:
                # Convert the entire body_q array to numpy first
                body_q_np = state.body_q.numpy()
                
                if len(body_q_np) > self.base_body_index:
                    # Extract base body transform (assuming it's a 7-element array: pos + quat)
                    base_transform = body_q_np[self.base_body_index]
                    
                    # Handle different formats of body transforms
                    if isinstance(base_transform, np.ndarray) and len(base_transform) >= 7:
                        # Direct array format [x, y, z, qx, qy, qz, qw]
                        position = base_transform[0:3]
                        quaternion = base_transform[3:7]
                        imu_data["orientation"]["quaternion"] = quaternion.tolist()
                        
                        # Convert quaternion to Euler angles (roll, pitch, yaw)
                        euler_angles = self._quaternion_to_euler(quaternion)
                        imu_data["orientation"]["euler_angles"] = euler_angles.tolist()
                    else:
                        # Try to interpret as separate position and quaternion components
                        # This is a fallback - the exact format depends on Newton's internal representation
                        print(f"Warning: Unexpected body transform format: {type(base_transform)}, shape: {getattr(base_transform, 'shape', 'unknown')}")
                        
            except Exception as e:
                print(f"Warning: Could not extract orientation data: {e}")
                # Set default orientation
                imu_data["orientation"]["quaternion"] = [0.0, 0.0, 0.0, 1.0]
                imu_data["orientation"]["euler_angles"] = [0.0, 0.0, 0.0]
        
        if state.body_qd is not None and self.base_body_index < len(state.body_qd):
            try:
                # Convert the entire body_qd array to numpy first
                body_qd_np = state.body_qd.numpy()
                
                if len(body_qd_np) > self.base_body_index:
                    # Extract base body spatial velocity (linear + angular)
                    base_velocity = body_qd_np[self.base_body_index]
                    
                    # Handle different formats of body velocities
                    if isinstance(base_velocity, np.ndarray) and len(base_velocity) >= 6:
                        # Direct array format [vx, vy, vz, wx, wy, wz]
                        linear_velocity = base_velocity[0:3]
                        angular_velocity = base_velocity[3:6]
                        imu_data["angular_velocity"] = angular_velocity.tolist()
                        
                        # Calculate linear acceleration if we have previous state
                        if (self.previous_state is not None and 
                            self.previous_timestamp is not None and 
                            timestamp is not None):
                            
                            dt = timestamp - self.previous_timestamp
                            if dt > 0:
                                # Get previous linear velocity
                                prev_velocity = self._get_previous_linear_velocity()
                                if prev_velocity is not None:
                                    # Calculate acceleration: (v_current - v_previous) / dt
                                    acceleration = (linear_velocity - prev_velocity) / dt
                                    
                                    # Add gravity compensation (assuming gravity in -Z direction)
                                    gravity = np.array([0.0, 0.0, -9.81])
                                    acceleration_with_gravity = acceleration - gravity
                                    
                                    imu_data["linear_acceleration"] = acceleration_with_gravity.tolist()
                    else:
                        print(f"Warning: Unexpected body velocity format: {type(base_velocity)}, shape: {getattr(base_velocity, 'shape', 'unknown')}")
                                
            except Exception as e:
                print(f"Warning: Could not extract velocity data: {e}")
        
        # Store current state for next acceleration calculation
        self._update_previous_state(state, timestamp)
        
        return imu_data
    
    def _quaternion_to_euler(self, quaternion: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
            quaternion (np.ndarray): Quaternion as [x, y, z, w]
            
        Returns:
            np.ndarray: Euler angles as [roll, pitch, yaw] in radians
        """
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def _get_previous_linear_velocity(self) -> Optional[np.ndarray]:
        """Get linear velocity from previous state."""
        if (self.previous_state is None or 
            not hasattr(self.previous_state, 'body_qd') or
            self.previous_state.body_qd is None or 
            len(self.previous_state.body_qd) <= self.base_body_index):
            return None
        
        try:
            prev_velocity = self.previous_state.body_qd[self.base_body_index]
            if isinstance(prev_velocity, np.ndarray):
                return prev_velocity[0:3] if len(prev_velocity) >= 3 else None
            else:
                return None
        except Exception:
            return None
    
    def _update_previous_state(self, state: newton.State, timestamp: float):
        """Store current state for next acceleration calculation."""
        # We only need to store the relevant body data, not the entire state
        self.previous_timestamp = timestamp
        
        # Create a lightweight copy of relevant state data
        if state.body_qd is not None and self.base_body_index < len(state.body_qd):
            class PreviousState:
                def __init__(self):
                    self.body_qd = None
            
            self.previous_state = PreviousState()
            
            try:
                # Copy the base body velocity
                body_qd_np = state.body_qd.numpy()
                if len(body_qd_np) > self.base_body_index:
                    base_velocity = body_qd_np[self.base_body_index]
                    if isinstance(base_velocity, np.ndarray):
                        velocity_copy = base_velocity.copy()
                    else:
                        velocity_copy = np.array([0, 0, 0, 0, 0, 0])
                    
                    # Store as a simple array
                    self.previous_state.body_qd = [velocity_copy]
            except Exception:
                self.previous_state.body_qd = [np.array([0, 0, 0, 0, 0, 0])]
    
    def extract_trajectory(self, states: List[newton.State], timestamps: List[float] = None) -> Dict:
        """
        Extract joint data from a sequence of simulation states (trajectory).
        
        Args:
            states (List[newton.State]): List of simulation states
            timestamps (List[float], optional): Timestamps for each state
            
        Returns:
            Dict: Complete trajectory data with joint positions/velocities over time
        """
        if timestamps is None:
            timestamps = list(range(len(states)))
        
        trajectory_data = {
            "metadata": {
                "frame_count": len(states),
                "joint_count": self.joint_count,
                "joint_names": self.joint_names,
                "start_time": timestamps[0] if timestamps else 0,
                "end_time": timestamps[-1] if timestamps else len(states)-1,
                "duration": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
            },
            "trajectory": []
        }
        
        # Extract data for each frame
        for i, state in enumerate(states):
            timestamp = timestamps[i] if i < len(timestamps) else i
            frame_data = self.extract_joint_state(state, timestamp)
            trajectory_data["trajectory"].append(frame_data)
        
        return trajectory_data
    
    def save_to_json(self, data: Dict, filepath: Union[str, Path], indent: int = 2) -> None:
        """
        Save joint data to a JSON file.
        
        Args:
            data (Dict): Joint data to save
            filepath (Union[str, Path]): Output file path
            indent (int): JSON indentation level
        """
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=indent)
        
        print(f"Joint data saved to: {filepath}")
    
    def load_from_json(self, filepath: Union[str, Path]) -> Dict:
        """
        Load joint data from a JSON file.
        
        Args:
            filepath (Union[str, Path]): Input file path
            
        Returns:
            Dict: Loaded joint data
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        print(f"Joint data loaded from: {filepath}")
        return data
    
    def get_joint_limits(self) -> Dict:
        """
        Extract joint limits from the model (if available).
        
        Returns:
            Dict: Joint limits data
        """
        limits_data = {
            "joint_names": self.joint_names,
            "limits": {}
        }
        
        # Try to extract joint limits if available in the model
        if hasattr(self.model, 'joint_limit_lower') and self.model.joint_limit_lower is not None:
            lower_limits = self.model.joint_limit_lower.numpy().tolist()
            limits_data["limits"]["lower"] = lower_limits
        
        if hasattr(self.model, 'joint_limit_upper') and self.model.joint_limit_upper is not None:
            upper_limits = self.model.joint_limit_upper.numpy().tolist()
            limits_data["limits"]["upper"] = upper_limits
        
        return limits_data


def create_quadruped_joint_mapping() -> Dict[str, List[str]]:
    """
    Create a standard joint mapping for quadruped robots (like Go2).
    
    Returns:
        Dict: Joint mapping by leg and joint type
    """
    return {
        "front_left": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
        "front_right": ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"], 
        "rear_left": ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
        "rear_right": ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
        "floating_base": ["floating_base"]
    }


def extract_quadruped_leg_angles(joint_data: Dict, leg_mapping: Dict[str, List[str]]) -> Dict:
    """
    Extract joint angles organized by leg for quadruped robots.
    
    Args:
        joint_data (Dict): Joint data from extract_joint_state()
        leg_mapping (Dict): Mapping of leg names to joint names
        
    Returns:
        Dict: Joint angles organized by leg
    """
    leg_angles = {}
    joints_by_name = joint_data.get("joints_by_name", {})
    
    for leg_name, joint_names in leg_mapping.items():
        if leg_name == "floating_base":
            if "floating_base" in joints_by_name:
                leg_angles[leg_name] = joints_by_name["floating_base"]
            continue
            
        leg_angles[leg_name] = {}
        for joint_name in joint_names:
            if joint_name in joints_by_name:
                joint_info = joints_by_name[joint_name]
                leg_angles[leg_name][joint_name] = {
                    "position": joint_info.get("position", 0.0),
                    "velocity": joint_info.get("velocity", 0.0)
                }
            else:
                print(f"Warning: Joint {joint_name} not found in joint data")
                leg_angles[leg_name][joint_name] = {"position": 0.0, "velocity": 0.0}
    
    return leg_angles


# Example usage functions
def example_extract_single_frame():
    """
    Example showing how to extract joint data from a single simulation frame.
    """
    print("\n=== Example: Single Frame Joint Extraction ===")
    
    # This would typically be called within a simulation loop
    # For demonstration, we'll show the structure
    
    example_code = """
    # Within your simulation loop:
    
    # Initialize extractor with your model
    extractor = JointDataExtractor(model)
    
    # Extract current joint state
    joint_data = extractor.extract_joint_state(current_state, sim_time)
    
    # Save to JSON
    extractor.save_to_json(joint_data, "joint_data_frame.json")
    
    # Access specific joint data
    if "joints_by_name" in joint_data:
        for joint_name, joint_info in joint_data["joints_by_name"].items():
            print(f"{joint_name}: pos={joint_info.get('position')}, vel={joint_info.get('velocity')}")
    """
    
    print(example_code)


def example_extract_trajectory():
    """
    Example showing how to extract joint data from multiple simulation frames.
    """
    print("\n=== Example: Trajectory Joint Extraction ===")
    
    example_code = """
    # Collect states during simulation
    states = []
    timestamps = []
    
    # During simulation loop:
    for frame in range(num_frames):
        # Run simulation step
        example.step()
        
        # Collect state and timestamp (note: storing references)
        states.append(example.state_0)  # Note: this stores reference, not copy
        timestamps.append(example.sim_time)
    
    # Extract complete trajectory
    extractor = JointDataExtractor(model)
    trajectory_data = extractor.extract_trajectory(states, timestamps)
    
    # Save trajectory to JSON
    extractor.save_to_json(trajectory_data, "robot_trajectory.json")
    
    # Extract quadruped-specific leg angles
    leg_mapping = create_quadruped_joint_mapping()
    for frame_data in trajectory_data["trajectory"]:
        leg_angles = extract_quadruped_leg_angles(frame_data, leg_mapping)
        print(f"Frame {frame_data['timestamp']}: {leg_angles}")
    """
    
    print(example_code)


if __name__ == "__main__":
    print("Newton Physics Engine - Joint Data Extractor")
    print("=" * 50)
    
    print("This utility provides functions to extract joint values from Newton simulations")
    print("and convert them to JSON format for low-level robot control.")
    
    print("\nKey Features:")
    print("- Extract joint positions and velocities")
    print("- Map joint indices to joint names") 
    print("- Convert to structured JSON format")
    print("- Support for single frames and trajectories")
    print("- Quadruped-specific joint mapping utilities")
    
    print("\nQuadruped Joint Mapping:")
    mapping = create_quadruped_joint_mapping()
    for leg, joints in mapping.items():
        print(f"  {leg}: {joints}")
    
    # Show example usage
    example_extract_single_frame()
    example_extract_trajectory()
    
    print("\nTo use this utility:")
    print("1. Import: from joint_data_extractor import JointDataExtractor")
    print("2. Initialize: extractor = JointDataExtractor(your_newton_model)")
    print("3. Extract: joint_data = extractor.extract_joint_state(simulation_state)")
    print("4. Save: extractor.save_to_json(joint_data, 'output.json')")
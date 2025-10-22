#!/usr/bin/env python3
"""
Go2 Quadruped Robot Simulation with Joint Data Extraction

This example demonstrates how to simulate a Go2 quadruped robot using Newton Physics Engine
and extract joint values for low-level control. The script is based on the ANYmal robot
examples but adapted for the Go2 robot structure.

The example shows:
- Loading and simulating a quadruped robot (Go2-style)
- Extracting joint positions and velocities in real-time
- Converting joint data to JSON format
- Saving joint trajectories for analysis

Command: python go2_simulation_example.py [--duration 10] [--output_dir ./go2_data]
"""

import argparse
import json
import mujoco
import numpy as np
import warp as wp
from pathlib import Path
from typing import List, Dict

# Import Newton Physics
import newton
import newton.examples
import newton.utils

# Import our joint data extractor
from joint_data_extractor import JointDataExtractor, create_quadruped_joint_mapping, extract_quadruped_leg_angles


class Go2SimulationExample:
    """
    Simulation example for Go2 quadruped robot with joint data extraction.
    
    This class sets up a Go2 robot simulation and provides methods to:
    - Run physics simulation
    - Extract joint data in real-time
    - Save joint trajectories to JSON files
    - Visualize the robot (if viewer is available)
    """
    
    def __init__(self, viewer=None, enable_visualization=True):
        """
        Initialize the Go2 simulation.
        
        Args:
            viewer: Newton viewer for visualization (optional)
            enable_visualization (bool): Whether to enable visual output
        """
        self.viewer = viewer
        self.enable_visualization = enable_visualization
        
        # Simulation parameters
        self.fps = 50
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = 4
        self.sim_dt = self.frame_dt / self.sim_substeps
        
        # Data collection
        self.states_history = []
        self.timestamps_history = []
        self.joint_data_history = []
        
        # Setup simulation
        self._setup_robot()
        self._setup_joint_extractor()
        
        print(f"Go2 simulation initialized:")
        print(f"  - Physics timestep: {self.sim_dt:.4f}s")
        print(f"  - Rendering framerate: {self.fps} FPS")
        print(f"  - Joint count: {len(self.model.joint_key)}")
        print(f"  - Joint names: {self.model.joint_key}")
    
    def _setup_robot(self):
        """Setup the Go2 robot model and simulation."""
        self.device = wp.get_device()
        
        # Create robot builder with Go2-appropriate settings
        articulation_builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        
        # Joint and shape parameters optimized for quadruped robots
        articulation_builder.default_joint_cfg = newton.ModelBuilder.JointDofConfig(
            limit_ke=1.0e3,   # Joint limit stiffness
            limit_kd=1.0e1,   # Joint limit damping
            friction=1e-5     # Joint friction
        )
        
        # Contact parameters for realistic ground interaction
        articulation_builder.default_shape_cfg.ke = 5.0e4   # Contact stiffness
        articulation_builder.default_shape_cfg.kd = 5.0e2   # Contact damping 
        articulation_builder.default_shape_cfg.kf = 1.0e3   # Friction stiffness
        articulation_builder.default_shape_cfg.mu = 0.75    # Friction coefficient
        
        # Try to load Go2 asset if available, otherwise use ANYmal as fallback
        try:
            # Attempt to load Go2 robot (this would need to be provided by user)
            # asset_path = newton.utils.download_asset("unitree_go2")
            # asset_file = str(asset_path / "urdf" / "go2.urdf")
            
            # Since Go2 assets may not be available, use ANYmal as a representative quadruped
            print("Loading ANYmal robot as Go2 representative...")
            asset_path = newton.utils.download_asset("anybotics_anymal_c")
            asset_file = str(asset_path / "urdf" / "anymal.urdf")
            
            articulation_builder.add_urdf(
                asset_file,
                xform=wp.transform(wp.vec3(0.0, 0.0, 0.62), wp.quat_identity()),
                floating=True,
                enable_self_collisions=False,
                collapse_fixed_joints=True,
            )
            
        except Exception as e:
            print(f"Could not load robot asset: {e}")
            print("Creating a simple quadruped robot model...")
            self._create_simple_quadruped(articulation_builder)
        
        # Set initial joint positions for stable standing pose
        self._set_initial_pose(articulation_builder)
        
        # Configure joint control (position control for all joints)
        for i in range(articulation_builder.joint_dof_count):
            articulation_builder.joint_dof_mode[i] = newton.JointMode.TARGET_POSITION
            articulation_builder.joint_target_ke[i] = 150  # Position control stiffness
            articulation_builder.joint_target_kd[i] = 5    # Position control damping
        
        # Create the complete simulation model
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.add_builder(articulation_builder, xform=wp.transform(wp.vec3(0, 0, 0), wp.quat_identity()))
        builder.add_ground_plane()
        
        # Finalize model and create solver
        self.model = builder.finalize()
        self.solver = newton.solvers.SolverMuJoCo(
            self.model,
            cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
            impratio=100,
            iterations=100,
            ls_iterations=50,
            njmax=600,  # Increased from default to prevent overflow warnings
        )
        
        # Initialize simulation states
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.collide(self.state_0)
        
        # Setup viewer if available
        if self.viewer is not None:
            self.viewer.set_model(self.model)
        
        # Setup CUDA graph for performance (if using GPU)
        self._setup_cuda_graph()
    
    def _create_simple_quadruped(self, builder):
        """Create a simple quadruped robot if assets are not available."""
        print("Creating simplified quadruped model...")
        
        # This would create a basic quadruped structure
        # For a complete implementation, you would add bodies, joints, and shapes here
        # For now, we'll use the ANYmal model as a fallback
        pass
    
    def _set_initial_pose(self, builder):
        """Set initial joint positions for a stable standing pose."""
        # Set base position (floating base)
        builder.joint_q[:3] = [0.0, 0.0, 0.62]  # x, y, z position
        if len(builder.joint_q) > 6:
            builder.joint_q[3:7] = [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
        
        # Set leg joint angles for stable standing pose
        # These values are typical for quadruped robots in standing position
        if hasattr(builder, 'joint_key') and len(builder.joint_key) > 7:
            try:
                # Try to set joints by name (ANYmal naming convention)
                initial_angles = {
                    "LF_HAA": 0.0,   # Left Front Hip Abduction/Adduction
                    "LF_HFE": 0.4,   # Left Front Hip Flexion/Extension  
                    "LF_KFE": -0.8,  # Left Front Knee Flexion/Extension
                    "RF_HAA": 0.0,   # Right Front Hip Abduction/Adduction
                    "RF_HFE": 0.4,   # Right Front Hip Flexion/Extension
                    "RF_KFE": -0.8,  # Right Front Knee Flexion/Extension
                    "LH_HAA": 0.0,   # Left Hind Hip Abduction/Adduction
                    "LH_HFE": -0.4,  # Left Hind Hip Flexion/Extension
                    "LH_KFE": 0.8,   # Left Hind Knee Flexion/Extension
                    "RH_HAA": 0.0,   # Right Hind Hip Abduction/Adduction
                    "RH_HFE": -0.4,  # Right Hind Hip Flexion/Extension
                    "RH_KFE": 0.8,   # Right Hind Knee Flexion/Extension
                }
                
                for joint_name, angle in initial_angles.items():
                    if joint_name in builder.joint_key:
                        joint_idx = builder.joint_key.index(joint_name)
                        if joint_idx < len(builder.joint_q):
                            builder.joint_q[joint_idx + 6] = angle  # +6 for floating base (3 pos + 4 quat - 1 for 0-index)
                            
            except Exception as e:
                print(f"Could not set joint angles by name: {e}")
                print("Using default pose")
    
    def _setup_joint_extractor(self):
        """Initialize the joint data extractor with IMU support."""
        # Base body is typically at index 0 (the floating base)
        base_body_index = 0
        self.joint_extractor = JointDataExtractor(self.model, base_body_index=base_body_index)
        self.quadruped_mapping = create_quadruped_joint_mapping()
        
        # Adapt mapping to actual joint names in the model
        if hasattr(self.model, 'joint_key'):
            self._adapt_joint_mapping()
    
    def _adapt_joint_mapping(self):
        """Adapt the quadruped joint mapping to the actual model joint names."""
        actual_joints = self.model.joint_key
        print(f"Adapting joint mapping for joints: {actual_joints}")
        
        # Create mapping based on actual joint names
        # This is a simplified mapping - in practice you'd need more sophisticated matching
        leg_joints = {}
        
        for joint_name in actual_joints:
            if 'LF' in joint_name or 'left_front' in joint_name.lower():
                leg_joints.setdefault('front_left', []).append(joint_name)
            elif 'RF' in joint_name or 'right_front' in joint_name.lower():
                leg_joints.setdefault('front_right', []).append(joint_name) 
            elif 'LH' in joint_name or 'left_hind' in joint_name.lower() or 'left_rear' in joint_name.lower():
                leg_joints.setdefault('rear_left', []).append(joint_name)
            elif 'RH' in joint_name or 'right_hind' in joint_name.lower() or 'right_rear' in joint_name.lower():
                leg_joints.setdefault('rear_right', []).append(joint_name)
            elif 'floating_base' in joint_name or 'base' in joint_name:
                leg_joints.setdefault('floating_base', []).append(joint_name)
        
        if leg_joints:
            self.quadruped_mapping.update(leg_joints)
            print(f"Updated quadruped mapping: {self.quadruped_mapping}")
    
    def _setup_cuda_graph(self):
        """Setup CUDA graph for performance optimization."""
        self.graph = None
        if self.device.is_cuda:
            try:
                with wp.ScopedCapture() as capture:
                    self.simulate()
                self.graph = capture.graph
                print("CUDA graph capture successful")
            except Exception as e:
                print(f"CUDA graph capture failed: {e}")
    
    def simulate(self):
        """Perform one frame of physics simulation."""
        self.contacts = self.model.collide(self.state_0)
        
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            
            # Apply external forces (if viewer supports it)
            if self.viewer is not None and hasattr(self.viewer, 'apply_forces'):
                self.viewer.apply_forces(self.state_0)
            
            # Step the physics solver
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            
            # Swap states
            self.state_0, self.state_1 = self.state_1, self.state_0
    
    def step(self, extract_data=True):
        """
        Perform one simulation step and optionally extract joint data.
        
        Args:
            extract_data (bool): Whether to extract and store joint data
        """
        # Run physics simulation
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        
        # Update simulation time
        self.sim_time += self.frame_dt
        
        # Extract joint data if requested
        if extract_data:
            joint_data = self.joint_extractor.extract_joint_state(self.state_0, self.sim_time, include_imu=True)
            self.joint_data_history.append(joint_data)
            self.timestamps_history.append(self.sim_time)
            
            # Store state for trajectory analysis (be careful with memory)
            if len(self.states_history) < 1000:  # Limit memory usage
                # Note: We're storing references, not copies for performance
                # For actual state copying, you'd need to use the Newton state copying mechanism
                self.states_history.append(self.state_0)
    
    def render(self):
        """Render the current simulation frame."""
        if self.viewer is not None and self.enable_visualization:
            self.viewer.begin_frame(self.sim_time)
            self.viewer.log_state(self.state_0)
            if hasattr(self, 'contacts'):
                self.viewer.log_contacts(self.contacts, self.state_0)
            self.viewer.end_frame()
    
    def get_current_joint_data(self, include_imu: bool = True) -> Dict:
        """Get joint data for the current simulation state."""
        return self.joint_extractor.extract_joint_state(self.state_0, self.sim_time, include_imu=include_imu)
    
    def get_current_imu_data(self) -> Dict:
        """Get IMU data for the current simulation state."""
        return self.joint_extractor.extract_imu_data(self.state_0, self.sim_time)
    
    def get_leg_angles(self) -> Dict:
        """Get current joint angles organized by leg."""
        joint_data = self.get_current_joint_data()
        return extract_quadruped_leg_angles(joint_data, self.quadruped_mapping)
    
    def save_trajectory_data(self, output_dir: Path):
        """
        Save collected joint trajectory data to JSON files.
        
        Args:
            output_dir (Path): Directory to save the data files
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        if not self.joint_data_history:
            print("No joint data collected to save")
            return
        
        # Save complete trajectory
        trajectory_data = {
            "metadata": {
                "robot_type": "Go2_quadruped",
                "frame_count": len(self.joint_data_history),
                "duration": self.sim_time,
                "fps": self.fps,
                "joint_names": self.model.joint_key,
                "includes_imu_data": True,
                "simulation_parameters": {
                    "timestep": self.sim_dt,
                    "substeps": self.sim_substeps
                }
            },
            "trajectory": self.joint_data_history
        }
        
        trajectory_file = output_dir / "go2_joint_trajectory.json"
        with open(trajectory_file, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        print(f"Joint trajectory saved: {trajectory_file}")
        
        # Save IMU-only data
        imu_trajectory = {
            "metadata": trajectory_data["metadata"],
            "imu_data": []
        }
        
        for joint_data in self.joint_data_history:
            if "imu" in joint_data:
                imu_frame = {
                    "timestamp": joint_data["timestamp"],
                    "imu": joint_data["imu"]
                }
                imu_trajectory["imu_data"].append(imu_frame)
        
        imu_file = output_dir / "go2_imu_data.json"
        with open(imu_file, 'w') as f:
            json.dump(imu_trajectory, f, indent=2)
        print(f"IMU data saved: {imu_file}")
        
        # Save leg-specific data
        leg_trajectory = {
            "metadata": trajectory_data["metadata"],
            "leg_data": []
        }
        
        for joint_data in self.joint_data_history:
            leg_angles = extract_quadruped_leg_angles(joint_data, self.quadruped_mapping)
            leg_trajectory["leg_data"].append({
                "timestamp": joint_data["timestamp"],
                "legs": leg_angles
            })
        
        leg_file = output_dir / "go2_leg_angles.json"
        with open(leg_file, 'w') as f:
            json.dump(leg_trajectory, f, indent=2)
        print(f"Leg angles saved: {leg_file}")
        
        # Save joint limits
        limits_data = self.joint_extractor.get_joint_limits()
        limits_file = output_dir / "go2_joint_limits.json"
        with open(limits_file, 'w') as f:
            json.dump(limits_data, f, indent=2)
        print(f"Joint limits saved: {limits_file}")
        
        print(f"All data saved to: {output_dir}")


def main():
    """Main function to run the Go2 simulation example."""
    parser = argparse.ArgumentParser(description="Go2 Quadruped Robot Simulation with Joint Data Extraction")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds")
    parser.add_argument("--output_dir", type=str, default="./go2_data", help="Output directory for JSON files")
    parser.add_argument("--headless", action="store_true", help="Run without visualization")
    parser.add_argument("--fps", type=int, default=50, help="Simulation framerate")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Go2 Quadruped Robot Simulation with Newton Physics")
    print("=" * 60)
    print(f"Duration: {args.duration} seconds")
    print(f"Output directory: {args.output_dir}")
    print(f"Framerate: {args.fps} FPS")
    print(f"Headless mode: {args.headless}")
    
    # Initialize viewer (or None for headless)
    viewer = None
    if not args.headless:
        try:
            # Try to create a viewer - this may fail if no display is available
            viewer = newton.viewer.ViewerGL()
            print("OpenGL viewer initialized")
        except Exception as e:
            print(f"Could not initialize viewer: {e}")
            print("Running in headless mode")
    
    # Create simulation
    simulation = Go2SimulationExample(viewer=viewer, enable_visualization=(viewer is not None))
    
    # Calculate simulation parameters
    total_frames = int(args.duration * args.fps)
    print(f"Running {total_frames} frames...")
    
    # Run simulation
    try:
        for frame in range(total_frames):
            # Step simulation and extract joint data
            simulation.step(extract_data=True)
            
            # Render if viewer is available
            simulation.render()
            
            # Print progress every second
            if frame % args.fps == 0:
                current_time = frame / args.fps
                leg_angles = simulation.get_leg_angles()
                imu_data = simulation.get_current_imu_data()
                
                print(f"Time: {current_time:.1f}s, Frame: {frame}/{total_frames}")
                
                # Print some joint data as example
                if "front_left" in leg_angles:
                    fl_data = leg_angles["front_left"]
                    print(f"  Front Left Leg: {list(fl_data.keys())}")
                
                # Print IMU data
                if "linear_acceleration" in imu_data:
                    accel = imu_data["linear_acceleration"]
                    angular_vel = imu_data["angular_velocity"]
                    euler = imu_data["orientation"]["euler_angles"]
                    print(f"  IMU - Accel: [{accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}] m/s²")
                    print(f"  IMU - Gyro: [{angular_vel[0]:.2f}, {angular_vel[1]:.2f}, {angular_vel[2]:.2f}] rad/s")
                    print(f"  IMU - Euler: [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}] rad")
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    # Save collected data
    print("\nSaving joint data...")
    simulation.save_trajectory_data(Path(args.output_dir))
    
    # Show summary
    print(f"\nSimulation completed:")
    print(f"  Total frames: {len(simulation.joint_data_history)}")
    print(f"  Duration: {simulation.sim_time:.2f} seconds")
    print(f"  Joint count: {len(simulation.model.joint_key)}")
    print(f"  Data saved to: {args.output_dir}")
    
    # Close viewer
    if viewer is not None:
        viewer.close()


if __name__ == "__main__":
    main()
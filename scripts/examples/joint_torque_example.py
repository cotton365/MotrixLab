#!/usr/bin/env python3
"""
Joint Torque API Example

This script demonstrates how to use MotrixSim's joint torque API to:
1. Read current joint torques
2. Set joint torques directly
3. Use PD control to compute torques
4. Monitor torque values during simulation

Run with:
    uv run scripts/examples/joint_torque_example.py
"""

import motrixsim as mtx
import numpy as np
from pathlib import Path


def main():
    print("=" * 60)
    print("MotrixSim Joint Torque API Example")
    print("=" * 60)

    # Load a simple pendulum model for demonstration
    model_path = Path(__file__).parent.parent.parent / "motrix_envs" / "src" / "motrix_envs" / "basic" / "pendulum" / "pendulum.xml"

    if not model_path.exists():
        print(f"Error: Model file not found at {model_path}")
        print("Please make sure you're running this from the MotrixLab root directory")
        return

    print(f"\n1. Loading model from: {model_path}")
    model = mtx.load_model(str(model_path))

    # Create simulation data with 4 parallel environments
    num_envs = 4
    print(f"2. Creating {num_envs} parallel simulation environments")
    data = mtx.create_data(model, num_envs=num_envs)

    # Print actuator information
    print(f"\n3. Actuator Information:")
    print(f"   Number of actuators: {model.num_actuators}")
    print(f"   Actuator names: {model.actuator_names}")
    print(f"   Control limits: {model.actuator_ctrl_limits}")

    # Initialize simulation
    print(f"\n4. Initializing simulation state")
    init_dof_pos = model.compute_init_dof_pos()
    for i in range(num_envs):
        data.set_dof_pos(init_dof_pos, model, env_id=i)

    # Example 1: Reading current torques
    print(f"\n5. Example 1: Reading Current Torques")
    print(f"   Initial actuator_ctrls shape: {data.actuator_ctrls.shape}")
    print(f"   Initial torques:\n{data.actuator_ctrls}")

    # Example 2: Setting torques directly
    print(f"\n6. Example 2: Setting Torques Directly")
    test_torques = np.array([[1.0], [2.0], [-1.0], [0.5]])
    print(f"   Setting torques to:\n{test_torques}")
    data.actuator_ctrls = test_torques
    print(f"   Current actuator_ctrls after setting:\n{data.actuator_ctrls}")

    # Example 3: PD control
    print(f"\n7. Example 3: PD Control Simulation")
    kp = 50.0  # Proportional gain
    kd = 2.0   # Derivative gain
    target_angle = 0.0  # Target angle (upright position)

    print(f"   PD parameters: Kp={kp}, Kd={kd}")
    print(f"   Target angle: {target_angle} rad")
    print(f"\n   Running simulation for 100 steps...")

    torque_history = []

    for step in range(100):
        # Get current state
        current_pos = data.dof_pos[:, 0]  # First DOF (pendulum angle)
        current_vel = data.dof_vel[:, 0]  # First DOF velocity

        # Compute torques using PD control
        torques = kp * (target_angle - current_pos) - kd * current_vel
        torques = torques.reshape(-1, 1)  # Reshape to [num_envs, 1]

        # Apply torques
        data.actuator_ctrls = torques

        # Step simulation
        mtx.step(model, data)

        # Record torques
        torque_history.append(data.actuator_ctrls.copy())

        # Print progress every 20 steps
        if (step + 1) % 20 == 0:
            mean_torque = np.mean(np.abs(data.actuator_ctrls))
            mean_pos = np.mean(current_pos)
            print(f"   Step {step + 1:3d}: Mean torque = {mean_torque:6.3f} N·m, Mean position = {mean_pos:6.3f} rad")

    # Example 4: Torque statistics
    print(f"\n8. Example 4: Torque Statistics")
    all_torques = np.concatenate(torque_history, axis=0)
    print(f"   Mean torque: {np.mean(all_torques):.4f} N·m")
    print(f"   Std torque: {np.std(all_torques):.4f} N·m")
    print(f"   Max absolute torque: {np.max(np.abs(all_torques)):.4f} N·m")
    print(f"   Min torque: {np.min(all_torques):.4f} N·m")
    print(f"   Max torque: {np.max(all_torques):.4f} N·m")

    # Example 5: Per-environment torque analysis
    print(f"\n9. Example 5: Per-Environment Analysis")
    for env_id in range(num_envs):
        env_torques = np.array([t[env_id] for t in torque_history])
        mean_env_torque = np.mean(np.abs(env_torques))
        print(f"   Environment {env_id}: Mean absolute torque = {mean_env_torque:.4f} N·m")

    # Example 6: Torque clipping
    print(f"\n10. Example 6: Torque Clipping")
    limits = model.actuator_ctrl_limits
    print(f"   Control limits: {limits}")

    test_torques = np.array([[100.0], [-100.0], [5.0], [-5.0]])  # Extreme values
    print(f"   Before clipping: {test_torques.T}")

    clipped_torques = np.clip(test_torques, limits[:, 0], limits[:, 1])
    print(f"   After clipping:  {clipped_torques.T}")

    print("\n" + "=" * 60)
    print("Example completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()

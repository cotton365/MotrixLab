# Joint Torque API Reference

This document provides detailed information about the Joint Torque API provided by MotrixSim.

## Core API: `data.actuator_ctrls`

MotrixSim provides the `actuator_ctrls` attribute to access and control actuator control values. For motor-type actuators, this value represents joint torques.

### API Location

```python
import motrixsim as mtx

# Load model
model = mtx.load_model("model.xml")

# Create simulation data
data = mtx.create_data(model, num_envs=1)

# Read current torque values
current_torques = data.actuator_ctrls  # Shape: [num_envs, num_actuators]

# Set torque values
data.actuator_ctrls = new_torques
```

### Data Type and Shape

- **Data Type**: `numpy.ndarray` (float32)
- **Shape**: `[num_envs, num_actuators]`
  - `num_envs`: Number of parallel simulation environments
  - `num_actuators`: Number of actuators defined in the model

## Reading Joint Torques

### Basic Usage

```python
# Get current control values for all actuators
torques = data.actuator_ctrls

# Get torques for a specific environment
env_0_torques = data.actuator_ctrls[0]  # First environment

# Get torque for a specific joint
joint_5_torque = data.actuator_ctrls[:, 5]  # Joint 5 across all environments
```

### Real-world Example

Using torque values in a reward function (from `motrix_envs/locomotion/go1/walk_np.py:326-328`):

```python
def _reward_torques(self, data: mtx.SceneData):
    """Penalize large torque consumption"""
    return np.sum(np.square(data.actuator_ctrls), axis=1)
```

## Setting Joint Torques

### Direct Torque Control

For `motor` type actuators, you can directly set torque values:

```python
def apply_action(self, actions, state):
    # Directly set torques
    state.data.actuator_ctrls = computed_torques
    return state
```

### PD Controller for Torque Computation

In practice, PD controllers are commonly used to convert position targets to torques (from `motrix_envs/locomotion/go1/walk_np.py:167-174`):

```python
def _compute_torques(self, actions, data):
    """Compute torques using PD controller"""
    actions_scaled = actions * self.cfg.control_config.action_scale

    # PD control: torque = Kp * (target - current) - Kd * velocity
    torques = self.kps * (
        actions_scaled + self.default_angles - self.get_dof_pos(data)
    ) - self.kds * self.get_dof_vel(data)

    return torques

def apply_action(self, actions, state):
    # Apply computed torques
    state.data.actuator_ctrls = self._compute_torques(actions, state.data)
    return state
```

## Actuator Types

### Motor Actuators (Torque Control)

XML definition example (from `motrix_envs/locomotion/go1/xmls/go1_motor_actuator.xml`):

```xml
<actuator>
    <motor class="abduction" name="FR_hip" joint="FR_hip_joint"
           ctrllimited="true" ctrlrange="-23.7 23.7"/>
    <motor class="hip" name="FR_thigh" joint="FR_thigh_joint"
           ctrllimited="true" ctrlrange="-23.7 23.7"/>
    <motor class="knee" name="FR_calf" joint="FR_calf_joint"
           ctrllimited="true" ctrlrange="-23.7 23.7"/>
</actuator>
```

For `motor` type actuators:
- `actuator_ctrls` represents **torque values** (units: N·m)
- `ctrlrange` defines minimum and maximum torque limits

### Position Actuators (Position Control)

```xml
<actuator>
    <position name="joint_pos" joint="my_joint" />
</actuator>
```

For `position` type actuators:
- `actuator_ctrls` represents **target positions** (units: radians)
- Internally converted to torques by the simulator

## Related APIs

### Model Information

```python
# Get number of actuators
num_actuators = model.num_actuators

# Get actuator names
actuator_names = model.actuator_names

# Get actuator control limits
ctrl_limits = model.actuator_ctrl_limits  # Shape: [num_actuators, 2]
```

### Joint Positions and Velocities

```python
# Get joint positions
dof_pos = data.dof_pos  # Shape: [num_envs, num_dof_pos]

# Get joint velocities
dof_vel = data.dof_vel  # Shape: [num_envs, num_dof_vel]

# Access specific joints via Body object
body = model.get_body("robot_base")
joint_pos = body.get_joint_dof_pos(data)
joint_vel = body.get_joint_dof_vel(data)
```

## Complete Example

Here's a complete torque control example:

```python
import motrixsim as mtx
import numpy as np

# 1. Load model
model = mtx.load_model("robot.xml")

# 2. Create simulation data
num_envs = 4
data = mtx.create_data(model, num_envs=num_envs)

# 3. Initialize PD control parameters
kp = 50.0  # Proportional gain
kd = 2.0   # Derivative gain
target_angles = np.array([0.0, 0.5, -1.0, 0.5] * 3)  # Target angles

# 4. Simulation loop
for step in range(1000):
    # Get current state
    current_pos = data.dof_pos
    current_vel = data.dof_vel

    # Compute torques (PD control)
    torques = kp * (target_angles - current_pos) - kd * current_vel

    # Set torques
    data.actuator_ctrls = torques

    # Execute simulation step
    mtx.step(model, data)

    # Read current torques (for monitoring or logging)
    applied_torques = data.actuator_ctrls

    # Print torque info every 100 steps
    if step % 100 == 0:
        print(f"Step {step}: Mean torque = {np.mean(np.abs(applied_torques)):.3f} N·m")
```

## Common Use Cases

### 1. Torque Penalty

In reinforcement learning, we typically want to minimize energy consumption:

```python
def compute_torque_penalty(data):
    """Compute torque penalty term"""
    # Square sum penalty
    torque_penalty = np.sum(np.square(data.actuator_ctrls), axis=1)

    # Or use absolute value penalty
    # torque_penalty = np.sum(np.abs(data.actuator_ctrls), axis=1)

    return torque_penalty
```

### 2. Torque Clipping

Ensure torques stay within safe limits:

```python
def apply_torque_with_limits(model, data, desired_torques):
    """Apply torques with limits"""
    # Get torque limits
    limits = model.actuator_ctrl_limits  # [num_actuators, 2]

    # Clip torques
    clipped_torques = np.clip(
        desired_torques,
        limits[:, 0],  # Minimum
        limits[:, 1]   # Maximum
    )

    data.actuator_ctrls = clipped_torques
```

### 3. Torque Monitoring and Logging

```python
class TorqueMonitor:
    """Torque monitor"""
    def __init__(self):
        self.torque_history = []
        self.max_torques = []

    def record(self, data):
        """Record current torques"""
        current_torques = data.actuator_ctrls.copy()
        self.torque_history.append(current_torques)
        self.max_torques.append(np.max(np.abs(current_torques)))

    def get_statistics(self):
        """Get torque statistics"""
        all_torques = np.concatenate(self.torque_history, axis=0)
        return {
            'mean': np.mean(all_torques),
            'std': np.std(all_torques),
            'max': np.max(np.abs(all_torques)),
            'peak_max': max(self.max_torques)
        }
```

## Important Notes

1. **Actuator Type**: Verify the actuator type defined in the XML file; `motor` type corresponds to torque control
2. **Units**: Torque units are Newton-meters (N·m), angle units are radians (rad)
3. **Array Shape**: `actuator_ctrls` is always a 2D array `[num_envs, num_actuators]`
4. **Control Range**: Use `model.actuator_ctrl_limits` to get control range for each actuator
5. **Performance**: Avoid frequent array copying; directly manipulating `data.actuator_ctrls` is more efficient

## References

- [MotrixSim Official Documentation](https://motrixsim.readthedocs.io/)
- [MJCF XML Format Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- MotrixLab Example Code:
  - `motrix_envs/locomotion/go1/walk_np.py`
  - `motrix_envs/locomotion/anymal_c/anymal_c_np.py`
  - `motrix_envs/basic/pendulum/pendulum_np.py`

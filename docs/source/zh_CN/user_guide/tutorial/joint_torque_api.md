# 关节力矩 API 说明

本文档详细介绍 MotrixSim 提供的关节力矩（Joint Torque）获取和设置 API。

## 核心 API：`data.actuator_ctrls`

MotrixSim 提供了 `actuator_ctrls` 属性来访问和控制执行器（actuator）的控制值。对于电机类型的执行器，这个值代表关节力矩。

### API 位置

```python
import motrixsim as mtx

# 加载模型
model = mtx.load_model("model.xml")

# 创建仿真数据
data = mtx.create_data(model, num_envs=1)

# 读取当前力矩值
current_torques = data.actuator_ctrls  # 形状: [num_envs, num_actuators]

# 设置力矩值
data.actuator_ctrls = new_torques
```

### 数据类型和形状

- **数据类型**: `numpy.ndarray` (float32)
- **形状**: `[num_envs, num_actuators]`
  - `num_envs`: 并行仿真环境数量
  - `num_actuators`: 模型中定义的执行器数量

## 读取关节力矩

### 基本用法

```python
# 获取当前所有执行器的控制值
torques = data.actuator_ctrls

# 获取特定环境的力矩
env_0_torques = data.actuator_ctrls[0]  # 第一个环境

# 获取特定关节的力矩
joint_5_torque = data.actuator_ctrls[:, 5]  # 所有环境的第5个关节
```

### 实际应用示例

在奖励函数中使用力矩值（来自 `motrix_envs/locomotion/go1/walk_np.py:326-328`）：

```python
def _reward_torques(self, data: mtx.SceneData):
    """惩罚过大的力矩消耗"""
    return np.sum(np.square(data.actuator_ctrls), axis=1)
```

## 设置关节力矩

### 直接力矩控制

对于 `motor` 类型的执行器，可以直接设置力矩值：

```python
def apply_action(self, actions, state):
    # 直接设置力矩
    state.data.actuator_ctrls = computed_torques
    return state
```

### PD 控制器计算力矩

实际应用中常使用 PD 控制器将位置目标转换为力矩（来自 `motrix_envs/locomotion/go1/walk_np.py:167-174`）：

```python
def _compute_torques(self, actions, data):
    """使用 PD 控制器计算力矩"""
    actions_scaled = actions * self.cfg.control_config.action_scale

    # PD 控制: torque = Kp * (target - current) - Kd * velocity
    torques = self.kps * (
        actions_scaled + self.default_angles - self.get_dof_pos(data)
    ) - self.kds * self.get_dof_vel(data)

    return torques

def apply_action(self, actions, state):
    # 应用计算出的力矩
    state.data.actuator_ctrls = self._compute_torques(actions, state.data)
    return state
```

## 执行器类型说明

### Motor 执行器（力矩控制）

XML 定义示例（`motrix_envs/locomotion/go1/xmls/go1_motor_actuator.xml`）：

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

对于 `motor` 类型执行器：
- `actuator_ctrls` 表示**力矩值**（单位：N·m）
- `ctrlrange` 定义力矩的最小值和最大值

### Position 执行器（位置控制）

```xml
<actuator>
    <position name="joint_pos" joint="my_joint" />
</actuator>
```

对于 `position` 类型执行器：
- `actuator_ctrls` 表示**目标位置**（单位：弧度）
- 内部由仿真器转换为力矩

## 相关 API

### 模型信息

```python
# 获取执行器数量
num_actuators = model.num_actuators

# 获取执行器名称列表
actuator_names = model.actuator_names

# 获取执行器控制范围
ctrl_limits = model.actuator_ctrl_limits  # 形状: [num_actuators, 2]
```

### 关节位置和速度

```python
# 获取关节位置
dof_pos = data.dof_pos  # 形状: [num_envs, num_dof_pos]

# 获取关节速度
dof_vel = data.dof_vel  # 形状: [num_envs, num_dof_vel]

# 通过 Body 对象访问特定关节
body = model.get_body("robot_base")
joint_pos = body.get_joint_dof_pos(data)
joint_vel = body.get_joint_dof_vel(data)
```

## 完整示例

以下是一个完整的力矩控制示例：

```python
import motrixsim as mtx
import numpy as np

# 1. 加载模型
model = mtx.load_model("robot.xml")

# 2. 创建仿真数据
num_envs = 4
data = mtx.create_data(model, num_envs=num_envs)

# 3. 初始化 PD 控制参数
kp = 50.0  # 比例增益
kd = 2.0   # 微分增益
target_angles = np.array([0.0, 0.5, -1.0, 0.5] * 3)  # 目标角度

# 4. 仿真循环
for step in range(1000):
    # 获取当前状态
    current_pos = data.dof_pos
    current_vel = data.dof_vel

    # 计算力矩（PD 控制）
    torques = kp * (target_angles - current_pos) - kd * current_vel

    # 设置力矩
    data.actuator_ctrls = torques

    # 执行仿真步
    mtx.step(model, data)

    # 读取当前力矩（用于监控或记录）
    applied_torques = data.actuator_ctrls

    # 每100步打印一次力矩信息
    if step % 100 == 0:
        print(f"Step {step}: Mean torque = {np.mean(np.abs(applied_torques)):.3f} N·m")
```

## 常见应用场景

### 1. 力矩惩罚（Torque Penalty）

在强化学习中，通常希望减少能量消耗：

```python
def compute_torque_penalty(data):
    """计算力矩惩罚项"""
    # 平方和惩罚
    torque_penalty = np.sum(np.square(data.actuator_ctrls), axis=1)

    # 或使用绝对值惩罚
    # torque_penalty = np.sum(np.abs(data.actuator_ctrls), axis=1)

    return torque_penalty
```

### 2. 力矩限制（Torque Clipping）

确保力矩在安全范围内：

```python
def apply_torque_with_limits(model, data, desired_torques):
    """应用带限制的力矩"""
    # 获取力矩限制
    limits = model.actuator_ctrl_limits  # [num_actuators, 2]

    # 裁剪力矩
    clipped_torques = np.clip(
        desired_torques,
        limits[:, 0],  # 最小值
        limits[:, 1]   # 最大值
    )

    data.actuator_ctrls = clipped_torques
```

### 3. 力矩监控和日志记录

```python
class TorqueMonitor:
    """力矩监控器"""
    def __init__(self):
        self.torque_history = []
        self.max_torques = []

    def record(self, data):
        """记录当前力矩"""
        current_torques = data.actuator_ctrls.copy()
        self.torque_history.append(current_torques)
        self.max_torques.append(np.max(np.abs(current_torques)))

    def get_statistics(self):
        """获取力矩统计信息"""
        all_torques = np.concatenate(self.torque_history, axis=0)
        return {
            'mean': np.mean(all_torques),
            'std': np.std(all_torques),
            'max': np.max(np.abs(all_torques)),
            'peak_max': max(self.max_torques)
        }
```

## 注意事项

1. **执行器类型**：确认 XML 文件中定义的执行器类型，`motor` 类型对应力矩控制
2. **单位**：力矩单位为牛顿·米（N·m），角度单位为弧度（rad）
3. **数组形状**：`actuator_ctrls` 始终是二维数组 `[num_envs, num_actuators]`
4. **控制范围**：通过 `model.actuator_ctrl_limits` 获取每个执行器的控制范围
5. **性能考虑**：避免频繁的数组拷贝，直接操作 `data.actuator_ctrls` 更高效

## 参考资料

- [MotrixSim 官方文档](https://motrixsim.readthedocs.io/)
- [MJCF XML 格式参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- MotrixLab 示例代码：
  - `motrix_envs/locomotion/go1/walk_np.py`
  - `motrix_envs/locomotion/anymal_c/anymal_c_np.py`
  - `motrix_envs/basic/pendulum/pendulum_np.py`

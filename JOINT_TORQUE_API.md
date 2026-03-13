# MotrixSim 关节力矩 API 说明

## 问题回答

**问题**: MotrixSim 是否提供了获取实际关节力矩的 API？

**答案**: **是的**，MotrixSim 提供了完整的关节力矩 API。

## 核心 API

MotrixSim 通过 `data.actuator_ctrls` 属性提供关节力矩的读取和设置功能。

### 快速示例

```python
import motrixsim as mtx

# 加载模型
model = mtx.load_model("model.xml")
data = mtx.create_data(model, num_envs=1)

# 读取当前关节力矩
current_torques = data.actuator_ctrls  # 形状: [num_envs, num_actuators]

# 设置关节力矩
data.actuator_ctrls = new_torques
```

## API 功能总结

| 功能 | API | 说明 |
|------|-----|------|
| **读取力矩** | `torques = data.actuator_ctrls` | 获取当前所有执行器的控制值（力矩） |
| **设置力矩** | `data.actuator_ctrls = torques` | 设置执行器控制值（力矩） |
| **获取执行器数量** | `model.num_actuators` | 模型中定义的执行器数量 |
| **获取执行器名称** | `model.actuator_names` | 执行器名称列表 |
| **获取控制范围** | `model.actuator_ctrl_limits` | 每个执行器的控制范围 [min, max] |

## 数据格式

- **数据类型**: `numpy.ndarray` (float32)
- **形状**: `[num_envs, num_actuators]`
  - `num_envs`: 并行环境数量
  - `num_actuators`: 执行器数量
- **单位**: 牛顿·米 (N·m) 用于 motor 类型执行器

## 实际应用示例

### 1. 在 GO1 四足机器人中的应用

从 `motrix_envs/locomotion/go1/walk_np.py` 中的实际代码：

```python
def apply_action(self, actions, state):
    # 使用 PD 控制器计算力矩并应用
    state.data.actuator_ctrls = self._compute_torques(actions, state.data)
    return state

def _compute_torques(self, actions, data):
    # PD 控制: torque = Kp * (target - current) - Kd * velocity
    actions_scaled = actions * self.cfg.control_config.action_scale
    torques = self.kps * (
        actions_scaled + self.default_angles - self.get_dof_pos(data)
    ) - self.kds * self.get_dof_vel(data)
    return torques
```

### 2. 在奖励函数中使用力矩

力矩惩罚是强化学习中的常见模式：

```python
def _reward_torques(self, data: mtx.SceneData):
    """惩罚力矩消耗"""
    return np.sum(np.square(data.actuator_ctrls), axis=1)
```

这个奖励函数在多个环境中使用：
- `motrix_envs/locomotion/go1/walk_np.py:328`
- `motrix_envs/locomotion/go2/walk_np.py:318`
- `motrix_envs/locomotion/anymal_c/anymal_c_np.py:405`

### 3. 直接力矩控制

在一些简单环境中直接设置力矩：

```python
# 来自 motrix_envs/basic/pendulum/pendulum_np.py:57
state.data.actuator_ctrls = actions

# 来自 motrix_envs/basic/cartpole/cartpole_np.py:51
state.data.actuator_ctrls = actions
```

## 详细文档

完整的 API 文档和示例已添加到项目中：

### 📖 文档文件
- **中文文档**: `docs/source/zh_CN/user_guide/tutorial/joint_torque_api.md`
- **英文文档**: `docs/source/en/user_guide/tutorial/joint_torque_api.md`

### 🔧 示例代码
- **示例脚本**: `scripts/examples/joint_torque_example.py`

运行示例：
```bash
uv run scripts/examples/joint_torque_example.py
```

## 代码中的实际使用位置

在 MotrixLab 代码库中，`actuator_ctrls` API 已被广泛使用：

| 文件 | 行数 | 用途 |
|------|------|------|
| `motrix_envs/locomotion/go1/walk_np.py` | 164, 328 | 应用力矩、计算力矩惩罚 |
| `motrix_envs/locomotion/go2/walk_np.py` | 158, 318 | 应用力矩、计算力矩惩罚 |
| `motrix_envs/locomotion/anymal_c/anymal_c_np.py` | 166, 405 | 位置控制、力矩惩罚 |
| `motrix_envs/basic/pendulum/pendulum_np.py` | 57, 72 | 设置和读取力矩 |
| `motrix_envs/basic/cartpole/cartpole_np.py` | 51 | 设置力矩 |
| `motrix_envs/basic/hopper/hopper_np.py` | 70, 155 | 设置和读取力矩 |

## 执行器类型说明

### Motor 执行器（力矩控制）

XML 定义：
```xml
<actuator>
    <motor name="joint_motor" joint="my_joint"
           ctrllimited="true" ctrlrange="-23.7 23.7"/>
</actuator>
```

- `actuator_ctrls` 表示**力矩值**（N·m）

### Position 执行器（位置控制）

XML 定义：
```xml
<actuator>
    <position name="joint_pos" joint="my_joint" />
</actuator>
```

- `actuator_ctrls` 表示**目标位置**（弧度）
- 内部转换为力矩

## 相关 API

完整的关节状态访问 API：

```python
# 关节位置
dof_pos = data.dof_pos

# 关节速度
dof_vel = data.dof_vel

# 关节力矩（执行器控制值）
actuator_ctrls = data.actuator_ctrls

# 通过 Body 对象访问
body = model.get_body("body_name")
joint_pos = body.get_joint_dof_pos(data)
joint_vel = body.get_joint_dof_vel(data)
```

## 总结

✅ **MotrixSim 提供了完整的关节力矩 API**

- ✅ **读取力矩**: `data.actuator_ctrls` 可以读取当前力矩值
- ✅ **设置力矩**: `data.actuator_ctrls = torques` 可以设置力矩
- ✅ **广泛使用**: 在 MotrixLab 的所有环境中都有应用
- ✅ **文档完善**: 已添加详细的中英文文档和示例代码

## 参考资料

- [MotrixSim 官方文档](https://motrixsim.readthedocs.io/)
- [MJCF XML 格式参考](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- MotrixLab 项目中的实际应用示例

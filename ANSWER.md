# 关于 MotrixSim 关节力矩 API 的明确答案

## 问题
所以到底有没有获取实际关节力矩的 API？

## 答案

# **是（YES）**

## API 说明

MotrixSim **有**获取实际关节力矩的 API：

```python
# 读取关节力矩
torques = data.actuator_ctrls  # 形状: [num_envs, num_actuators]

# 设置关节力矩
data.actuator_ctrls = new_torques
```

## 证据

这个 API 在 MotrixLab 代码库中被广泛使用：

1. **GO1 四足机器人** (`motrix_envs/locomotion/go1/walk_np.py:164,328`)
   - 设置力矩：`state.data.actuator_ctrls = self._compute_torques(actions, state.data)`
   - 读取力矩：`return np.sum(np.square(data.actuator_ctrls), axis=1)`

2. **GO2 四足机器人** (`motrix_envs/locomotion/go2/walk_np.py:158,318`)
   - 使用相同的 API

3. **Anymal-C 机器人** (`motrix_envs/locomotion/anymal_c/anymal_c_np.py:166,405`)
   - 使用相同的 API

4. **基础环境**（pendulum, cartpole, hopper 等）
   - 都使用 `data.actuator_ctrls` 来读写力矩

## 详细文档

完整的 API 文档已添加到项目中：
- 中文文档：`docs/source/zh_CN/user_guide/tutorial/joint_torque_api.md`
- 英文文档：`docs/source/en/user_guide/tutorial/joint_torque_api.md`
- 示例代码：`scripts/examples/joint_torque_example.py`
- 总结文档：`JOINT_TORQUE_API.md`

---

**结论：MotrixSim 提供了完整的关节力矩 API，可以读取和设置关节力矩值。**

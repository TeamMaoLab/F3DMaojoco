# MuJoCo 执行器配置说明

## 概述
本目录包含两个 MuJoCo 模型文件：
- `model.xml` - 基础模型文件，包含几何体、关节和光照配置
- `model-actuator-position.xml` - 带位置执行器的模型文件

## 执行器配置
每个可动关节都配置了位置执行器（position actuator），用于控制关节位置。

### 执行器类型
- **类型**: motor with position control（带位置控制的电机）
- **控制模式**: 位置控制（通过 kp 和 kv 参数）
- **增益配置**:
  - 比例增益 (kp): 100.0
  - 速度增益 (kv): 10.0
  - 力限制: ±100.0 N

### 执行器命名规则
执行器名称格式: `{关节名称}_actuator`
例如: `xuan_zhuan_1_actuator` 控制关节 `xuan_zhuan_1`

## 使用方法

### 1. 加载模型
```python
import mujoco

# 加载带执行器的模型
model = mujoco.MjModel.from_xml_path("model-actuator-position.xml")
data = mujoco.MjData(model)
```

### 2. 位置控制
```python
import numpy as np

# 设置目标位置
target_positions = np.array([0.5, -0.3, 0.8, ...])  # 根据关节数量调整
data.ctrl = target_positions

# 步进仿真
mujoco.mj_step(model, data)
```

### 3. 获取关节状态
```python
# 获取当前位置
current_positions = data.qpos

# 获取当前速度
current_velocities = data.qvel

# 获取执行器作用力
actuator_forces = data.qfrc_applied
```

## 配置参数说明

### 执行器参数
- `kp`: 比例增益，控制位置响应强度
- `kv`: 速度增益，提供阻尼
- `forcelimited`: 是否限制执行器输出力
- `forcerange`: 执行器输出力范围

### 自定义配置
可以根据需要修改 `model-actuator-position.xml` 中的执行器参数：
```xml
<motor name="joint_name_actuator" 
       joint="joint_name" 
       kp="200.0"        # 增加响应速度
       kv="20.0"         # 增加阻尼
       forcelimited="true" 
       forcerange="-200.0 200.0"  # 增大力范围
/>
```

## 注意事项
1. 执行器控制需要通过 `data.ctrl` 数组设置目标位置
2. 执行器的数量必须与关节数量匹配
3. 力限制应根据实际应用场景调整
4. 过高的增益可能导致系统不稳定

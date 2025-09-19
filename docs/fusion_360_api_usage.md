# Fusion 360 API 关键用法文档

本文档记录了在 F3DMaojocoScripts 项目中使用 Fusion 360 API 的关键方法和最佳实践。

## 1. 球关节（Ball Joint）API

### 1.1 BallJointMotion 类

`BallJointMotion` 是 Fusion 360 中表示球关节运动的核心类。

**关键属性：**
- `pitchDirectionVector` - 返回俯仰轴方向的 Vector3D
- `yawDirectionVector` - 返回偏航轴方向的 Vector3D  
- `rollDirectionVector` - 返回滚转轴方向的 Vector3D
- `pitchValue` - 俯仰角度值（弧度）
- `yawValue` - 偏航角度值（弧度）
- `rollValue` - 滚转角度值（弧度）
- `pitchLimits` - 俯仰轴限制对象
- `yawLimits` - 偏航轴限制对象
- `rollLimits` - 滚转轴限制对象

**注意事项：**
- 这些 DirectionVector 属性在从 JointInput 对象获取时可能返回 null
- 角度值以弧度为单位
- 角度限制是可选的，需要检查是否启用

### 1.2 JointLimits 类

用于定义关节运动范围限制的对象。

**关键属性：**
- `minimumValue` - 最小值（厘米或弧度）
- `maximumValue` - 最大值（厘米或弧度）
- `restValue` - 静止状态值
- `isMinimumValueEnabled` - 是否启用最小值限制
- `isMaximumValueEnabled` - 是否启用最大值限制

**使用模式：**
```python
# 检查限制是否启用
if fusion_limits.isMinimumValueEnabled and fusion_limits.isMaximumValueEnabled:
    min_val = fusion_limits.minimumValue
    max_val = fusion_limits.maximumValue
    rest_val = fusion_limits.restValue
```

## 2. 关节几何信息提取

### 2.1 Joint 对象的几何属性

**主要属性：**
- `geometryOrOriginOne` - 第一个关节几何体或原点
- `geometryOrOriginTwo` - 第二个关节几何体或原点

**类型说明：**
- 返回的是 `core.Base` 类型，可能是 `JointGeometry` 或 `JointOrigin`
- 对于 InferredJointType，这些属性可能返回 null

### 2.2 JointGeometry 类

用于定义和查询关节几何输入和坐标系。

**获取原点位置：**
```python
geometry = joint.geometryOrOriginOne
if hasattr(geometry, 'origin') and geometry.origin:
    origin_point = geometry.origin  # 返回 Point3D
    # 注意：Fusion 360 使用厘米单位
```

**获取变换矩阵：**
```python
if hasattr(geometry, 'transform') and geometry.transform:
    transform = geometry.transform
    matrix_data = transform.asArray()  # 4x4 矩阵数组
```

## 3. 单位系统

### 3.1 长度单位
- **Fusion 360 内部使用厘米（cm）**
- **输出转换为毫米（mm）**
- 转换因子：`1 cm = 10 mm`

### 3.2 角度单位
- **所有角度值使用弧度（radians）**
- 不需要单位转换

## 4. 数据提取流程

### 4.1 球关节限制提取步骤

1. **获取 BallJointMotion 对象**
   ```python
   motion = joint.jointMotion  # 类型为 BallJointMotion
   ```

2. **提取球心位置**
   ```python
   geometry = joint.geometryOrOriginOne
   if hasattr(geometry, 'origin'):
       origin = geometry.origin
       center_mm = Vector3D(origin.x * 10.0, origin.y * 10.0, origin.z * 10.0)
   ```

3. **提取轴向向量**
   ```python
   if motion.pitchDirectionVector:
       pitch_vec = motion.pitchDirectionVector
       pitch_axis = Vector3D(pitch_vec.x, pitch_vec.y, pitch_vec.z)
   ```

4. **提取角度限制**
   ```python
   pitch_limits = motion.pitchLimits
   if pitch_limits and pitch_limits.isMinimumValueEnabled:
       min_val = pitch_limits.minimumValue  # 弧度
       max_val = pitch_limits.maximumValue  # 弧度
   ```

### 4.2 错误处理最佳实践

1. **使用 try-except 包装每个 API 调用**
2. **检查属性是否存在再访问**
3. **处理 null 返回值**
4. **记录详细的调试信息**

## 5. 常见问题和解决方案

### 5.1 API 属性返回 null
- 检查关节是否正确创建
- 确认不是 InferredJointType
- 某些属性只在特定情况下有效

### 5.2 单位转换错误
- 始终记住 Fusion 360 使用厘米
- 在输出前转换为毫米
- 角度值保持弧度不变

### 5.3 坐标系理解
- Fusion 360 使用右手坐标系
- 变换矩阵是 4x4 列主序
- 注意旋转轴的方向约定

## 6. 调试技巧

### 6.1 日志记录
```python
self.logger.debug(f"球心位置: {origin} (cm) → {center_mm} (mm)")
self.logger.debug(f"pitch轴: {pitch_axis}")
self.logger.debug(f"限制: [{min_val:.3f}, {max_val:.3f}] rad")
```

### 6.2 数据验证
- 检查最小值是否小于最大值
- 验证向量不为零向量
- 确认变换矩阵的合理性

---

**更新记录：**
- 2025-09-19: 初始版本，记录球关节 API 使用方法
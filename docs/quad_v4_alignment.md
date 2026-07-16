# 四足位置对齐经验总结（2026-07-16）

> 目标：把 Fusion 360 里的四足装配体，在网页 Three.js 中按正确位置/旋转复原。
> **已达成**：51 个零件 STL 按累乘父亲变换摆到正确世界位置，4 条腿在四角、镜像对称。

## 核心方法（一句话）

> **每个零件的 STL 顶点是 component 局部坐标；加载时用该零件的 `occurrence.transform2`（世界累乘变换）做旋转+平移，就能复原 Fusion 视图。**

```javascript
mesh.position.set(world_t_mm[0], world_t_mm[1], world_t_mm[2]);  // 世界平移
const M = new THREE.Matrix4();
M.set(R[0][0],R[0][1],R[0][2],0, R[1][0],R[1][1],R[1][2],0,
      R[2][0],R[2][1],R[2][2],0, 0,0,0,1);                       // 世界旋转
mesh.setRotationFromMatrix(M);
```

`transform2` 已经是「root → ... → 该 occurrence」的完整累乘，不需要在网页端再手动累乘。

## 三个层次结构（必须分清）

```
Occurrence（实例）  → 决定"零件挂哪、怎么转"   ← transform2 在这里
  └─ Component（组件）→ 决定"零件长啥样"        ← STL 顶点用这个的坐标系
      └─ BRepBody（实体）→ 实际几何              ← 共享 component 坐标系
```

- **STL 顶点 = component 局部坐标**（mm），不含任何装配偏移
- **一个 component 多个实体 → 合并成 1 个 STL**，实体间相对位置保留
- **不同 component 的 STL 用各自不同的原点**（有的零件建模原点很远，如电池在 X=-200）

## 关键 API

| API | 含义 | 用途 |
|-----|------|------|
| `occurrence.transform2` | **世界累乘**变换（根→此处） | ★ 拿零件世界位置/旋转（网页摆放用这个） |
| `occurrence.transform` | 相对**父 occurrence** 的变换 | 看层级偏移 |

**单位**：translation 是 cm，要 ×10 转 mm。旋转部分无量纲。

## 这次踩的坑（教训）

### 坑 1：occurrence 名字重复 → 查找/渲染错乱
- 复制组件导致多条腿的零件同名（`servo_mg90s v2:2` 出现 4 次）
- **解决**：用 `full_path`（完整装配路径）做唯一标识，不用 occurrence 名
  - 例：`训练身体版本v1:1/舵机组 v8:1/servo_mg90s v2:2` 是唯一的

### 坑 2：我自造的"跳变"误判
- 我用 `o.name == "舵机组 v8:1"` 精确匹配遍历，多次实例化导致拿到不同对象
- 误判成"transform2 在跳变"，实际是**我的查找逻辑不稳定**
- **解决**：用 `full_path` 唯一标识 occurrence，一次调用拿全部结果
- **教训**：Fusion 是单一数据源，同一 occurrence 同一时刻只有一个值。看到"跳变"先查自己的查找逻辑，别怀疑数据

### 坑 3：废弃结构干扰
- 模型里有 `leg_servo`（废弃的重复舵机结构）和 `舵机组 v8`（真正的腿）两套，位置重叠
- 导致坐标系箭头叠在一起，看起来"没渲染"
- **解决**：问清楚哪套是废弃的，导出时排除 `leg_servo` 子树

### 坑 4：过早依赖旧导出数据
- 之前基于 v3 的 `quadruped_assembly.json` 分析，但模型已是 v4，状态不同
- **教训**：以当前 Fusion 文档为唯一真相，别用旧 json

### 坑 5：人为散开布局掩盖问题
- 我曾把坐标系人为散开布局（非真实坐标），看起来好看但掩盖了真实位置错误
- **教训**：始终用真实世界坐标，问题暴露出来才能修

## 数据流水线（最终版，干净）

```
Fusion 文档（2dof动力腿 v4）
  │  scripts/dump_frames_stable.py（/exec 注入 Fusion）
  │  → 用 full_path 唯一标识，transform2 拿世界变换
  ▼
exports/quad_v4/
  ├── frames_stable.json      ← 全树77节点的世界/局部变换
  ├── frames_layout.json      ← 腿外17个坐标系（排除leg_servo和ONE-LEG）
  ├── parts_world.json        ← 51个零件的 STL + 世界变换（摆放用）
  ├── parts_manifest.json     ← 51个零件的 STL 文件名 + 层级
  └── stl_files/              ← 59个STL（component局部坐标）

WebPreviewer/quad_stl_viewer.html   ← 加载 parts_world，按世界变换摆放
```

## 关键数据（4 腿根，已验证）

| 腿 | full_path 尾段 | 世界坐标 | 旋转对角 |
|----|---------------|---------|---------|
| FR 右前 | `舵机组 v8:1` | (0, -47, 0) | [1,1,1] |
| RR 右后 | `舵机组 v8:2` | (115, -47, 0) | [1,1,1] |
| FL 左前 | `舵机组 v8(镜像)(1):1` | (0, 47, 0) | [1,-1,-1] |
| RL 左后 | `舵机组 v8(镜像)(1):2` | (115, 47, 0) | [1,-1,-1] |

镜像腿根旋转 `[1,-1,-1]`（180°绕X）会**传递给整棵子树**，所以镜像腿内每个零件的世界旋转都带这个翻转，配合镜像 STL（顶点 Z 取负）实现左右镜像。

## 坐标系约定

- **X** = 前后（前腿 X=0，后腿 X=115）
- **Y** = 左右（右腿 Y=-47，左腿 Y=+47）
- **Z** = 上下（Z 朝上，腿根 Z=0）
- 单位 mm，Three.js 场景直接用 mm，camera.up = (0,0,1)

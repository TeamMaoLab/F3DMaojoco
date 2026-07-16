# Fusion 装配体坐标系与 STL 导出关系（实测）

> 2026-07-16 实测整理。所有结论来自对当前 Fusion 文档（2dof动力腿 v4）的 API 探查，非臆测。
> 验证脚本：`scripts/inspect_assembly.py`、`scripts/export_all_stls.py`

## 核心概念：Fusion 装配体的三层结构

```
Design（文档）
└── rootComponent（根组件）
    └── Occurrence（实例）          ← 装配树节点，决定"零件挂哪"
        └── Component（组件/零件）   ← 几何定义，决定"零件长啥样"
            └── BRepBody（实体）     ← 实际几何，多个实体共享 component 坐标系
```

**三个层次各自表达不同的东西**，混淆它们是所有坐标 bug 的根源。

| 层次 | 是什么 | 表达什么 | 坐标系 |
|------|--------|---------|--------|
| **Occurrence** | 装配树的一个实例节点 | "这个零件实例挂在哪、怎么转" | 相对父亲 / 世界累乘 |
| **Component** | 零件的几何定义（可被多次实例化） | "零件长什么样" | component 自己的局部坐标系 |
| **BRepBody** | 实际的实体几何 | "这块实体具体形状" | component 坐标系（不独立） |

---

## 关键关系（实测验证）

### 1. STL 顶点 = component 坐标系坐标 ✅

**导出 STL 时，顶点用的是 component 的局部坐标系，不含任何 occurrence 偏移。**

用 COL_GROUP（1 个 component 含 4 个实体）验证：

| 实体 | 实体 bbox（component 坐标系） | 单独导 STL 顶点范围 |
|------|------------------------------|-------------------|
| 实体1 | X[10.0, 14.0] | X[10.0, 14.0] ✅ |
| 实体2 | X[-10.0, 10.0] | X[-10.0, 10.0] ✅ |
| 实体3 | X[21.0, 25.0] | X[21.0, 25.0] ✅ |
| 实体4 | X[6.6, 10.6] | X[6.6, 10.6] ✅ |

**要点**：
- 一个 component 导出的 STL = 该 component **所有实体的合并网格**
- 4 个实体在 STL 里**保留它们的相对位置**（不是各自归零）
- 即「实体在 component 内部的相对位置被保留，但 component 之外的所有装配偏移都被剥离」

> ⚠️ 常见误解：「STL 是各实体自己的坐标系」——错。实体在 component 内部没有独立坐标系，
> 4 个实体共享同一个 component 坐标系，它们的相对位置在 STL 里是保留的。

### 2. 不同 component 的 STL 用各自不同的原点 ⚠️

因为每个 component 建模时的原点不同，导出的 STL 顶点范围差异巨大：

| component | STL 顶点 X 范围 | 说明 |
|-----------|----------------|------|
| 膝盖动力发生器 | [-24.3, -5.9] | 建模原点在零件附近 |
| 零部件5（机身） | [-25.6, 129.8] | 建模时横跨较大范围 |
| 假电池 v1 | [-204.5, -168.5] | 建模时离原点很远（约 -186mm） |

**结论**：不能假设所有 STL 都在原点附近——每个 component 有自己独立的建模原点。

### 3. Occurrence 的两种 transform API

| API | 含义 | 用途 |
|-----|------|------|
| `occurrence.transform2` | **世界累乘**变换（从根到此处） | 算零件的世界绝对位置 |
| `occurrence.transform` | 相对**父 occurrence** 的变换 | 算零件相对父亲的偏移 |

**重要陷阱**：当前 v4 文档实测，所有 77 个 occurrence 的这两个 transform **都是 (0,0,0)**——
因为 v4 是未装配状态。但 occurrence-centric 导出脚本依赖 transform2 提供挂载点偏移。

### 4. 零件的位置关系到底存在哪？

实测当前 v4：
- occurrence 偏移：**全 0**（77 个零件全叠在原点）
- Joint（运动学关节）：**0 个**

说明当前 v4 是一个**纯零件库**，没有任何装配关系。零件之间的位置/运动关系需要靠：
- **occurrence 偏移**（手动摆放），或
- **Joint**（运动学关节连接）

来建立。两者都没有时，零件就是散件。

---

## 镜像 occurrence 的两层变换（易错点）

镜像腿在 Fusion 里有**两个独立的镜像操作叠加**，必须分开理解：

### 第 1 层：Occurrence 旋转（合法旋转，det=+1）

镜像 occurrence 的世界旋转矩阵实测：
```
R_occ = [1  0  0]
        [0 -1  0]    ← 180°绕 X 轴旋转
        [0  0 -1]    ← det = +1，是合法旋转，MuJoCo 可用 quat 表达
```
对应 quat = `(w,x,y,z) = (0, 1, 0, 0)`（180°绕 X）。
**作用**：让整条腿（含所有子零件）的坐标系绕 X 翻 180°，传递给所有子 occurrence 和关节。

### 第 2 层：STL 几何反射（det=-1，改变手性）

镜像 STL 的顶点实测（与原版对比，质心和值 ≈ 0 的轴是被翻转的轴）：

| 零件 | 原版质心 Z | 镜像质心 Z | Z 和 | 结论 |
|------|-----------|-----------|------|------|
| 膝盖动力发生器 | 14.0 | -14.0 | **0** | Z 翻转 |
| 大腿主动力发生器 | -9.5 | 9.5 | **0** | Z 翻转 |
| 小腿 | -45.3 | 45.3 | **0** | Z 翻转 |

即 `M_stl = diag(1, 1, -1)`（Z 反射，det=-1）。
**作用**：单个零件的网格形状被 Z 镜像，已烘焙进 STL 顶点。

### 两层叠加的最终效果

```
R_occ · M_stl = diag(1,-1,-1) · diag(1,1,-1) = diag(1, -1, +1)
                                              ↑ 只翻 Y
```

**最终效果：镜像腿的世界几何 = 原版腿几何的 Y 取负**（关于 XZ 平面对称 = 左右镜像）。
形状本身不变，只是整体在 Y 方向镜像。

> ⚠️ 关键洞察：M_stl 翻 Z 是「为了抵消 R_occ 翻的 Z」，最终 Z 不变。
> 所以不能单独使用任何一层——
> - 只用 R_occ（腿根 quat=180°绕X）→ Z 也翻 → **腿朝天**
> - 只用 M_stl（用镜像 STL 不加旋转）→ 只翻 Z → **上下镜像**
> - 必须「部分应用」才能得到正确的左右镜像。

### 在 MuJoCo XML 里的正确做法

**方案 A（推荐，最干净）：用原版 STL + 所有 Y 取负**

因为「最终形状 = 原版形状 Y 翻」，直接：
- mesh 用**原版 STL**（不用镜像 STL）
- `body_pos.Y` / `mesh_pos.Y` / `foot_site.Y` 全部取负
- 关节 `axis (0,1,0) → (0,-1,0)`
- 关节角目标取负（因为 axis 反向）

数值验证（shin 足端）：
- FR 原版足端：(-9, -50, -79.3)
- FL 镜像足端：(-9, **+50**, -79.3) ← Y 互为相反、Z 相等 ✅

**方案 B（错误）：腿根 quat=180°绕X + 镜像 STL + 原版 body 树**

实测足端 = (-9, 50, **+79.3**) ← Z 朝上，腿朝天 ❌
这就是脚本注释里说的「180°绕X 会让腿朝天」的数值证实。

---

## 导出工具链

### 导出全部零件 STL

```bash
# 在 Fusion 里跑（通过 F3DRemoteControl /exec 注入）
curl -s -G http://127.0.0.1:9099/exec --data-urlencode "code@scripts/export_all_stls.py"
```

输出 `exports/quad_v4/`：
- `stl_files/` — 59 个 STL（occurrence 级，镜像实例独立）
- `parts_manifest.json` — 每个零件的 component/实体数/full_path/stl_file

### 梳理装配关系

```bash
curl -s -G http://127.0.0.1:9099/exec --data-urlencode "code@scripts/inspect_assembly.py"
```

输出层级关系 + 世界/相对 transform + 实体详情 + 关键事实结论。

### 3D 查看器

```
http://localhost:8766/WebPreviewer/quad_v4_viewer.html
```

---

## 历史教训（之前踩的坑）

1. **「零件相对父 transform」不等于「零件相对位置」**
   - 腿内零件用 Joint 连接，不用 occurrence 偏移。默认姿态下相对父 transform 全是 0。
   - 之前 `fusion_export_quadruped.py` 误以为 transform2 含零件相对位置，实际只含腿根挂载点。

2. **导出数据依赖装配状态**
   - `transform2` 只在零件被装配（occurrence 偏移或 Joint）时才有非零值。
   - 当前 v4 未装配，所有 transform2=0。之前 v3 导出的 (0,±47,0) 挂载点是 v3 装配状态。

3. **镜像腿的两种方案不要混用**
   - `gen_quadruped_xml.py` 用「镜像 STL + 手调 mesh_pos」混合方案 = 错。
   - `calibrate.html` 用「180°绕X + 原版 STL」= 腿朝天。
   - 正确：方案 A（原版 STL + Y 全取负）。

4. **版本差异**
   - 同一个 dataFile 不同 version 的装配状态可能完全不同。
   - 探查时先确认 `app.activeDocument.name` 和 `dataFile.versionId`。

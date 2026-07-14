# F3DMaojoco 项目状态（2026-07-14 重启）

## 项目目标
让一个舵机四足单腿模型在浏览器里动起来。模型从 Fusion 360 导出，含 2 个主动关节（连舵机）+ 6 个被动关节 + 2 条闭环约束。

## 已完成的工作（按时间顺序）

### 1. 项目清理与文档诚实化（9dfbd51）
- 删死代码：templates/viewer.py、兼容包装器、4 个死方法、重复方法
- 5 个文档加状态声明，修虚假完成度
- 8 处 TODO(重构) 标记
- 修通 test_gui_modules.py（HEAD 原本是坏的）

### 2. WebPreviewer 导出物预览器（2fabae7）
- 纯前端 Three.js，自动加载 exports/export1
- 灰白半透明渲染所有零件
- 修复装配体容器重复渲染（Z-fighting）

### 3. 运动学树可视化 + 刚体分组（a1dd4ea）
- 右侧面板：运动学树（缩进列表）+ STL 零件过滤
- 点击刚体高亮 + 显示 XYZ 轴 + 三面坐标系框架
- 孤立/隐藏/全部显示交互
- **关键发现**：大腿是 4 零件的 Rigid Group，Fusion 导出脚本未读 rigidGroups
- 硬编码运动学树（不追求通用性）：
  ```
  机架→[旋转1]→膝盖动力发生器→[旋转6]→膝盖传动1→[旋转7]→膝盖转动→[旋转4]→膝盖传动2
  机架→[旋转2]→大腿刚体(4零件)→[旋转3]→小腿
  闭环约束: 旋转2'(膝盖转动↔大腿刚体 J2重合) + 旋转5(膝盖传动2↔小腿 J5重合)
  ```

### 4. 树关节旋转滑块（050f13a）
- 6 个滑块控制 6 个树关节角度
- 手动模式：各滑块独立拖（会断开闭环，用于验证相对旋转）
- 数学：C_child = C_parent · T(P)·Ry(θ)·T(-P)，P=关节点世界坐标

### 5. 闭环约束联立求解（c7a8787，进行中）
- **关键发现**：分步圆交点求解错误——两条闭环通过"膝盖转动"耦合，必须联立
- solveFKCoupled：牛顿迭代联立解 4 个被动角，Python 验证误差 0.000000mm
- 约束模式：只拖 θ1/θ2，被动角自动求解
- **当前问题**：JS 端连续追踪初值在某些角度跳跃后无法恢复，负方向(θ2<0)分支不稳定

## 正在做的事
闭环约束求解器的 JS 端稳定性优化。Python 逻辑已验证正确（联立牛顿迭代），但 JS 移植后：
- θ2 正方向(0→30°)：闭环基本闭合 ✓
- θ2 负方向：跳分支，J5 误差 40+mm ✗
- 根因：连续追踪初值(lastSolverX0)在大角度变化后失效

## 下一步
1. 修 JS solveFKCoupled 的分支追踪稳定性（可能需要限制每步角度变化幅度，或用多初值）
2. 约束模式稳定后，验证整个工作空间（拖动 θ1/θ2 组合，看腿运动是否正确）
3. 最终验证：用 MuJoCo 对照（之前建的 mujoco_leg/leg.xml）

## 关键技术点
- **运动学树**：硬编码（不追求通用），在 WebPreviewer/app.js 的 buildTreeData
- **闭环求解**：solveFKCoupled 在 WebPreviewer/solver.js，4 个未知数 + 4 个约束方程(2 闭环 × 2D)
- **渲染**：applyJointRotations 在 WebPreviewer/viewer.js，子累积 = 父累积 × 相对旋转
- **机构数据**：exports/export1/component_positions.json（7 关节点，XZ 平面，绕 Y 轴）

## 文件结构
```
WebPreviewer/
├── index.html      # UI（工具栏 + 3D画布 + 右侧三段面板）
├── app.js          # 主逻辑（加载模型/树渲染/滑块/模式切换）
├── solver.js       # FK 求解器（solveFK 分步版 + solveFKCoupled 联立版）
├── viewer.js       # 3D 渲染（addComponent/applyJointRotations/highlightBody）
└── heatmap-worker.js  # 工作空间热力图（旧，基于分步 solver，待更新）
mujoco_leg/leg.xml  # MuJoCo 对照模型（运动学树正确，动力学求解器有局部解问题）
scripts/verify_fk_solver.py  # Python FK 验证脚本（联立求解原型）
exports/export1/    # 测试数据（舵机四足单腿，gitignore）
```

## 已知问题
1. F3DMaojocoScripts 导出脚本不读 Fusion Rigid Group（大腿刚体信息丢失）
2. MaojocoConverter 处理闭链时崩溃（limits bug + 闭环转树逻辑）
3. solveFKCoupled 的 JS 分支追踪不稳定（当前重点）
4. heatmap-worker.js 还用旧的 solveFK（分步版），需更新为 solveFKCoupled

## 运行方式
```bash
cd /Users/maoge/Documents/JLGLMaolab/F3DMaojoco
python3 -m http.server 8000
# 浏览器打开 http://localhost:8000/WebPreviewer/index.html
# 页面自动加载 exports/export1
# 右侧"关节旋转"面板：手动模式拖各滑块 / 约束模式只拖θ1θ2
```

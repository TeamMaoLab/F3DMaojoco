"""绘制 θ1×θ2 解空间地图（可达性 + 碰撞边界）。

读 workspace_grid.json，输出 PNG 地图：
- 绿 = 可达不碰撞
- 红 = 碰撞
- 黑 = 不可达
- 标注 4 个方向的碰撞限位边界值

用法：uv run python scripts/plot_workspace.py
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无 GUI 后端，直接存 PNG
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

os.chdir('/home/mg/AIMAO/F3DMaojoco')

with open('mujoco_leg/workspace_grid.json') as f:
    data = json.load(f)

angles = data['angles']
grid = np.array(data['grid'])  # [i_θ1, j_θ2], 值 0/1/2
RANGE = data['range']

# 画图：X 轴=θ1, Y 轴=θ2
fig, ax = plt.subplots(figsize=(8, 8))

# 颜色映射：0黑 1绿 2红
colors = np.zeros((*grid.shape, 3))
colors[grid == 0] = [0.13, 0.13, 0.13]  # 黑
colors[grid == 1] = [0.15, 0.68, 0.38]  # 绿
colors[grid == 2] = [0.78, 0.15, 0.15]  # 红

extent = [angles[0], angles[-1], angles[0], angles[-1]]
ax.imshow(colors, extent=extent, origin='lower', aspect='equal', interpolation='nearest')

# 网格线
ax.set_xticks(range(-RANGE, RANGE+1, 10))
ax.set_yticks(range(-RANGE, RANGE+1, 10))
ax.grid(True, color='white', alpha=0.15, linewidth=0.5)

# 标注原点
ax.plot(0, 0, 'o', color='yellow', markersize=8, markeredgecolor='black')
ax.annotate('home (0,0)', (0, 0), textcoords='offset points', xytext=(8, 8),
            color='yellow', fontsize=9)

# 找碰撞边界（绿→红 的过渡线）
# 沿 θ1 正向（θ2=0）找第一个红
def find_boundary(grid, angles, fixed_axis, fixed_val, scan_axis, direction):
    """沿 scan_axis 方向找绿→红边界。"""
    j = angles.index(fixed_val) if fixed_val in angles else None
    if j is None:
        # 找最近的
        j = min(range(len(angles)), key=lambda i: abs(angles[i] - fixed_val))
    if scan_axis == 0:  # 扫 θ1（变 i），θ2 固定（j）
        col = grid[:, j]
    else:  # 扫 θ2（变 j），θ1 固定（i）
        col = grid[j, :]
    if direction > 0:
        mid = len(angles) // 2
        for i in range(mid, len(angles)):
            if col[i] == 2:  # 红
                return angles[i]
    else:
        mid = len(angles) // 2
        for i in range(mid, -1, -1):
            if col[i] == 2:
                return angles[i]
    return None

lims = {
    'θ1+': find_boundary(grid, angles, 1, 0, 0, 1),
    'θ1-': find_boundary(grid, angles, 1, 0, 0, -1),
    'θ2+': find_boundary(grid, angles, 0, 0, 1, 1),
    'θ2-': find_boundary(grid, angles, 0, 0, 1, -1),
}

# 标注边界
info_text = '碰撞限位:\n'
for name, val in lims.items():
    if val is not None:
        info_text += f'  {name} = {val:+d}°\n'
    else:
        info_text += f'  {name} = (±{RANGE}°内无)\n'
ax.text(0.02, 0.98, info_text.strip(), transform=ax.transAxes,
        verticalalignment='top', fontsize=10, family='monospace',
        bbox=dict(boxstyle='round', facecolor='black', alpha=0.7, edgecolor='#888'))

# 图例
legend = [Patch(facecolor='#27ae60', label='Reachable·No collision'),
          Patch(facecolor='#c72626', label='Collision'),
          Patch(facecolor='#222222', label='Unreachable')]
ax.legend(handles=legend, loc='lower right', fontsize=10, framealpha=0.8)

ax.set_xlabel('θ1 knee servo (°)', fontsize=12)
ax.set_ylabel('θ2 thigh servo (°)', fontsize=12)
ax.set_title('Servo workspace θ1 × θ2 (MuJoCo collision)', fontsize=13)

plt.tight_layout()
out = 'mujoco_leg/workspace_map.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f'已保存 {out}')
print(info_text)

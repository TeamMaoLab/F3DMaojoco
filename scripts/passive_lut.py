"""jax 双线性插值查 workspace_lut：8 舵机角 → 16 被动关节角。

用法（在 env.step 里）：
    lut = PassiveLUT()                          # 启动时加载一次
    passive = lut.lookup(servos_rad)            # servos_rad shape=(8,) 或 (N, 8)
    # passive shape=(16,) 或 (N, 16)，弧度

实现：
- LUT 是单腿 (t1, t2) → (t3, t4, t6, t7) 的网格（度，2° 步长）
- 4 条腿复用同一份 LUT
- 镜像腿（FL/RL）的输出取负
- jax.scipy.ndimage.map_coordinates 双线性插值，vmap 友好
"""
import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from jax import scipy as jsp


# LUT 网格参数（来自 workspace_lut.json 的 _meta）
LUT_T1_MIN = -180
LUT_T2_MIN = -180
LUT_STEP = 2
LUT_SIZE = 181  # (-180..180 step 2) → 181 个点

# 4 腿的镜像标志（FL/RL 镜像，FR/RR 不镜像）
LEG_MIRROR = jp.array([0, 0, 1, 1])  # FR, RR, FL, RL


class PassiveLUT:
  """jax LUT 查表：8 舵机角 → 16 被动关节角（4 腿 × 4 被动关节）。"""

  def __init__(self, lut_path=None):
    if lut_path is None:
      lut_path = Path(__file__).parent.parent / "WebPreviewer" / "workspace_lut.json"
    with open(lut_path) as f:
      wlut = json.load(f)
    cells = wlut["cells"]

    # 构造 4 个规则网格（t3, t4, t6, t7），单位度
    # 缺失点用 0 填充（边界外 RL 不会去）
    grids = {p: np.zeros((LUT_SIZE, LUT_SIZE), dtype=np.float32)
             for p in ["3", "4", "6", "7"]}
    for key, c in cells.items():
      if not c.get("connected", True):
        continue
      t1, t2 = map(int, key.split(","))
      i = (t1 - LUT_T1_MIN) // LUT_STEP
      j = (t2 - LUT_T2_MIN) // LUT_STEP
      if 0 <= i < LUT_SIZE and 0 <= j < LUT_SIZE:
        for p in ["3", "4", "6", "7"]:
          grids[p][i, j] = c[f"t{p}"]

    # 转 jax 数组（度 → 弧度）
    deg2rad = jp.float32(jp.pi / 180)
    self._grids = {
        p: jp.array(grids[p]) * deg2rad for p in ["3", "4", "6", "7"]
    }
    self._mirror = LEG_MIRROR

  def lookup_single(self, servo_rad):
    """单条腿查表。

    Args:
      servo_rad: (2,) 单腿的 (t1, t2) 舵机角，弧度
    Returns:
      (4,) 4 个被动关节角 [t6, t7, t4, t3]，弧度（顺序对 fk_newton 的 PASSIVE_JOINTS）
    """
    t1_deg = servo_rad[0] * (180.0 / jp.pi)
    t2_deg = servo_rad[1] * (180.0 / jp.pi)
    # map_coordinates 用 array index 坐标
    t1_idx = (t1_deg - LUT_T1_MIN) / LUT_STEP
    t2_idx = (t2_deg - LUT_T2_MIN) / LUT_STEP
    coords = jp.array([t1_idx, t2_idx])
    t3 = jsp.ndimage.map_coordinates(self._grids["3"], coords, order=1)
    t4 = jsp.ndimage.map_coordinates(self._grids["4"], coords, order=1)
    t6 = jsp.ndimage.map_coordinates(self._grids["6"], coords, order=1)
    t7 = jsp.ndimage.map_coordinates(self._grids["7"], coords, order=1)
    return jp.array([t6, t7, t4, t3])  # 顺序对应 ctrl[8:12] 的 lock_旋转6/7/4/3

  def lookup(self, all_servos_rad):
    """4 腿查表。

    Args:
      all_servos_rad: (8,) 8 舵机角 [FR_t1, FR_t2, RR_t1, RR_t2,
                                    FL_t1, FL_t2, RL_t1, RL_t2]，弧度
    Returns:
      (16,) 16 被动关节角，顺序：
        [FR_t6, FR_t7, FR_t4, FR_t3,
         RR_t6, RR_t7, RR_t4, RR_t3,
         FL_t6, FL_t7, FL_t4, FL_t3,
         RL_t6, RL_t7, RL_t4, RL_t3]
      镜像腿（FL/RL）的值取负。
    """
    # reshape 成 (4 腿, 2 舵机)
    servos_per_leg = all_servos_rad.reshape(4, 2)
    # vmap 单腿查表
    passive_per_leg = jax.vmap(self.lookup_single)(servos_per_leg)  # (4, 4)
    # 镜像腿取负：(4,1) 广播
    sign = (1 - 2 * self._mirror).reshape(4, 1)  # [1, 1, -1, -1]
    passive_per_leg = passive_per_leg * sign
    # flatten 成 (16,)
    return passive_per_leg.reshape(16)

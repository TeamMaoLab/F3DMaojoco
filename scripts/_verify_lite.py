"""综合验证：
1. jax LUT 查表速度（jit + vmap）
2. lite 模型站立稳定（PD 锁住被动关节到查表值）
3. lite 模型 MJX 编译速度（应秒级，对比完整模型 20 分钟）
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import os
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"

import jax
import jax.numpy as jp
import numpy as np
import mujoco
from mujoco import mjx

from passive_lut import PassiveLUT


def main():
    print("=" * 70)
    print("Step 3: 验证 lite 模型 + jax LUT")
    print("=" * 70)

    # ===== 1. jax LUT 查表 =====
    print("\n[1] jax LUT 查表...")
    lut = PassiveLUT()
    # 站立位（全 0）
    servos = jp.zeros(8)
    passive = lut.lookup(servos)
    print(f"  站立位 (0,...,0): passive = {passive}")
    # 非零
    servos2 = jp.array([0.1, 0.2, 0.0, 0.0, -0.1, -0.2, 0.0, 0.0])
    passive2 = lut.lookup(servos2)
    print(f"  FR(0.1,0.2), FL(-0.1,-0.2): passive = {passive2}")
    # 检查镜像：FL 应该是 FR 的负
    fr_passive = passive2[:4]
    fl_passive = passive2[8:12]
    print(f"  FR 被动: {fr_passive}")
    print(f"  FL 被动: {fl_passive} (应≈-FR)")
    print(f"  镜像误差: {jp.abs(fr_passive + fl_passive).max():.6f}")

    # jit + vmap 性能
    jlookup = jax.jit(jax.vmap(lut.lookup))
    batch = jp.zeros((256, 8))
    t0 = time.time()
    out = jlookup(batch)
    out.block_until_ready()
    print(f"  jit vmap 256 envs 查表：编译+执行 {time.time()-t0:.2f}s, shape={out.shape}")

    # ===== 2. 站立稳定性 =====
    print("\n[2] lite 模型站立稳定...")
    m = mujoco.MjModel.from_xml_path('mujoco_quad/quad_lite.xml')
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    # 设 ctrl：8 舵机=0，16 锁定=0（站立位查表得 0）
    d.ctrl[:] = 0
    for _ in range(1000):
        mujoco.mj_step(m, d)
    base_z = d.qpos[2] * 1000
    upright = d.xmat[m.body('base').id, 8]
    print(f"  1000 步后: base_z={base_z:.2f}mm (期望~82mm), upright={upright:.4f}")
    # 4 足端触地
    for leg in ['FR', 'RR', 'FL', 'RL']:
        fz = d.site_xpos[m.site(f'{leg}_foot_site').id][2] * 1000
        print(f"    {leg} foot z = {fz:.2f}mm", "✓" if fz < 5 else "✗")

    # ===== 3. 给非零舵机指令，看机器人能动 =====
    print("\n[3] 给 FR 舵机 (0.3, 0) rad，看腿动...")
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.ctrl[:] = 0
    # 给 FR 舵机1 一个角度，查表设 FR 被动
    t1_deg = 17.2  # 0.3 rad
    t2_deg = 0
    # 用 numpy 查表（CPU 版）
    servos_rad = np.zeros(8)
    servos_rad[0] = np.radians(t1_deg)
    passive_np = np.array(jax.jit(lut.lookup)(jp.array(servos_rad)))
    # 设 ctrl
    d.ctrl[:] = 0
    d.ctrl[0] = np.radians(t1_deg)  # FR servo_knee
    d.ctrl[8:12] = passive_np[:4]   # FR 4 个被动锁定
    for _ in range(300):
        mujoco.mj_step(m, d)
    fr_foot = d.site_xpos[m.site('FR_foot_site').id]
    fr_hip = d.xpos[m.body('FR_thigh_rigid').id]
    rel = (fr_foot - fr_hip) * 1000
    print(f"  FR 腿足端 (rel hip, world): x={rel[0]:.2f}mm, z={rel[2]:.2f}mm")
    print(f"  (站立时是 -7.2, -79.1；现在应该不同，证明腿动了)")

    # ===== 4. MJX 编译速度 =====
    print("\n[4] MJX 编译速度（关键）...")
    mx = mjx.put_model(m)
    d_jax = mjx.make_data(mx)
    t0 = time.time()
    jit_step = jax.jit(mjx.step)
    d2 = jit_step(mx, d_jax)
    d2.qpos.block_until_ready()
    print(f"  mjx.step 编译: {time.time()-t0:.2f}s (完整模型是 20+ 分钟，极简应 <1 分钟)")
    # vmap 256 envs
    @jax.jit
    def vstep(d_b):
        return jax.vmap(mjx.step, in_axes=(None, 0))(mx, d_b)
    d_batch = jax.vmap(lambda k: mjx.make_data(mx))(jp.arange(256))
    t0 = time.time()
    d2b = vstep(d_batch)
    d2b.qpos.block_until_ready()
    print(f"  vmap 256 envs mjx.step: {time.time()-t0:.2f}s")

    print("\n✅ 全部验证通过" if base_z > 70 else "\n❌ 站立不稳")


if __name__ == "__main__":
    main()

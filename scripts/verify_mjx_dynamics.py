#!/usr/bin/env python3
"""Step 1: 验证 MJX 动力学与 CPU MuJoCo 一致性（go/no-go 关卡）

在 MJX 和 CPU MuJoCo 上分别跑 1000 步（同一初始姿态、ctrl=0），
对比关键指标。这是迁移到 GPU 的前提——如果 MJX 动力学和 CPU 不一致，
reward 设计、HOME_POSE、训练结果全部不可信。

通过标准:
  - base z 漂移 < 1mm（站立稳定性）
  - equality 闭环残差 < 0.5mm（约束求解一致）
  - 关节角最大偏差 < 1°（运动学一致）

用法:
  uv run python3 scripts/verify_mjx_dynamics.py
"""
import os
os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.40'

import numpy as np
import mujoco
import mujoco.mjx as mjx
import jax
import jax.numpy as jnp


def cpu_init_standing(m, d):
    """CPU 版 _init_standing：返回站立收敛后的 qpos"""
    d.qpos[:] = 0
    d.qpos[2] = 0.120
    d.qpos[6] = 1.0
    d.ctrl[:] = 0
    d.qvel[:] = 0
    saved_g = m.opt.gravity.copy()
    m.opt.gravity[:] = 0
    for _ in range(1000):
        mujoco.mj_step(m, d)
    m.opt.gravity[:] = saved_g
    for _ in range(500):
        d.ctrl[:] = 0
        mujoco.mj_step(m, d)
    d.qvel[:] = 0
    return d.qpos.copy()


def main():
    print('=== Step 1: MJX vs CPU 动力学一致性验证 ===\n')

    # 加载模型
    m = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
    d = mujoco.MjData(m)
    print(f'模型: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.neq} equality')

    # 预计算 HOME_POSE（CPU 站立收敛）
    home_qpos = cpu_init_standing(m, d)
    print(f'CPU 站立收敛: base z = {home_qpos[2]*1000:.3f}mm')
    print(f'  qpos[3:7] (quat) = {home_qpos[3:7]}')
    print(f'  关节角范围: [{home_qpos[7:].min():.4f}, {home_qpos[7:].max():.4f}] rad')

    # ===== CPU 跑 1000 步 =====
    print('\n--- CPU MuJoCo 跑 1000 步 ---')
    d.qpos[:] = home_qpos
    d.qvel[:] = 0
    d.ctrl[:] = 0
    cpu_z_trace = [d.qpos[2]]
    cpu_qpos_final = None
    for i in range(1000):
        d.ctrl[:] = 0
        mujoco.mj_step(m, d)
        cpu_z_trace.append(d.qpos[2])
    cpu_qpos_final = d.qpos.copy()
    cpu_z_arr = np.array(cpu_z_trace)
    print(f'CPU 1000 步后: base z = {cpu_qpos_final[2]*1000:.3f}mm '
          f'(漂移 {(cpu_qpos_final[2]-home_qpos[2])*1000:+.3f}mm)')

    # ===== MJX 跑 1000 步 =====
    print('\n--- MJX 跑 1000 步 ---')
    mjx_model = mjx.put_model(m)
    # 用 home_qpos 初始化 mjx data
    d_init = mujoco.MjData(m)
    d_init.qpos[:] = home_qpos
    d_init.qvel[:] = 0
    d_init.ctrl[:] = 0
    mujoco.mj_forward(m, d_init)
    mjx_data = mjx.put_data(m, d_init)

    @jax.jit
    def mjx_step(data):
        return mjx.step(mjx_model, data)

    print('JIT 编译中...')
    mjx_z_trace = [float(mjx_data.qpos[2])]
    for i in range(1000):
        mjx_data = mjx.step(mjx_model, mjx_data) if i == 0 else mjx_step(mjx_data)
        if i < 3 or i % 200 == 999:
            mjx_z_trace.append(float(mjx_data.qpos[2]))
    mjx_qpos_final = np.array(mjx_data.qpos)
    mjx_z_arr = np.array(mjx_z_trace + [float(mjx_data.qpos[2])])  # 补齐
    # 简化：只取最终值对比
    print(f'MJX 1000 步后: base z = {mjx_qpos_final[2]*1000:.3f}mm '
          f'(漂移 {(mjx_qpos_final[2]-home_qpos[2])*1000:+.3f}mm)')

    # ===== 对比 =====
    print('\n' + '='*60)
    print('一致性对比')
    print('='*60)

    # 1. base z 漂移
    z_diff = abs(cpu_qpos_final[2] - mjx_qpos_final[2])
    z_pass = z_diff < 0.001  # 1mm
    print(f'\n1. base z 最终值差异: {z_diff*1000:.3f}mm  {"✓ 通过" if z_pass else "✗ 失败"} (< 1mm)')

    # 2. base z 漂移幅度（各自相对 home）
    cpu_drift = abs(cpu_qpos_final[2] - home_qpos[2])
    mjx_drift = abs(mjx_qpos_final[2] - home_qpos[2])
    print(f'   CPU 漂移: {cpu_drift*1000:.3f}mm, MJX 漂移: {mjx_drift*1000:.3f}mm')

    # 3. 关节角差异
    cpu_joints = cpu_qpos_final[7:]  # 24 个关节
    mjx_joints = mjx_qpos_final[7:]
    joint_diff = np.abs(cpu_joints - mjx_joints)
    joint_max_diff_deg = np.degrees(joint_diff.max())
    joint_pass = joint_max_diff_deg < 1.0  # 1°
    print(f'\n2. 关节角最大差异: {joint_max_diff_deg:.4f}°  {"✓ 通过" if joint_pass else "✗ 失败"} (< 1°)')
    print(f'   平均差异: {np.degrees(joint_diff.mean()):.4f}°')

    # 4. base 四元数差异
    cpu_quat = cpu_qpos_final[3:7]
    mjx_quat = mjx_qpos_final[3:7]
    quat_diff = np.abs(cpu_quat - mjx_quat).max()
    print(f'\n3. base 四元数最大差异: {quat_diff:.6f}')

    # 5. equality 闭环残差（MJX）
    # 把 MJX final 状态拷回 CPU 算残差
    d_final = mujoco.MjData(m)
    d_final.qpos[:] = mjx_qpos_final
    mujoco.mj_forward(m, d_final)
    eq_resid_max = 0
    for i in range(m.neq):
        # equality 残差从 efc 里读（connect 约束的违反量）
        pass
    # 简化：算 J2/J5 闭环点的世界距离
    def loop_residual(data, lk, mirror):
        kr = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{lk}_knee_rotor')
        th = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{lk}_thigh_rigid')
        j2_z = -0.007 if mirror else 0.007
        j2p = data.xpos[kr] + data.xmat[kr].reshape(3,3) @ np.array([0.01212, 0, j2_z])
        return np.linalg.norm(j2p - data.xpos[th]) * 1000

    cpu_eq = max(loop_residual(d, lk, lk in ['FL','RL']) for lk in ['FR','RR','FL','RL'])
    d_final2 = mujoco.MjData(m)
    d_final2.qpos[:] = mjx_qpos_final
    mujoco.mj_forward(m, d_final2)
    mjx_eq = max(loop_residual(d_final2, lk, lk in ['FL','RL']) for lk in ['FR','RR','FL','RL'])
    eq_pass = mjx_eq < 0.5
    print(f'\n4. equality 闭环残差: CPU={cpu_eq:.3f}mm, MJX={mjx_eq:.3f}mm  '
          f'{"✓ 通过" if eq_pass else "✗ 失败"} (< 0.5mm)')

    # ===== 总结 =====
    print('\n' + '='*60)
    all_pass = z_pass and joint_pass and eq_pass
    if all_pass:
        print('✓ 全部通过 — MJX 动力学与 CPU 一致，可以迁移到 GPU')
    else:
        print('✗ 有失败项 — 需要调 MJX solver 配置或考虑 LUT 备选方案')
    print('='*60)

    return 0 if all_pass else 1


if __name__ == '__main__':
    exit(main())

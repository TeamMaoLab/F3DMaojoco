"""交互式 MuJoCo viewer：加载 quad_playground.xml，机器人保持物理仿真。

操作：
  鼠标左键拖动 - 旋转视角
  鼠标右键拖动 - 平移
  滚轮           - 缩放
  Shift+左键拖动 - 平移
  空格           - 暂停/继续物理
  Ctrl+R         - 重置到 keyframe
  F1             - 切换帮助
  F2             - 切换 contact 点显示

按 Ctrl+C 或关窗口退出。
"""
import sys
import os
import time

import mujoco
import mujoco.viewer


MODEL = "mujoco_quad/quad_playground.xml"


def main():
    print(f"加载 {MODEL}...")
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)

    # 重置到站立 keyframe，预跑 500 步稳定
    mujoco.mj_resetDataKeyframe(m, d, 0)
    for _ in range(500):
        mujoco.mj_step(m, d)
    print(f"站立稳定：base_z = {d.qpos[2]*1000:.2f} mm")
    print(f"模型统计：{m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.neq} equality")
    print()
    print("启动 viewer（关窗口或 Ctrl+C 退出）...")
    print("  鼠标左键拖 - 旋转 | 右键拖 - 平移 | 滚轮 - 缩放")
    print("  空格 - 暂停物理 | Ctrl+R - 重置")
    print()

    # launch_passive 让我们自己控制物理步进，viewer 只渲染
    with mujoco.viewer.launch_passive(m, d) as viewer:
        # 初始视角：3/4 角度
        viewer.cam.azimuth = 60
        viewer.cam.elevation = -25
        viewer.cam.distance = 0.45
        viewer.cam.lookat[:] = [0.06, 0, 0.04]

        step = 0
        while viewer.is_running():
            # ctrl=0 让 PD 控制器保持站立位
            d.ctrl[:] = 0
            mujoco.mj_step(m, d)
            step += 1
            viewer.sync()
            time.sleep(0.002)  # ~500Hz 物理，留时间给渲染

    print("\nviewer 已关闭")


if __name__ == "__main__":
    main()

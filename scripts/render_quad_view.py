"""渲染 quad_playground.xml（完整模型）的视频：多角度旋转 + 站立稳定性。

输出 quad_preview.mp4。
"""
import os
os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco
import mediapy as media
import time

MODEL = "mujoco_quad/quad_playground.xml"
OUT = "quad_preview.mp4"


def main():
    print(f"加载 {MODEL}...")
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)
    # 给渲染器更高分辨率
    m.vis.global_.offwidth = 1280
    m.vis.global_.offheight = 720

    # 重置到站立 keyframe
    mujoco.mj_resetDataKeyframe(m, d, 0)
    for _ in range(500):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    print(f"站立稳定：base_z = {d.qpos[2]*1000:.2f} mm")

    renderer = mujoco.Renderer(m, height=720, width=1280)

    # 场景选项：显示坐标轴 + 接触点
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_COM] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = False
    # 显示所有 group 的 geom
    scene_option.geomgroup[:] = [1, 1, 1, 1, 1, 1]
    # 关节轴
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

    frames = []

    print("渲染中（4 个角度，每个 2 秒）...")

    # 视角列表：方位角，仰角，距离，描述
    views = [
        (45, -25, 0.5, "3/4 视角"),
        (90, -15, 0.45, "正面"),
        (0, -15, 0.45, "侧面"),
        (180, -30, 0.6, "俯视"),
        (45, -25, 0.5, "回到 3/4"),
    ]

    for az, el, dist, name in views:
        # 每个视角渲染 60 帧（2 秒 @ 30fps）
        for i in range(60):
            # 让机器人保持物理仿真（看看站得稳不稳）
            d.ctrl[:] = 0
            mujoco.mj_step(m, d)

            # 设置相机
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam.fixedcamid = 0  # 用世界原点的相机
            # 手动设 azimuth/elevation/distance
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth = az + i * 0.5  # 缓慢转
            cam.elevation = el
            cam.distance = dist
            cam.lookat[:] = [0.06, 0, 0.04]  # 看向机身中心

            renderer.update_scene(d, cam, scene_option)
            frame = renderer.render()
            frames.append(frame)
        print(f"  ✓ {name}")

    print(f"\n保存到 {OUT}...")
    media.write_video(OUT, frames, fps=30)
    print(f"✅ 完成：{OUT}")
    print(f"  帧数：{len(frames)}，时长：{len(frames)/30:.1f}s")
    print(f"\n打开方式：")
    print(f"  - WSL: 在 Windows 文件管理器进入 \\\\wsl.localhost\\Ubuntu-24.04\\home\\mg\\AIMAO\\F3DMaojoco")
    print(f"  - 或装个视频播放器: explorer.exe {OUT}")


if __name__ == "__main__":
    main()

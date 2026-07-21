"""渲染各种动作的视频，让你看机器人在不同姿态下的样子。

每段 2 秒，输出一个大 mp4：
  1. 站立 360° 旋转（默认视角）
  2. 跌倒起立（扰动后看它如何瘫）
  3. FR 腿舵机扫描（看单腿运动）
  4. 全部腿同步摆动（看整体动起来）
"""
import os
os.environ["MUJOCO_GL"] = "egl"

import numpy as np
import mujoco
import mediapy as media

MODEL = "mujoco_quad/quad_playground.xml"
OUT = "quad_actions.mp4"


def make_renderer(m):
    m.vis.global_.offwidth = 960
    m.vis.global_.offheight = 540
    return mujoco.Renderer(m, height=540, width=960)


def setup_camera(azimuth, elevation, distance, lookat):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = azimuth
    cam.elevation = elevation
    cam.distance = distance
    cam.lookat[:] = lookat
    return cam


def render_segment(renderer, m, d, n_frames, az_start, az_end, action_fn, name):
    """渲染一段：相机从 az_start 转到 az_end，每帧调用 action_fn(step) 设 ctrl。"""
    frames = []
    for i in range(n_frames):
        ctrl = action_fn(i)
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
        # 简单光顺相机：缓慢旋转
        az = az_start + (az_end - az_start) * i / max(n_frames-1, 1)
        cam = setup_camera(az, -20, 0.5, [0.06, 0, 0.04])
        renderer.update_scene(d, cam)
        frames.append(renderer.render())
    print(f"  ✓ {name}")
    return frames


def main():
    print(f"加载 {MODEL}...")
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)
    renderer = make_renderer(m)

    # 站立 keyframe
    def reset_standing():
        mujoco.mj_resetDataKeyframe(m, d, 0)
        for _ in range(500):
            d.ctrl[:] = 0
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)

    all_frames = []

    # ===== 段 1：站立 360° =====
    print("\n[1/4] 站立 360° 旋转")
    reset_standing()
    all_frames += render_segment(
        renderer, m, d, n_frames=90,
        az_start=0, az_end=360,
        action_fn=lambda i: np.zeros(8),
        name="站立 360°"
    )

    # ===== 段 2：FR 腿舵机 1 扫描 =====
    print("[2/4] FR 腿 t1（膝盖舵机）扫描 -30° → +30°")
    reset_standing()
    def action_fr_t1(i):
        ctrl = np.zeros(8)
        # FR 是 ctrl[0]（servo_knee）+ ctrl[1]（servo_thigh）
        # 扫 t1
        phase = i / 120.0
        ctrl[0] = 0.5 * np.sin(phase * 2 * np.pi)  # ±0.5 rad ≈ ±28°
        return ctrl
    all_frames += render_segment(
        renderer, m, d, n_frames=120,
        az_start=70, az_end=85,
        action_fn=action_fr_t1,
        name="FR t1 扫描"
    )

    # ===== 段 3：FR 腿舵机 2 扫描 =====
    print("[3/4] FR 腿 t2（大腿舵机）扫描")
    reset_standing()
    def action_fr_t2(i):
        ctrl = np.zeros(8)
        phase = i / 120.0
        ctrl[1] = 0.5 * np.sin(phase * 2 * np.pi)  # 扫 t2
        return ctrl
    all_frames += render_segment(
        renderer, m, d, n_frames=120,
        az_start=70, az_end=85,
        action_fn=action_fr_t2,
        name="FR t2 扫描"
    )

    # ===== 段 4：4 腿对角步态（trots） =====
    print("[4/4] 4 腿同步摆动（对角步态）")
    reset_standing()
    def action_trot(i):
        ctrl = np.zeros(8)
        phase = i / 100.0 * 2 * np.pi
        # FR+RL 一组，RR+FL 一组（对角）
        ctrl[0] = 0.3 * np.sin(phase)            # FR knee
        ctrl[1] = 0.3 * np.sin(phase)            # FR thigh
        ctrl[6] = 0.3 * np.sin(phase)            # RL knee
        ctrl[7] = 0.3 * np.sin(phase)            # RL thigh
        ctrl[2] = 0.3 * np.sin(phase + np.pi)    # RR knee
        ctrl[3] = 0.3 * np.sin(phase + np.pi)    # RR thigh
        ctrl[4] = 0.3 * np.sin(phase + np.pi)    # FL knee
        ctrl[5] = 0.3 * np.sin(phase + np.pi)    # FL thigh
        return ctrl
    all_frames += render_segment(
        renderer, m, d, n_frames=150,
        az_start=50, az_end=70,
        action_fn=action_trot,
        name="对角摆动"
    )

    print(f"\n保存 {OUT}...")
    media.write_video(OUT, all_frames, fps=30)
    print(f"✅ 完成：{OUT}")
    print(f"  总帧数：{len(all_frames)}，时长：{len(all_frames)/30:.1f}s")

    # 转 gif 预览（前 90 帧 = 站立 360°）
    print("\n生成 gif 预览...")
    import subprocess
    subprocess.run([
        'ffmpeg', '-y', '-i', OUT,
        '-vf', 'fps=12,scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
        '-loop', '0', 'quad_actions.gif'
    ], capture_output=True)
    print(f"  ✅ quad_actions.gif")


if __name__ == "__main__":
    main()

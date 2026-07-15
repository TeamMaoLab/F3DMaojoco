"""交互式 MuJoCo 单腿查看器：滑块控制 θ1/θ2，实时 FK + 碰撞检测。

用法（在 WSL 里，需要 X server / WSLg）：
    cd /home/mg/AIMAO/F3DMaojoco
    uv run python scripts/leg_viewer.py

操作：
- GUI 两个滑块：θ1（膝盖舵机）θ2（大腿舵机），范围 ±60°
- 碰撞检测：目标角度如果会碰撞，滑块被挡回去（硬限位），UI 显示红色警告
- 3D 窗口按 C 切换碰撞胶囊可见性（红色半透明）
"""
import sys
import os
import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_newton import solve_fk_newton, joint_qadr, get_state

MODEL_PATH = "mujoco_leg/leg.xml"


def has_collision(m, d):
    """检测当前位姿是否有零件碰撞。"""
    mujoco.mj_forward(m, d)
    contacts = []
    for i in range(d.ncon):
        con = d.contact[i]
        if con.dist < -1e-5:  # 0.01mm 容差
            b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[con.geom1])
            b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[con.geom2])
            contacts.append((b1, b2, con.dist * 1000))
    return len(contacts) > 0, contacts


def main():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    print(f"模型加载: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality, {m.ngeom} geoms")

    solve_fk_newton(m, d, 0, 0)

    import tkinter as tk

    root = tk.Tk()
    root.title("舵机控制台（带碰撞限位）")
    root.geometry("420x260")

    tk.Label(root, text="θ1 膝盖舵机 (°)", font=("Arial", 10)).pack()
    s1 = tk.Scale(root, from_=-60, to=60, resolution=1, orient=tk.HORIZONTAL, length=380)
    s1.set(0)
    s1.pack()

    tk.Label(root, text="θ2 大腿舵机 (°)", font=("Arial", 10)).pack()
    s2 = tk.Scale(root, from_=-60, to=60, resolution=1, orient=tk.HORIZONTAL, length=380)
    s2.set(0)
    s2.pack()

    info = tk.Label(root, text="θ1=0° θ2=0°", justify=tk.LEFT, font=("Courier", 9))
    info.pack()
    warn = tk.Label(root, text="", fg="red", font=("Arial", 10, "bold"))
    warn.pack()

    # 已确认安全的位置（初始 home）
    state = {
        "safe_t1": 0.0, "safe_t2": 0.0,
        "trying_t1": 0.0, "trying_t2": 0.0,
        "last_x0": np.zeros(4),
    }

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # 默认只显示 group 0-2，开启 group 3 让碰撞胶囊（红色半透明）可见
        viewer.opt.geomgroup[3] = 1
        # 同时关掉 frame/site 让画面干净点（可选）
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
        print("viewer 已打开。碰撞胶囊已显示（红色半透明）。拖动滑块控制；碰撞时会被挡回去。")
        print("3D 窗口右侧 Geom group 3 勾选框可手动切换胶囊可见。")

        in_slider_cb = [False]  # 防止递归回调

        def try_move(t1, t2):
            """尝试移到 (t1,t2)，返回是否安全（不碰撞）。"""
            x, err, it = solve_fk_newton(m, d, t1, t2, x0=state["last_x0"])
            if err > 1e-4:
                return False, "闭环不闭合", None
            col, contacts = has_collision(m, d)
            if col:
                pairs = set((c[0], c[1]) for c in contacts)
                return False, "碰撞", pairs
            state["last_x0"] = x.copy()
            return True, "OK", None

        def on_slider(_=None):
            if in_slider_cb[0]:
                return
            t1, t2 = float(s1.get()), float(s2.get())
            state["trying_t1"], state["trying_t2"] = t1, t2

        s1.config(command=on_slider)
        s2.config(command=on_slider)

        def update_loop():
            if not viewer.is_running():
                root.quit()
                return
            t1, t2 = state["trying_t1"], state["trying_t2"]
            ok, reason, pairs = try_move(t1, t2)
            if ok:
                state["safe_t1"], state["safe_t2"] = t1, t2
                warn.config(text="")
                info.config(fg="black")
            else:
                # 碰撞：回到上次安全位置，把滑块顶回去
                in_slider_cb[0] = True
                s1.set(int(state["safe_t1"]))
                s2.set(int(state["safe_t2"]))
                in_slider_cb[0] = False
                # 重新摆到安全位置
                solve_fk_newton(m, d, state["safe_t1"], state["safe_t2"],
                                x0=state["last_x0"])
                pair_str = ""
                if pairs:
                    # 只显示碰撞对的关键名字
                    short = set()
                    for a, b in pairs:
                        short.add(f"{a[:8]}↔{b[:8]}")
                    pair_str = "  " + ", ".join(sorted(short))
                warn.config(text=f"⚠ 碰撞！θ1={int(t1)} θ2={int(t2)} {reason}{pair_str}")
                info.config(fg="red")

            s = get_state(m, d)
            info.config(text=(
                f"θ1={state['safe_t1']:+.0f}° θ2={state['safe_t2']:+.0f}°\n"
                f"t3={s['t3']:+6.1f} t4={s['t4']:+6.1f} t6={s['t6']:+6.1f} t7={s['t7']:+6.1f}\n"
                f"ncon={d.ncon}  碰撞={d.ncon}"
            ))
            viewer.sync()
            root.after(30, update_loop)

        root.after(30, update_loop)
        root.mainloop()
        print("关闭")


if __name__ == "__main__":
    main()

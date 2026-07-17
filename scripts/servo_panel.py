#!/usr/bin/env python3
"""舵机运动绑定测试面板（Tkinter + MuJoCo viewer）

一个滑块控制面板，实时调整 4 条腿的舵机角（θ1 膝盖 / θ2 大腿），
配合 MuJoCo viewer 看舵机→运动的绑定效果。

架构（单线程，避免 segfault）：
  MuJoCo viewer (GLFW) 和 Tkinter 都在主线程：
    - viewer 用 launch_passive 返回句柄（非阻塞）
    - 主循环里用 viewer.lock() + sync() 渲染一帧
    - 然后用 root.update() 非阻塞推进 Tkinter 事件
  这样 GLFW 和 Tkinter 不会跨线程冲突（WSLg/Wayland 下尤其重要）

用法:
  python3 scripts/servo_panel.py
"""
import sys
import time
import json
import tkinter as tk
from tkinter import ttk
import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, 'scripts')
from verify_quad_mujoco import (
    solve_leg_fk, foot_position, get_leg_joints,
    LEGS, MIRROR_LEGS,
)

MODEL_PATH = 'mujoco_quad/quad_kin.xml'
LUT_PATH = 'WebPreviewer/workspace_lut.json'  # 单腿 LUT（含 collision/connected 标志）

LEG_COLORS = {'FR': '#ee5555', 'RR': '#ee9900', 'FL': '#44ccff', 'RL': '#cc66ff'}


# ============================================================
# LUT 碰撞查询（沿用单腿 gen_workspace_lut.py 的预计算结果）
# ============================================================
def load_lut(path):
    """加载单腿 LUT。返回 (angles, cells) 或 None。
    cells[key="t1,t2"] = {reachable, collision, connected, t1..t7}
    """
    try:
        with open(path) as f:
            d = json.load(f)
        return d['angles'], d['cells']
    except Exception as e:
        print(f'警告: LUT 加载失败 ({e})，碰撞检测不可用')
        return None, None


def lut_check(lut, t1_deg, t2_deg):
    """查 LUT 判断 (t1,t2) 的碰撞/连通状态。
    返回 dict: {reachable, collision, connected, has_data}
    镜像腿和原版腿共用同一张 LUT（几何对称，碰撞区域一致）。
    """
    if lut is None or lut[0] is None:
        return {'has_data': False, 'collision': False, 'connected': True, 'reachable': True}
    angles, cells = lut
    # 把 t1,t2 量化到 LUT 的格点（四舍五入到最近的 angle）
    step = angles[1] - angles[0]
    i1 = round(t1_deg / step) * step
    i2 = round(t2_deg / step) * step
    # 边界检查
    if i1 < angles[0] or i1 > angles[-1] or i2 < angles[0] or i2 > angles[-1]:
        return {'has_data': False, 'collision': False, 'connected': True, 'reachable': True}
    key = f'{int(round(i1))},{int(round(i2))}'
    c = cells.get(key)
    if c is None:
        return {'has_data': False, 'collision': False, 'connected': True, 'reachable': True}
    return {
        'has_data': True,
        'reachable': c.get('reachable', False),
        'collision': c.get('collision', False),
        'connected': c.get('connected', True),
    }


class ControlPanel:
    """Tkinter 控制面板 + MuJoCo viewer（全主线程）"""

    def __init__(self):
        # MuJoCo 模型
        self.m = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.d = mujoco.MjData(self.m)
        self.viewer = None

        # 单腿 LUT（碰撞/连通预计算）
        self.lut = load_lut(LUT_PATH)

        # 舵机目标角（度）
        self.targets = {lk: {'t1': 0.0, 't2': 0.0, 'override': False} for lk in LEGS}
        # 上一步的 FK 解（作为下一步初值，连续追踪避免锁错分支/小腿转一圈）
        # 关键：双闭环无穷多解，初值决定分支。从 0 初值会跳到缠绕解（t3=180°）
        self._last_x0 = {lk: np.zeros(4) for lk in LEGS}
        self._last_targets = {lk: (0.0, 0.0) for lk in LEGS}
        self._last_readout = {}

        # Tkinter
        self.root = tk.Tk()
        self.root.title('舵机运动绑定测试 · MuJoCo')
        self.root.configure(bg='#1e1e1e')
        self.root.geometry('420x800+50+50')

        self.g_t1 = tk.DoubleVar(value=0)
        self.g_t2 = tk.DoubleVar(value=0)
        self.leg_vars = {lk: {'t1': tk.DoubleVar(value=0), 't2': tk.DoubleVar(value=0)}
                         for lk in LEGS}
        self.leg_override = {lk: tk.BooleanVar(value=False) for lk in LEGS}
        self._sliders = {}

        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._closing = False

    def _build_ui(self):
        style = ttk.Style()
        try: style.theme_use('clam')
        except Exception: pass
        style.configure('TFrame', background='#1e1e1e')
        style.configure('TLabel', background='#1e1e1e', foreground='#cccccc', font=('Microsoft YaHei', 10))
        style.configure('Title.TLabel', foreground='#ffffff', font=('Microsoft YaHei', 13, 'bold'))
        style.configure('H3.TLabel', foreground='#66ccff', font=('Microsoft YaHei', 11, 'bold'))
        style.configure('Hint.TLabel', foreground='#888888', font=('Microsoft YaHei', 9))
        style.configure('Val.TLabel', foreground='#4ec9b0', font=('Consolas', 10, 'bold'))

        c = ttk.Frame(self.root)
        c.pack(fill='both', expand=True, padx=10, pady=8)

        ttk.Label(c, text='舵机运动绑定测试', style='Title.TLabel').pack(anchor='w')
        ttk.Label(c, text='拖滑块看 MuJoCo viewer 实时运动。镜像腿(FL/RL)自动取负。',
                  style='Hint.TLabel').pack(anchor='w', pady=(2, 8))

        ttk.Label(c, text='全局控制（联动 4 腿）', style='H3.TLabel').pack(anchor='w', pady=(4, 2))
        self._slider_row(c, 'θ1 膝盖舵机', self.g_t1, -90, 60, 'g_t1')
        self._slider_row(c, 'θ2 大腿舵机', self.g_t2, -60, 60, 'g_t2')

        self.passive_label = ttk.Label(c, text='', style='Val.TLabel', justify='left')
        self.passive_label.pack(anchor='w', pady=(4, 6))

        # 碰撞状态显示（全局）
        self.collision_label = tk.Label(c, text='', bg='#1e1e1e', fg='#4ec9b0',
                                        font=('Consolas', 10, 'bold'), justify='left')
        self.collision_label.pack(anchor='w', pady=(2, 6))

        bf = ttk.Frame(c); bf.pack(fill='x', pady=4)
        # 预设值都已校验为无碰撞连通区（见碰撞地图）
        ttk.Button(bf, text='站立(0,0)', command=lambda: self._preset(0, 0)).pack(side='left', padx=2)
        ttk.Button(bf, text='膝盖弯', command=lambda: self._preset(40, 0)).pack(side='left', padx=2)
        ttk.Button(bf, text='大腿抬', command=lambda: self._preset(0, 30)).pack(side='left', padx=2)
        ttk.Button(bf, text='收缩', command=lambda: self._preset(40, 20)).pack(side='left', padx=2)

        ttk.Separator(c).pack(fill='x', pady=8)
        ttk.Label(c, text='单腿分控（勾选覆盖全局）', style='H3.TLabel').pack(anchor='w', pady=(2, 4))
        for lk in LEGS:
            self._leg_card(c, lk)

        ttk.Separator(c).pack(fill='x', pady=8)
        ttk.Label(c, text='足端世界坐标（mm，验证对称性）', style='H3.TLabel').pack(anchor='w', pady=(2, 4))
        self.foot_label = ttk.Label(c, text='', style='Val.TLabel', justify='left', font=('Consolas', 9))
        self.foot_label.pack(anchor='w')

    def _slider_row(self, parent, label, var, lo, hi, key):
        row = ttk.Frame(parent); row.pack(fill='x', pady=2)
        head = ttk.Frame(row); head.pack(fill='x')
        ttk.Label(head, text=label).pack(side='left')
        val_lbl = ttk.Label(head, text='0°', style='Val.TLabel', width=8); val_lbl.pack(side='right')
        s = tk.Scale(row, from_=lo, to=hi, orient='horizontal', variable=var,
                     showvalue=False, bg='#2a2a2a', fg='#4ec9b0', troughcolor='#3a3a3a',
                     highlightthickness=0, length=380)
        s.pack(fill='x')
        s.bind('<ButtonRelease-1>', lambda e, k=key: self._on_global_change(k))
        s.bind('<B1-Motion>', lambda e, k=key: self._on_global_change(k))
        self._sliders[key] = (s, val_lbl)

    def _leg_card(self, parent, lk):
        color = LEG_COLORS[lk]
        mirror_tag = ' (镜像)' if lk in MIRROR_LEGS else ''
        card = tk.Frame(parent, bg='#2a2a2a', highlightbackground=color, highlightthickness=2, bd=0)
        card.pack(fill='x', pady=3, padx=2)
        head = tk.Frame(card, bg='#2a2a2a'); head.pack(fill='x', padx=6, pady=(3, 1))
        tk.Label(head, text='●', fg=color, bg='#2a2a2a', font=('Arial', 12)).pack(side='left')
        tk.Label(head, text=f'{lk}{mirror_tag}', bg='#2a2a2a', fg='#ffffff',
                 font=('Microsoft YaHei', 10, 'bold')).pack(side='left', padx=4)
        tk.Checkbutton(head, text='覆盖', variable=self.leg_override[lk],
                       bg='#2a2a2a', fg='#cccccc', selectcolor='#1a1a1a',
                       activebackground='#2a2a2a', activeforeground='#ffffff',
                       command=lambda l=lk: self._on_override_toggle(l),
                       font=('Microsoft YaHei', 8)).pack(side='right')
        for joint_key, joint_name, lo, hi in [('t1', 'θ1', -90, 60), ('t2', 'θ2', -60, 60)]:
            mini = tk.Frame(card, bg='#2a2a2a'); mini.pack(fill='x', padx=6, pady=1)
            lr = tk.Frame(mini, bg='#2a2a2a'); lr.pack(fill='x')
            tk.Label(lr, text=joint_name, bg='#2a2a2a', fg='#999999',
                     font=('Microsoft YaHei', 8)).pack(side='left')
            vlbl = tk.Label(lr, text='0°', bg='#2a2a2a', fg='#4ec9b0',
                            font=('Consolas', 8, 'bold'), width=8); vlbl.pack(side='right')
            var = self.leg_vars[lk][joint_key]
            s = tk.Scale(mini, from_=lo, to=hi, orient='horizontal', variable=var,
                         showvalue=False, bg='#2a2a2a', fg='#4ec9b0', troughcolor='#3a3a3a',
                         highlightthickness=0, length=340)
            s.pack(fill='x')
            s.bind('<B1-Motion>', lambda e, l=lk: self._on_leg_change(l))
            s.bind('<ButtonRelease-1>', lambda e, l=lk: self._on_leg_change(l))

    # ===== 事件 =====
    def _on_global_change(self, key):
        self._sliders[key][1].configure(
            text=f'{(self.g_t1 if key=="g_t1" else self.g_t2).get():+.0f}°')
        t1, t2 = self.g_t1.get(), self.g_t2.get()
        self.targets = {lk: {'t1': float(t1), 't2': float(t2), 'override': False} for lk in LEGS}
        for lk in LEGS:
            if self.leg_override[lk].get():
                self.leg_override[lk].set(False)
                self.leg_vars[lk]['t1'].set(t1)
                self.leg_vars[lk]['t2'].set(t2)

    def _on_leg_change(self, lk):
        if self.leg_override[lk].get():
            t1, t2 = self.leg_vars[lk]['t1'].get(), self.leg_vars[lk]['t2'].get()
            self.targets[lk] = {'t1': float(t1), 't2': float(t2), 'override': True}

    def _on_override_toggle(self, lk):
        if self.leg_override[lk].get():
            t1, t2 = self.g_t1.get(), self.g_t2.get()
            self.leg_vars[lk]['t1'].set(t1); self.leg_vars[lk]['t2'].set(t2)
            self.targets[lk] = {'t1': float(t1), 't2': float(t2), 'override': True}
        else:
            self.targets = {lk: {'t1': self.g_t1.get(), 't2': self.g_t2.get(), 'override': False}
                            for lk in LEGS}

    def _preset(self, t1, t2):
        self.g_t1.set(t1); self.g_t2.set(t2)
        if 'g_t1' in self._sliders: self._sliders['g_t1'][1].configure(text=f'{t1:+.0f}°')
        if 'g_t2' in self._sliders: self._sliders['g_t2'][1].configure(text=f'{t2:+.0f}°')
        for lk in LEGS:
            if self.leg_override[lk].get(): self.leg_override[lk].set(False)
            self.leg_vars[lk]['t1'].set(t1); self.leg_vars[lk]['t2'].set(t2)
        self.targets = {lk: {'t1': float(t1), 't2': float(t2), 'override': False} for lk in LEGS}

    # ===== FK 求解 =====
    def _solve_leg_continuous(self, lk, t1, t2):
        """单腿连续追踪求解（避免锁错分支/小腿转一圈）。

        双闭环机构有无穷多解，初值决定分支。直接从 0 初值跳到大角度会收敛到
        缠绕解（t3=180° 等）。用上一步解作初值；目标大跳变时插值追踪。
        """
        last_t1, last_t2 = self._last_targets[lk]
        dt1 = t1 - last_t1
        dt2 = t2 - last_t2
        # 大跳变（>20°）：分步插值追踪，每步 ≤10°
        if abs(dt1) > 20 or abs(dt2) > 20:
            n_steps = max(int(max(abs(dt1), abs(dt2)) / 10) + 1, 2)
            for s in range(1, n_steps + 1):
                ti1 = last_t1 + dt1 * s / n_steps
                ti2 = last_t2 + dt2 * s / n_steps
                x, err, it = solve_leg_fk(self.m, self.d, lk, ti1, ti2, x0=self._last_x0[lk])
                if x is not None and err < 0.01:
                    self._last_x0[lk] = x.copy()
        else:
            x, err, it = solve_leg_fk(self.m, self.d, lk, t1, t2, x0=self._last_x0[lk])
            if x is not None and err < 0.01:
                self._last_x0[lk] = x.copy()
        self._last_targets[lk] = (t1, t2)
        return err

    def _solve_all(self):
        """解 4 腿 FK + 查 LUT 碰撞 + 刷新读数"""
        readout = {}
        any_collision = False
        for lk in LEGS:
            t = self.targets[lk]
            if lk in MIRROR_LEGS:
                t1, t2 = -t['t1'], -t['t2']
            else:
                t1, t2 = t['t1'], t['t2']
            err = self._solve_leg_continuous(lk, t1, t2)
            # 查 LUT 碰撞状态（镜像腿和原版腿共用同一张表）
            col = lut_check(self.lut, t['t1'], t['t2'])
            if col['collision']:
                any_collision = True
            readout[lk] = {
                'err_mm': err * 1000,
                'joints': get_leg_joints(self.m, self.d, lk),
                'foot_mm': foot_position(self.m, self.d, lk) * 1000,
                'collision': col['collision'],
                'connected': col['connected'],
                'reachable': col['reachable'],
            }
        mujoco.mj_forward(self.m, self.d)
        self._last_readout = readout
        self._any_collision = any_collision

    def _refresh_labels(self):
        r = self._last_readout
        if r and 'FR' in r:
            j = r['FR']['joints']; err = r['FR']['err_mm']
            self.passive_label.configure(
                text=f'被动角(LUT解): t3={j["t3"]:+.1f}° t4={j["t4"]:+.1f}° '
                     f't6={j["t6"]:+.1f}° t7={j["t7"]:+.1f}°  | 闭环 {err:.3f}mm')
            # 全局碰撞警告
            collision_legs = [lk for lk in LEGS if r[lk].get('collision')]
            if collision_legs:
                self.collision_label.configure(
                    text=f'⚠ 碰撞警告: {",".join(collision_legs)} 腿处于碰撞区（大腿↔小腿穿透）',
                    fg='#ff6666')
            elif r['FR'].get('connected') is False:
                self.collision_label.configure(text='⚠ 当前姿态不连通（孤岛解，拖不到）', fg='#e8c530')
            else:
                self.collision_label.configure(text='✓ 无碰撞 · 可达连通', fg='#4ec9b0')
            # 足端坐标（碰撞的腿标红 ✗）
            lines = []
            for lk in LEGS:
                if lk in r:
                    f = r[lk]['foot_mm']; e = r[lk]['err_mm']
                    col = r[lk].get('collision', False)
                    mark = ' ✗碰撞' if col else (' ⚠残差' if e > 0.5 else '')
                    lines.append(f'{lk}: x={f[0]:+7.1f}  y={f[1]:+7.1f}  z={f[2]:+7.1f}{mark}')
            self.foot_label.configure(text='\n'.join(lines))

    def _on_close(self):
        self._closing = True
        try:
            if self.viewer:
                self.viewer.close()
        except Exception:
            pass
        self.root.destroy()

    # ===== 主循环（viewer + Tkinter 都在主线程）=====
    def run(self):
        # 先让 Tk 窗口显示
        self.root.update()
        # launch viewer（主线程，返回非阻塞句柄）
        with mujoco.viewer.launch_passive(self.m, self.d) as v:
            self.viewer = v
            last_solve = 0
            while not self._closing:
                if not v.is_running():
                    break
                # 每 50ms solve 一次（够流畅，又不卡）
                now = time.time()
                if now - last_solve > 0.05:
                    self._solve_all()
                    self._refresh_labels()
                    last_solve = now
                # viewer 渲染一帧（持锁）
                with v.lock():
                    v.sync()
                # Tkinter 非阻塞推进事件（关键：不用 mainloop，用 update）
                try:
                    self.root.update()
                except tk.TclError:
                    break  # Tk 窗口被关
                time.sleep(0.005)


def main():
    print('启动舵机运动绑定测试面板...')
    print('  MuJoCo viewer 窗口 + 控制面板窗口（都在主线程，避免冲突）')
    print('拖动滑块，viewer 里的机器人会实时跟随运动。')
    panel = ControlPanel()
    panel.run()


if __name__ == '__main__':
    main()

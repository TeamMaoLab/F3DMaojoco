"""从 e9 导出的 colliders 生成 MuJoCo leg.xml（用 Fusion 画的圆柱做碰撞体）。

读取 exports/e9/component_positions.json，把每个 component 的 colliders
转成 MuJoCo <geom type="cylinder">，替换掉之前手猜的胶囊。

策略：
- body 映射：Fusion component 名 → MuJoCo body 名
- collider 坐标是世界坐标（mm），要转成 body 局部坐标（减去 body 原点，×0.001 转 m）
- 沿 Y 轴的圆柱 → MuJoCo cylinder（默认轴就是 Y，不用旋转）
- 排除相邻/闭环体对（保持之前的 <contact><exclude>）
"""
import json
import os

EXPORT_DIR = 'exports/e9'
JSON_PATH = f'{EXPORT_DIR}/component_positions.json'
OUTPUT_XML = 'mujoco_leg/leg.xml'

# Fusion component 名 → MuJoCo body 名 + body 原点世界坐标 (mm, XZ)
BODY_MAP = {
    '膝盖动力发生器': {'body': 'knee_driver',  'origin': [-10.9, 0, 14.4]},
    '膝盖传动1':      {'body': 'knee_link1',   'origin': [-21.29, 0, 8.4]},
    '膝盖转动':       {'body': 'knee_rotor',   'origin': [-12.12, 0, -7]},
    '膝盖传动2':      {'body': 'knee_link2',   'origin': [12.12, 0, 7]},
    '大腿部分':       {'body': 'thigh_rigid',  'origin': [0, 0, 0]},
    '小腿':           {'body': 'shin',         'origin': [25, 0, -43.3]},
}

# 归属修正：Fusion 里某些 COL_ 挂错了父零件，按几何位置（端点世界坐标）
# 重新指定到正确的 body。key = collider name，value = 应属的 body。
# 世界坐标在 load_colliders 里用，这里只标记重定向。
COLLIDER_REDIRECT = {
    # COL_小腿传动 在 J4(12.12,7) 位置，应属膝盖传动2（knee_link2），
    # 但 Fusion 里错挂在膝盖传动1 下。
    'COL_小腿传动': '膝盖传动2',
}


def load_colliders():
    """读取 JSON，按 body 分组返回 colliders。

    应用 COLLIDER_REDIRECT：某些 collider 在 Fusion 里挂错了父零件，
    这里按名字重定向到正确 body（用目标 body 的 origin 算局部坐标）。
    """
    with open(JSON_PATH) as f:
        data = json.load(f)
    by_body = {}  # mujoco_body_name -> [collider_dict, ...]
    for comp in data['components']:
        comp_name = comp['name']
        if comp_name not in BODY_MAP:
            continue
        for col in comp.get('colliders', []):
            # 检查是否需要重定向归属
            redirect_target = COLLIDER_REDIRECT.get(col['name'])
            if redirect_target and redirect_target in BODY_MAP:
                target_info = BODY_MAP[redirect_target]
                body_name = target_info['body']
                body_origin = target_info['origin']
            else:
                body_info = BODY_MAP[comp_name]
                body_name = body_info['body']
                body_origin = body_info['origin']
            # 世界坐标 mm → body 局部坐标 mm（减 body 原点）
            ep1 = [col['endpoints'][0][i] - body_origin[i] for i in range(3)]
            ep2 = [col['endpoints'][1][i] - body_origin[i] for i in range(3)]
            by_body.setdefault(body_name, []).append({
                'name': col['name'],
                'radius_m': col['radius'] / 1000.0,  # mm → m
                'from_local': [v / 1000.0 for v in ep1],  # mm → m
                'to_local': [v / 1000.0 for v in ep2],
            })
    return by_body


def gen_geom_xml(collider):
    """生成一个 <geom type="cylinder"> 的 XML 字符串（body 局部坐标，米）。"""
    f = collider['from_local']
    t = collider['to_local']
    r = collider['radius_m']
    # MuJoCo cylinder 用 fromto（两端点）+ size（半径）
    return (f'      <geom class="collider" type="cylinder" '
            f'fromto="{f[0]:.5f} {f[1]:.5f} {f[2]:.5f} '
            f'{t[0]:.5f} {t[1]:.5f} {t[2]:.5f}" '
            f'size="{r:.5f}"/>')


def main():
    by_body = load_colliders()
    print('=== 按 body 分组的碰撞体 ===')
    for body, cols in by_body.items():
        print(f'\n{body} ({len(cols)} 个):')
        for c in cols:
            print(f"  {c['name']}: r={c['radius_m']*1000:.1f}mm "
                  f"from={[round(v*1000,1) for v in c['from_local']]} "
                  f"to={[round(v*1000,1) for v in c['to_local']]}")

    # 生成 collider geom 列表（按 body）
    collider_xml = {}
    for body, cols in by_body.items():
        lines = []
        for c in cols:
            lines.append(gen_geom_xml(c))
        collider_xml[body] = lines

    # 输出供查看（实际写入 leg.xml 需要插入到对应 body 里）
    print('\n=== 生成的 MuJoCo geom（贴到对应 body 的 mesh geom 后）===')
    for body, lines in collider_xml.items():
        print(f'\n  <!-- {body} 的碰撞体 -->')
        for ln in lines:
            print(ln)

    # 保存到中间文件，供手动/自动合并到 leg.xml
    out = 'mujoco_leg/colliders_generated.txt'
    with open(out, 'w') as f:
        for body, lines in collider_xml.items():
            f.write(f'<!-- {body} -->\n')
            for ln in lines:
                f.write(ln + '\n')
            f.write('\n')
    print(f'\n已保存到 {out}')


if __name__ == '__main__':
    os.chdir('/home/mg/AIMAO/F3DMaojoco')
    main()

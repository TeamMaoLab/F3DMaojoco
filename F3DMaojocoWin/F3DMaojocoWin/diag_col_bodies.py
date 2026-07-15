"""独立诊断脚本：检查 COL_ 组件的 body 结构。

在 Fusion 360 → Scripts and Add-Ins → 选择这个脚本 → Run。
会弹窗显示所有 COL 开头组件的 body 信息（类型、face 数、每个面的 surfaceType）。

目的：搞清楚为什么 body.faces 遍历拿不到圆柱面。
"""
import adsk.core
import adsk.fusion


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        if not design:
            ui.messageBox('没有活动的设计')
            return

        root = design.rootComponent
        lines = []

        def scan_occurrence(occ):
            try:
                comp = occ.component
                if comp and comp.name.startswith('COL'):
                    lines.append(f"\n=== {occ.name} (component: {comp.name}) ===")
                    lines.append(f"  bRepBodies.count = {comp.bRepBodies.count}")
                    for i in range(comp.bRepBodies.count):
                        body = comp.bRepBodies.item(i)
                        lines.append(f"  body[{i}]:")
                        lines.append(f"    objectType = {body.objectType}")
                        lines.append(f"    name = {body.name}")
                        try:
                            lines.append(f"    isSurface = {body.isSurface}")
                        except:
                            lines.append(f"    isSurface = (无此属性)")
                        try:
                            fc = body.faces.count
                            lines.append(f"    faces.count = {fc}")
                            # 打印前 5 个面的 surfaceType
                            for j in range(min(fc, 5)):
                                f = body.faces.item(j)
                                try:
                                    st = f.surfaceType
                                    lines.append(f"    face[{j}].surfaceType = {st}")
                                    if str(st).find('Cylinder') >= 0:
                                        cyl = f.geometry
                                        lines.append(f"      → Cylinder! radius={cyl.radius}cm axis=({cyl.axis.x},{cyl.axis.y},{cyl.axis.z})")
                                except Exception as e:
                                    lines.append(f"    face[{j}] surfaceType 读取失败: {e}")
                        except Exception as e:
                            lines.append(f"    faces 访问失败: {e}")
                        # 也试 edges
                        try:
                            ec = body.edges.count
                            lines.append(f"    edges.count = {ec}")
                        except:
                            pass
                for child in occ.childOccurrences:
                    scan_occurrence(child)
            except Exception as e:
                lines.append(f"  扫描 {occ.name} 失败: {e}")

        for occ in root.allOccurrences:
            scan_occurrence(occ)

        if not any('COL' in line for line in lines):
            ui.messageBox('没有找到 COL 开头的组件')
            return

        # 输出到文件 + 弹窗
        import os
        out = os.path.join(os.path.dirname(__file__), 'col_diagnosis.txt')
        with open(out, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
        # 弹窗显示（截断到 3000 字符避免太长）
        msg = '\n'.join(lines)
        if len(msg) > 3000:
            msg = msg[:3000] + f"\n... (完整输出见 {out})"
        ui.messageBox(msg, 'COL 诊断结果')
    except Exception as e:
        if ui:
            ui.messageBox('失败: ' + str(e))

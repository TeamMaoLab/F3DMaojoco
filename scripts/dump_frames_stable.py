"""稳定导出坐标系树：用 full_path 唯一标识，一次调用拿全部。
通过 /exec 注入：curl -s -G http://127.0.0.1:9099/exec --data-urlencode "code@scripts/dump_frames_stable.py"
输出 _result，含每个 occurrence 的世界坐标+旋转+相对父坐标+旋转。
"""
import adsk.core
import adsk.fusion
import json
import traceback

def mat4(mat):
    a = mat.asArray()
    return [[round(a[i], 6) for i in [0,1,2,3]],
            [round(a[i], 6) for i in [4,5,6,7]],
            [round(a[i], 6) for i in [8,9,10,11]],
            [0,0,0,1]]

def t_mm(m):
    return [round(m[0][3]*10, 3), round(m[1][3]*10, 3), round(m[2][3]*10, 3)]

def full_path(o):
    chain = []
    x = o
    while x is not None:
        chain.append(x.name)
        x = x.assemblyContext
    return "/".join(reversed(chain))

def main():
    app = adsk.core.Application.get()
    design = app.activeProduct
    if not design:
        return {"error": "无活动 Design"}
    root = design.rootComponent

    frames = []
    def walk(o, depth):
        comp = o.component
        nb = comp.bRepBodies.count if comp else 0
        fp = full_path(o)
        parent = o.assemblyContext
        parent_fp = full_path(parent) if parent else ""
        world = mat4(o.transform2)
        local = mat4(o.transform)
        frames.append({
            "full_path": fp,
            "occurrence": o.name,
            "component": comp.name if comp else "?",
            "depth": depth,
            "parent": parent_fp,
            "bodies": nb,
            "world_t_mm": t_mm(world),
            "world_rot": [world[0][:3], world[1][:3], world[2][:3]],
            "local_t_mm": t_mm(local),
            "local_rot": [local[0][:3], local[1][:3], local[2][:3]],
        })
        for c in o.childOccurrences:
            walk(c, depth+1)

    for o in root.occurrences:
        walk(o, 0)

    return {
        "document": app.activeDocument.name,
        "frame_count": len(frames),
        "frames": frames,
    }

try:
    _result = main()
except Exception as e:
    _result = {"error": str(e), "tb": traceback.format_exc()}

"""导出当前 Fusion 文档全部带实体零件的 STL（occurrence 级，含镜像实例独立）。

通过 /exec 注入 Fusion 运行：
    curl -s -G http://127.0.0.1:9099/exec --data-urlencode "code@scripts/export_all_stls.py"

输出目录由 OUTPUT_DIR 变量指定（注入前替换）。
每个带实体的 occurrence 导出一个 STL（多实体合并），文件名 = occurrence 安全名。
STL 顶点 = body 局部坐标（component 坐标系，mm）。

同时输出 parts_manifest.json：每个零件的 component/实体数/STL文件名/full_path。
"""
import adsk.core
import adsk.fusion
import json
import os
import math
import traceback

OUTPUT_DIR = r"\\wsl.localhost\Ubuntu-24.04\home\mg\AIMAO\F3DMaojoco\exports\quad_v4"


def safe_filename(name):
    out = ""
    for ch in name:
        if ch.isalnum() or ch in "._-":
            out += ch
        else:
            out += "_"
    return out


def main():
    app = adsk.core.Application.get()
    design = app.activeProduct
    if not design:
        return {"error": "无活动 Design"}
    root = design.rootComponent

    stl_dir = os.path.join(OUTPUT_DIR, "stl_files")
    os.makedirs(stl_dir, exist_ok=True)

    manifest = []
    exported = 0
    failed = 0
    used_names = {}

    def walk(o, depth):
        nonlocal exported, failed
        comp = o.component
        nb = comp.bRepBodies.count if comp else 0
        if nb == 0:
            for c in o.childOccurrences:
                walk(c, depth + 1)
            return

        # full path
        chain = []
        x = o
        while x is not None:
            chain.append(x.name)
            x = x.assemblyContext
        full_path = "/".join(reversed(chain))

        # 文件名：occurrence 名 + 实例序号去重
        base = safe_filename(o.name)
        if base in used_names:
            used_names[base] += 1
            fname = f"{base}_{used_names[base]}.stl"
        else:
            used_names[base] = 1
            fname = f"{base}.stl"

        fpath = os.path.join(stl_dir, fname)

        # 导出 STL（occurrence 级，镜像 occurrence 自动导镜像几何）
        try:
            em = design.exportManager
            opt = em.createSTLExportOptions(o, fpath)
            try:
                opt.angleTolerance = math.radians(8)
                opt.surfaceTolerance = 0.05
            except Exception:
                pass
            ok = em.execute(opt)
        except Exception as e:
            ok = False

        if ok:
            exported += 1
            manifest.append({
                "occurrence": o.name,
                "component": comp.name,
                "bodies_count": nb,
                "stl_file": "stl_files/" + fname,
                "full_path": full_path,
                "depth": depth,
            })
        else:
            failed += 1
            manifest.append({
                "occurrence": o.name,
                "component": comp.name,
                "bodies_count": nb,
                "stl_file": None,
                "error": "STL导出失败",
                "full_path": full_path,
            })

        for c in o.childOccurrences:
            walk(c, depth + 1)

    for o in root.occurrences:
        walk(o, 0)

    # 写 manifest
    manifest_path = os.path.join(OUTPUT_DIR, "parts_manifest.json")
    out = {
        "source_document": app.activeDocument.name,
        "stl_count": exported,
        "failed": failed,
        "note": "occurrence 级导出。STL 顶点 = body 局部坐标(component 坐标系, mm)。镜像 occurrence 导出镜像几何。当前文档无装配偏移(transform 全 0)。各 STL 都在各自 component 原点。",
        "parts": manifest,
    }
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)

    return {
        "ok": True,
        "导出目录": OUTPUT_DIR,
        "成功": exported,
        "失败": failed,
        "manifest": manifest_path,
    }


try:
    _result = main()
except Exception as e:
    _result = {"error": str(e), "tb": traceback.format_exc()}

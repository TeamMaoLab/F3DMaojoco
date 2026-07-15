"""把 exports/export1 的 STL 序列化成单一 geometry.json（顶点去重 + base64 压缩）。

输出格式（混合：base64 positions + 文本 indices）:
{
  "_meta": {...},
  "parts": {
    "stl_files/小腿_小腿.stl": {
      "positions": "<base64 Float32>",   # 去重后唯一顶点，base64 编码的小端 Float32
      "indices":  [i,j,k, i,j,k, ...]    # 三角面顶点索引（文本 Uint32）
    },
    ...
  }
}

网页解码：atob → Uint8Array → Float32Array(positions)；indices 直接用。
"""
import struct
import json
import base64
import os
import numpy as np

os.chdir('/home/mg/AIMAO/F3DMaojoco')
STL_DIR = 'exports/export1/stl_files'
OUTPUT = 'WebPreviewer/geometry.json'
QUANTUM = 1000  # 1μm 精度（mm × 1000 → 整数去重）


def parse_stl(path):
    """读二进制 STL，返回 (N,3) 顶点数组（每面3个，含重复）。"""
    with open(path, 'rb') as f:
        f.read(80)  # header
        ntri = struct.unpack('<I', f.read(4))[0]
        if ntri == 0:
            return np.zeros((0, 3), dtype=np.float32)
        raw = np.frombuffer(f.read(ntri * 50), dtype=np.uint8).reshape(-1, 50)
        # 顶点在 offset 12~48（跳过法线12字节）
        verts = np.frombuffer(raw[:, 12:48].tobytes(), dtype='<f4').reshape(-1, 3)
    return verts


def dedup_vertices(verts, quantum):
    """顶点去重：量化到整数去重，返回 (唯一顶点, 索引)。"""
    if len(verts) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)
    quantized = np.round(verts * quantum).astype(np.int64)
    unique, inverse = np.unique(quantized, axis=0, return_inverse=True)
    # 反量化回浮点（mm）
    dequant = (unique.astype(np.float32) / quantum).ravel()
    return dequant, inverse.astype(np.uint32)


def main():
    stl_files = sorted(f for f in os.listdir(STL_DIR) if f.endswith('.stl'))
    print(f"转换 {len(stl_files)} 个 STL → {OUTPUT}")
    print(f"量化精度: 1/{QUANTUM} mm = {1000/QUANTUM:.1f}μm\n")

    parts = {}
    total_stl = 0
    total_raw_verts = 0
    total_unique = 0
    total_faces = 0

    print(f"{'文件':<48} {'STL':>7} {'原始顶点':>8} {'去重':>6} {'压缩':>6}")
    print('-' * 80)
    for fname in stl_files:
        path = os.path.join(STL_DIR, fname)
        stl_size = os.path.getsize(path)
        verts = parse_stl(path)
        nfaces = len(verts) // 3
        dequant, indices = dedup_vertices(verts, QUANTUM)

        key = 'stl_files/' + fname
        # positions: base64(Float32 小端)，紧凑且 web 端易解码
        pos_f32 = dequant.reshape(-1).astype('<f4')
        parts[key] = {
            'positions': base64.b64encode(pos_f32.tobytes()).decode('ascii'),
            'indices': indices.reshape(-1, 3).tolist(),
        }

        total_stl += stl_size
        total_raw_verts += len(verts)
        total_unique += len(dequant) // 3
        total_faces += nfaces
        ratio = 100 - len(dequant) / 3 / len(verts) * 100 if len(verts) else 0
        print(f"{fname[:47]:<48} {stl_size//1024:>5}K {len(verts):>8} {len(dequant)//3:>6} {ratio:>5.0f}%")

    print('-' * 80)
    print(f"{'合计':<48} {total_stl//1024:>5}K {total_raw_verts:>8} {total_unique:>6} "
          f"{100-total_unique/total_raw_verts*100:>5.0f}%")

    out = {
        '_meta': {
            'description': 'STL 几何序列化（顶点去重+量化）。网页直接喂 BufferGeometry。',
            'quantum': QUANTUM,
            'precision_um': 1000 / QUANTUM,
            'source': 'exports/export1/stl_files',
            'unit': 'mm',
        },
        'parts': parts,
    }

    with open(OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(out, f, separators=(',', ':'))  # 紧凑无空格

    out_size = os.path.getsize(OUTPUT)
    print(f"\n输出: {OUTPUT}")
    print(f"  体积: {out_size/1024/1024:.2f} MB (STL 原始 {total_stl/1024/1024:.2f} MB，"
          f"压缩 {100-out_size/total_stl*100:.0f}%)")
    print(f"  顶点: {total_raw_verts} → {total_unique}（去重 {total_raw_verts-total_unique} 个）")
    print(f"  三角面: {total_faces}")


if __name__ == '__main__':
    main()

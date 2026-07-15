"""把 exports/export1 的 STL 序列化成 geometry.json（顶点去重 + 量化压缩 + base64）。

优化策略:
  1. 顶点去重：三角面共享顶点，去重 83%
  2. indices: Uint32 → Uint16（最大索引<65535 时省 50%）
  3. positions: 可选量化到 Int16 定点（存 scale + offset，省 50%）

  positions 编码（quantize_pos 开启时）:
    quantized = round((value - offset) / scale)  → Int16
    解码: value = quantized * scale + offset
    scale = (max - min) / 65535, offset = min

用法:
  # Uint16 indices + Float32 positions（默认，视觉无损）
  uv run python scripts/gen_geometry_json.py

  # 满精度：Uint16 indices + Float32 positions
  uv run python scripts/gen_geometry_json.py --quantum 1000

  # 扫描多精度对比
  uv run python scripts/gen_geometry_json.py --scan

  # 激进压缩：positions 也量化到 Int16
  uv run python scripts/gen_geometry_json.py --quantize-pos
"""
import struct
import json
import base64
import argparse
import os
import numpy as np

os.chdir('/home/mg/AIMAO/F3DMaojoco')
STL_DIR = 'exports/export1/stl_files'
OUTPUT = 'WebPreviewer/geometry.json'


def parse_stl(path):
    """读二进制 STL，返回 (N,3) 顶点数组（每面3个，含重复）。"""
    with open(path, 'rb') as f:
        f.read(80)  # header
        ntri = struct.unpack('<I', f.read(4))[0]
        if ntri == 0:
            return np.zeros((0, 3), dtype=np.float32)
        raw = np.frombuffer(f.read(ntri * 50), dtype=np.uint8).reshape(-1, 50)
        verts = np.frombuffer(raw[:, 12:48].tobytes(), dtype='<f4').reshape(-1, 3)
    return verts


def dedup_vertices(verts, quantum):
    """顶点去重：量化到整数去重，返回 (唯一顶点 Float32, 索引 Uint32)。"""
    if len(verts) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)
    quantized = np.round(verts * quantum).astype(np.int64)
    unique, inverse = np.unique(quantized, axis=0, return_inverse=True)
    dequant = (unique.astype(np.float32) / quantum).reshape(-1, 3)
    return dequant, inverse.astype(np.uint32)


def encode_positions(verts, quantize):
    """编码顶点。quantize=False 用 Float32 base64；True 用 Int16 定点 base64。
    返回 (encoded_dict, bytes_count)。"""
    if not quantize:
        b64 = base64.b64encode(verts.astype('<f4').tobytes()).decode('ascii')
        return {'positions': b64}, len(b64)
    # Int16 定点量化：每个零件独立 scale/offset
    # 用 32767 而非 65535（Int16 非负半区 0~32767，避免溢出）
    vmin = verts.min(axis=0)
    vmax = verts.max(axis=0)
    scale = (vmax - vmin) / 32767.0
    scale[scale == 0] = 1e-6  # 避免除0（单点情况）
    quantized = np.round((verts - vmin) / scale).astype(np.int64)
    quantized = np.clip(quantized, 0, 32767).astype('<i2')  # 安全裁剪到 Int16 范围
    b64 = base64.b64encode(quantized.tobytes()).decode('ascii')
    # 精度 = scale 的最大值
    precision_um = float(scale.max() * 1000)
    return {
        'positions': b64,
        'posScale': [float(s) for s in scale],
        'posOffset': [float(o) for o in vmin],
        'posType': 'int16',
    }, len(b64), precision_um


def encode_indices(indices):
    """Uint16 如果够用否则 Uint32，返回 (dict, bytes_count)。"""
    if len(indices) == 0:
        return {'indices': ''}, 0
    max_idx = int(indices.max())
    if max_idx < 65535:
        b64 = base64.b64encode(indices.astype('<u2').tobytes()).decode('ascii')
        return {'indices': b64, 'idxType': 'uint16'}, len(b64)
    b64 = base64.b64encode(indices.astype('<u4').tobytes()).decode('ascii')
    return {'indices': b64, 'idxType': 'uint32'}, len(b64)


def build_geometry(stl_files, quantum, quantize_pos=False, verbose=False):
    parts = {}
    total_stl = 0
    total_raw = 0
    total_unique = 0
    total_faces = 0
    max_precision = 0.0

    for fname in stl_files:
        path = os.path.join(STL_DIR, fname)
        verts = parse_stl(path)
        stl_size = os.path.getsize(path)
        nfaces = len(verts) // 3
        dequant, indices = dedup_vertices(verts, quantum)
        nunique = len(dequant)

        pos_enc = encode_positions(dequant, quantize_pos)
        precision_um = pos_enc[2] if quantize_pos else round(1000 / quantum, 3)
        max_precision = max(max_precision, precision_um)

        idx_enc = encode_indices(indices)
        key = 'stl_files/' + fname
        parts[key] = {**pos_enc[0], **idx_enc[0]}

        total_stl += stl_size
        total_raw += len(verts)
        total_unique += nunique
        total_faces += nfaces

    out = {
        '_meta': {
            'quantum': quantum,
            'dedup_precision_um': round(1000 / quantum, 3),
            'pos_quantized': quantize_pos,
            'max_precision_um': round(max_precision, 3),
            'unit': 'mm',
        },
        'parts': parts,
    }
    json_bytes = len(json.dumps(out, separators=(',', ':')).encode('utf-8'))
    stats = {
        'quantum': quantum,
        'quantize_pos': quantize_pos,
        'json_mb': json_bytes / 1024 / 1024,
        'stl_mb': total_stl / 1024 / 1024,
        'raw_verts': total_raw,
        'unique_verts': total_unique,
        'faces': total_faces,
        'dedup_pct': round(100 - total_unique / total_raw * 100, 1),
        'compression_pct': round(100 - json_bytes / total_stl * 100, 1),
        'precision_um': round(max_precision, 3),
    }
    return out, stats


def main():
    ap = argparse.ArgumentParser(description='STL → geometry.json')
    ap.add_argument('--quantum', type=int, default=1000, help='去重量化精度（默认1000=1μm）')
    ap.add_argument('--quantize-pos', action='store_true', help='positions 也量化到 Int16 定点')
    ap.add_argument('--scan', action='store_true', help='扫描多精度对比表')
    ap.add_argument('-o', '--output', default=OUTPUT)
    args = ap.parse_args()

    stl_files = sorted(f for f in os.listdir(STL_DIR) if f.endswith('.stl'))

    if args.scan:
        print(f"扫描精度对比（{len(stl_files)} STL，去重 + Uint16 indices）\n")
        configs = [
            (1000, False, 'Float32 pos + u16 idx (无损基准)'),
            (1000, True,  'Int16 pos + u16 idx (1μm级)'),
            (500, True,   'Int16 pos + u16 idx (2μm级)'),
            (200, True,   'Int16 pos + u16 idx (5μm级)'),
            (100, True,   'Int16 pos + u16 idx (10μm级)'),
            (50, True,    'Int16 pos + u16 idx (20μm级)'),
            (20, True,    'Int16 pos + u16 idx (50μm级)'),
        ]
        print(f"{'方案':<42} {'JSON':>7} {'压缩':>6} {'实际精度':>9}")
        print('-' * 68)
        for q, qp, label in configs:
            _, stats = build_geometry(stl_files, q, quantize_pos=qp)
            print(f"{label:<42} {stats['json_mb']:>5.2f}MB {stats['compression_pct']:>5.0f}% "
                  f"{stats['precision_um']:>7.1f}μm")
        print('-' * 68)
        return

    out, stats = build_geometry(stl_files, args.quantum, args.quantize_pos, verbose=False)
    print(f"精度: 去重 1/{args.quantum}mm ({1000/args.quantum:.3f}μm)"
          + (f" + positions Int16 量化" if args.quantize_pos else " + positions Float32"))
    print(f"  实际最差精度: {stats['precision_um']:.3f}μm")
    print(f"  体积: {stats['json_mb']:.2f} MB (STL {stats['stl_mb']:.2f}MB, 压缩 {stats['compression_pct']:.0f}%)")
    print(f"  顶点: {stats['raw_verts']} → {stats['unique_verts']}（去重 {stats['dedup_pct']:.0f}%）")

    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(out, f, separators=(',', ':'))
    print(f"  输出: {args.output}")


if __name__ == '__main__':
    main()

# geometry-loader.js

从 `geometry.json` 解码零件几何的零依赖库。任何项目都能用，不绑定 Three.js。

## 安装

把 `geometry-loader.js` 复制到你的项目即可（单文件，无依赖）。

## 编码格式

`geometry.json` 由 `scripts/gen_geometry_json.py` 生成，每个 part 的结构：

```json
{
  "positions": "<base64 Float32 或 Int16>",
  "posType": "int16",
  "posScale": [sx, sy, sz],
  "posOffset": [ox, oy, oz],
  "indices": "<base64 Uint16 或 Uint32>",
  "idxType": "uint16"
}
```

- `posType` 缺省 = Float32（无损）；`"int16"` = 定点量化（`value = q * scale + offset`）
- `idxType` 缺省 = 文本数组；`"uint16"`/`"uint32"` = base64 二进制

## 用法

### 纯数据（不依赖 Three.js）

```js
import { decodePart, decodeAll } from './geometry-loader.js';

// 单个零件 → { positions: Float32Array, indices: Uint16Array }
const { positions, indices } = decodePart(part);

// 整个文件 → { "stl_files/xxx.stl": {positions, indices}, ... }
const parts = decodeAll(geoJson);
```

`positions` 是扁平的 `x,y,z,x,y,z,...`，`indices` 是扁平的 `i,j,k,i,j,k,...`。

### Three.js

```js
import { toGeometry } from './geometry-loader.js';

// 返回 THREE.BufferGeometry（已算顶点法线）
const geometry = toGeometry(part);
const mesh = new THREE.Mesh(geometry, material);
```

如果 THREE 不是全局，传入：`toGeometry(part, THREE)`。

### 非 ES Module（老式 script 标签）

`geometry-loader.js` 用了 `export`。如果用 `<script>` 标签加载（非 `type="module"`），需要改成 UMD。简单办法：把文件内容包进 IIFE 挂全局：

```html
<script>
  // 复制 geometry-loader.js 内容，去掉 export，改成：
  window.GeometryLoader = { decodePart, decodeAll, toGeometry };
</script>
```

## Python 端（生成 geometry.json）

```bash
# 默认（Float32 positions + Uint16 indices，1μm 去重，视觉无损）
uv run python scripts/gen_geometry_json.py

# 更小体积（Int16 量化 positions，1.5μm，省 24%）
uv run python scripts/gen_geometry_json.py --quantize-pos

# 对比各精度体积
uv run python scripts/gen_geometry_json.py --scan
```

## 体积参考（12 个 STL，原 5.17MB）

| 方案 | 体积 | 压缩 | 精度 |
|---|---|---|---|
| Float32 pos + Uint16 idx | 1.65MB | 68% | 1μm（无损） |
| Int16 pos + Uint16 idx | 1.24MB | 76% | 1.5μm |

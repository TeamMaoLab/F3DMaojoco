/**
 * geometry-loader.js — geometry.json 解码库（零依赖，任何项目可用）
 *
 * 编码格式（见 scripts/gen_geometry_json.py）：
 *   {
 *     "positions": "<base64>",        // Float32 或 Int16 量化
 *     "posType": "int16" | undefined, // 缺省=Float32
 *     "posScale": [sx,sy,sz],         // Int16 量化的 scale（value = q*scale+offset）
 *     "posOffset": [ox,oy,oz],
 *     "indices": "<base64>",          // Uint16 或 Uint32
 *     "idxType": "uint16" | "uint32"  // 缺省=文本数组
 *   }
 *
 * 用法（纯数据，不依赖 Three.js）：
 *   import { decodePart } from './geometry-loader.js';
 *   const { positions, indices } = decodePart(packed);
 *   // positions: Float32Array(x,y,z, x,y,z, ...)
 *   // indices:   Uint16Array 或 Uint32Array(i,j,k, i,j,k, ...)
 *
 * 用法（Three.js）：
 *   import { toGeometry } from './geometry-loader.js';
 *   const geometry = toGeometry(packed);  // 返回 THREE.BufferGeometry
 *   // 需先全局加载 THREE，或传 THREE 参数：toGeometry(packed, THREE)
 */

// base64 字符串 → Uint8Array（无需 TextEncoder，兼容性好）
function b64ToBytes(b64) {
    const bin = atob(b64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return bytes;
}

/**
 * 解码单个零件的几何数据（纯数据，无渲染依赖）
 * @param {Object} packed - geometry.json 里的一个 part 对象
 * @returns {{positions: Float32Array, indices: Uint16Array|Uint32Array}}
 */
export function decodePart(packed) {
    // --- positions ---
    const posBytes = b64ToBytes(packed.positions);
    let positions;
    if (packed.posType === 'int16') {
        // Int16 定点量化：value = q * scale + offset
        const q = new Int16Array(posBytes.buffer);
        const scale = packed.posScale, off = packed.posOffset;
        positions = new Float32Array(q.length);
        for (let i = 0; i < q.length; i += 3) {
            positions[i]     = q[i]     * scale[0] + off[0];
            positions[i + 1] = q[i + 1] * scale[1] + off[1];
            positions[i + 2] = q[i + 2] * scale[2] + off[2];
        }
    } else {
        // Float32（无损）
        positions = new Float32Array(posBytes.buffer);
    }

    // --- indices ---
    let indices;
    if (packed.idxType === 'uint16') {
        indices = new Uint16Array(b64ToBytes(packed.indices).buffer);
    } else if (packed.idxType === 'uint32') {
        indices = new Uint32Array(b64ToBytes(packed.indices).buffer);
    } else if (Array.isArray(packed.indices)) {
        // 旧格式：嵌套数组 [[i,j,k],...] 或扁平 [i,j,k,...]
        indices = new Uint32Array(packed.indices.flat ? packed.indices.flat() : packed.indices);
    } else if (typeof packed.indices === 'string') {
        // base64 但未指定类型，默认 Uint16
        indices = new Uint16Array(b64ToBytes(packed.indices).buffer);
    }

    return { positions, indices };
}

/**
 * 解码整个 geometry.json
 * @param {Object} geoJson - geometry.json 解析后的对象
 * @returns {Map<string, {positions, indices}>} key = stl 路径
 */
export function decodeAll(geoJson) {
    const result = {};
    for (const key in geoJson.parts) {
        result[key] = decodePart(geoJson.parts[key]);
    }
    return result;
}

/**
 * 转成 Three.js BufferGeometry（可选，需要 THREE）
 * @param {Object} packed - 单个 part
 * @param {Object} [THREE] - Three.js 命名空间（缺省用全局 THREE）
 * @returns {THREE.BufferGeometry}
 */
export function toGeometry(packed, THREE) {
    const T = THREE || (typeof window !== 'undefined' ? window.THREE : undefined);
    if (!T) throw new Error('toGeometry 需要 Three.js（传 THREE 参数或全局加载）');
    const { positions, indices } = decodePart(packed);
    const geometry = new T.BufferGeometry();
    geometry.setAttribute('position', new T.BufferAttribute(positions, 3));
    geometry.setIndex(new T.BufferAttribute(indices, 1));
    geometry.computeVertexNormals();
    return geometry;
}

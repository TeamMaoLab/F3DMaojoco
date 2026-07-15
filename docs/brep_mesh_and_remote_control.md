# Fusion 360 → 网页：BRep 精确网格化 + 远程控制探索总结

> 2026-07-15 第二阶段探索。在第一阶段（STL 网格 + geometry.json 序列化）基础上，
> 解决了"STL 网格粗糙"和"改代码要重启 Fusion"两个核心痛点。

---

## 成果一览

### 1. BRep 精确网格化（替代 STL 默认网格）

**问题**：STL 导出的网格质量固定（Fusion medium 质量），圆柱面棱角分明；而且体积大（5.2MB）。

**解决**：用 Fusion 的 `TriangleMeshCalculator` 直接从精确 BRep 曲面 tessellate，精度可控。

| 指标 | STL 默认 | BRep 低精度 | BRep 高精度（最终） |
|---|---|---|---|
| 顶点 | 31395 | 4397 | **23000** |
| 三角面 | 62974 | 4442 | **25042** |
| 体积 | 5.2MB | 132KB | **756KB** |
| 圆柱面 | 棱角 | 很粗糙 | **圆滑** |

**关键技术点**：
- Fusion API：`body.meshManager.createMeshCalculator()` → `setQuality(15)` → `calculate()`
- `setQuality` 枚举：8(Low)/11(Normal)/13(High)/15(VeryHigh)，越大越精细
- 额外可用 `surfaceTolerance`（cm，越小越精细）、`maxSideLength`、`maxNormalDeviation`
- trim 自动处理（MeshManager 输出已裁剪的真实面，不需要手动处理 edge loop）

### 2. Fusion 远程控制 add-in（F3DRemoteControl）

**问题**：每次改 Fusion 脚本代码都要重启 Fusion（Python 模块缓存在 sys.modules，进程隔离无法外部清除）。

**解决**：驻留型 add-in，开本地 HTTP 服务（127.0.0.1:9099），外部可远程控制 Fusion 进程。

**API**：
| 端点 | 功能 |
|---|---|
| `GET /ping` | 测试连通性 |
| `GET /reload` | 热重载（清 inf3d/common 模块缓存 + __pycache__） |
| `GET /exec?code=...` | 在 Fusion 进程内执行任意 Python 代码 |
| `GET /modules` | 查看已加载的模块 |

**核心价值**：
- 改代码后 `curl /reload` + 点 Run，不用重启 Fusion
- 用 `/exec` 直接触发导出，绕过 Script UI（不需要手动选目录）
- `/exec` 可执行任意代码——调试 API、查询模型状态、自动化操作

---

## 关键踩坑记录

### 坑 1：TriangleMeshCalculator 的属性名

**现象**：设 `angleTolerance`/`maxEdgeLength` 不报错但完全无效，网格精度不变。

**根因**：这两个属性名**根本不存在**。Fusion API 的 `TriangleMeshCalculator` 真实属性是：
- `maxNormalDeviation`（弧度，法线偏差）
- `maxSideLength`（cm，最大边长）
- `surfaceTolerance`（cm，网格偏离曲面距离）
- `setQuality(枚举)`（一键设精度）

**怎么发现的**：在 `_tessellate_body` 里加 `dir(calc)` 诊断，打印出全部可用属性，对照 Fusion 自带的 API 源码（`adsk/defs/adsk/fusion.py`）确认。

**教训**：Fusion API 文档可能和实际不符，**用 `dir()` + `objectType` 诊断是最可靠的**。`setattr(calc, 'xxx', val)` 被 `except: pass` 静默吞掉，问题无从发现——诊断日志必须打出来。

### 坑 2：Fusion 的 Python 模块缓存

**现象**：改了 `.py` 文件，磁盘上是新的，但 Fusion 运行时还是旧代码。

**根因**：Python 模块首次 import 后缓存在 `sys.modules`，后续 import 直接用缓存。Fusion 进程的 `sys.modules` 只有自己能改。

**多层缓存**：
1. `sys.modules`（进程内存，最顽固）
2. `__pycache__/*.pyc`（磁盘字节码，可删）
3. WSL 路径 `\\wsl.localhost\...` 可能有同步延迟

**解决**：`_hot_reload()` 清除时必须：
- 倒序删（子模块先于父包，避免半残状态）
- 清 `__dict__`（释放模块内部引用）
- `importlib.invalidate_caches()`（清查找缓存）
- 删 `__pycache__` 目录

否则 Python 3.14 会报 `KeyError: 'common.backup_manager'`（_bootstrap 内部状态损坏）。

### 坑 3：Fusion 内嵌 Python 环境

**发现**：
- 解释器路径：`C:\Users\...\Autodesk\webdeploy\...\Fusion360.exe`
- **Python 版本 3.14.0**（很新！）
- 独立 `Python/python.exe` 存在，但命令行调它 `import adsk` 失败（adsk 是 C++ 扩展，只有 Fusion 进程内才加载）
- adsk 的 Python 绑定源码在 `API/Python/packages/adsk/defs/adsk/fusion.py`（可读，查 API 定义用）

**局限**：没法在命令行直接调 Fusion API，必须通过 add-in 的 `/exec` 在 Fusion 进程内执行。

### 坑 4：Windows GBK 编码

**现象**：`open(file).read()` 读含中文注释的 .py 报 GBK 解码错误。

**解决**：`open(file, encoding='utf-8')`。Windows 默认编码是 GBK，读 UTF-8 文件必须显式指定。

---

## 工作流（最终形态）

### 日常开发循环

```
改代码（inf3d/ 或 common/）
    ↓
curl http://127.0.0.1:9099/reload    ← 清模块缓存（不用重启 Fusion）
    ↓
Fusion Scripts → Run                  ← 加载最新代码
    ↓
看结果
```

### 远程触发导出（绕过 UI）

```bash
curl -G http://127.0.0.1:9099/exec --data-urlencode 'code=
from inf3d.fusion_export_manager import FusionExportManager
from common.data_types import MeshQuality
mgr = FusionExportManager(mesh_quality=MeshQuality.MEDIUM)
result = mgr.export_assembly(r"输出目录路径")
_result = {"success": result.success, "stl": len(result.stl_files)}
'
```

### /exec 调试技巧

- 末尾赋值 `_result = {...}` 取返回值
- 中文文件用 `open(path, encoding="utf-8")`
- 复杂代码避免 URL 特殊字符：写到 `/tmp/xxx.py`，用 `exec(open(r"\\wsl.localhost\...\xxx.py", encoding="utf-8").read())`
- 查 API：`dir(obj)` 列属性，对照 `adsk/defs/adsk/fusion.py`

---

## 文件索引（本阶段新增/改动）

| 文件 | 作用 |
|---|---|
| `F3DMaojocoScripts/inf3d/brep_mesh_exporter.py` | BRep 精确网格化导出（自包含，用 setQuality） |
| `F3DMaojocoScripts/inf3d/brep_mesh_extractor.py` | BRep 网格提取器（早期版本，已被 exporter 内联取代） |
| `F3DMaojocoScripts/inf3d/brep_extractor.py` | BRep 曲面参数提取（Plane/Cylinder/... 数学定义） |
| `F3DRemoteControl/F3DRemoteControl/` | 远程控制 add-in（HTTP 服务 + 热重载 + /exec） |
| `F3DMaojocoWin/F3DMaojocoWin/F3DMaojocoWin.py` | Windows 入口（含热重载逻辑） |
| `WebPreviewer/app.js` | 加载优先级：brep_geometry.json > geometry.json > STL |

## 下一步方向

### RemoteControl 增强（用 remote + 脚本增强 Fusion）

1. **`/export` 端点**：封装导出流程，参数化输出目录 + 精度，一行命令完成
2. **`/model_info` 端点**：查询当前装配体结构（零件/关节/材质），不用导出就能看
3. **`/select` 端点**：程序化选择面/边/顶点，配合几何分析
4. **`/modify` 端点**：程序化修改参数（改尺寸/移动零件），实现设计自动化
5. **批量导出**：一次导出多个配置（不同精度/不同视角截图）

### BRep 增强

1. **精度可配**：导出时指定 `setQuality` 级别，按需选择精度/体积平衡
2. **NURBS 处理**：39% 的面是 NurbsSurface，探索 NURBS 控制点导出（更小体积）
3. **曲面参数重建**：规则几何（Plane/Cylinder）用参数重建，NURBS 用网格 fallback（混合方案）

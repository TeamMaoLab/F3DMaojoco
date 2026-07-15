/**
 * F3DMaojoco 导出物预览器 - 应用主逻辑
 *
 * 工作流：
 * 1. 用户通过 webkitdirectory 选择整个导出目录
 * 2. 从 File 列表里找 component_positions.json 并解析
 * 3. 按 components[].stl_file 相对路径从 File 集合里匹配 STL
 * 4. 用 FileReader 读取 STL（不用 fetch，绕过本地 CORS）
 * 5. 调用 ExportViewer 渲染
 */
(function () {
    const dirInput = document.getElementById('dirInput');
    const openBtn = document.getElementById('openBtn');
    const jointsToggle = document.getElementById('jointsToggle');
    const axesToggle = document.getElementById('axesToggle');
    const bgSelect = document.getElementById('bgSelect');
    const fitBtn = document.getElementById('fitBtn');
    const dropzone = document.getElementById('dropzone');
    const loadingEl = document.getElementById('loading');
    const statusEl = document.getElementById('status');
    const componentListEl = document.getElementById('componentList');

    const viewer = new ExportViewer(document.getElementById('viewport'));

    // ---- 状态提示 ----
    function showStatus(msg, isError) {
        statusEl.textContent = msg;
        statusEl.className = isError ? 'error info' : 'info';
    }
    function hideStatus() { statusEl.className = ''; }
    function showLoading(msg) { loadingEl.textContent = msg; loadingEl.classList.add('show'); }
    function hideLoading() { loadingEl.classList.remove('show'); }

    // 工作空间可达性地图相关
    const workspaceBtn = document.getElementById('workspaceBtn');
    const wsSolveBtn = document.getElementById('wsSolveBtn');
    const wsRangeSelect = document.getElementById('wsRangeSelect');
    const wsFill = document.getElementById('wsFill');
    const wsText = document.getElementById('wsText');
    const wsStats = document.getElementById('wsStats');
    const wsReadout = document.getElementById('wsReadout');
    const wsCanvas = document.getElementById('wsCanvas');
    const wsCtx = wsCanvas.getContext('2d');
    let currentMechData = null;   // 加载模型后提取的机构数据
    let currentHeatmap = null;    // 计算完成的热力图数据
    let currentAngles = null;     // 角度序列
    let currentJointInit = null;  // 初始关节位置（XZ）
    let currentTreeData = null;   // 运动学树数据
    let treeVisible = false;      // 树结构可视化开关
    let currentHighlight = null;  // 当前高亮的零件名
    let bodyVisibility = {};      // 刚体名 -> bool(是否显示)，初始全 true
    let isolatedBody = null;      // 当前孤立的刚体名（null=无孤立）
    let currentJointAngles = null; // 树关节角度 {刚体名: 角度}
    let constraintMode = false;   // false=手动模式, true=约束模式(只拖θ1θ2)
    let lastSolverX0 = null;      // solveFKCoupled 上次解（连续追踪初值，避免跳分支）

    // 工作空间拖拽状态
    let wsRange = 90;             // 扫描范围（±wsRange 度）
    let wsPointer = { t1: 0, t2: 0 };  // 当前舵机角（用于画指针）
    let isWsDragging = false;     // 是否正在拖拽
    let dragTarget = { t1: 0, t2: 0 }; // 拖拽目标角
    let dragAnimId = null;        // rAF handle

    // 零件 -> 其旋转关节点A（零件绕此点转）。来自机构分析。
    const partJointMap = {
        '膝盖动力发生器:1':  '旋转 1',
        '膝盖传动1:1':       '旋转 6',
        '膝盖转动:1':        '旋转 7',
        '膝盖传动2:1':       '旋转 4',
        '大腿主动力发生器:1': '旋转 2',
        '小腿:1':            '旋转 3',
    };

    // ---- 文件选择 ----
    openBtn.addEventListener('click', () => dirInput.click());

    dirInput.addEventListener('change', async (e) => {
        const files = Array.from(e.target.files);
        if (files.length === 0) return;
        await loadFromFiles(files);
    });

    // ---- 主加载流程 ----
    async function loadFromFiles(files) {
        try {
            hideStatus();
            showLoading('正在扫描文件…');

            // 1. 找 component_positions.json
            const jsonFile = files.find(f =>
                f.name === 'component_positions.json' || f.webkitRelativePath.endsWith('component_positions.json')
            );
            if (!jsonFile) {
                showStatus('❌ 未找到 component_positions.json。请选择包含该文件的导出目录（含 stl_files/ 子目录）。', true);
                hideLoading();
                return;
            }

            // 2. 解析 JSON
            showLoading('正在解析 component_positions.json…');
            const text = await jsonFile.text();
            const data = JSON.parse(text);

            // 构建相对路径 -> File 的映射（用 webkitRelativePath 去掉前导目录）
            const baseDir = jsonFile.webkitRelativePath.replace(/\/component_positions\.json$/, '');
            const fileMap = new Map();
            for (const f of files) {
                let rel = f.webkitRelativePath;
                // 去掉 baseDir 前缀，得到导出目录内的相对路径
                if (baseDir && rel.startsWith(baseDir + '/')) {
                    rel = rel.substring(baseDir.length + 1);
                }
                fileMap.set(rel, f);
                fileMap.set(f.name, f); // 也按纯文件名存一份作 fallback
            }

            // 3. 加载并渲染
            await renderData(data, fileMap);
            dropzone.classList.add('hide');
            hideLoading();
        } catch (err) {
            console.error(err);
            hideLoading();
            showStatus('❌ 加载失败：' + err.message, true);
        }
    }

    // ---- 直接从 exports/export1 加载（fetch 方式，无需选目录）----
    const EXPORT1_BASE = '../exports/export1/';
    async function loadFromExport1() {
        try {
            hideStatus();
            showLoading('正在加载 exports/export1…');
            const resp = await fetch(EXPORT1_BASE + 'component_positions.json');
            if (!resp.ok) throw new Error('fetch component_positions.json 失败: ' + resp.status);
            const data = await resp.json();
            // 构造 fake fileMap：stlPath → { arrayBuffer: () => fetch(...).then(r=>r.arrayBuffer()) }
            const fileMap = new Map();
            fileMap.get = function(key) {
                if (!key) return undefined;
                // key 可能是 "stl_files/xxx.stl" 或纯文件名
                const path = key.startsWith('stl_files/') ? key : 'stl_files/' + key;
                return {
                    arrayBuffer: async () => {
                        const r = await fetch(EXPORT1_BASE + path);
                        if (!r.ok) throw new Error('STL 加载失败: ' + path);
                        return r.arrayBuffer();
                    }
                };
            };
            await renderData(data, fileMap);
            dropzone.classList.add('hide');
            hideLoading();
        } catch (err) {
            console.error(err);
            hideLoading();
            showStatus('❌ 加载 exports/export1 失败：' + err.message + '（需通过 http.server 访问，不能直接双击打开）', true);
        }
    }

    // 页面启动时自动加载 export1
    loadFromExport1();

    // ---- 渲染 ----
    async function renderData(data, fileMap) {
        viewer.clear();

        const components = data.components || [];
        const joints = data.joints || [];
        const meta = data.meta || {};

        showLoading(`正在加载 ${components.length} 个零件的 STL…`);

        // 4. 加载零件：按 stl_file 分组，复用 geometry
        const stlLoader = new THREE.STLLoader();
        const geometryCache = new Map(); // stlFilePath -> geometry
        let loadedCount = 0;
        let skippedContainerCount = 0;
        const failedComponents = [];

        for (const comp of components) {
            const stlPath = comp.stl_file;
            const displayName = comp.occurrence_name || comp.name || '未命名';
            const matrix = comp.world_transform ? comp.world_transform.matrix : null;

            // 跳过纯装配体容器：has_children=true 且 bodies_count=0
            // 这类零件自身无实体几何，其 STL 是子零件几何的合并副本，
            // 与子零件一起渲染会导致 Z-fighting（画面闪烁）。
            // F3DMaojocoScripts 导出时对装配体 occurrence 会生成合并 STL。
            if (comp.has_children && (comp.bodies_count === 0 || comp.bodies_count === '0')) {
                skippedContainerCount++;
                continue;
            }

            if (!matrix) {
                console.warn(`零件 ${displayName} 无 world_transform，跳过`);
                failedComponents.push(displayName);
                continue;
            }

            let geometry = geometryCache.get(stlPath);
            if (!geometry) {
                const file = fileMap.get(stlPath) || fileMap.get(stlPath.split('/').pop());
                if (!file) {
                    console.warn(`未找到 STL 文件: ${stlPath}`);
                    failedComponents.push(displayName);
                    continue;
                }
                try {
                    const buf = await file.arrayBuffer();
                    geometry = stlLoader.parse(buf);
                    geometryCache.set(stlPath, geometry);
                } catch (e) {
                    console.warn(`解析 STL 失败: ${stlPath}`, e);
                    failedComponents.push(displayName);
                    continue;
                }
            }
            viewer.addComponent(stlPath, geometry, matrix, displayName);
            loadedCount++;
        }

        // 5. 构建关节
        const jointData = [];
        for (const j of joints) {
            const gt = j.geometry || {};
            const transform = gt.geometry_one_transform || gt.geometry_two_transform;
            if (!transform || !transform.matrix) {
                console.warn(`关节 ${j.name} 无 geometry transform，跳过`);
                continue;
            }
            const hasLimits = !!j.limits && !!j.limits.revolute_limits;
            jointData.push({
                name: j.name,
                matrix4x4: transform.matrix,
                hasLimits: hasLimits,
            });
        }
        viewer.buildJoints(jointData);

        // 6. 坐标轴
        viewer.buildAxes();
        viewer.setAxesVisible(axesToggle.checked);

        // 7. 适配相机
        viewer.fitCamera();

        // 8. 更新 UI
        renderComponentList(components);

        if (failedComponents.length > 0) {
            showStatus(`⚠️ ${failedComponents.length} 个零件加载失败（STL 缺失或解析错误），详见控制台。`, false);
            setTimeout(hideStatus, 5000);
        } else if (skippedContainerCount > 0) {
            showStatus(`已跳过 ${skippedContainerCount} 个装配体容器（合并几何，避免与子零件重复渲染）。`, false);
            setTimeout(hideStatus, 4000);
        }

        // 提取机构数据，启用工作空间按钮
        currentMechData = window.FKSolver.extractMechanism(data);
        currentJointInit = currentMechData.joints;
        workspaceBtn.disabled = false;

        // 构建运动学树数据并渲染树结构列表
        currentTreeData = buildTreeData(data);
        renderTreeList(currentTreeData);
        renderJointSliders(currentTreeData);
    }

    /**
     * 运动学树（硬编码，针对舵机四足单腿 export1）
     *
     * 树结构:
     *   机架 →[旋转1]→ 膝盖动力发生器 →[旋转6]→ 膝盖传动1 →[旋转7]→ 膝盖转动 →[旋转4]→ 膝盖传动2
     *   机架 →[旋转2]→ 大腿刚体(4零件) →[旋转3]→ 小腿
     *
     * 闭环约束（断开的边，用 equality 约束补回）:
     *   旋转2' : 膝盖转动 ↔ 大腿刚体（在 J2 点重合）
     *   旋转5  : 膝盖传动2 ↔ 小腿（在 J5 点重合）
     *
     * 刚体分组（Fusion Rigid Group，导出脚本未读，手动指定）:
     *   大腿刚体 = 大腿主动力发生器 + 小腿保持架 + 髋关节保持架 + 大腿盖板
     */
    function buildTreeData(data) {
        // 关节世界坐标
        const jointWorld = {};
        for (const j of data.joints) {
            const g = j.geometry || {};
            const t = g.geometry_one_transform || g.geometry_two_transform;
            if (t) {
                const m = t.matrix;
                jointWorld[j.name] = [m[0][3], m[1][3], m[2][3]];
            }
        }
        // 运动学树：[刚体名, 父刚体名(或null=机架), 树关节名, [零件occurrence列表]]
        // 第一个零件是主零件（连着关节）
        const treeBodies = [
            ['膝盖动力发生器', null,             '旋转 1', ['膝盖动力发生器:1']],
            ['膝盖传动1',      '膝盖动力发生器', '旋转 6', ['膝盖传动1:1']],
            ['膝盖转动',       '膝盖传动1',      '旋转 7', ['膝盖转动:1']],
            ['膝盖传动2',      '膝盖转动',       '旋转 4', ['膝盖传动2:1']],
            ['大腿刚体',       null,             '旋转 2', ['大腿主动力发生器:1', '小腿保持架:1', '髋关节保持架:1', '大腿盖板:1']],
            ['小腿',           '大腿刚体',       '旋转 3', ['小腿:1']],
        ];
        // 闭环约束（断开的关节边）
        const loopConstraints = [
            { joint: '旋转 2', body1: '膝盖转动', body2: '大腿刚体', desc: '膝盖转动↔大腿刚体 在J2重合' },
            { joint: '旋转 5', body1: '膝盖传动2', body2: '小腿',   desc: '膝盖传动2↔小腿 在J5重合' },
        ];
        return {
            bodies: treeBodies.map(([name, parent, joint, parts]) => ({
                name: name,
                parent: parent,
                joint: joint,
                jointWorld: jointWorld[joint] || null,
                parts: parts,
                partCount: parts.length,
            })),
            loopConstraints: loopConstraints,
        };
    }
    // 统一应用可见性到 3D（根据 bodyVisibility 状态更新所有 mesh）
    function applyVisibility() {
        for (const mesh of viewer.componentMeshes) {
            const name = mesh.userData.name;
            // 找该零件属于哪个刚体
            let visible = true;
            for (const body of currentTreeData.bodies) {
                if (body.parts.includes(name)) {
                    visible = bodyVisibility[body.name] !== false;
                    break;
                }
            }
            mesh.visible = visible;
        }
    }

    // 更新树列表的视觉状态（隐藏项灰掉、孤立项标记）
    function updateTreeItemStyles() {
        const treeListEl = document.getElementById('treeList');
        treeListEl.querySelectorAll('.tree-item').forEach(el => {
            const name = el.dataset.body;
            const visible = bodyVisibility[name] !== false;
            el.style.opacity = visible ? '1' : '0.4';
            el.style.textDecoration = visible ? '' : 'line-through';
            const isoBtn = el.querySelector('.btn-iso');
            if (isoBtn) isoBtn.textContent = isolatedBody === name ? '●' : '◎';
        });
    }

    // 渲染运动学树（缩进列表，可点击高亮，带孤立/隐藏按钮）
    function renderTreeList(treeData) {
        const treeListEl = document.getElementById('treeList');
        // 过滤掉舵机刚体（机架，不参与运动学树展示）
        const bodies = treeData.bodies.filter(b => !b.name.includes('servo'));
        // 初始化可见性
        if (Object.keys(bodyVisibility).length === 0) {
            for (const b of bodies) bodyVisibility[b.name] = true;
        }
        // 计算深度
        const depthMap = new Map();
        function getDepth(b) {
            if (depthMap.has(b.name)) return depthMap.get(b.name);
            const d = b.parent ? (getDepth(bodies.find(x => x.name === b.parent)) || 0) + 1 : 0;
            depthMap.set(b.name, d);
            return d;
        }
        treeListEl.innerHTML = bodies.map(b => {
            const d = getDepth(b);
            const indent = '　'.repeat(d);
            const parentLabel = b.parent ? ` ← ${b.parent}` : ' ← 机架';
            const jointLabel = b.joint ? `[${b.joint}]` : '';
            const partsLabel = b.parts.length > 1 ? ` (${b.parts.length}零件)` : '';
            return `<div class="tree-item" data-body="${b.name}" style="padding:4px 8px;cursor:pointer;font-size:12px;border-radius:3px;display:flex;align-items:center;gap:6px;">
                <span style="flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;">${indent}${b.name}${partsLabel} <span style="color:#888;font-size:11px">${jointLabel}${parentLabel}</span></span>
                <button class="btn-iso" data-body="${b.name}" title="孤立显示" style="background:none;border:1px solid #555;color:#aaa;cursor:pointer;font-size:11px;padding:1px 5px;border-radius:2px;">◎</button>
                <button class="btn-hide" data-body="${b.name}" title="隐藏" style="background:none;border:1px solid #555;color:#aaa;cursor:pointer;font-size:11px;padding:1px 5px;border-radius:2px;">✕</button>
            </div>`;
        }).join('');
        // 闭环约束（断开的关节边）
        const loops = treeData.loopConstraints || [];
        if (loops.length > 0) {
            treeListEl.innerHTML += `<div style="padding:4px 8px;margin-top:6px;border-top:1px solid #3c3c3c;font-size:11px;color:#888;">闭环约束（断开的边）:</div>`;
            for (const lc of loops) {
                treeListEl.innerHTML += `<div style="padding:3px 8px;font-size:11px;color:#e8c44a;">⟳ [${lc.joint}] ${lc.body1} ↔ ${lc.body2}</div>`;
            }
        }

        // 点击行 = 高亮
        treeListEl.querySelectorAll('.tree-item').forEach(el => {
            el.addEventListener('click', (e) => {
                if (e.target.tagName === 'BUTTON') return;  // 按钮点击不触发高亮
                const bodyName = el.dataset.body;
                if (currentHighlight === bodyName) {
                    currentHighlight = null;
                    viewer.highlightBody(null);
                    treeListEl.querySelectorAll('.tree-item').forEach(x => x.style.background = '');
                } else {
                    currentHighlight = bodyName;
                    treeListEl.querySelectorAll('.tree-item').forEach(x => x.style.background = '');
                    el.style.background = '#3a3d41';
                    const body = bodies.find(b => b.name === bodyName);
                    viewer.highlightBody(body ? body.parts : null);
                }
            });
        });
        // 孤立按钮
        treeListEl.querySelectorAll('.btn-iso').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const name = btn.dataset.body;
                if (isolatedBody === name) {
                    // 取消孤立，恢复全部显示
                    isolatedBody = null;
                    for (const b of bodies) bodyVisibility[b.name] = true;
                } else {
                    // 孤立：只显示这个
                    isolatedBody = name;
                    for (const b of bodies) bodyVisibility[b.name] = (b.name === name);
                }
                applyVisibility();
                updateTreeItemStyles();
            });
        });
        // 隐藏按钮
        treeListEl.querySelectorAll('.btn-hide').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const name = btn.dataset.body;
                // 取消孤立状态（隐藏时孤立无意义）
                isolatedBody = null;
                bodyVisibility[name] = !(bodyVisibility[name] !== false);
                applyVisibility();
                updateTreeItemStyles();
            });
        });
        updateTreeItemStyles();
    }

    // 渲染树关节旋转滑块
    // 手动模式: 6个滑块独立拖（验证相对旋转，但会断开闭环）
    // 约束模式: 只拖θ1(膝盖舵机)θ2(大腿舵机), 被动角由 solveFK 自动求解, 闭环约束生效
    function renderJointSliders(treeData) {
        const container = document.getElementById('jointSliders');
        const bodies = treeData.bodies;
        if (!currentJointAngles) {
            currentJointAngles = {};
            for (const b of bodies) currentJointAngles[b.name] = 0;
        }
        // 约束模式下哪些滑块可用（只有主动关节）
        const isActive = (b) => !constraintMode || b.name === '膝盖动力发生器' || b.name === '大腿刚体';

        container.innerHTML = bodies.map(b => {
            const parentLabel = b.parent ? `(${b.parent}→${b.name})` : `(机架→${b.name})`;
            const jw = b.jointWorld;
            const jwStr = jw ? `P=(${jw[0].toFixed(1)},${jw[1].toFixed(1)},${jw[2].toFixed(1)})` : '';
            const active = isActive(b);
            const isDriven = constraintMode && !active;  // 被动关节（约束模式下属自动求解）
            const drivenTag = isDriven ? ' <span style="color:#e8c44a;">[自动]</span>' : '';
            const activeTag = (constraintMode && active) ? ' <span style="color:#4ec9b0;">[主动]</span>' : '';
            return `<div style="margin-bottom:8px;opacity:${active||isDriven?1:1};">
                <div style="display:flex;justify-content:space-between;font-size:11px;color:#aaa;margin-bottom:2px;">
                    <span>[${b.joint}] ${parentLabel}${activeTag}${drivenTag}</span>
                    <span class="angle-val" data-body="${b.name}">0°</span>
                </div>
                <input type="range" min="-180" max="180" value="0" step="1" class="joint-slider"
                       data-body="${b.name}" style="width:100%;" ${active?'':'disabled'}>
                <div style="font-size:10px;color:#666;">${jwStr}</div>
            </div>`;
        }).join('') + `<div style="margin-top:6px;font-size:11px;color:#888;">${constraintMode?'约束模式: 拖θ1/θ2, 被动角自动求解, 闭环生效':'手动模式: 各滑块独立, 闭环会断开'}</div>
        <div style="margin-top:4px;"><button id="resetJointsBtn" style="background:#3a3d41;border:none;color:#ccc;cursor:pointer;font-size:11px;padding:4px 10px;border-radius:3px;">复位</button></div>`;

        container.querySelectorAll('.joint-slider').forEach(sl => {
            sl.addEventListener('input', () => {
                const name = sl.dataset.body;
                currentJointAngles[name] = parseFloat(sl.value);
                if (constraintMode) {
                    // 约束模式: 重新求解被动角
                    solveAndUpdatePassive();
                }
                updateSliderDisplay();
                applyAllJointRotations();
            });
        });
        document.getElementById('resetJointsBtn').addEventListener('click', () => {
            for (const b of bodies) currentJointAngles[b.name] = 0;
            updateSliderDisplay();
            applyAllJointRotations();
        });
        updateSliderDisplay();
    }

    // 更新所有滑块和角度显示（约束模式下被动滑块显示求解值）
    function updateSliderDisplay() {
        const container = document.getElementById('jointSliders');
        if (!container) return;
        container.querySelectorAll('.joint-slider').forEach(sl => {
            const name = sl.dataset.body;
            const val = currentJointAngles[name] || 0;
            const valEl = container.querySelector(`.angle-val[data-body="${name}"]`);
            if (sl.disabled) {
                // 被动关节: 滑块位置和文字都更新（但不能拖）
                sl.value = Math.round(val);
            }
            if (valEl) valEl.textContent = val.toFixed(0) + '°';
        });
    }

    // 约束模式: 用 solveFKCoupled 求解被动角（与 applyAllJointRotatives 一致）
    // coupled 解出的 passive.{th3,th4,th6,th7} 就是树关节相对角，直接用
    function solveAndUpdatePassive() {
        if (!currentMechData) return;
        const t1 = currentJointAngles['膝盖动力发生器'] || 0;  // θ1 = 旋转1
        const t2 = currentJointAngles['大腿刚体'] || 0;        // θ2 = 旋转2
        const result = window.FKSolver.solveFKCoupled(t1, t2, currentMechData, lastSolverX0);
        if (!result.ok) {
            showStatus(`θ1=${t1}° θ2=${t2}° → 无解: ${result.reason}`, true);
            return;
        }
        hideStatus();
        lastSolverX0 = result.passive ? [result.passive.th3, result.passive.th4, result.passive.th6, result.passive.th7] : lastSolverX0;
        const p = result.passive;
        currentJointAngles['膝盖传动1'] = p.th6 * 180 / Math.PI;
        currentJointAngles['膝盖转动'] = p.th7 * 180 / Math.PI;
        currentJointAngles['膝盖传动2'] = p.th4 * 180 / Math.PI;
        currentJointAngles['小腿'] = p.th3 * 180 / Math.PI;
    }

    // 应用所有树关节旋转
    // 约束模式: 用 solver 的关节点新位置直接摆零件（闭链可靠）
    // 手动模式: 用相对角累积旋转（会断开闭环）
    function applyAllJointRotations() {
        if (!currentTreeData) return;
        const bodies = currentTreeData.bodies;
        if (constraintMode) {
            // 约束模式：用 solveFKCoupled 联立求解（连续追踪初值避免跳分支）
            const t1 = currentJointAngles['膝盖动力发生器'] || 0;
            const t2 = currentJointAngles['大腿刚体'] || 0;
            const result = window.FKSolver.solveFKCoupled(t1, t2, currentMechData, lastSolverX0);
            if (!result.ok) {
                showStatus(`θ1=${t1}° θ2=${t2}° → 无解: ${result.reason}`, true);
                viewer.resetPose();
                return;
            }
            hideStatus();
            lastSolverX0 = result.passive ? [result.passive.th3, result.passive.th4, result.passive.th6, result.passive.th7] : lastSolverX0;
            // passive 返回的就是树关节相对角(th3/th4/th6/th7)，直接用于 applyJointRotations
            const p = result.passive;
            currentJointAngles['膝盖传动1'] = p.th6 * 180 / Math.PI;
            currentJointAngles['膝盖转动'] = p.th7 * 180 / Math.PI;
            currentJointAngles['膝盖传动2'] = p.th4 * 180 / Math.PI;
            currentJointAngles['小腿'] = p.th3 * 180 / Math.PI;
            updateSliderDisplay();
            // 走 applyJointRotations（和手动模式相同路径，保证一致性）
            const bodyRotations = bodies.map(b => ({
                name: b.name, parent: b.parent, jointWorld: b.jointWorld,
                angle: currentJointAngles[b.name] || 0, parts: b.parts,
            }));
            viewer.applyJointRotations(bodyRotations);
            // 同步工作空间地图指针（滑块/地图拖拽都走这里）
            if (currentAngles) {
                wsPointer.t1 = t1; wsPointer.t2 = t2;
                drawWsMap();
                wsReadout.textContent = `θ1=${t1.toFixed(0)}° θ2=${t2.toFixed(0)}°`;
            }
        } else {
            // 手动模式：用相对角累积
            const bodyRotations = bodies.map(b => ({
                name: b.name, parent: b.parent, jointWorld: b.jointWorld,
                angle: currentJointAngles[b.name] || 0, parts: b.parts,
            }));
            viewer.applyJointRotations(bodyRotations);
        }
    }

    function renderComponentList(components) {
        if (components.length === 0) {
            componentListEl.innerHTML = '<div style="color:#666;font-size:12px">无零件</div>';
            return;
        }
        componentListEl.innerHTML = components.map(comp => {
            const name = comp.occurrence_name || comp.name || '未命名';
            const isContainer = comp.has_children && (comp.bodies_count === 0 || comp.bodies_count === '0');
            if (isContainer) {
                return `<div class="item" style="opacity:0.5">
                    <span class="swatch" style="background:#666"></span>
                    <span class="name" title="${name}（装配体，已跳过）">${name}</span>
                    <span class="meta">装配体</span>
                </div>`;
            }
            const stl = comp.stl_file ? comp.stl_file.split('/').pop() : '无STL';
            return `<div class="item">
                <input type="checkbox" checked data-part="${name}" class="stl-toggle">
                <span class="name" title="${name}">${name}</span>
                <span class="meta" title="${comp.stl_file || ''}">${stl}</span>
            </div>`;
        }).join('');
        // 显示/隐藏开关
        componentListEl.querySelectorAll('.stl-toggle').forEach(cb => {
            cb.addEventListener('change', () => {
                const name = cb.dataset.part;
                const mesh = viewer.componentMeshes.find(m => m.userData.name === name);
                if (mesh) mesh.visible = cb.checked;
            });
        });
    }

    // ---- 控件事件 ----
    jointsToggle.addEventListener('change', () => {
        viewer.setJointsVisible(jointsToggle.checked);
    });
    axesToggle.addEventListener('change', () => {
        viewer.setAxesVisible(axesToggle.checked);
    });
    bgSelect.addEventListener('change', () => {
        viewer.setBackgroundColor(bgSelect.value);
    });
    fitBtn.addEventListener('click', () => viewer.fitCamera());

    // 全部显示 / 全部隐藏
    document.getElementById('showAllBtn').addEventListener('click', () => {
        isolatedBody = null;
        for (const b of currentTreeData.bodies) bodyVisibility[b.name] = true;
        applyVisibility();
        updateTreeItemStyles();
    });
    document.getElementById('hideAllBtn').addEventListener('click', () => {
        isolatedBody = null;
        for (const b of currentTreeData.bodies) {
            if (!b.name.includes('servo')) bodyVisibility[b.name] = false;
        }
        applyVisibility();
        updateTreeItemStyles();
    });

    // 模式切换：手动 / 约束
    document.getElementById('modeManualBtn').addEventListener('click', () => {
        constraintMode = false;
        document.getElementById('modeManualBtn').style.background = '#0e639c';
        document.getElementById('modeManualBtn').style.color = '#fff';
        document.getElementById('modeConstraintBtn').style.background = '#3a3d41';
        document.getElementById('modeConstraintBtn').style.color = '#ccc';
        if (currentTreeData) renderJointSliders(currentTreeData);
        applyAllJointRotations();
    });
    document.getElementById('modeConstraintBtn').addEventListener('click', () => {
        constraintMode = true;
        document.getElementById('modeConstraintBtn').style.background = '#0e639c';
        document.getElementById('modeConstraintBtn').style.color = '#fff';
        document.getElementById('modeManualBtn').style.background = '#3a3d41';
        document.getElementById('modeManualBtn').style.color = '#ccc';
        // 切换到约束模式时先复位，再求解一次
        for (const b of currentTreeData.bodies) currentJointAngles[b.name] = 0;
        solveAndUpdatePassive();
        if (currentTreeData) renderJointSliders(currentTreeData);
        applyAllJointRotations();
    });

    // ---- 工作空间可达性地图 ----
    // 坐标约定: Canvas x 轴 = θ1, y 轴 = θ2（原点左上，与角度序列索引一致）
    // angles[i] 对应 θ1，angles[j] 对应 θ2（drawHeatmap 里 i→row, j→col）

    workspaceBtn.addEventListener('click', () => { if (currentMechData) runWsScan(); });
    wsSolveBtn.addEventListener('click', () => { if (currentMechData) runWsScan(); });

    wsRangeSelect.addEventListener('change', () => {
        wsRange = parseInt(wsRangeSelect.value, 10);
        if (currentMechData && currentHeatmap) runWsScan();
    });

    function runWsScan() {
        wsText.textContent = '计算中…';
        wsFill.style.width = '0%';
        wsStats.textContent = '';
        const worker = new Worker('heatmap-worker.js');
        worker.onmessage = (e) => {
            const msg = e.data;
            if (msg.type === 'progress') {
                const pct = (msg.done / msg.total * 100).toFixed(0);
                wsFill.style.width = pct + '%';
                wsText.textContent = `${msg.done}/${msg.total} (${pct}%)`;
            } else if (msg.type === 'done') {
                currentHeatmap = msg.heatmap;
                currentAngles = msg.angles;
                drawWsMap();
                const s = msg.stats;
                wsText.textContent = '完成 · 可拖动';
                wsFill.style.width = '100%';
                wsStats.textContent = `无解 ${s.noSolution} · 超极限 ${s.solvable - s.inLimits} · 极限内 ${s.inLimits} / 共 ${s.total}`;
                worker.terminate();
            }
        };
        worker.postMessage({ mechData: currentMechData, rangeMin: -wsRange, rangeMax: wsRange, step: 2 });
    }

    // 画底图（无指针）
    function drawWsMap() {
        const N = currentAngles.length;
        const cell = wsCanvas.width / N;
        const colors = ['#222222', '#e74c3c', '#27ae60'];
        for (let i = 0; i < N; i++) {
            for (let j = 0; j < N; j++) {
                wsCtx.fillStyle = colors[currentHeatmap[i][j]] || '#222';
                wsCtx.fillRect(j * cell, i * cell, Math.ceil(cell), Math.ceil(cell));
            }
        }
        // 画原点十字（0,0 参考线）
        wsCtx.strokeStyle = 'rgba(255,255,255,0.25)';
        wsCtx.lineWidth = 1;
        const mid = wsCanvas.width / 2;
        wsCtx.beginPath();
        wsCtx.moveTo(mid, 0); wsCtx.lineTo(mid, wsCanvas.height);
        wsCtx.moveTo(0, mid); wsCtx.lineTo(wsCanvas.width, mid);
        wsCtx.stroke();
        drawWsPointer();
    }

    // 在当前舵机角位置画十字指针（含 + 标记）
    function drawWsPointer() {
        if (!currentAngles) return;
        const [px, py] = angleToPixel(wsPointer.t1, wsPointer.t2);
        // 外圈光晕
        wsCtx.strokeStyle = isWsDragging ? '#fff' : 'rgba(255,255,255,0.9)';
        wsCtx.lineWidth = 2;
        wsCtx.beginPath();
        wsCtx.moveTo(px - 8, py); wsCtx.lineTo(px + 8, py);
        wsCtx.moveTo(px, py - 8); wsCtx.lineTo(px, py + 8);
        wsCtx.stroke();
        wsCtx.fillStyle = isWsDragging ? '#ffd700' : '#fff';
        wsCtx.beginPath();
        wsCtx.arc(px, py, 3, 0, Math.PI * 2);
        wsCtx.fill();
    }

    // 角度 → Canvas 像素（与 angles 序列索引对齐，避免重复计算）
    function angleToPixel(t1, t2) {
        const N = currentAngles.length;
        // 找最近的角度格索引
        let i = 0, best = 1e9;
        for (let k = 0; k < N; k++) { const d = Math.abs(currentAngles[k] - t1); if (d < best) { best = d; i = k; } }
        let j = 0; best = 1e9;
        for (let k = 0; k < N; k++) { const d = Math.abs(currentAngles[k] - t2); if (d < best) { best = d; j = k; } }
        const cell = wsCanvas.width / N;
        return [(j + 0.5) * cell, (i + 0.5) * cell];
    }

    // 像素 → 最近的 {i, j} 网格索引 + 对应角度
    function pixelToGrid(e) {
        const rect = wsCanvas.getBoundingClientRect();
        const x = Math.max(0, Math.min(rect.width - 1, e.clientX - rect.left)) / rect.width;
        const y = Math.max(0, Math.min(rect.height - 1, e.clientY - rect.top)) / rect.height;
        const N = currentAngles.length;
        const i = Math.min(N - 1, Math.max(0, Math.floor(y * N)));
        const j = Math.min(N - 1, Math.max(0, Math.floor(x * N)));
        return { i, j, t1: currentAngles[i], t2: currentAngles[j], code: currentHeatmap[i][j] };
    }

    // 查询某角度格是否有解（code!=0）
    function isSolvable(t1, t2) {
        if (!currentHeatmap || !currentAngles) return false;
        const N = currentAngles.length;
        let i = 0, best = 1e9;
        for (let k = 0; k < N; k++) { const d = Math.abs(currentAngles[k] - t1); if (d < best) { best = d; i = k; } }
        let j = 0; best = 1e9;
        for (let k = 0; k < N; k++) { const d = Math.abs(currentAngles[k] - t2); if (d < best) { best = d; j = k; } }
        return currentHeatmap[i][j] !== 0;
    }

    // 更新指针位置 + readout（不改舵机，仅显示）
    function refreshPointerDisplay() {
        if (!currentAngles) return;
        drawWsMap();  // 重画底图+指针
        wsReadout.textContent = `θ1=${wsPointer.t1.toFixed(0)}° θ2=${wsPointer.t2.toFixed(0)}°`;
    }

    // ---- 拖拽控制（核心：步进插值，防 solveFKCoupled 跳分支）----
    // 鼠标松开时刷新地图
    function onPointerUp() {
        isWsDragging = false;
        wsCanvas.classList.remove('dragging');
        if (dragAnimId) { cancelAnimationFrame(dragAnimId); dragAnimId = null; }
        refreshPointerDisplay();
    }

    wsCanvas.addEventListener('mousedown', (e) => {
        if (!currentHeatmap) return;
        const g = pixelToGrid(e);
        if (g.code === 0) {
            showStatus(`θ1=${g.t1}° θ2=${g.t2}° 无解区，不可进入`, true);
            return;
        }
        hideStatus();
        isWsDragging = true;
        wsCanvas.classList.add('dragging');
        dragTarget.t1 = g.t1; dragTarget.t2 = g.t2;
        if (!dragAnimId) dragAnimId = requestAnimationFrame(stepDrag);
    });

    wsCanvas.addEventListener('mousemove', (e) => {
        if (!currentHeatmap) return;
        const g = pixelToGrid(e);
        if (isWsDragging) {
            // 仅更新目标，不直接求解（由 rAF 步进追赶）
            if (g.code !== 0) { dragTarget.t1 = g.t1; dragTarget.t2 = g.t2; }
            // 拖到无解区：目标保持在上一个有效点（实现"停在边界"）
        }
        wsCanvas.title = `θ1=${g.t1}° θ2=${g.t2}° ${g.code === 0 ? '(无解)' : g.code === 2 ? '(极限内)' : '(超极限)'}`;
    });

    wsCanvas.addEventListener('mouseleave', onPointerUp);
    window.addEventListener('mouseup', onPointerUp);

    // 每帧步进：从当前角朝目标走 MAX_STEP，求解并渲染
    function stepDrag() {
        const MAX_STEP = 2;  // 单帧最大角增量（度），防 lastSolverX0 丢连续性
        const curT1 = currentJointAngles['膝盖动力发生器'] || 0;
        const curT2 = currentJointAngles['大腿刚体'] || 0;
        let d1 = dragTarget.t1 - curT1;
        let d2 = dragTarget.t2 - curT2;
        const dist = Math.hypot(d1, d2);
        if (dist < 0.1) {
            // 已到目标
            dragAnimId = null;
            refreshPointerDisplay();
            return;
        }
        // 等比缩放，保证两轴同步且单帧总位移 ≤ MAX_STEP
        const scale = Math.min(1, MAX_STEP / dist);
        let nextT1 = curT1 + d1 * scale;
        let nextT2 = curT2 + d2 * scale;
        // 进入无解区 → 卡在当前（不前进）
        if (!isSolvable(nextT1, nextT2)) {
            // 尝试只走单轴
            if (isSolvable(curT1 + d1 * scale, curT2)) { nextT1 = curT1 + d1 * scale; nextT2 = curT2; }
            else if (isSolvable(curT1, curT2 + d2 * scale)) { nextT1 = curT1; nextT2 = curT2 + d2 * scale; }
            else { dragAnimId = null; refreshPointerDisplay(); return; }  // 两轴都进不去，停
        }
        // 自动切约束模式（若不在）
        if (!constraintMode) {
            constraintMode = true;
            document.getElementById('modeConstraintBtn').click();
        }
        currentJointAngles['膝盖动力发生器'] = nextT1;
        currentJointAngles['大腿刚体'] = nextT2;
        wsPointer.t1 = nextT1; wsPointer.t2 = nextT2;
        applyAllJointRotations();   // 内部用 solveFKCoupled + lastSolverX0 连续追踪
        updateSliderDisplay();
        // 实时刷新指针（轻量重画）
        refreshPointerDisplay();
        dragAnimId = requestAnimationFrame(stepDrag);
    }

    // 滑块拖动时也同步指针位置（约束模式下 θ1/θ2 滑块拖动反映到地图）
    // （通过在 applyAllJointRotations 后更新 wsPointer 实现，见下方 hook）

    // 初始建坐标轴（即使没加载数据也能看到）
    viewer.buildAxes();
    viewer.setAxesVisible(true);
})();

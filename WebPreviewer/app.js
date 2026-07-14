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

    // 工作空间热力图相关
    const workspaceBtn = document.getElementById('workspaceBtn');
    const heatmapPanel = document.getElementById('heatmapPanel');
    const hpCloseBtn = document.getElementById('hpCloseBtn');
    const hpFill = document.getElementById('hpFill');
    const hpText = document.getElementById('hpText');
    const hpStats = document.getElementById('hpStats');
    const heatmapCanvas = document.getElementById('heatmapCanvas');
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

    // 约束模式: 用 solveFK 求解被动角，转换成树关节相对角
    function solveAndUpdatePassive() {
        if (!currentMechData) return;
        const t1 = currentJointAngles['膝盖动力发生器'] || 0;  // θ1 = 旋转1
        const t2 = currentJointAngles['大腿刚体'] || 0;        // θ2 = 旋转2
        const result = window.FKSolver.solveFK(t1, t2, currentMechData);
        if (!result.ok) {
            showStatus(`θ1=${t1}° θ2=${t2}° → 无解: ${result.reason}`, true);
            return;
        }
        hideStatus();
        // solver 的 angles 是世界角，转换成树关节相对角
        // solver angles: '旋转 1'..'旋转 7'
        const sa = result.angles;
        // 树关节相对角 = 子世界角 - 父世界角
        // 膝盖动力(父=机架): 相对 = θ1
        // 膝盖传动1(父=膝盖动力): 相对 = solver6 - θ1
        // 膝盖转动(父=膝盖传动1): 相对 = solver7 - solver6
        // 膝盖传动2(父=膝盖转动): 相对 = solver4 - solver7
        // 大腿刚体(父=机架): 相对 = θ2
        // 小腿(父=大腿刚体): 相对 = solver3 - θ2
        currentJointAngles['膝盖动力发生器'] = t1;
        currentJointAngles['膝盖传动1'] = sa['旋转 6'] - t1;
        currentJointAngles['膝盖转动'] = sa['旋转 7'] - sa['旋转 6'];
        currentJointAngles['膝盖传动2'] = sa['旋转 4'] - sa['旋转 7'];
        currentJointAngles['大腿刚体'] = t2;
        currentJointAngles['小腿'] = sa['旋转 3'] - t2;
    }

    // 应用所有树关节旋转
    // 约束模式: 用 solver 的关节点新位置直接摆零件（闭链可靠）
    // 手动模式: 用相对角累积旋转（会断开闭环）
    function applyAllJointRotations() {
        if (!currentTreeData) return;
        const bodies = currentTreeData.bodies;
        if (constraintMode) {
            // 约束模式：用 applyJointPositions
            const t1 = currentJointAngles['膝盖动力发生器'] || 0;
            const t2 = currentJointAngles['大腿刚体'] || 0;
            const result = window.FKSolver.solveFK(t1, t2, currentMechData);
            if (!result.ok) { viewer.resetPose(); return; }
            // solver 给的 jointPositions 是 [x,z]（2D），补 Y（初始值）
            const initPos = {};
            for (const jn in currentJointInit) {
                const jw = currentTreeData.bodies.flatMap(b => b.jointWorld ? [] : []).length; // placeholder
            }
            // 从原始数据取初始 3D 关节位置（含 Y）
            const jw3d = {};
            for (const b of bodies) {
                if (b.jointWorld) jw3d[b.joint] = b.jointWorld;
            }
            // solver 的 jointPositions 键名是 '旋转 1' 等，值是 [x,z]
            const newPos = {};
            for (const jn in result.jointPositions) {
                const p2 = result.jointPositions[jn];
                const init3 = jw3d[jn];
                newPos[jn] = [p2[0], init3 ? init3[1] : 0, p2[1]];
            }
            const initPos3d = {};
            for (const jn in jw3d) initPos3d[jn] = jw3d[jn];
            // 刚体定义：每个刚体由哪两个关节点确定姿态
            const bodyDefs = [
                { name: '膝盖动力发生器', parts: bodies.find(b=>b.name==='膝盖动力发生器').parts, jA: '旋转 1', jB: '旋转 6' },
                { name: '膝盖传动1',      parts: bodies.find(b=>b.name==='膝盖传动1').parts,      jA: '旋转 6', jB: '旋转 7' },
                { name: '膝盖转动',       parts: bodies.find(b=>b.name==='膝盖转动').parts,       jA: '旋转 7', jB: '旋转 4' },
                { name: '膝盖传动2',      parts: bodies.find(b=>b.name==='膝盖传动2').parts,      jA: '旋转 4', jB: '旋转 5' },
                { name: '大腿刚体',       parts: bodies.find(b=>b.name==='大腿刚体').parts,       jA: '旋转 2', jB: '旋转 3' },
                { name: '小腿',           parts: bodies.find(b=>b.name==='小腿').parts,           jA: '旋转 3', jB: '旋转 5' },
            ];
            viewer.applyJointPositions(newPos, initPos3d, bodyDefs);
            // 同步更新被动角的显示
            const sa = result.angles;
            currentJointAngles['膝盖传动1'] = sa['旋转 6'] - t1;
            currentJointAngles['膝盖转动'] = sa['旋转 7'] - sa['旋转 6'];
            currentJointAngles['膝盖传动2'] = sa['旋转 4'] - sa['旋转 7'];
            currentJointAngles['小腿'] = sa['旋转 3'] - t2;
            updateSliderDisplay();
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

    // ---- 工作空间热力图 ----
    workspaceBtn.addEventListener('click', () => {
        if (!currentMechData) return;
        heatmapPanel.classList.add('show');
        runHeatmapScan();
    });
    hpCloseBtn.addEventListener('click', () => heatmapPanel.classList.remove('show'));

    function runHeatmapScan() {
        hpText.textContent = '启动计算…';
        hpFill.style.width = '0%';
        hpStats.textContent = '';

        const worker = new Worker('heatmap-worker.js');
        worker.onmessage = (e) => {
            const msg = e.data;
            if (msg.type === 'progress') {
                const pct = (msg.done / msg.total * 100).toFixed(0);
                hpFill.style.width = pct + '%';
                hpText.textContent = `${msg.done}/${msg.total} (${pct}%)`;
            } else if (msg.type === 'done') {
                currentHeatmap = msg.heatmap;
                currentAngles = msg.angles;
                drawHeatmap(msg.heatmap, msg.angles);
                const s = msg.stats;
                hpText.textContent = '完成';
                hpFill.style.width = '100%';
                hpStats.textContent = `无解 ${s.noSolution} · 超极限 ${s.solvable - s.inLimits} · 极限内 ${s.inLimits} / 共 ${s.total}`;
                worker.terminate();
            }
        };
        worker.postMessage({ mechData: currentMechData, rangeMin: -90, rangeMax: 90, step: 2 });
    }

    function drawHeatmap(heatmap, angles) {
        const ctx = heatmapCanvas.getContext('2d');
        const N = angles.length;
        const cellW = heatmapCanvas.width / N;
        const cellH = heatmapCanvas.height / N;
        // 颜色: 0=无解(深), 1=超极限(红), 2=极限内(绿)
        const colors = ['#222222', '#e74c3c', '#27ae60'];
        for (let i = 0; i < N; i++) {
            for (let j = 0; j < N; j++) {
                ctx.fillStyle = colors[heatmap[i][j]] || '#222';
                ctx.fillRect(j * cellW, i * cellH, Math.ceil(cellW), Math.ceil(cellH));
            }
        }
    }

    // 点击热力图 → 3D 腿摆到该角度组合的姿态
    heatmapCanvas.addEventListener('click', (e) => {
        if (!currentHeatmap || !currentAngles || !currentMechData) return;
        const rect = heatmapCanvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width;
        const y = (e.clientY - rect.top) / rect.height;
        const N = currentAngles.length;
        const j = Math.min(N - 1, Math.floor(x * N));
        const i = Math.min(N - 1, Math.floor(y * N));
        const t1 = currentAngles[i];
        const t2 = currentAngles[j];
        const result = window.FKSolver.solveFK(t1, t2, currentMechData);
        if (result.ok) {
            const limOk = window.FKSolver.checkLimits(result, currentMechData);
            viewer.applyPose(result.partRotations, result.jointPositions, currentJointInit, partJointMap);
            showStatus(`θ1=${t1}° θ2=${t2}° → ${limOk ? '✅有解·极限内' : '⚠️有解·超极限'}`, false);
        } else {
            viewer.resetPose();
            showStatus(`θ1=${t1}° θ2=${t2}° → ❌无解: ${result.reason}（已复位）`, true);
        }
    });

    // 鼠标在热力图上移动时显示当前角度
    heatmapCanvas.addEventListener('mousemove', (e) => {
        if (!currentAngles) return;
        const rect = heatmapCanvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width;
        const y = (e.clientY - rect.top) / rect.height;
        const N = currentAngles.length;
        const j = Math.min(N - 1, Math.max(0, Math.floor(x * N)));
        const i = Math.min(N - 1, Math.max(0, Math.floor(y * N)));
        heatmapCanvas.title = `θ1=${currentAngles[i]}° θ2=${currentAngles[j]}°`;
    });

    // 初始建坐标轴（即使没加载数据也能看到）
    viewer.buildAxes();
    viewer.setAxesVisible(true);
})();

/**
 * F3DMaojoco 导出物预览器 - 应用主逻辑（LUT 驱动版）
 *
 * 不再有 JS FK 求解器。所有关节角来自预计算的查找表 workspace_lut.json
 * （由 scripts/gen_workspace_lut.py 用 MuJoCo 求解生成）。
 * 拖 θ1/θ2 → 查表 + 双线性插值 → viewer.applyJointRotations 驱动 3D 腿。
 *
 * 工作流：
 * 1. 加载 exports/export1（零件 STL + 关节坐标）
 * 2. 加载 workspace_lut.json（关节角查找表）+ workspace_grid.json（解空间地图）
 * 3. 用户拖 θ1/θ2 滑块 或在地图上拖动 → 插值出 6 个关节角 → 驱动 3D
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

    // ---- 状态 ----
    function showStatus(msg, isError) {
        statusEl.textContent = msg;
        statusEl.className = isError ? 'error info' : 'info';
    }
    function hideStatus() { statusEl.className = ''; }
    function showLoading(msg) { loadingEl.textContent = msg; loadingEl.classList.add('show'); }
    function hideLoading() { loadingEl.classList.remove('show'); }

    let currentTreeData = null;
    let currentJointInit = null;
    let bodyVisibility = {};
    let isolatedBody = null;

    // LUT 数据
    let lutData = null;       // workspace_lut.json
    let gridData = null;      // workspace_grid.json
    let lutAngles = null;     // [-180,-178,...,180]
    let lutAngleIndex = null; // angle → index 映射

    const wsCanvas = document.getElementById('wsCanvas');
    const wsCtx = wsCanvas.getContext('2d');
    const wsReadout = document.getElementById('wsReadout');

    // ---- 加载导出目录 ----
    const EXPORT1_BASE = '../exports/export1/';
    async function loadFromExport1() {
        try {
            hideStatus();
            showLoading('正在加载 exports/export1…');
            const resp = await fetch(EXPORT1_BASE + 'component_positions.json');
            if (!resp.ok) throw new Error('fetch component_positions.json 失败: ' + resp.status);
            const data = await resp.json();
            const fileMap = new Map();
            fileMap.get = function (key) {
                if (!key) return undefined;
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
            showStatus('❌ 加载 exports/export1 失败：' + err.message + '（需通过 http.server 访问）', true);
        }
    }

    // ---- 加载 LUT ----
    async function loadLUT() {
        try {
            const [r1, r2] = await Promise.all([
                fetch('workspace_lut.json'),
                fetch('workspace_grid.json'),
            ]);
            lutData = await r1.json();
            gridData = await r2.json();
            lutAngles = lutData.angles;
            lutAngleIndex = {};
            lutAngles.forEach((a, i) => { lutAngleIndex[a] = i; });
            drawWorkspaceMap();
            showStatus(`解空间已加载（${lutAngles.length}×${lutAngles.length} 格点）`);
        } catch (e) {
            console.warn('LUT 加载失败，地图不可用:', e);
            showStatus('⚠ 解空间数据加载失败（workspace_lut.json），仅可手动拖滑块', true);
        }
    }

    // ---- 渲染 ----
    async function renderData(data, fileMap) {
        viewer.clear();
        const components = data.components || [];
        const joints = data.joints || [];
        showLoading(`正在加载 ${components.length} 个零件的 STL…`);

        const stlLoader = new THREE.STLLoader();
        const geometryCache = new Map();
        const failedComponents = [];

        for (const comp of components) {
            const stlPath = comp.stl_file;
            const displayName = comp.occurrence_name || comp.name || '未命名';
            const matrix = comp.world_transform ? comp.world_transform.matrix : null;
            if (comp.has_children && (comp.bodies_count === 0 || comp.bodies_count === '0')) continue;
            if (!matrix) { failedComponents.push(displayName); continue; }
            let geometry = geometryCache.get(stlPath);
            if (!geometry) {
                const file = fileMap.get(stlPath) || fileMap.get(stlPath.split('/').pop());
                if (!file) { failedComponents.push(displayName); continue; }
                try {
                    const buf = await file.arrayBuffer();
                    geometry = stlLoader.parse(buf);
                    geometryCache.set(stlPath, geometry);
                } catch (e) { failedComponents.push(displayName); continue; }
            }
            viewer.addComponent(stlPath, geometry, matrix, displayName);
        }

        // 关节
        const jointWorld = {};
        for (const j of joints) {
            const g = j.geometry || {};
            const t = g.geometry_one_transform || g.geometry_two_transform;
            if (t) {
                const m = t.matrix;
                jointWorld[j.name] = [m[0][3], m[1][3], m[2][3]];
            }
        }
        currentJointInit = jointWorld;
        // buildJoints 需要 {name, matrix4x4} 格式
        viewer.buildJoints(joints.map(j => {
            const g = j.geometry || {};
            const t = g.geometry_one_transform || g.geometry_two_transform;
            return { name: j.name, matrix4x4: t ? t.matrix : null, hasLimits: !!(j.limits) };
        }));
        viewer.buildAxes();
        viewer.setAxesVisible(true);

        // 运动学树
        currentTreeData = buildTreeData(data);
        for (const body of currentTreeData.bodies) bodyVisibility[body.name] = true;
        renderTreeList(currentTreeData);
        renderComponentList(components);
        renderServoControls();

        if (failedComponents.length > 0) {
            const names = failedComponents.slice(0, 3).join(", ");
            showStatus("⚠ " + failedComponents.length + " 个零件加载失败: " + names + "…", true);
        } else {
            showStatus("✓ 加载完成: " + components.length + " 个零件, " + joints.length + " 个关节");
        }
        viewer.fitCamera();
    }

    // ---- 运动学树（硬编码） ----
    function buildTreeData(data) {
        const jointWorld = {};
        for (const j of data.joints) {
            const g = j.geometry || {};
            const t = g.geometry_one_transform || g.geometry_two_transform;
            if (t) { const m = t.matrix; jointWorld[j.name] = [m[0][3], m[1][3], m[2][3]]; }
        }
        const treeBodies = [
            ['膝盖动力发生器', null,             '旋转 1', ['膝盖动力发生器:1']],
            ['膝盖传动1',      '膝盖动力发生器', '旋转 6', ['膝盖传动1:1']],
            ['膝盖转动',       '膝盖传动1',      '旋转 7', ['膝盖转动:1']],
            ['膝盖传动2',      '膝盖转动',       '旋转 4', ['膝盖传动2:1']],
            ['大腿刚体',       null,             '旋转 2', ['大腿主动力发生器:1', '小腿保持架:1', '髋关节保持架:1', '大腿盖板:1']],
            ['小腿',           '大腿刚体',       '旋转 3', ['小腿:1']],
        ];
        return {
            bodies: treeBodies.map(([name, parent, joint, parts]) => ({
                name, parent, joint,
                jointWorld: jointWorld[joint] || null,
                parts,
            })),
        };
    }

    // ---- 树列表 ----
    function renderTreeList(treeData) {
        const el = document.getElementById('treeList');
        el.innerHTML = treeData.bodies.map(b => {
            const color = viewer._colorForName ? '#888' : '#888';
            return `<div class="tree-item" data-body="${b.name}" style="display:flex;align-items:center;gap:8px;padding:5px 6px;font-size:12px;cursor:pointer;border-radius:3px;">
                <span class="swatch" style="width:10px;height:10px;border-radius:50%;background:${color};flex-shrink:0;"></span>
                <span class="name" style="flex:1;">${b.name}</span>
                <span class="meta" style="color:#666;font-size:11px;">${b.parts.length}件</span>
            </div>`;
        }).join('');
        el.querySelectorAll('.tree-item').forEach(item => {
            const name = item.dataset.body;
            item.addEventListener('click', () => {
                viewer.highlightBody(currentTreeData.bodies.find(b => b.name === name).parts);
            });
        });
    }

    function applyVisibility() {
        for (const mesh of viewer.componentMeshes) {
            const name = mesh.userData.name;
            let visible = true;
            for (const body of currentTreeData.bodies) {
                if (body.parts.includes(name)) { visible = bodyVisibility[body.name] !== false; break; }
            }
            mesh.visible = visible;
        }
    }

    // ---- 零件列表 ----
    function renderComponentList(components) {
        componentListEl.innerHTML = components
            .filter(c => !(c.has_children && (c.bodies_count === 0 || c.bodies_count === '0')))
            .map(c => {
                const dn = c.occurrence_name || c.name;
                const colorNum = viewer._colorForName(c.name);
                const colorHex = '#' + colorNum.toString(16).padStart(6, '0');
                return `<div class="item"><span class="swatch" style="background:${colorHex};"></span><span class="name">${dn}</span></div>`;
            }).join('');
    }

    // ---- 舵机控制面板 ----
    function renderServoControls() {
        const el = document.getElementById('jointSliders');
        el.innerHTML = `
            <div style="margin:8px 0;">
                <label style="font-size:12px;color:#aaa;">θ1 膝盖舵机 <span id="t1val" style="color:#4ec9b0;">0°</span></label>
                <input type="range" id="t1Slider" min="-180" max="180" value="0" step="1" style="width:100%;">
            </div>
            <div style="margin:8px 0;">
                <label style="font-size:12px;color:#aaa;">θ2 大腿舵机 <span id="t2val" style="color:#4ec9b0;">0°</span></label>
                <input type="range" id="t2Slider" min="-180" max="180" value="0" step="1" style="width:100%;">
            </div>
            <div id="passiveAngles" style="margin-top:10px;padding:8px;background:#2a2d2e;border-radius:4px;font-family:monospace;font-size:11px;color:#888;">
                被动关节（自动求解）:<br>
                <span style="color:#ddd;">t3=<span id="p_t3">0.0</span>° t4=<span id="p_t4">0.0</span>°<br>
                t6=<span id="p_t6">0.0</span>° t7=<span id="p_t7">0.0</span>°</span>
            </div>
            <div id="servoWarn" style="margin-top:6px;color:#e74c3c;font-size:11px;min-height:14px;"></div>
        `;
        document.getElementById('t1Slider').addEventListener('input', onServoChange);
        document.getElementById('t2Slider').addEventListener('input', onServoChange);
    }

    // ---- LUT 插值（容错版：忽略个别噪声不可达点）----
    function lutInterp(t1, t2) {
        if (!lutData) return null;
        const a = lutAngles;
        const n = a.length;
        let i1 = 0;
        for (let k = 0; k < n - 1; k++) {
            if (a[k] <= t1 && a[k + 1] >= t1) { i1 = k; break; }
            if (k === n - 2) i1 = k;
        }
        let i2 = 0;
        for (let k = 0; k < n - 1; k++) {
            if (a[k] <= t2 && a[k + 1] >= t2) { i2 = k; break; }
            if (k === n - 2) i2 = k;
        }
        const t1a = a[i1], t1b = a[Math.min(i1 + 1, n - 1)];
        const t2a = a[i2], t2b = a[Math.min(i2 + 1, n - 1)];
        const fx = t1b === t1a ? 0 : (t1 - t1a) / (t1b - t1a);
        const fy = t2b === t2a ? 0 : (t2 - t2a) / (t2b - t2a);
        const cells = lutData.cells;
        // 4 个角点，标记可达性（容错：个别噪声不可达点当 0 权重跳过）
        const corners = [
            { c: cells[t1a + ',' + t2a], w: (1 - fx) * (1 - fy) },
            { c: cells[t1b + ',' + t2a], w: fx * (1 - fy) },
            { c: cells[t1a + ',' + t2b], w: (1 - fx) * fy },
            { c: cells[t1b + ',' + t2b], w: fx * fy },
        ];
        const valid = corners.filter(x => x.c && x.c.reachable);
        // 如果一个可达点都没有，真不可达
        if (valid.length === 0) return null;
        // 重新归一化权重（跳过不可达点）
        const wsum = valid.reduce((s, x) => s + x.w, 0);
        if (wsum < 1e-9) {
            // 权重全在不可达点上，但有可达点 → 用最近可达点
            const nearest = valid[0];
            return { t1: nearest.c.t1, t2: nearest.c.t2, t3: nearest.c.t3, t4: nearest.c.t4,
                     t6: nearest.c.t6, t7: nearest.c.t7, collision: nearest.c.collision,
                     connected: !!nearest.c.connected };
        }
        const lerp = (key) => {
            let s = 0;
            for (const x of valid) { s += x.c[key] * (x.w / wsum); }
            return s;
        };
        // 碰撞判断（保守）：任何可达角点碰撞 → 整格碰撞
        collision = valid.some(x => x.c.collision);
        // 连通判断（保守）：任何可达角点不连通（孤岛）→ 整格不可进
        const connected = valid.every(x => x.c.connected !== false);
        return {
            t1: lerp('t1'), t2: lerp('t2'),
            t3: lerp('t3'), t4: lerp('t4'),
            t6: lerp('t6'), t7: lerp('t7'),
            collision,
            connected,
        };
    }

    // ---- 应用姿态 ----
    function applyPoseFromLUT(t1, t2) {
        const r = lutInterp(t1, t2);
        if (!r) {
            document.getElementById('servoWarn').textContent = '⚠ 不可达区域';
            return false;
        }
        if (r.collision) {
            document.getElementById('servoWarn').textContent = '⚠ 碰撞区域（舵机限位）';
        } else {
            document.getElementById('servoWarn').textContent = '';
        }
        // 构造 bodyRotations 喂给 viewer.applyJointRotations
        const bodyAngles = {
            '膝盖动力发生器': r.t1,
            '膝盖传动1': r.t6,
            '膝盖转动': r.t7,
            '膝盖传动2': r.t4,
            '大腿刚体': r.t2,
            '小腿': r.t3,
        };
        const bodyRotations = currentTreeData.bodies.map(b => ({
            name: b.name,
            parent: b.parent,
            angle: bodyAngles[b.name] || 0,
            jointWorld: b.jointWorld,
            parts: b.parts,
        }));
        viewer.applyJointRotations(bodyRotations);
        // 更新被动角显示
        document.getElementById('p_t3').textContent = r.t3.toFixed(1);
        document.getElementById('p_t4').textContent = r.t4.toFixed(1);
        document.getElementById('p_t6').textContent = r.t6.toFixed(1);
        document.getElementById('p_t7').textContent = r.t7.toFixed(1);
        return !r.collision;
    }

    // ---- 滑块/地图交互：连续追踪，不跳变 ----
    // safeT1/safeT2 = 当前实际渲染位置（绿色区内）
    // targetT1/targetT2 = 用户想去的点（滑块/地图目标）
    // 动画循环每帧小步（MAX_STEP°）追向 target，每步检查是否还在绿色区，红区挡回
    let safeT1 = 0, safeT2 = 0;       // 当前安全位置
    let targetT1 = 0, targetT2 = 0;   // 目标位置
    const MAX_STEP = 5;              // 每帧最大步进（度）
    let stepping = false;

    // 检查 (t1,t2) 是否在绿色可达区（可达 + 不碰撞 + 与原点连通，即非孤岛）
    function isGreen(t1, t2) {
        const r = lutInterp(t1, t2);
        return r && !r.collision && r.connected;
    }

    // 综合：可达 + 不碰撞（红色碰撞区/不可达区都不能进）
    function isAllowed(t1, t2) {
        return isGreen(t1, t2);
    }

    // 设置目标（滑块或地图拖动调用）
    function setTarget(t1, t2) {
        targetT1 = t1;
        targetT2 = t2;
        stepping = true;
    }

    // 每帧小步追向 target（由 viewer 的渲染循环驱动，避免双 rAF 不同步闪烁）
    function stepTowards() {
        if (!stepping) return;
        const dt1 = targetT1 - safeT1;
        const dt2 = targetT2 - safeT2;
        const dist = Math.hypot(dt1, dt2);
        if (dist < 0.5) {
            // 到达目标
            stepping = false;
            renderAt(safeT1, safeT2);
            return;
        }
        // 步进方向（单位向量）× MAX_STEP
        const step = Math.min(MAX_STEP, dist);
        const nx = safeT1 + (dt1 / dist) * step;
        const ny = safeT2 + (dt2 / dist) * step;
        // 检查下一个点是否允许（可达 + 不碰撞 + 黄框内）
        if (isAllowed(nx, ny)) {
            safeT1 = nx;
            safeT2 = ny;
        } else {
            // 碰墙：尝试只走 θ1 或只走 θ2（贴边滑动）
            let moved = false;
            if (isAllowed(safeT1 + (dt1 / dist) * step, safeT2)) {
                safeT1 += (dt1 / dist) * step; moved = true;
            }
            if (isAllowed(safeT1, safeT2 + (dt2 / dist) * step)) {
                safeT2 += (dt2 / dist) * step; moved = true;
            }
            if (!moved) {
                // 完全挡住，停止追踪
                stepping = false;
            }
        }
        renderAt(safeT1, safeT2);
    }

    // 在 (t1,t2) 渲染：更新 3D + 滑块 + 地图指针 + 被动角
    let suppressSliderEvent = false;  // 防止 renderAt 同步滑块时触发 input → setTarget 死循环
    function renderAt(t1, t2) {
        // 滑块同步（不触发 input 事件）
        suppressSliderEvent = true;
        document.getElementById('t1Slider').value = Math.round(t1);
        document.getElementById('t2Slider').value = Math.round(t2);
        suppressSliderEvent = false;
        document.getElementById('t1val').textContent = Math.round(t1) + '°';
        document.getElementById('t2val').textContent = Math.round(t2) + '°';
        wsReadout.textContent = 'θ1=' + Math.round(t1) + '° θ2=' + Math.round(t2) + '°';
        // 3D 姿态
        const r = lutInterp(t1, t2);
        if (!r) {
            document.getElementById('servoWarn').textContent = '⚠ 不可达';
            drawWorkspacePointer(t1, t2, false);
            return;
        }
        document.getElementById('servoWarn').textContent = r.collision ? '⚠ 碰撞限位' : '';
        const bodyAngles = {
            '膝盖动力发生器': r.t1, '膝盖传动1': r.t6, '膝盖转动': r.t7,
            '膝盖传动2': r.t4, '大腿刚体': r.t2, '小腿': r.t3,
        };
        const bodyRotations = currentTreeData.bodies.map(b => ({
            name: b.name, parent: b.parent, angle: bodyAngles[b.name] || 0,
            jointWorld: b.jointWorld, parts: b.parts,
        }));
        viewer.applyJointRotations(bodyRotations);
        document.getElementById('p_t3').textContent = r.t3.toFixed(1);
        document.getElementById('p_t4').textContent = r.t4.toFixed(1);
        document.getElementById('p_t6').textContent = r.t6.toFixed(1);
        document.getElementById('p_t7').textContent = r.t7.toFixed(1);
        drawWorkspacePointer(t1, t2, !r.collision);
    }

    // 滑块拖动 → 设目标
    function onServoChange() {
        if (suppressSliderEvent) return;  // renderAt 同步滑块时不响应
        const t1 = parseInt(document.getElementById('t1Slider').value);
        const t2 = parseInt(document.getElementById('t2Slider').value);
        setTarget(t1, t2);
    }

    // ---- 解空间地图 ----
    // grid[i][j]: i=θ1 索引, j=θ2 索引
    // 绘制：X 轴=θ1（横向，左-右+），Y 轴=θ2（纵向，下-上+）
    function angleToCanvas(t1, t2) {
        const rng = gridData.range;
        return [
            ((t1 + rng) / (2 * rng)) * wsCanvas.width,
            wsCanvas.height - ((t2 + rng) / (2 * rng)) * wsCanvas.height
        ];
    }

    function drawWorkspaceMap() {
        if (!gridData) return;
        const grid = gridData.grid;
        const N = gridData.angles.length;
        const cellW = wsCanvas.width / N;
        const cellH = wsCanvas.height / N;
        const colors = ['#222222', '#27ae60', '#c72626', '#e8c530']; // 不可达/可达·连通/碰撞/可达·孤岛(黄)
        for (let i = 0; i < N; i++) {       // i = θ1
            for (let j = 0; j < N; j++) {   // j = θ2
                wsCtx.fillStyle = colors[grid[i][j]] || '#222';
                wsCtx.fillRect(i * cellW, (N - 1 - j) * cellH, Math.ceil(cellW), Math.ceil(cellH));
            }
        }
        // θ1=0 / θ2=0 十字参考线（原点居中）
        wsCtx.strokeStyle = 'rgba(255,255,255,0.35)';
        wsCtx.lineWidth = 1;
        const [zx, zy] = angleToCanvas(0, 0);
        wsCtx.beginPath();
        wsCtx.moveTo(zx, 0); wsCtx.lineTo(zx, wsCanvas.height); // θ1=0 竖线
        wsCtx.moveTo(0, zy); wsCtx.lineTo(wsCanvas.width, zy);   // θ2=0 横线
        wsCtx.stroke();
        // 原点小圆点
        wsCtx.fillStyle = '#ffffff';
        wsCtx.beginPath();
        wsCtx.arc(zx, zy, 2, 0, Math.PI * 2);
        wsCtx.fill();
    }

    function drawWorkspacePointer(t1, t2, ok) {
        if (!gridData) return;
        drawWorkspaceMap();
        const [x, y] = angleToCanvas(t1, t2);
        // 当前位置（实心）
        wsCtx.fillStyle = ok ? '#ffcc00' : '#ff4444';
        wsCtx.beginPath();
        wsCtx.arc(x, y, 5, 0, Math.PI * 2);
        wsCtx.fill();
        // 目标位置（空心，如果和当前位置不同）
        if (Math.abs(targetT1 - t1) > 0.5 || Math.abs(targetT2 - t2) > 0.5) {
            const [tx, ty] = angleToCanvas(targetT1, targetT2);
            wsCtx.strokeStyle = '#88ccff';
            wsCtx.lineWidth = 1.5;
            wsCtx.beginPath();
            wsCtx.arc(tx, ty, 4, 0, Math.PI * 2);
            wsCtx.stroke();
        }
    }

    // 地图拖动 → 设目标（不直接跳，由 stepTowards 追过去）
    let wsDragging = false;
    wsCanvas.addEventListener('mousedown', (e) => {
        wsDragging = true;
        wsCanvas.style.cursor = 'grabbing';
        handleWsDrag(e);
    });
    wsCanvas.addEventListener('mousemove', (e) => {
        if (wsDragging) handleWsDrag(e);
    });
    window.addEventListener('mouseup', () => {
        wsDragging = false;
        wsCanvas.style.cursor = 'crosshair';
    });
    function handleWsDrag(e) {
        if (!gridData) return;
        const rect = wsCanvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width;
        const y = (e.clientY - rect.top) / rect.height;
        const rng = gridData.range;
        const t1 = Math.round(x * 2 * rng - rng);
        const t2 = Math.round((1 - y) * 2 * rng - rng);
        const t1c = Math.max(-rng, Math.min(rng, t1));
        const t2c = Math.max(-rng, Math.min(rng, t2));
        // 设为目标，由 stepTowards 一步步追过去；红色区由 isAllowed 挡回
        setTarget(t1c, t2c);
    }

    // ---- 工具栏 ----
    openBtn.addEventListener('click', () => dirInput.click());
    dirInput.addEventListener('change', (e) => {
        const files = e.target.files;
        if (!files.length) return;
        // 简化：仍用 export1（目录加载逻辑保留但默认走 export1）
        loadFromExport1();
    });
    jointsToggle.addEventListener('change', () => viewer.setJointsVisible(jointsToggle.checked));
    axesToggle.addEventListener('change', () => viewer.setAxesVisible(axesToggle.checked));
    bgSelect.addEventListener('change', () => viewer.setBackgroundColor(bgSelect.value));
    fitBtn.addEventListener('click', () => viewer.fitCamera());
    document.getElementById('showAllBtn').addEventListener('click', () => {
        for (const b of currentTreeData.bodies) bodyVisibility[b.name] = true;
        applyVisibility();
    });
    document.getElementById('hideAllBtn').addEventListener('click', () => {
        for (const b of currentTreeData.bodies) bodyVisibility[b.name] = false;
        applyVisibility();
    });

    // ---- 启动 ----
    // 把 stepTowards 挂到 viewer 渲染循环，每帧渲染前更新姿态（避免双 rAF 闪烁）
    viewer.onBeforeRender = stepTowards;
    loadFromExport1().then(() => {
        return loadLUT();
    }).then(() => {
        // LUT 加载完后，强制初始渲染（画地图 + 初始姿态）
        console.log('启动完成，gridData=', !!gridData, 'lutData=', !!lutData, 'treeData=', !!currentTreeData);
        drawWorkspaceMap();
        drawWorkspacePointer(0, 0, true);
        // 应用初始姿态（home 位）
        if (currentTreeData) {
            renderAt(0, 0);
        }
    }).catch(e => {
        console.error('启动失败:', e);
        showStatus('❌ 启动失败: ' + e.message, true);
    });
})();

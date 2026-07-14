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
    const summaryEl = document.getElementById('summary');
    const componentListEl = document.getElementById('componentList');
    const jointListEl = document.getElementById('jointList');

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
        renderSummary(meta, components.length, joints.length, loadedCount, failedComponents.length, skippedContainerCount);
        renderComponentList(components);
        renderJointList(joints);

        if (failedComponents.length > 0) {
            showStatus(`⚠️ ${failedComponents.length} 个零件加载失败（STL 缺失或解析错误），详见控制台。`, false);
            setTimeout(hideStatus, 5000);
        } else if (skippedContainerCount > 0) {
            showStatus(`已跳过 ${skippedContainerCount} 个装配体容器（合并几何，避免与子零件重复渲染）。`, false);
            setTimeout(hideStatus, 4000);
        }

        // 提取机构数据，启用工作空间按钮
        currentMechData = window.FKSolver.extractMechanism(data);
        workspaceBtn.disabled = false;
    }

    // ---- UI 渲染 ----
    function renderSummary(meta, compCount, jointCount, loaded, failed, skippedContainers) {
        const rows = [
            ['导出时间', meta.export_time || '-'],
            ['几何单位', meta.geometry_unit || '-'],
            ['格式版本', meta.format_version || '-'],
            ['零件总数', compCount],
            ['关节总数', jointCount],
            ['成功加载', loaded],
            ['跳过容器', skippedContainers],
            ['加载失败', failed],
        ];
        summaryEl.innerHTML = '<dl>' + rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('') + '</dl>';
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
                // 装配体容器：未渲染（合并几何已由子零件体现）
                return `<div class="item" style="opacity:0.5">
                    <span class="swatch" style="background:#666"></span>
                    <span class="name" title="${name}（装配体，已跳过）">${name}</span>
                    <span class="meta">装配体</span>
                </div>`;
            }
            const color = colorCssForName(name);
            const stl = comp.stl_file ? comp.stl_file.split('/').pop() : '无STL';
            return `<div class="item">
                <span class="swatch" style="background:${color}"></span>
                <span class="name" title="${name}">${name}</span>
                <span class="meta" title="${comp.stl_file || ''}">${stl}</span>
            </div>`;
        }).join('');
    }

    function renderJointList(joints) {
        if (joints.length === 0) {
            jointListEl.innerHTML = '<div style="color:#666;font-size:12px">无关节</div>';
            return;
        }
        jointListEl.innerHTML = joints.map(j => {
            const hasLimits = !!j.limits && !!j.limits.revolute_limits;
            const limits = hasLimits ? j.limits.revolute_limits.rotation_limits : null;
            const range = limits
                ? `[${(limits.minimum_value).toFixed(0)}°, ${(limits.maximum_value).toFixed(0)}°]`
                : '无限制';
            const type = j.joint_type || '?';
            return `<div class="item joint">
                <span class="swatch"></span>
                <span class="name" title="${j.name}">${j.name}</span>
                <span class="meta">${type} · ${range}</span>
            </div>`;
        }).join('');
    }

    // 与 viewer.js 的 _colorForName 保持一致的 CSS 版本
    function colorCssForName(name) {
        const palette = [
            '#4e79a7', '#f28e2b', '#e15759', '#76b7b2', '#59a14f',
            '#edc948', '#b07aa1', '#ff9da7', '#9c755f', '#bab0ac'
        ];
        let h = 0;
        for (let i = 0; i < name.length; i++) {
            h = ((h << 5) - h + name.charCodeAt(i)) | 0;
        }
        return palette[Math.abs(h) % palette.length];
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

    // 点击热力图查看该角度组合的求解结果
    heatmapCanvas.addEventListener('click', (e) => {
        if (!currentHeatmap || !currentAngles) return;
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
            let info = `θ1=${t1}° θ2=${t2}°\n状态: ${limOk ? '有解·极限内' : '有解·超极限'}\n`;
            for (const k in result.angles) info += `${k}: ${result.angles[k].toFixed(1)}°\n`;
            alert(info);
        } else {
            alert(`θ1=${t1}° θ2=${t2}°\n无解: ${result.reason}`);
        }
    });

    // 初始建坐标轴（即使没加载数据也能看到）
    viewer.buildAxes();
    viewer.setAxesVisible(true);
})();

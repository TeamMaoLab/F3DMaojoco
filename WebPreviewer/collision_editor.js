/**
 * 碰撞体配置器（XZ 平面标记版）。
 *
 * 核心简化：所有圆柱沿 Y 轴（旋转都绕 Y），所以每个圆柱只需：
 *   - 中心 XZ 位置（在正视 XZ 平面点一下）
 *   - 半径 r
 *   - 高度 L（Y 方向长度）
 *   - Y 偏移（中心 Y 坐标，默认 0）
 *
 * 圆柱跟随所选 body 运动（body 绕其 pivot 旋转）。
 */
(function () {
    const EXPORT_BASE = '../exports/e3/';
    const CONFIG_URL = 'collision_config.json';

    // body 定义：[旋转中心 XZ, 颜色]
    const BODIES = {
        thigh_rigid:  { pivot: [0, 0],        color: 0xff44aa, label: '大腿' },
        knee_driver:  { pivot: [-10.9, 14.4], color: 0xff8844, label: '膝盖动力' },
        knee_link1:   { pivot: [-21.29, 8.4], color: 0x44ff44, label: '传动1' },
        knee_rotor:   { pivot: [-12.12, -7],  color: 0x4488ff, label: '膝盖转动' },
        knee_link2:   { pivot: [12.12, 7],    color: 0xffff44, label: '传动2' },
        shin:         { pivot: [25, -43.3],   color: 0x44aaff, label: '小腿' },
    };

    // 关节销（自动加）
    const JOINT_POINTS = [
        { name: 'J1', xz: [-10.9, 14.4],  body: 'knee_driver' },
        { name: 'J2', xz: [0, 0],         body: 'thigh_rigid' },
        { name: 'J3', xz: [25, -43.3],    body: 'thigh_rigid' },
        { name: 'J4', xz: [12.12, 7],     body: 'knee_rotor' },
        { name: 'J5', xz: [37.12, -36.3], body: 'knee_link2' },
        { name: 'J6', xz: [-21.29, 8.4],  body: 'knee_driver' },
        { name: 'J7', xz: [-12.12, -7],   body: 'knee_link1' },
    ];

    const container = document.getElementById('viewport');
    const statusEl = document.getElementById('status');
    const crosshair = document.getElementById('crosshair');

    // ---- Three.js ----
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1e1e1e);
    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const l1 = new THREE.DirectionalLight(0xffffff, 0.6); l1.position.set(1, -1, 1); scene.add(l1);

    let viewportW = container.clientWidth || 800;
    let viewportH = container.clientHeight || 600;
    const aspect = viewportW / viewportH;

    // 两套相机：透视 + 正交
    const perspCam = new THREE.PerspectiveCamera(50, aspect, 0.1, 100000);
    perspCam.position.set(80, -80, 60); perspCam.up.set(0, 0, 1); perspCam.lookAt(10, 0, -15);

    const orthoSize = 70;  // 正交视野半高（mm）
    const orthoCam = new THREE.OrthographicCamera(-orthoSize*aspect, orthoSize*aspect, orthoSize, -orthoSize, -1000, 1000);
    // 正交正视 XZ：相机在 +Y 方向往 -Y 看（Z-up）
    orthoCam.position.set(10, 200, -15); orthoCam.up.set(0, 0, 1); orthoCam.lookAt(10, 0, -15);

    let camera = orthoCam;  // 默认正交
    let orthoMode = true;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(viewportW, viewportH);
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.1;
    controls.target.set(10, 0, -15);
    // 正交模式下只允许平移和缩放，不允许旋转（保持正视）
    controls.enableRotate = false;

    // 坐标轴
    const axesHelper = new THREE.AxesHelper(30);
    scene.add(axesHelper);

    // 零件参考组（半透明，不包含 COL_GROUP）
    const partsGroup = new THREE.Group(); scene.add(partsGroup);
    // 圆柱碰撞体组
    const cylinderGroup = new THREE.Group(); scene.add(cylinderGroup);
    // 标记预览（鼠标悬停的虚影）
    const previewGroup = new THREE.Group(); scene.add(previewGroup);

    // ---- 加载零件（参考，不含 COL_GROUP）----
    async function loadParts() {
        const resp = await fetch(EXPORT_BASE + 'component_positions.json');
        const data = await resp.json();
        const loader = new THREE.STLLoader();
        for (const comp of data.components) {
            if (comp.has_children && (comp.bodies_count === 0)) continue;
            if (comp.name === 'COL_GROUP') continue;  // 不加载，避免干扰
            if (!comp.stl_file) continue;
            try {
                const r = await fetch(EXPORT_BASE + comp.stl_file);
                const buf = await r.arrayBuffer();
                const geo = loader.parse(buf);
                const mat = new THREE.MeshPhongMaterial({
                    color: 0x999999, transparent: true, opacity: 0.18, side: THREE.DoubleSide,
                });
                const mesh = new THREE.Mesh(geo, mat);
                partsGroup.add(mesh);
            } catch (e) { console.warn('加载失败', comp.stl_file); }
        }
    }

    // ---- 圆柱 mesh（沿 Y 轴）----
    function makeCylinderMesh(xz, radius, height, yOffset, color) {
        // CylinderGeometry 默认轴沿 Y，高度=height，正好不需要旋转
        const geo = new THREE.CylinderGeometry(radius, radius, height, 20, 1, false);
        const mat = new THREE.MeshPhongMaterial({ color: color || 0xff4444, transparent: true, opacity: 0.55 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(xz[0], yOffset, xz[1]);  // X=x, Y=yOffset, Z=z
        // 加两端半球（capsule 外观）
        const ballGeo = new THREE.SphereGeometry(radius, 14, 10);
        const b1 = new THREE.Mesh(ballGeo, mat);
        b1.position.set(0, height/2, 0); mesh.add(b1);
        const b2 = new THREE.Mesh(ballGeo, mat);
        b2.position.set(0, -height/2, 0); mesh.add(b2);
        return mesh;
    }

    // ---- 碰撞体数据 ----
    let cylinders = [];  // {id, name, xz:[x,z], radius, height, yOffset, body, mesh}
    let nextId = 1;
    let selectedId = null;

    function addCylinder(xz, radius, height, yOffset, body, name) {
        const info = BODIES[body];
        const cyl = {
            id: nextId++,
            name: name || `${body}_${cylinders.filter(c => c.body === body).length + 1}`,
            xz: [...xz], radius, height, yOffset, body,
        };
        cyl.mesh = makeCylinderMesh(xz, radius, height, yOffset, info.color);
        cyl.mesh.userData.cylId = cyl.id;
        cylinderGroup.add(cyl.mesh);
        cylinders.push(cyl);
        renderList();
        return cyl;
    }

    function removeCylinder(id) {
        const i = cylinders.findIndex(c => c.id === id);
        if (i < 0) return;
        cylinderGroup.remove(cylinders[i].mesh);
        if (selectedId === id) selectedId = null;
        cylinders.splice(i, 1);
        renderList();
    }

    function addJointCylinders() {
        for (const j of JOINT_POINTS) {
            addCylinder(j.xz, 3.0, 6.0, 0, j.body, `joint_${j.name}`);
        }
        showStatus(`已添加 ${JOINT_POINTS.length} 个关节销（r=3 高度6 沿Y）`);
    }

    // ---- 点击标记（XZ 平面拾取）----
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    // 拾取平面：Y = 当前 yOffset（默认 0）
    function getPickYOffset() { return parseFloat(document.getElementById('yOffsetInput').value) || 0; }
    let markingMode = false;

    function pickXZ(e) {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(mouse, camera);
        const yOff = getPickYOffset();
        const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -yOff);  // y = yOff
        const p = new THREE.Vector3();
        const hit = raycaster.ray.intersectPlane(plane, p);
        return hit ? [p.x, p.z] : null;  // 返回 [x, z]
    }

    let previewMesh = null;
    function onCanvasMove(e) {
        if (!markingMode) { if (previewMesh) { previewGroup.remove(previewMesh); previewMesh = null; } return; }
        const xz = pickXZ(e);
        if (!xz) return;
        const radius = parseFloat(document.getElementById('radiusInput').value) || 5;
        const height = parseFloat(document.getElementById('heightInput').value) || 8;
        const yOff = getPickYOffset();
        if (previewMesh) previewGroup.remove(previewMesh);
        previewMesh = makeCylinderMesh(xz, radius, height, yOff, 0xffcc00);
        previewMesh.material.opacity = 0.3;
        previewGroup.add(previewMesh);
        // 更新坐标读数
        coordReadout.textContent = `XZ = (${xz[0].toFixed(1)}, ${xz[1].toFixed(1)})`;
    }

    function onCanvasClick(e) {
        if (!markingMode) {
            // 非标记模式：点选圆柱
            const rect = renderer.domElement.getBoundingClientRect();
            mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
            raycaster.setFromCamera(mouse, camera);
            const hits = raycaster.intersectObjects(cylinderGroup.children, true);
            if (hits.length > 0) {
                let obj = hits[0].object;
                while (obj && !obj.userData.cylId) obj = obj.parent;
                if (obj) selectCylinder(obj.userData.cylId);
            } else {
                selectCylinder(null);
            }
            return;
        }
        const xz = pickXZ(e);
        if (!xz) return;
        const radius = parseFloat(document.getElementById('radiusInput').value) || 5;
        const height = parseFloat(document.getElementById('heightInput').value) || 8;
        const yOff = getPickYOffset();
        const body = document.getElementById('bodySelect').value;
        addCylinder(xz, radius, height, yOff, body);
        showStatus(`✓ 添加 [${BODIES[body].label}] 圆柱 @ (${xz[0].toFixed(1)}, ${xz[1].toFixed(1)}) r=${radius} L=${height} yOff=${yOff}`);
    }

    renderer.domElement.addEventListener('click', onCanvasClick);
    renderer.domElement.addEventListener('mousemove', onCanvasMove);

    function selectCylinder(id) {
        selectedId = id;
        // 高亮
        for (const c of cylinders) {
            c.mesh.material.color.setHex(c.id === id ? 0xffff00 : BODIES[c.body].color);
        }
        renderList();
        // 填入编辑框
        if (id) {
            const c = cylinders.find(x => x.id === id);
            document.getElementById('radiusInput').value = c.radius;
            document.getElementById('heightInput').value = c.height;
            document.getElementById('yOffsetInput').value = c.yOffset;
            document.getElementById('bodySelect').value = c.body;
        }
    }

    // ---- 标记模式开关 ----
    function setMarking(on) {
        markingMode = on;
        document.getElementById('markBtn').textContent = on ? '✋ 标记中（点空白处放置）' : '➕ 标记圆柱';
        document.getElementById('markBtn').classList.toggle('active', on);
        if (on) {
            controls.enableRotate = false;  // 正交本来就不能转
            showStatus('标记模式：鼠标移动看预览（黄色虚影），点击放置。半径/高度/Y偏移在左侧调。');
        } else {
            if (previewMesh) { previewGroup.remove(previewMesh); previewMesh = null; }
        }
    }
    document.getElementById('markBtn').addEventListener('click', () => setMarking(!markingMode));
    document.getElementById('addJointBtn').addEventListener('click', addJointCylinders);

    // 选中圆柱后改参数 → 实时更新
    function updateSelected() {
        if (!selectedId) return;
        const c = cylinders.find(x => x.id === selectedId);
        if (!c) return;
        c.radius = parseFloat(document.getElementById('radiusInput').value);
        c.height = parseFloat(document.getElementById('heightInput').value);
        c.yOffset = parseFloat(document.getElementById('yOffsetInput').value);
        c.body = document.getElementById('bodySelect').value;
        // 重建 mesh
        cylinderGroup.remove(c.mesh);
        c.mesh = makeCylinderMesh(c.xz, c.radius, c.height, c.yOffset, 0xffff00);
        c.mesh.userData.cylId = c.id;
        cylinderGroup.add(c.mesh);
        renderList();
    }
    ['radiusInput', 'heightInput', 'yOffsetInput'].forEach(id => {
        document.getElementById(id).addEventListener('input', updateSelected);
    });
    document.getElementById('bodySelect').addEventListener('change', updateSelected);

    // ---- 视角切换 ----
    document.getElementById('viewBtn').addEventListener('click', () => {
        orthoMode = !orthoMode;
        camera = orthoMode ? orthoCam : perspCam;
        controls.object = camera;
        controls.enableRotate = orthoMode ? false : true;
        document.getElementById('viewBtn').textContent = orthoMode ? '📐 正交（点切透视）' : '🎥 透视（点切正交）';
    });

    document.getElementById('fitBtn').addEventListener('click', () => {
        // 重置正交视野到包含所有零件
        const box = new THREE.Box3().setFromObject(partsGroup);
        const center = box.getCenter(new THREE.Vector3());
        const size = box.getSize(new THREE.Vector3());
        const fitSize = Math.max(size.x, size.z) / 2 + 10;
        orthoCam.left = -fitSize * aspect; orthoCam.right = fitSize * aspect;
        orthoCam.top = fitSize; orthoCam.bottom = -fitSize;
        orthoCam.position.set(center.x, 200, center.z);
        orthoCam.lookAt(center.x, 0, center.z);
        orthoCam.updateProjectionMatrix();
        controls.target.set(center.x, 0, center.z);
        controls.update();
    });

    // ---- 显示开关 ----
    document.getElementById('showParts').addEventListener('change', e => partsGroup.visible = e.target.checked);
    document.getElementById('showCylinders').addEventListener('change', e => cylinderGroup.visible = e.target.checked);

    // ---- 清空 ----
    document.getElementById('clearBtn').addEventListener('click', () => {
        if (!confirm('清空所有碰撞体？')) return;
        for (const c of cylinders) cylinderGroup.remove(c.mesh);
        cylinders = []; selectedId = null;
        renderList();
    });

    // ---- 保存/加载 ----
    document.getElementById('saveBtn').addEventListener('click', () => {
        const config = {
            _meta: { description: '碰撞体配置（XZ 平面标记）', units: 'mm', note: '圆柱均沿 Y 轴' },
            cylinders: cylinders.map(c => ({
                name: c.name, xz: c.xz, radius: c.radius, height: c.height,
                yOffset: c.yOffset, body: c.body,
            })),
        };
        const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'collision_config.json';
        a.click();
        showStatus(`✓ 保存 ${cylinders.length} 个碰撞体（下载到本地，放到 WebPreviewer/ 覆盖）`);
    });

    document.getElementById('loadBtn').addEventListener('click', async () => {
        try {
            const r = await fetch(CONFIG_URL + '?t=' + Date.now());
            if (!r.ok) throw new Error('未找到');
            const config = await r.json();
            for (const c of cylinders) cylinderGroup.remove(c.mesh);
            cylinders = [];
            for (const c of config.cylinders || []) {
                addCylinder(c.xz, c.radius, c.height, c.yOffset || 0, c.body, c.name);
            }
            showStatus(`✓ 加载 ${cylinders.length} 个碰撞体`);
        } catch (e) { showStatus('加载失败: ' + e.message, true); }
    });

    // ---- 列表 ----
    const coordReadout = document.getElementById('coordReadout');
    function renderList() {
        const el = document.getElementById('cylinderList');
        document.getElementById('cylCount').textContent = `(${cylinders.length})`;
        if (cylinders.length === 0) {
            el.innerHTML = '<div style="color:#666;font-size:12px;padding:8px;">还没有碰撞体。</div>';
            return;
        }
        el.innerHTML = cylinders.map(c => `
            <div class="cyl-item ${c.id === selectedId ? 'selected' : ''}" data-id="${c.id}" onclick="window._sel(${c.id})">
                <div class="head">
                    <span class="name" style="color:#${BODIES[c.body].color.toString(16).padStart(6,'0')}">${c.name}</span>
                    <button class="del" onclick="event.stopPropagation();window._del(${c.id})">删除</button>
                </div>
                <div class="info">@(${c.xz[0].toFixed(1)}, ${c.xz[1].toFixed(1)}) r=${c.radius} L=${c.height} yOff=${c.yOffset}</div>
            </div>
        `).join('');
    }
    window._del = removeCylinder;
    window._sel = selectCylinder;

    function showStatus(msg, isError) {
        statusEl.textContent = msg;
        statusEl.style.background = isError ? 'rgba(180,0,0,0.85)' : 'rgba(0,0,0,0.75)';
    }

    // ---- 渲染循环 ----
    function onResize() {
        viewportW = container.clientWidth; viewportH = container.clientHeight;
        const a = viewportW / viewportH;
        perspCam.aspect = a; perspCam.updateProjectionMatrix();
        orthoCam.left = -orthoSize * a; orthoCam.right = orthoSize * a;
        orthoCam.updateProjectionMatrix();
        renderer.setSize(viewportW, viewportH);
    }
    window.addEventListener('resize', onResize);
    new ResizeObserver(onResize).observe(container);

    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }

    onResize();
    animate();
    loadParts().then(() => {
        showStatus('零件已加载。默认正交正视 XZ。点「标记圆柱」然后在图上点位置放置。');
        renderList();
    }).catch(e => showStatus('加载失败: ' + e.message, true));
})();

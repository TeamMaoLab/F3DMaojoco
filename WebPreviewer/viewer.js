/**
 * F3DMaojoco 导出物预览器 - 3D 渲染封装
 *
 * 移植自 VistaQuickViewer（pyvista）的核心渲染逻辑：
 * - 零件按 world_transform.matrix（4x4 行优先，毫米）变换
 * - 关节用红色球 + 标签
 * - 坐标轴 XYZ 红绿蓝
 * - 相机自适应场景 bounds
 */
class ExportViewer {
    constructor(container) {
        this.container = container;

        // 场景对象集合
        this.componentMeshes = [];   // 所有零件 mesh
        this.jointGroup = null;      // 关节组（统一显隐）
        this.axesGroup = null;       // 坐标轴组
        this.allObjects = [];        // 所有需计入 bounds 的对象

        // 缓存：同一 STL 文件被多个 occurrence 引用时复用 geometry
        this._geometryCache = new Map(); // stlFilePath -> THREE.BufferGeometry

        this._initScene();
        this._initRenderer();
        this._initControls();
        this._initResizeObserver();
        this._startRenderLoop();
    }

    _initScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1e1e1e);

        // 灯光：环境光 + 两个方向光（正面+背面，避免背光面全黑）
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        const light1 = new THREE.DirectionalLight(0xffffff, 0.8);
        light1.position.set(1, -1, 1);
        this.scene.add(light1);
        const light2 = new THREE.DirectionalLight(0xffffff, 0.4);
        light2.position.set(-1, 1, -1);
        this.scene.add(light2);
    }

    _initRenderer() {
        const w = this.container.clientWidth || 800;
        const h = this.container.clientHeight || 600;
        this.camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 100000);
        // 初始相机位置：看向原点，Z-up（与 MuJoCo/Fusion 一致）
        this.camera.position.set(150, -150, 120);
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt(0, 0, 0);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(w, h);
        this.container.appendChild(this.renderer.domElement);
    }

    _initControls() {
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        // 左键旋转、右键平移、滚轮缩放（OrbitControls 默认即此）
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        // 双击 fit
        this.renderer.domElement.addEventListener('dblclick', () => this.fitCamera());
    }

    _initResizeObserver() {
        const ro = new ResizeObserver(() => this._onResize());
        ro.observe(this.container);
    }

    _onResize() {
        const w = this.container.clientWidth;
        const h = this.container.clientHeight;
        if (w === 0 || h === 0) return;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    _startRenderLoop() {
        const animate = () => {
            requestAnimationFrame(animate);
            this.controls.update();
            this.renderer.render(this.scene, this.camera);
        };
        animate();
    }

    /**
     * 清空场景中的零件/关节/坐标轴（保留灯光）
     */
    clear() {
        for (const m of this.componentMeshes) {
            this.scene.remove(m);
            // geometry 由缓存管理，不在此 dispose（缓存统一管）
            if (m.material) m.material.dispose();
        }
        this.componentMeshes = [];
        this._geometryCache.clear();

        if (this.jointGroup) { this.scene.remove(this.jointGroup); this.jointGroup = null; }
        if (this.axesGroup) { this.scene.remove(this.axesGroup); this.axesGroup = null; }
        this.allObjects = [];
    }

    /**
     * 从 4x4 行优先矩阵（JSON 里的 matrix 字段）构造 THREE.Matrix4
     * 并清理浮点噪声（<1e-10 归零，移植自 VistaQuickViewer._apply_transform）
     */
    _matrixFromJson(matrix4x4) {
        const m = new THREE.Matrix4();
        const cleaned = [];
        for (let r = 0; r < 4; r++) {
            for (let c = 0; c < 4; c++) {
                let v = matrix4x4[r][c];
                if (Math.abs(v) < 1e-10) v = 0.0;
                cleaned.push(v);
            }
        }
        // THREE.Matrix4.set 接受行优先参数
        m.set(
            cleaned[0], cleaned[1], cleaned[2], cleaned[3],
            cleaned[4], cleaned[5], cleaned[6], cleaned[7],
            cleaned[8], cleaned[9], cleaned[10], cleaned[11],
            cleaned[12], cleaned[13], cleaned[14], cleaned[15]
        );
        return m;
    }

    /**
     * 按 occurrence 名称生成颜色（移植自 VistaQuickViewer._generate_color）
     * 返回 {css, hex}
     */
    _colorForName(name) {
        const palette = [
            0x4e79a7, 0xf28e2b, 0xe15759, 0x76b7b2, 0x59a14f,
            0xedc948, 0xb07aa1, 0xff9da7, 0x9c755f, 0xbab0ac
        ];
        // 简单字符串哈希
        let h = 0;
        for (let i = 0; i < name.length; i++) {
            h = ((h << 5) - h + name.charCodeAt(i)) | 0;
        }
        const hex = palette[Math.abs(h) % palette.length];
        return {
            hex: hex,
            css: '#' + hex.toString(16).padStart(6, '0')
        };
    }

    /**
     * 添加一个零件
     * @param {string} stlKey - STL 相对路径（缓存键）
     * @param {THREE.BufferGeometry} geometry - 已解析的 STL geometry
     * @param {number[][]} matrix4x4 - world_transform.matrix
     * @param {string} displayName - 显示名（occurrence_name）
     */
    addComponent(stlKey, geometry, matrix4x4, displayName) {
        // 统一灰白半透明材质
        const material = new THREE.MeshPhongMaterial({
            color: 0xcccccc,
            transparent: true,
            opacity: 0.6,
            shininess: 30
        });
        const mesh = new THREE.Mesh(geometry, material);
        // 不 bake 矩阵到几何，改用 mesh.matrix 驱动（便于姿态更新时只改矩阵，无需克隆几何）
        const initMatrix = this._matrixFromJson(matrix4x4);
        mesh.matrix.copy(initMatrix);
        mesh.matrixAutoUpdate = false;
        mesh.matrixWorldNeedsUpdate = true;
        mesh.userData = {
            name: displayName,
            color: '#cccccc',
            initMatrix: initMatrix.clone(), // 保存初始矩阵，applyPose 基于此叠加旋转
        };
        this.scene.add(mesh);
        this.componentMeshes.push(mesh);
        this.allObjects.push(mesh);
        return mesh;
    }

    /**
     * 应用姿态：根据求解结果更新所有运动零件的矩阵
     * @param {Object} partRotations - {零件名: 旋转角(度,绕Y轴)}
     * @param {Object} jointPositions - {关节名: [x,z]} 新位置
     * @param {Object} jointInitPositions - {关节名: [x,z]} 初始位置（用于算旋转中心）
     * @param {Object} partJointMap - {零件名: 关节点A名} 每个零件绕哪个关节点转
     */
    applyPose(partRotations, jointPositions, jointInitPositions, partJointMap) {
        for (const mesh of this.componentMeshes) {
            const name = mesh.userData.name;
            const rotDeg = partRotations[name];
            if (rotDeg === undefined) continue; // 静止零件不动
            const jointA = partJointMap[name];
            if (!jointA) continue;

            const initMatrix = mesh.userData.initMatrix;
            const A_init = jointInitPositions[jointA];
            const A_new = jointPositions[jointA];
            if (!A_init || !A_new) continue;

            // M_new = T(A_new) × Ry(θ) × T(-A_init) × M_init
            // A 是世界坐标 [x,z]，Y 分量取 0（平面机构）
            const theta = rotDeg * Math.PI / 180;
            const Ry = new THREE.Matrix4().makeRotationY(theta);
            const T_back = new THREE.Matrix4().makeTranslation(-A_init[0], 0, -A_init[1]);
            const T_fwd = new THREE.Matrix4().makeTranslation(A_new[0], 0, A_new[1]);

            const M = new THREE.Matrix4();
            M.multiplyMatrices(T_fwd, Ry);
            M.multiply(T_back);
            M.multiply(initMatrix);
            mesh.matrix.copy(M);
            mesh.matrixWorldNeedsUpdate = true;
        }
    }

    /** 复位所有零件到初始姿态 */
    resetPose() {
        for (const mesh of this.componentMeshes) {
            if (mesh.userData.initMatrix) {
                mesh.matrix.copy(mesh.userData.initMatrix);
                mesh.matrixWorldNeedsUpdate = true;
            }
        }
    }

    /**
     * 应用树关节旋转（核心运动学方法）
     *
     * 原理: 子坐标系相对父坐标系，绕关节点旋转。
     *   每个刚体的世界变换 = 父刚体变换 × 绕关节点旋转
     *   按树深度从根到叶顺序应用，子节点继承父节点的累积变换。
     *
     * @param {Array} bodyRotations - [{name, parent, jointWorld:[x,y,z], angle(度), parts:[occurrence名]}]
     *   jointWorld: 关节点在【父刚体局部坐标系】下的坐标（用于相对旋转）
     *   但初始位姿下所有零件原点在(0,0,0)且无旋转，父局部=世界，所以 jointWorld 用世界坐标即可
     */
    applyJointRotations(bodyRotations) {
        // 1. 先复位所有零件到初始姿态
        for (const mesh of this.componentMeshes) {
            if (mesh.userData.initMatrix) {
                mesh.matrix.copy(mesh.userData.initMatrix);
            }
        }

        // 2. 按树深度排序（父先处理）
        //    计算每个刚体的累积变换矩阵
        const cumTransforms = {};  // bodyName -> THREE.Matrix4 (累积变换)

        // 找根节点（parent=null）先处理
        function getDepth(b) {
            if (!b.parent) return 0;
            const p = bodyRotations.find(x => x.name === b.parent);
            return p ? getDepth(p) + 1 : 0;
        }
        const sorted = [...bodyRotations].sort((a, b) => getDepth(a) - getDepth(b));

        // 3. 逐个刚体：计算累积变换，应用到其所有零件
        for (const body of sorted) {
            let cumMatrix;
            if (!body.parent) {
                // 根节点：绕世界坐标系的关节点转
                cumMatrix = this._rotationAroundPoint(body.angle, body.jointWorld);
            } else {
                // 子节点：父累积变换 × (绕父局部坐标系的关节点转)
                const parentCum = cumTransforms[body.parent] || new THREE.Matrix4();
                // 关节点世界坐标 → 父局部坐标（用父 initMatrix 的逆）
                // 但初始位姿下父局部=世界，且父已经转了，所以关节点在父当前局部 = 父initMatrix逆 × jointWorld
                // 简化：因为所有零件初始都在原点无旋转，jointWorld 在父局部 = jointWorld
                const localRot = this._rotationAroundPoint(body.angle, body.jointWorld);
                cumMatrix = new THREE.Matrix4().multiplyMatrices(parentCum, localRot);
            }
            cumTransforms[body.name] = cumMatrix;

            // 应用到该刚体的所有零件
            for (const partName of body.parts) {
                const mesh = this.componentMeshes.find(m => m.userData.name === partName);
                if (!mesh || !mesh.userData.initMatrix) continue;
                const M = new THREE.Matrix4().multiplyMatrices(cumMatrix, mesh.userData.initMatrix);
                mesh.matrix.copy(M);
                mesh.matrixWorldNeedsUpdate = true;
            }
        }
    }

    /**
     * 用关节点新位置直接确定刚体姿态（闭链最可靠方式）
     *
     * 原理: 每个刚体由两个关节点确定姿态（连杆方向）。
     *   初始方向 = atan2(JB_init.z - JA_init.z, JB_init.x - JA_init.x) (XZ平面)
     *   新方向   = atan2(JB_new.z  - JA_new.z,  JB_new.x  - JA_new.x)
     *   旋转角   = 新方向 - 初始方向 (绕Y轴)
     *   平移     = JA_new - 旋转(JA_init)  (= JA_new, 因为零件原点在0时简化)
     *
     * 大腿刚体的4个零件共享同一刚体变换（由J2→J3方向确定）。
     *
     * @param {Object} newPositions - {关节名: [x,y,z]} 新世界坐标
     * @param {Object} initPositions - {关节名: [x,y,z]} 初始世界坐标
     * @param {Array} bodyDefs - [{name, parts:[occurrence], jA:关节名, jB:关节名}]
     *   jA/jB: 确定该刚体姿态的两个关节点
     */
    applyJointPositions(newPositions, initPositions, bodyDefs) {
        for (const body of bodyDefs) {
            const JAInit = initPositions[body.jA];
            const JBInit = initPositions[body.jB];
            const JANew = newPositions[body.jA];
            const JBNew = newPositions[body.jB];
            if (!JAInit || !JBInit || !JANew || !JBNew) continue;

            // 初始方向和新方向（XZ 平面，绕 Y 轴）
            const a0 = Math.atan2(JBInit[2] - JAInit[2], JBInit[0] - JAInit[0]);
            const a1 = Math.atan2(JBNew[2] - JANew[2], JBNew[0] - JANew[0]);
            let theta = a1 - a0;
            while (theta > Math.PI) theta -= 2 * Math.PI;
            while (theta < -Math.PI) theta += 2 * Math.PI;

            // 刚体变换: 先绕 JA_init 转 theta, 再平移使 JA_init 到 JA_new
            // M = T(JA_new) · Ry(theta) · T(-JA_init)
            const Ry = new THREE.Matrix4().makeRotationY(theta);
            const T_back = new THREE.Matrix4().makeTranslation(-JAInit[0], -JAInit[1], -JAInit[2]);
            const T_fwd = new THREE.Matrix4().makeTranslation(JANew[0], JANew[1], JANew[2]);
            const bodyM = new THREE.Matrix4();
            bodyM.multiplyMatrices(T_fwd, Ry);
            bodyM.multiply(T_back);

            // 应用到该刚体的所有零件
            for (const partName of body.parts) {
                const mesh = this.componentMeshes.find(m => m.userData.name === partName);
                if (!mesh || !mesh.userData.initMatrix) continue;
                const M = new THREE.Matrix4().multiplyMatrices(bodyM, mesh.userData.initMatrix);
                mesh.matrix.copy(M);
                mesh.matrixWorldNeedsUpdate = true;
            }
        }
    }

    /** 构造"绕指定点转指定角度(度,绕Y轴)"的变换矩阵: T(p) × Ry(θ) × T(-p) */
    _rotationAroundPoint(angleDeg, point) {
        const theta = angleDeg * Math.PI / 180;
        const Ry = new THREE.Matrix4().makeRotationY(theta);
        const T_back = new THREE.Matrix4().makeTranslation(-point[0], -point[1], -point[2]);
        const T_fwd = new THREE.Matrix4().makeTranslation(point[0], point[1], point[2]);
        const M = new THREE.Matrix4();
        M.multiplyMatrices(T_fwd, Ry);
        M.multiply(T_back);
        return M;
    }

    /**
     * 高亮一个刚体的所有零件：变亮黄色 + 在第一个零件(主零件)显示坐标系框架
     * @param {Array|null} partNames - 零件 occurrence_name 列表，null 表示取消高亮
     */
    highlightBody(partNames) {
        // 清除之前的高亮
        for (const mesh of this.componentMeshes) {
            if (mesh.userData.highlighted) {
                mesh.material.color.setHex(0xcccccc);
                mesh.material.opacity = 0.6;
                mesh.userData.highlighted = false;
            }
        }
        // 清除之前的坐标系框架
        if (this._coordFrame) {
            this.scene.remove(this._coordFrame);
            this._coordFrame = null;
        }
        if (!partNames || partNames.length === 0) return;

        // 高亮该刚体的所有零件
        for (const name of partNames) {
            const mesh = this.componentMeshes.find(m => m.userData.name === name);
            if (!mesh) continue;
            mesh.material.color.setHex(0xffdd00);
            mesh.material.opacity = 0.9;
            mesh.userData.highlighted = true;
        }

        // 在主零件（第一个）原点显示坐标系框架（XYZ轴+三面）
        const mainMesh = this.componentMeshes.find(m => m.userData.name === partNames[0]);
        if (!mainMesh) return;
        const frame = new THREE.Group();
        const size = 8; // 框架尺寸 mm

        // 三根轴
        const makeAxis = (dir, color) => {
            const geo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0,0,0), dir.clone().multiplyScalar(size)
            ]);
            return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, linewidth: 3 }));
        };
        frame.add(makeAxis(new THREE.Vector3(1,0,0), 0xff0000)); // X 红
        frame.add(makeAxis(new THREE.Vector3(0,1,0), 0x00cc00)); // Y 绿
        frame.add(makeAxis(new THREE.Vector3(0,0,1), 0x0066ff)); // Z 蓝

        // 三个半透明面
        const planeGeo = new THREE.PlaneGeometry(size, size);
        const pXY = new THREE.Mesh(planeGeo, new THREE.MeshBasicMaterial({ color: 0x0066ff, transparent: true, opacity: 0.15, side: THREE.DoubleSide }));
        frame.add(pXY);
        const pYZ = new THREE.Mesh(planeGeo, new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.15, side: THREE.DoubleSide }));
        pYZ.rotation.y = Math.PI / 2;
        frame.add(pYZ);
        const pXZ = new THREE.Mesh(planeGeo, new THREE.MeshBasicMaterial({ color: 0x00cc00, transparent: true, opacity: 0.15, side: THREE.DoubleSide }));
        pXZ.rotation.x = -Math.PI / 2;
        frame.add(pXZ);

        // 定位到主零件原点，朝向跟随其 initMatrix
        const partPos = new THREE.Vector3().setFromMatrixPosition(mainMesh.userData.initMatrix);
        frame.position.copy(partPos);
        const partQuat = new THREE.Quaternion().setFromRotationMatrix(mainMesh.userData.initMatrix);
        frame.quaternion.copy(partQuat);

        this._coordFrame = frame;
        this.scene.add(frame);
    }

    /**
     * 批量构建关节（红色球 + 标签）
     * @param {Array} joints - [{name, matrix4x4, hasLimits, rotationAxis}]
     */
    buildJoints(joints) {
        if (this.jointGroup) { this.scene.remove(this.jointGroup); }
        const group = new THREE.Group();
        group.visible = false; // 默认隐藏
        const sphereGeo = new THREE.SphereGeometry(2.0, 16, 12); // 半径 2mm

        for (const j of joints) {
            const mat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
            const sphere = new THREE.Mesh(sphereGeo, mat);
            sphere.applyMatrix4(this._matrixFromJson(j.matrix4x4));
            group.add(sphere);

            // 标签：用 canvas 生成 sprite
            const label = this._makeLabelSprite(j.name + (j.hasLimits ? '' : ' ⚠'));
            const pos = new THREE.Vector3();
            this._matrixFromJson(j.matrix4x4).decompose(
                pos, new THREE.Quaternion(), new THREE.Vector3()
            );
            label.position.copy(pos);
            label.position.y += 4; // 球上方
            group.add(label);
        }

        this.jointGroup = group;
        this.scene.add(group);
        // 关节不计入 allObjects（不参与相机 fit，避免球半径干扰）
    }

    _makeLabelSprite(text) {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        canvas.width = 256; canvas.height = 64;
        ctx.fillStyle = 'rgba(0,0,0,0.6)';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.font = 'bold 28px sans-serif';
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, canvas.width / 2, canvas.height / 2);
        const tex = new THREE.CanvasTexture(canvas);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
        sprite.scale.set(16, 4, 1);
        return sprite;
    }

    setJointsVisible(visible) {
        if (this.jointGroup) this.jointGroup.visible = visible;
    }

    /**
     * 构建坐标轴（XYZ 红绿蓝，移植自 VistaQuickViewer，axis_length=50mm）
     */
    buildAxes() {
        if (this.axesGroup) { this.scene.remove(this.axesGroup); }
        const group = new THREE.Group();
        const len = 50;
        const make = (dir, color) => {
            const geo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0, 0, 0),
                dir.clone().multiplyScalar(len)
            ]);
            const mat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
            return new THREE.Line(geo, mat);
        };
        group.add(make(new THREE.Vector3(1, 0, 0), 0xff0000)); // X 红
        group.add(make(new THREE.Vector3(0, 1, 0), 0x00cc00)); // Y 绿
        group.add(make(new THREE.Vector3(0, 0, 1), 0x0066ff)); // Z 蓝
        this.axesGroup = group;
        this.scene.add(group);
    }

    setAxesVisible(visible) {
        if (this.axesGroup) this.axesGroup.visible = visible;
        else if (visible) { this.buildAxes(); }
    }

    setBackgroundColor(cssColor) {
        this.scene.background = new THREE.Color(cssColor);
    }

    /**
     * 适配相机：计算所有零件 bounds，定位相机（移植自 VistaQuickViewer._setup_camera）
     */
    fitCamera() {
        if (this.componentMeshes.length === 0) return;
        const box = new THREE.Box3();
        for (const m of this.componentMeshes) {
            box.expandByObject(m);
        }
        if (box.isEmpty()) return;

        const center = box.getCenter(new THREE.Vector3());
        const size = box.getSize(new THREE.Vector3());
        const maxDim = Math.max(size.x, size.y, size.z, 50);
        const distance = maxDim * 2.5;

        this.camera.position.set(
            center.x + distance,
            center.y - distance,
            center.z + distance * 0.8
        );
        this.camera.up.set(0, 0, 1); // Z-up（与 MuJoCo/Fusion 一致）
        this.camera.lookAt(center);
        this.controls.target.copy(center);
        this.controls.update();
    }
}

// 暴露到全局
window.ExportViewer = ExportViewer;

/**
 * 双闭环平面机构正向运动学求解器（JS 移植自 Python 验证脚本）
 *
 * 机构拓扑（XZ 平面，Y 是旋转轴）：
 *   闭环1（膝盖四连杆）: J1 ── J6 ── J7 ── (闭合到 J1)
 *   闭环2（腿+膝盖传动）: J2 ── J3 ── J5 ── J4 ── (闭合到 J2)
 *   耦合: J7 和 J4 都在"膝盖转动"零件上（刚体约束 |J4-J7| 不变）
 *
 * 主动角: θ1 (J1, 膝盖舵机), θ2 (J2, 大腿舵机)
 * 用圆交点解析法（非数值迭代），无收敛问题、确定性、毫秒级。
 */

/**
 * 从 component_positions.json 提取关节点初始位置（XZ 平面）
 * @param {Object} data - 解析后的 component_positions.json
 * @returns {Object} { joints: {name: [x,z]}, limits: {name: [minDeg,maxDeg]|null} }
 */
function extractMechanism(data) {
    const joints = {};
    const limits = {};
    for (const j of data.joints) {
        const g = j.geometry || {};
        const t = g.geometry_one_transform || g.geometry_two_transform;
        if (t && t.matrix) {
            const m = t.matrix;
            joints[j.name] = [m[0][3], m[2][3]]; // [X, Z]
        }
        const lim = j.limits;
        if (lim && lim.revolute_limits && lim.revolute_limits.rotation_limits) {
            const rl = lim.revolute_limits.rotation_limits;
            limits[j.name] = [rl.minimum_value * 180 / Math.PI, rl.maximum_value * 180 / Math.PI];
        } else {
            limits[j.name] = null;
        }
    }
    return { joints, limits };
}

/** 二维向量距离 */
function dist(a, b) {
    const dx = a[0] - b[0], dz = a[1] - b[1];
    return Math.sqrt(dx * dx + dz * dz);
}

/** 2D 旋转矩阵作用 */
function rotApply(angle, v, center) {
    const c = Math.cos(angle), s = Math.sin(angle);
    const dx = v[0] - center[0], dz = v[1] - center[1];
    return [center[0] + c * dx - s * dz, center[1] + s * dx + c * dz];
}

/**
 * 两圆相交点（平面 XZ）。
 * @returns {Array} 0/1/2 个 [x,z] 点
 */
function circleIntersect(c1, r1, c2, r2) {
    const dx = c2[0] - c1[0], dz = c2[1] - c1[1];
    const d = Math.sqrt(dx * dx + dz * dz);
    const eps = 1e-9;
    if (d > r1 + r2 + eps || d < Math.abs(r1 - r2) - eps || d < eps) return [];
    const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    let hSq = r1 * r1 - a * a;
    if (hSq < 0) hSq = 0;
    const h = Math.sqrt(hSq);
    const mx = c1[0] + a * dx / d;
    const mz = c1[1] + a * dz / d;
    // 垂直方向
    const px = -dz / d, pz = dx / d;
    return [
        [mx + h * px, mz + h * pz],
        [mx - h * px, mz - h * pz]
    ];
}

/** 向量角度（atan2） */
function angleOf(v) { return Math.atan2(v[1], v[0]); }

/**
 * 联立求解双闭环 FK（给定 θ1/θ2，牛顿迭代解 4 个被动角使两闭环同时闭合）
 *
 * 与 solveFK（分步圆交点）不同，此版本联立求解，正确处理闭环耦合。
 * 返回关节点新位置（XZ 平面），供 applyJointPositions 用。
 *
 * @param {number} theta1Deg - 主动角1（度）
 * @param {number} theta2Deg - 主动角2（度）
 * @param {Object} mech - extractMechanism 返回的机构数据
 * @param {Array} x0 - [θ3,θ4,θ6,θ7] 初始猜测（弧度），用于连续追踪避免跳分支
 * @returns {Object} { ok, jointPositions:{关节名:[x,z]}, angles:{...度}, err, passive }
 */
function solveFKCoupled(theta1Deg, theta2Deg, mech, x0) {
    const J = mech.joints;
    const th1 = theta1Deg * Math.PI / 180;
    const th2 = theta2Deg * Math.PI / 180;

    function rot(p, ang, center) {
        const c = Math.cos(ang), s = Math.sin(ang);
        const dx = p[0] - center[0], dz = p[1] - center[1];
        return [center[0] + c * dx - s * dz, center[1] + s * dx + c * dz];
    }

    function computePos(th3, th4, th6, th7) {
        const J1 = J['旋转 1'], J2 = J['旋转 2'], J3 = J['旋转 3'], J4 = J['旋转 4'],
              J5 = J['旋转 5'], J6 = J['旋转 6'], J7 = J['旋转 7'];
        // 膝盖链累积: th1(J1) -> th6(J6) -> th7(J7) -> th4(J4)
        let J2_rotor = rot(rot(rot(J2, th7, J7), th6, J6), th1, J1);
        let J5_link2 = rot(rot(rot(rot(J5, th4, J4), th7, J7), th6, J6), th1, J1);
        // 大腿链: th2(J2) -> th3(J3)
        let J2_thigh = J2;
        let J5_shin = rot(rot(J5, th3, J3), th2, J2);
        return { J2_rotor, J2_thigh, J5_link2, J5_shin };
    }

    function constraints(th3, th4, th6, th7) {
        const p = computePos(th3, th4, th6, th7);
        return [
            p.J2_rotor[0] - p.J2_thigh[0],
            p.J2_rotor[1] - p.J2_thigh[1],
            p.J5_link2[0] - p.J5_shin[0],
            p.J5_link2[1] - p.J5_shin[1],
        ];
    }

    // 牛顿迭代
    let x = x0 ? x0.slice() : [0, 0, 0, 0.01];
    const eps = 1e-6;
    let err = 1e9;
    for (let it = 0; it < 100; it++) {
        const f = constraints(x[0], x[1], x[2], x[3]);
        err = Math.hypot(f[0], f[1], f[2], f[3]);
        if (err < 1e-9) break;
        // 数值雅可比 4x4
        const Jac = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
        for (let j = 0; j < 4; j++) {
            const dx = [0,0,0,0]; dx[j] = eps;
            const f2 = constraints(x[0]+dx[0], x[1]+dx[1], x[2]+dx[2], x[3]+dx[3]);
            for (let i = 0; i < 4; i++) Jac[i][j] = (f2[i] - f[i]) / eps;
        }
        // 解 Jac * dx = f, 更新 x -= dx (高斯消元)
        const sol = solveLinear(Jac, f);
        if (!sol) return { ok: false, reason: '雅可比奇异' };
        for (let i = 0; i < 4; i++) x[i] -= sol[i];
    }
    if (err > 0.01) return { ok: false, reason: '不收敛, err=' + err.toFixed(4) };

    // 计算所有关节点新位置
    const th3=x[0], th4=x[1], th6=x[2], th7=x[3];
    const p = computePos(th3, th4, th6, th7);
    // 算各关节点完整位置
    const J1=J['旋转 1'],J6=J['旋转 6'],J7=J['旋转 7'],J4=J['旋转 4'],J3=J['旋转 3'],J2=J['旋转 2'],J5=J['旋转 5'];
    const J1n = J1.slice();
    const J6n = rot(J6, th1, J1);
    const J7n = rot(rot(J7, th6, J6), th1, J1);
    const J4n = rot(rot(rot(J4, th7, J7), th6, J6), th1, J1);
    const J5n = p.J5_link2;  // = 膝盖传动2上的J5 = 小腿上的J5 (闭环闭合)
    const J2n = J2.slice();  // J2是大腿旋转中心,不动
    const J3n = rot(J3, th2, J2);

    return {
        ok: true,
        err: err,
        passive: { th3, th4, th6, th7 },
        jointPositions: {
            '旋转 1': J1n, '旋转 2': J2n, '旋转 3': J3n, '旋转 4': J4n,
            '旋转 5': J5n, '旋转 6': J6n, '旋转 7': J7n,
        },
        // 世界角(用于显示)
        angles: {
            '旋转 1': theta1Deg, '旋转 2': theta2Deg,
            '旋转 3': (th3 + th2) * 180 / Math.PI,  // 小腿世界角
            '旋转 4': (th4 + th7 + th6 + th1) * 180 / Math.PI,
            '旋转 6': (th6 + th1) * 180 / Math.PI,
            '旋转 7': (th7 + th6 + th1) * 180 / Math.PI,
        }
    };
}

/** 4x4 线性方程求解（高斯消元） */
function solveLinear(A, b) {
    const M = A.map((row, i) => row.concat(b[i])); // 增广矩阵
    for (let i = 0; i < 4; i++) {
        // 选主元
        let maxRow = i;
        for (let k = i + 1; k < 4; k++) {
            if (Math.abs(M[k][i]) > Math.abs(M[maxRow][i])) maxRow = k;
        }
        [M[i], M[maxRow]] = [M[maxRow], M[i]];
        if (Math.abs(M[i][i]) < 1e-12) return null;
        for (let k = i + 1; k < 4; k++) {
            const f = M[k][i] / M[i][i];
            for (let j = i; j <= 4; j++) M[k][j] -= f * M[i][j];
        }
    }
    const x = [0,0,0,0];
    for (let i = 3; i >= 0; i--) {
        let s = M[i][4];
        for (let j = i + 1; j < 4; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return x;
}

/**
 * 求解正向运动学（分步圆交点版本，旧逻辑，手动模式用）
 * @param {number} theta1Deg - 主动角1（度），膝盖舵机
 * @param {number} theta2Deg - 主动角2（度），大腿舵机
 * @param {Object} mech - extractMechanism 返回的机构数据
 * @returns {Object} { ok: bool, angles: {...deg}, jointPositions: {...[x,z]}, reason: string }
 */
function solveFK(theta1Deg, theta2Deg, mech) {
    const J = mech.joints;
    const theta1 = theta1Deg * Math.PI / 180;
    const theta2 = theta2Deg * Math.PI / 180;

    const J1 = J['旋转 1'], J2 = J['旋转 2'], J3 = J['旋转 3'], J4 = J['旋转 4'];
    const J5 = J['旋转 5'], J6 = J['旋转 6'], J7 = J['旋转 7'];

    // 连杆长度（刚体，不变）
    const L_1_6 = dist(J6, J1);
    const L_6_7 = dist(J7, J6);
    const L_7_1 = dist(J1, J7);
    const L_2_3 = dist(J3, J2);
    const L_3_5 = dist(J5, J3);
    const L_5_4 = dist(J4, J5);
    const L_4_2 = dist(J2, J4);
    const L_7_4 = dist(J4, J7); // 耦合两闭环的刚体

    // 步骤1: 主动角驱动 J6 和 J3
    const J1n = J1.slice();
    const J6n = rotApply(theta1, J6, J1);
    const J2n = J2.slice();
    const J3n = rotApply(theta2, J3, J2);

    // 辅助：选旋转角绝对值最小的候选（连续运动假设，而非位置最近）
    function normA(a) { while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
    function pickMinRotation(candidates, computeAngle) {
        let best = candidates[0], bestAbs = 1e9;
        for (const p of candidates) {
            const a = Math.abs(normA(computeAngle(p)));
            if (a < bestAbs) { bestAbs = a; best = p; }
        }
        return best;
    }
    // thigh 逆变换：世界坐标 → thigh frame（绕 J2 逆转 theta2）
    function thighInv(p) {
        return rotApply(-theta2, p, J2n);
    }

    // 步骤2: 闭环1 求 J7（选 θ6 绝对值最小的解）
    const J7c = circleIntersect(J6n, L_6_7, J1n, L_7_1);
    if (J7c.length === 0) return { ok: false, reason: '闭环1 无交点' };
    const J7n = pickMinRotation(J7c, p =>
        angleOf([p[0]-J6n[0], p[1]-J6n[1]]) - angleOf([J7[0]-J6[0], J7[1]-J6[1]]));

    // 步骤3: 闭环2 求 J4（选 J4-J7 方向变化最小的解）
    const J4c = circleIntersect(J7n, L_7_4, J2n, L_4_2);
    if (J4c.length === 0) return { ok: false, reason: '闭环2 J4 无交点' };
    const J4n = pickMinRotation(J4c, p =>
        angleOf([p[0]-J7n[0], p[1]-J7n[1]]) - angleOf([J4[0]-J7[0], J4[1]-J7[1]]));

    // 步骤4: 闭环2 求 J5（选 θ3 绝对值最小的解，θ3 在 thigh frame 里算）
    const J5c = circleIntersect(J3n, L_3_5, J4n, L_5_4);
    if (J5c.length === 0) return { ok: false, reason: '闭环2 J5 无交点' };
    const J5n = pickMinRotation(J5c, p => {
        const pThigh = thighInv(p);
        return angleOf([pThigh[0]-J3[0], pThigh[1]-J3[1]]) - angleOf([J5[0]-J3[0], J5[1]-J3[1]]);
    });

    // 步骤5: 计算被动角（相对初始位姿的转角，用正确的 frame 变换）
    const angles = {
        '旋转 1': theta1Deg,
        '旋转 2': theta2Deg,
        // θ3: 小腿相对大腿，在 thigh frame 里算
        '旋转 3': normA(angleOf([thighInv(J5n)[0]-J3[0], thighInv(J5n)[1]-J3[1]]) - angleOf([J5[0]-J3[0], J5[1]-J3[1]])) * 180 / Math.PI,
        // θ4: 膝盖传动2 绕 J4
        '旋转 4': normA(angleOf([J5n[0]-J4n[0], J5n[1]-J4n[1]]) - angleOf([J5[0]-J4[0], J5[1]-J4[1]])) * 180 / Math.PI,
        '旋转 5': normA(angleOf([J4n[0]-J5n[0], J4n[1]-J5n[1]]) - angleOf([J4[0]-J5[0], J4[1]-J5[1]])) * 180 / Math.PI,
        // θ6: 膝盖传动1 绕 J6
        '旋转 6': normA(angleOf([J7n[0]-J6n[0], J7n[1]-J6n[1]]) - angleOf([J7[0]-J6[0], J7[1]-J6[1]])) * 180 / Math.PI,
        // θ7: 膝盖传动1→膝盖转动 绕 J7
        '旋转 7': normA(angleOf([J1n[0]-J7n[0], J1n[1]-J7n[1]]) - angleOf([J1[0]-J7[0], J1[1]-J7[1]])) * 180 / Math.PI,
    };

    // 角度归一化到 [-180, 180]
    for (const k in angles) {
        if (k !== '旋转 1' && k !== '旋转 2') {
            let a = angles[k] % 360;
            if (a > 180) a -= 360;
            if (a < -180) a += 360;
            angles[k] = a;
        }
    }

    // 计算每个运动零件绕 Y 轴的旋转角（度）
    // 零件姿态由其两端关节点 A->B 的方向确定：旋转角 = 新方向角 - 初始方向角
    // 这些角度用于更新 Three.js mesh 的变换矩阵
    function partAngle(JA_init, JB_init, JA_new, JB_new) {
        const a0 = angleOf([JB_init[0] - JA_init[0], JB_init[1] - JA_init[1]]);
        const a1 = angleOf([JB_new[0] - JA_new[0], JB_new[1] - JA_new[1]]);
        let d = (a1 - a0) * 180 / Math.PI;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }
    const partRotations = {
        '膝盖动力发生器:1':  partAngle(J1, J6, J1n, J6n),
        '膝盖传动1:1':       partAngle(J6, J7, J6n, J7n),
        '膝盖转动:1':        partAngle(J7, J4, J7n, J4n),
        '膝盖传动2:1':       partAngle(J4, J5, J4n, J5n),
        '大腿主动力发生器:1': partAngle(J2, J3, J2n, J3n),
        '小腿:1':            partAngle(J3, J5, J3n, J5n),
    };

    return {
        ok: true,
        angles,
        partRotations,
        jointPositions: { '旋转 1': J1n, '旋转 2': J2n, '旋转 3': J3n, '旋转 4': J4n, '旋转 5': J5n, '旋转 6': J6n, '旋转 7': J7n },
        reason: 'OK'
    };
}

/**
 * 检查被动角是否在关节极限内
 * @returns {bool} 全部在极限内（无极限的关节视为通过）
 */
function checkLimits(result, mech) {
    if (!result.ok) return false;
    for (const name in result.angles) {
        const lim = mech.limits[name];
        if (lim) {
            const a = result.angles[name];
            if (a < lim[0] - 0.5 || a > lim[1] + 0.5) return false;
        }
    }
    return true;
}

// 暴露到全局（Web Worker 里用 self，主线程用 window）
if (typeof self !== 'undefined') {
    self.extractMechanism = extractMechanism;
    self.solveFK = solveFK;
    self.checkLimits = checkLimits;
}
if (typeof window !== 'undefined') {
    window.FKSolver = { extractMechanism, solveFK, solveFKCoupled, checkLimits };
}

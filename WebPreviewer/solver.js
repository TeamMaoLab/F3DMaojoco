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
 * 求解正向运动学
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

    // 步骤2: 闭环1 求 J7
    const J7c = circleIntersect(J6n, L_6_7, J1n, L_7_1);
    if (J7c.length === 0) return { ok: false, reason: '闭环1 无交点' };
    const J7n = J7c.reduce((best, p) => dist(p, J7) < dist(best, J7) ? p : best, J7c[0]);

    // 步骤3: 闭环2 求 J4（J4 距 J7 = L_7_4，距 J2 = L_4_2）
    const J4c = circleIntersect(J7n, L_7_4, J2n, L_4_2);
    if (J4c.length === 0) return { ok: false, reason: '闭环2 J4 无交点' };
    const J4n = J4c.reduce((best, p) => dist(p, J4) < dist(best, J4) ? p : best, J4c[0]);

    // 步骤4: 闭环2 求 J5（J5 距 J3 = L_3_5，距 J4 = L_5_4）
    const J5c = circleIntersect(J3n, L_3_5, J4n, L_5_4);
    if (J5c.length === 0) return { ok: false, reason: '闭环2 J5 无交点' };
    const J5n = J5c.reduce((best, p) => dist(p, J5) < dist(best, J5) ? p : best, J5c[0]);

    // 步骤5: 计算被动角（相对初始位姿的方向变化）
    const angles = {
        '旋转 1': theta1Deg,
        '旋转 2': theta2Deg,
        '旋转 3': (angleOf([J5n[0]-J3n[0], J5n[1]-J3n[1]]) - angleOf([J5[0]-J3[0], J5[1]-J3[1]])) * 180 / Math.PI,
        '旋转 4': (angleOf([J5n[0]-J4n[0], J5n[1]-J4n[1]]) - angleOf([J5[0]-J4[0], J5[1]-J4[1]])) * 180 / Math.PI,
        '旋转 5': (angleOf([J4n[0]-J5n[0], J4n[1]-J5n[1]]) - angleOf([J4[0]-J5[0], J4[1]-J5[1]])) * 180 / Math.PI,
        '旋转 6': (angleOf([J7n[0]-J6n[0], J7n[1]-J6n[1]]) - angleOf([J7[0]-J6[0], J7[1]-J6[1]])) * 180 / Math.PI,
        '旋转 7': (angleOf([J1n[0]-J7n[0], J1n[1]-J7n[1]]) - angleOf([J1[0]-J7[0], J1[1]-J7[1]])) * 180 / Math.PI,
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
    window.FKSolver = { extractMechanism, solveFK, checkLimits };
}

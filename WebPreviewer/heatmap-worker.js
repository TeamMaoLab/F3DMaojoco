/**
 * 工作空间可达性扫描器（BFS 洪水填充 + coupled solver 初值传染）
 *
 * 关键修正: 旧版用 solveFK（分步圆交点 + pick_min_rotation 启发式），
 *   会把真实有解的格子误判为无解（假黑色区）。改用 solveFKCoupled（牛顿迭代），
 *   并用 BFS 从原点扩散，每个格子用相邻已解格的解做初值 → 分支连续追踪，
 *   画出真实可达区。
 *
 * 接收: { mechData, rangeMin, rangeMax, step }
 * 回传: { type:'progress', done, total } 和 { type:'done', heatmap, angles, stats }
 *   heatmap[i][j]: 0=无解, 1=有解超极限, 2=有解极限内
 */
importScripts('solver.js');  // 复用 extractMechanism / checkLimits

self.onmessage = function (e) {
    const { mechData, rangeMin, rangeMax, step } = e.data;
    const rMin = rangeMin !== undefined ? rangeMin : -90;
    const rMax = rangeMax !== undefined ? rangeMax : 90;
    const st = step !== undefined ? step : 2;

    const angles = [];
    for (let a = rMin; a <= rMax + 0.001; a += st) angles.push(Math.round(a * 10) / 10);
    const N = angles.length;

    // grid[i][j]: 0=未解, 1=已解超极限, 2=已解极限内
    const grid = [];
    for (let i = 0; i < N; i++) grid.push(new Array(N).fill(0));
    const x0grid = [];
    for (let i = 0; i < N; i++) x0grid.push(new Array(N).fill(null));

    // 原点 (0,0) 求解（无初值）
    const mid = findNearestIndex(angles, 0);
    const x0Origin = solveCoupled(0, 0, mechData, null);
    if (x0Origin) {
        grid[mid][mid] = 2;  // 原点必在极限内
        x0grid[mid][mid] = x0Origin;
    }
    const total = N * N;
    let done = 1;
    self.postMessage({ type: 'progress', done, total });

    // BFS 从原点向四邻扩散
    const queue = [[mid, mid]];
    const dirs = [[0, 1], [0, -1], [1, 0], [-1, 0]];
    while (queue.length > 0) {
        const [i, j] = queue.shift();
        const x0 = x0grid[i][j];
        for (const [di, dj] of dirs) {
            const ni = i + di, nj = j + dj;
            if (ni < 0 || ni >= N || nj < 0 || nj >= N) continue;
            if (grid[ni][nj] !== 0) continue;  // 已解
            const t1 = angles[ni], t2 = angles[nj];
            const x0Next = solveCoupled(t1, t2, mechData, x0);
            if (x0Next) {
                // 检查关节极限（用 solveFKCoupled 的返回格式构造 result 给 checkLimits）
                const inLim = checkLimitsAt(t1, t2, x0Next, mechData);
                grid[ni][nj] = inLim ? 2 : 1;
                x0grid[ni][nj] = x0Next;
                queue.push([ni, nj]);
            } else {
                grid[ni][nj] = -1;  // 标记尝试过但无解（防止重复尝试）
            }
            done++;
            if (done % 200 === 0) {
                self.postMessage({ type: 'progress', done: Math.min(done, total), total });
            }
        }
    }

    // 统计 + 清理 -1 → 0
    let solvable = 0, inLimits = 0, noSolution = 0;
    for (let i = 0; i < N; i++) {
        for (let j = 0; j < N; j++) {
            if (grid[i][j] === -1) grid[i][j] = 0;
            if (grid[i][j] === 0) noSolution++;
            else { solvable++; if (grid[i][j] === 2) inLimits++; }
        }
    }

    self.postMessage({
        type: 'done',
        heatmap: grid,
        angles,
        stats: { solvable, inLimits, noSolution, total }
    });
};

/** 找 angles 里最接近 target 的索引 */
function findNearestIndex(angles, target) {
    let best = 0, bestD = 1e9;
    for (let k = 0; k < angles.length; k++) {
        const d = Math.abs(angles[k] - target);
        if (d < bestD) { bestD = d; best = k; }
    }
    return best;
}

/**
 * coupled 牛顿迭代求解（与 solver.js solveFKCoupled 相同逻辑，但只返回 passive 数组）
 * 返回 [th3,th4,th6,th7] 弧度 或 null
 */
function solveCoupled(theta1Deg, theta2Deg, mech, x0) {
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
        const J2_rotor = rot(rot(rot(J2, th7, J7), th6, J6), th1, J1);
        const J5_link2 = rot(rot(rot(rot(J5, th4, J4), th7, J7), th6, J6), th1, J1);
        const J2_thigh = J2;
        const J5_shin = rot(rot(J5, th3, J3), th2, J2);
        return [J2_rotor, J2_thigh, J5_link2, J5_shin];
    }
    function constraints(th3, th4, th6, th7) {
        const p = computePos(th3, th4, th6, th7);
        return [
            p[0][0] - p[1][0], p[0][1] - p[1][1],
            p[2][0] - p[3][0], p[2][1] - p[3][1],
        ];
    }

    let x = x0 ? x0.slice() : [0, 0, 0, 0.01];
    const eps = 1e-6;
    let err = 1e9;
    for (let it = 0; it < 60; it++) {
        const f = constraints(x[0], x[1], x[2], x[3]);
        err = Math.hypot(f[0], f[1], f[2], f[3]);
        if (err < 1e-9) break;
        const Jac = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
        for (let j = 0; j < 4; j++) {
            const dx = [0,0,0,0]; dx[j] = eps;
            const f2 = constraints(x[0]+dx[0], x[1]+dx[1], x[2]+dx[2], x[3]+dx[3]);
            for (let i = 0; i < 4; i++) Jac[i][j] = (f2[i] - f[i]) / eps;
        }
        const sol = solveLinear(Jac, f);
        if (!sol) return null;
        for (let i = 0; i < 4; i++) x[i] -= sol[i];
    }
    if (err > 0.01) return null;
    return x;
}

/** 4x4 高斯消元 */
function solveLinear(A, b) {
    const M = A.map((row, i) => row.concat(b[i]));
    for (let i = 0; i < 4; i++) {
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
 * 检查给定 passive 解是否在关节极限内
 * 用世界角比较（与 solver.js solveFKCoupled 的 angles 字段一致）
 */
function checkLimitsAt(t1d, t2d, x, mech) {
    const th1 = t1d * Math.PI / 180, th2 = t2d * Math.PI / 180;
    const th3 = x[0], th4 = x[1], th6 = x[2], th7 = x[3];
    const angles = {
        '旋转 1': t1d,
        '旋转 2': t2d,
        '旋转 3': (th3 + th2) * 180 / Math.PI,
        '旋转 4': (th4 + th7 + th6 + th1) * 180 / Math.PI,
        '旋转 6': (th6 + th1) * 180 / Math.PI,
        '旋转 7': (th7 + th6 + th1) * 180 / Math.PI,
    };
    for (const name in angles) {
        const lim = mech.limits[name];
        if (lim) {
            const a = angles[name];
            if (a < lim[0] - 0.5 || a > lim[1] + 0.5) return false;
        }
    }
    return true;
}

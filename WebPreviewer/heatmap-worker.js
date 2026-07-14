/**
 * Web Worker: 扫描 180×180 主动角组合，生成可解性热力图
 *
 * 接收: { mechData, rangeMin, rangeMax, step }
 *   mechData: 从 component_positions.json 提取的机构数据（extractMechanism 结果）
 *   rangeMin/rangeMax: 主动角扫描范围（度），默认 [-90, 90]
 *   step: 步长（度），默认 2
 * 回传: { type:'progress', done, total } 和 { type:'done', heatmap, stats }
 *   heatmap: 二维数组 [i][j] = 状态码
 *     0 = 无解, 1 = 有解但超关节极限, 2 = 有解且在极限内
 *   stats: { solvable, inLimits, noSolution, total }
 */
importScripts('solver.js');

self.onmessage = function (e) {
    const { mechData, rangeMin, rangeMax, step } = e.data;
    const rMin = rangeMin !== undefined ? rangeMin : -90;
    const rMax = rangeMax !== undefined ? rangeMax : 90;
    const st = step !== undefined ? step : 2;

    // 生成角度序列
    const angles = [];
    for (let a = rMin; a <= rMax + 0.001; a += st) angles.push(Math.round(a * 10) / 10);
    const N = angles.length;

    const heatmap = [];
    let solvable = 0, inLimits = 0, noSolution = 0;
    const total = N * N;
    let done = 0;

    for (let i = 0; i < N; i++) {
        const row = [];
        for (let j = 0; j < N; j++) {
            const t1 = angles[i];
            const t2 = angles[j];
            const result = solveFK(t1, t2, mechData);
            let code = 0;
            if (result.ok) {
                solvable++;
                if (checkLimits(result, mechData)) {
                    code = 2;
                    inLimits++;
                } else {
                    code = 1;
                }
            } else {
                noSolution++;
            }
            row.push(code);
            done++;
        }
        heatmap.push(row);
        // 每行回传一次进度
        self.postMessage({ type: 'progress', done, total, rowIdx: i });
    }

    self.postMessage({
        type: 'done',
        heatmap,
        angles,
        stats: { solvable, inLimits, noSolution, total }
    });
};

// 生成 foot_lut.json：遍历 workspace_lut 所有可达·连通点，
// 用 quad_leg_viewer 的 applyLeg 摆出姿态，读 shin 足端世界坐标。
//
// 足端定义：shin mesh 局部坐标系里，距腿根（0,-47,0）最远的那个顶点（固定索引）。
// 用固定索引追踪同一个物理足端，避免"最远顶点随姿态变"。
//
// 运行：node scripts/gen_foot_lut.js
// 输出：exports/quad_v4/foot_lut.json

const puppeteer = require('puppeteer-core');
const fs = require('fs');
const path = require('path');

const ROOT = '/home/mg/AIMAO/F3DMaojoco';
const URL = 'http://localhost:8766/WebPreviewer/quad_leg_viewer.html';
const OUT = path.join(ROOT, 'exports/quad_v4/foot_lut.json');

(async () => {
  const lut = JSON.parse(fs.readFileSync(path.join(ROOT, 'WebPreviewer/workspace_lut.json')));
  // 只取可达·连通点（绿区）
  const samples = [];
  for (const [k, v] of Object.entries(lut.cells)) {
    if (!v.collision && v.connected !== false) {
      const [t1, t2] = k.split(',').map(Number);
      samples.push([t1, t2]);
    }
  }
  console.log(`可达·连通点：${samples.length}`);

  const browser = await puppeteer.launch({
    executablePath: '/usr/bin/chromium-browser',
    args: ['--no-sandbox','--use-gl=swiftshader','--enable-unsafe-swiftshader'],
    headless: 'new',
  });
  const page = await browser.newPage();
  await page.goto(URL, {waitUntil:'domcontentloaded'});
  await page.waitForFunction(() => !document.getElementById('loading') || document.getElementById('loading').style.display==='none', {timeout:30000}).catch(()=>{});
  await new Promise(r=>setTimeout(r,2500));

  // 先在 (0,0) 找出 shin 距腿根最远的顶点索引（在 shin geometry 里的索引）
  await page.evaluate(() => window.setLegImmediate('FR', 0, 0));
  await new Promise(r=>setTimeout(r,300));
  const tipInfo = await page.evaluate(() => {
    const lg = window.legGroups.FR;
    const shinGroup = lg.bodies['shin'].group;
    let tip = null;
    shinGroup.traverse(obj => {
      if (obj.isMesh && obj.geometry && !tip) {
        obj.updateWorldMatrix(true, true);
        const pos = obj.geometry.attributes.position;
        const root = new THREE.Vector3(0, -47, 0);
        let maxD = -1, maxIdx = -1;
        const v = new THREE.Vector3();
        for (let i=0; i<pos.count; i++) {
          v.fromBufferAttribute(pos, i);
          obj.localToWorld(v);  // 局部→世界
          const d = v.distanceTo(root);
          if (d > maxD) { maxD = d; maxIdx = i; }
        }
        tip = {meshFound: true, tipIndex: maxIdx, tipLocal: [pos.getX(maxIdx), pos.getY(maxIdx), pos.getZ(maxIdx)]};
      }
    });
    return tip;
  });
  console.log('足端顶点索引:', tipInfo.tipIndex, '局部坐标:', tipInfo.tipLocal);

  // 遍历所有可达点，读该顶点的世界坐标
  const out = {
    leg_root: [0, -47, 0],
    tip_index: tipInfo.tipIndex,
    tip_local: tipInfo.tipLocal,
    samples: []
  };

  const startT = Date.now();
  for (let i = 0; i < samples.length; i++) {
    const [t1, t2] = samples[i];
    await page.evaluate(([a,b]) => window.setLegImmediate('FR', a, b), [t1, t2]);
    // 不等固定时间，applyLeg 是同步的，setLegImmediate 内部 applyLeg 同步完成
    const foot = await page.evaluate((tipIdx) => {
      const lg = window.legGroups.FR;
      const shinGroup = lg.bodies['shin'].group;
      let result = null;
      shinGroup.traverse(obj => {
        if (obj.isMesh && obj.geometry && !result) {
          obj.updateWorldMatrix(true, true);
          const pos = obj.geometry.attributes.position;
          const v = new THREE.Vector3(pos.getX(tipIdx), pos.getY(tipIdx), pos.getZ(tipIdx));
          obj.localToWorld(v);
          result = [v.x, v.z];  // 只要 X 和 Z
        }
      });
      return result;
    }, tipInfo.tipIndex);
    out.samples.push({theta: [t1, t2], foot: foot});
    if ((i+1) % 500 === 0) {
      const elapsed = ((Date.now()-startT)/1000).toFixed(1);
      console.log(`  ${i+1}/${samples.length}  (${elapsed}s)`);
    }
  }

  fs.writeFileSync(OUT, JSON.stringify(out));
  console.log(`\n写入 ${OUT}`);
  console.log(`  ${out.samples.length} 个样本`);
  // 样例
  console.log('  样例:');
  for (const s of out.samples.slice(0, 5)) {
    console.log(`    θ=[${s.theta}] → foot=[${s.foot.map(x=>x.toFixed(1))}]`);
  }
  // footX / footZ 范围
  const xs = out.samples.map(s=>s.foot[0]);
  const zs = out.samples.map(s=>s.foot[1]);
  console.log(`  footX 范围: [${Math.min(...xs).toFixed(1)}, ${Math.max(...xs).toFixed(1)}]`);
  console.log(`  footZ 范围: [${Math.min(...zs).toFixed(1)}, ${Math.max(...zs).toFixed(1)}]`);

  await browser.close();
})();

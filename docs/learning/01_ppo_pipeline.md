# 笔记 1：PPO 训练管线全景

> **目标**：读完这篇你能回答 ——
> 1. 调一次 `train()` 从开始到返回，主要分几个阶段？每个阶段在干什么？
> 2. 训练日志里的 "step / reward / episode_length" 是谁打印的？多久打印一次？
> 3. 为什么我们昨天 5M 步训练"看不到日志"？怎么改才能看见？
>
> **对应能力**：看得懂 PPO 训练在干什么、看得懂日志结构、知道怎么让自己能看见训练过程。

---

## 0. 一句话总览

Brax 的 PPO `train()` 是一个**纯函数**：吃 env + 超参 + 一个 `progress_fn` 回调，吐 `(make_policy, params, metrics)`。它的内部就是三层嵌套循环：

```
外层 epoch 循环（progress_fn 在这里被调用，约 num_evals 次）
└─ training_step（rollout 采样 + SGD 一次）
   ├─ generate_unroll × N（采一批数据）
   └─ sgd_step × num_updates_per_batch（在这批数据上做 K 轮梯度更新）
      └─ minibatch_step × num_minibatches（把数据切成 M 块逐块算梯度）
         └─ loss_and_pgrad（PPO loss + 反向传播）
```

理解这四层循环 + 一个回调，就理解了 80% 的 PPO 训练流程。

---

## 1. 源码定位

所有路径都是：
- `BRAX = .venv/lib/python3.12/site-packages/brax/training/`
- 主文件：`BRAX/agents/ppo/train.py`（875 行）

关键函数与行号：

| 内容 | 位置 |
|---|---|
| `train()` 函数签名 + 超参默认值 | `train.py:176-242` |
| `TrainingState` 数据类 | `train.py:52-60` |
| `env_step_per_training_step` 公式 | `train.py:367-369` |
| `num_evals_after_init` / `num_training_steps_per_epoch` 公式 | `train.py:370-381` |
| 内层 `minibatch_step`（loss + grad + optimizer） | `train.py:493-520` |
| 中层 `sgd_step`（切 minibatch + scan） | `train.py:522-556` |
| 外层 `training_step`（rollout + SGD） | `train.py:558-655` |
| `training_epoch`（外层 scan + pmap） | `train.py:657-676` |
| 初始 eval + `progress_fn(0, ...)` | `train.py:786-796` |
| 主循环 + 循环内 `progress_fn(current_step, ...)` | `train.py:806-856` |

---

## 2. train() 的 13 个阶段（A-M）

> 不需要全记，记住 **G（TrainingState 初始化）、K（初始 eval）、L（主循环）** 就够 debug 用。

| 阶段 | 行号 | 做什么 | 重要程度 |
|---|---|---|---|
| **A** 设备与步数预算 | `347-381` | 检测 GPU 数，算 `env_step_per_training_step`（每个 training step 跑多少 env 步） | ⭐⭐⭐ |
| B RNG 拆分 | `383-391` | 拆 key 给 env/network/eval | ⭐ |
| C Env 转换 | `395-427` | 调 `wrap_env_fn` 包 env（Vmap + Episode + AutoReset） | ⭐⭐ |
| D 网络构建 | `432-449` | 调 `network_factory` 建 policy/value 网络 | ⭐⭐ |
| E Optimizer + loss 装配 | `451-491` | adam + clip + 装配 `loss_fn` | ⭐⭐ |
| F 三个嵌套 inner 函数 | `493-655` | minibatch_step / sgd_step / training_step（真正的训练逻辑） | ⭐⭐⭐ |
| **G** TrainingState 初始化 | `705-723` | 网络 forward 一次拿初始 params，组装 TrainingState | ⭐⭐ |
| H Checkpoint 恢复 | `725-743` | 如果给了 restore_checkpoint_path，从这里加载 | ⭐ |
| I 特殊早退 | `745-754` | `num_timesteps == 0` 直接返回（debug 用） | ⭐ |
| J Eval 环境构建 | `756-778` | 单独建一个 eval env（不 vmap 到 device） | ⭐ |
| **K** 初始 eval | `784-804` | 跑一次 eval，调 `progress_fn(0, metrics)` | ⭐⭐⭐ |
| **L** 主训练循环 | `806-856` | `for it in range(num_evals_after_init)` 循环，每个 it 跑一个 epoch + eval + 保存 ckpt + 调 `progress_fn` | ⭐⭐⭐ |
| M 收尾 | `858-875` | assert 步数够了，`_unpmap` 出最终 params | ⭐ |

### 阶段 A 最关键的公式

```python
# train.py:367-369
env_step_per_training_step = batch_size * unroll_length * num_minibatches * action_repeat
```

**这是 PPO 一次"外层迭代"消耗的 env 步数**。比如我们 quad 的配置：
```
batch_size=256 × unroll_length=20 × num_minibatches=32 × action_repeat=1
= 163,840 步/training_step
```

记住这个数：**16 万步/training_step**。这决定了 checkpoint 文件名为什么是 `000001638400` —— 这正是我们跑了 1 个 training_step（× num_resets_per_eval=10）后保存的检查点。

> 🔍 **为什么 checkpoint 文件名是 1638400**？因为 `num_resets_per_eval=10`，一个 eval 周期跑 10 个 training_step × 163840 步/step = 1,638,400 步。我们昨天训练"卡住"的真相：**不是卡住，是跑完了一个 eval 周期、写完第一个 checkpoint，但因为编译期 + 第一个 epoch 用了 2-3 分钟、看起来没动静就被停了**。

---

## 3. 三层循环细节（阶段 F）

### 3.1 最内层：`minibatch_step`（`train.py:493-520`）

一次梯度更新。流程：
1. 调 `loss_and_pgrad_fn(params, normalizer_params, data, key)` → `(metrics, grads)`
2. 如果开了 adaptive KL lr，用 `metrics['kl_mean']` 调学习率
3. `optimizer.update(grads)` + `optax.apply_updates(params, update)`

**这一层对应 PPO 论文里的"在 minibatch 上做 K 步梯度上升"**。

### 3.2 中层：`sgd_step`（`train.py:522-556`）

把一批数据切成 `num_minibatches` 块，对每块跑一次 `minibatch_step`：

```python
# train.py:549-554 简化
data = jax.random.permutation(key, data)         # 打乱
data = data.reshape(num_minibatches, batch_size, unroll_length, ...)  # 切块
carry, _ = jax.lax.scan(minibatch_step, carry, data)  # 逐块更新
```

**一次 sgd_step = 一份 rollout 数据被完整扫一遍（32 个 minibatch 各跑一次梯度更新）**。

### 3.3 外层：`training_step`（`train.py:558-655`）

这是 PPO 的主迭代单元：
1. **rollout 采样**：`jax.lax.scan(generate_unroll, ...)` 跑 `batch_size * num_minibatches // num_envs` 次，凑够一个 training_step 需要的数据（`train.py:586-591`）
2. **normalizer 更新**：用刚采的数据更新 obs 归一化统计量（`train.py:611-619`）
3. **SGD**：`jax.lax.scan(sgd_step, ..., length=num_updates_per_batch)`（`train.py:621-628`）—— **同一批 rollout 数据被重复用 `num_updates_per_batch=4` 次**

> 🔍 **关键概念「数据复用」**：PPO 是 on-policy 算法，严格说一批数据只能用一次。但实践中"重复用 K 次"（K=4 在 Go1/quad 都用 4）能大幅提速，代价是数据逐渐变 off-policy（因为每次梯度更新后策略就变了）。`clipping_epsilon` 就是用来限制"策略变化不能太大"的，正是为了容忍这种数据复用。

---

## 4. 主循环（阶段 L）—— `progress_fn` 的真相

```python
# train.py:806-856（简化）
for it in range(num_evals_after_init):                              # 外层循环
    for _ in range(max(num_resets_per_eval, 1)):                    # 每 eval 周期 reset 几次
        (training_state, env_state, training_metrics) = (
            training_epoch_with_timing(training_state, env_state, epoch_keys)
        )
        current_step = int(_unpmap(training_state.env_steps))

    if process_id != 0: continue                                    # 只有进程 0 才继续

    policy_params_fn(current_step, make_policy, params)             # 回调 1：保存 policy
    if save_checkpoint_path: checkpoint.save(...)                   # 保存 checkpoint
    if num_evals > 0:
        metrics = evaluator.run_evaluation(params, training_metrics)
        progress_fn(current_step, metrics)                          # 回调 2：进度回调 ★
```

**`progress_fn` 只在两个地方被调用**：

| 调用点 | 行号 | 触发条件 | 频率 |
|---|---|---|---|
| 初始 eval | `train.py:796` | `num_evals > 1 and run_evals` | 1 次（step=0） |
| 循环内 | `train.py:856` | `process_id == 0 and num_evals > 0` | `num_evals_after_init` 次 |

### 触发次数公式

```
progress_fn 总调用次数 = 1（初始 eval） + num_evals_after_init（循环内）
                       = 1 + max(num_evals - 1, 1)
                       ≈ num_evals（当 num_evals > 1）
```

**整个训练只回调 `num_evals` 次**。默认 `num_evals=1` 时 → `num_evals_after_init=1` → 循环内调 1 次，**且初始 eval 因 `num_evals > 1` 不满足被跳过** → 总共 1 次（训练快结束时才调）。

---

## 5. ⭐ 专章：为什么我们昨天"看不到日志"

> 这是昨天你停训练的真正原因。也是这篇笔记最重要的部分。

### 5.1 现象

昨天跑 `train_quad_pg.py`，配置 `num_evals=5`，目标 5M 步。结果 2-3 分钟没看到任何输出，你以为是卡住了，手动停了。

### 5.2 真相（用实际配置算给你看）

我们 `train_quad_pg.py:46-49` 的配置：
```python
num_timesteps = 5_000_000
num_evals     = 5
num_resets_per_eval = 10   # brax 默认值
```

带入 `train.py:370-381` 公式：
```
env_step_per_training_step = 256 × 20 × 32 × 1 = 163,840 步
num_evals_after_init       = max(5 - 1, 1) = 4
num_training_steps_per_epoch = ceil(5,000,000 / (4 × 163,840 × 10)) = 1
```

**所以整个 5M 训练里，`progress_fn` 只回调 5 次**：

| 时刻 | env 步数 | 触发位置 |
|---|---|---|
| 编译完、初始 eval | 0 | `train.py:796` |
| 第 1 个 epoch 跑完 | 1,638,400 | `train.py:856` |
| 第 2 个 epoch 跑完 | 3,276,800 | `train.py:856` |
| 第 3 个 epoch 跑完 | 4,915,200 | `train.py:856` |
| 第 4 个 epoch 跑完 | 6,553,600（cap 在 5M） | `train.py:856` |

### 5.3 为什么"看起来卡住"

```
启动 → [JIT 编译，无输出] → 1-2 分钟 → progress_fn(0) 打印初始 eval
     → [训练 1 个 epoch ≈ 30-60s，无输出] → progress_fn(1638400)
     → ...
```

编译期（1-2 分钟）一次都不打印 → 你以为卡住，停了。
checkpoint 文件名 `000001638400` = 第一个 epoch 跑完写的，证明**训练实际在跑**。

### 5.4 三种解法（任选其一）

#### 解法 A（推荐）：调大 `num_evals`

把 `num_evals` 从 5 改到 50。代价：每个 eval 跑一次完整 episode（1000 步 × num_eval_envs），eval 会有开销，但能看到更密的日志。

```python
# train_quad_pg.py:48
num_evals = 50   # 从 5 改到 50 → 大约每 100k 步一次日志
```

⚠️ 副作用：`num_evals` 变大 → `num_training_steps_per_epoch` 变小（公式 `train.py:374-381`）→ 每个 epoch 内 SGD 次数变少 → 可能影响收敛速度。但只要 `env_step_per_training_step` 不变，总训练步数不变。

#### 解法 B：开 `log_training_metrics`（细粒度训练 loss）

`train()` 有个隐藏参数 `log_training_metrics`（`train.py:231`，默认 `False`）。开了之后，每 `training_metrics_steps` 步（默认 = `env_step_per_training_step`）会通过 `EpisodeMetricsLogger` 调一次 `progress_fn`，metrics 里多出 `training/loss_mean`、`training/policy_loss_mean`、`training/kl_mean` 等**训练 loss 指标**。

```python
# train_quad_pg.py 的 train_fn 里加两个参数
train_fn = functools.partial(
    ppo.train,
    ...
    log_training_metrics=True,             # 开启训练 loss 上报
    training_metrics_steps=50_000,         # 每 5 万步报一次（默认是 16 万步太稀）
)
```

**优点**：能看到 PPO loss 曲线（policy_loss / v_loss / entropy_loss / kl_mean），是判断训练健康度的关键信号。
**缺点**：会增加少量 host-side 开销（`jax.debug.callback`）。

#### 解法 C：在 `progress_fn` 里加自己的心跳

Brax 训练过程中其实**有内部 logging**（`logging.info(...)`），但走的是 absl 默认 logger，默认不显示到 stdout。在脚本最前面加：

```python
# train_quad_pg.py 开头
import absl.logging
absl.logging.set_verbosity('info')  # 或 'debug'
absl.logging.use_python_logging()   # 接到 python logging
```

这样能看到 brax 内部的 `starting iteration 0 123.45` 这种行（`train.py:807`）—— 至少知道训练循环在动。

### 5.5 推荐组合

实际调参时，把三种组合起来：

```python
ppo_params = dict(
    ...
    num_evals=20,                       # 解法 A：每 ~25 万步报一次 eval
    log_training_metrics=True,          # 解法 B：开训练 loss 上报
    training_metrics_steps=50_000,      # 解法 B：训练 loss 每 5 万步报一次
)
# 脚本开头加 absl.logging.set_verbosity('info')  # 解法 C
```

这样下次训练你至少能看到：
- 每 5 万步：training loss 曲线（看 loss 在不在线性下降）
- 每 25 万步：eval reward（看 policy 在变好）
- 实时：brax 内部 "starting iteration N" 心跳

---

## 6. 我们 `train_quad_pg.py` 哪里对应哪个阶段

| Brax train() 阶段 | 我们代码对应 |
|---|---|
| 创建 env | `train_quad_pg.py:111` `env = QuadJoystick()` |
| 配置 PPO 超参 | `train_quad_pg.py:118-145` `ppo_params` + `network_factory` |
| 构造 `train_fn` | `train_quad_pg.py:147-154` `functools.partial(ppo.train, ...)` |
| 触发 train() | `train_quad_pg.py:161` `train_fn(environment=env, progress_fn=progress, ...)` |
| `progress_fn` 实现 | `train_quad_pg.py:72-90` `make_progress_fn` —— 这里**就是你要改"看不见日志"的地方** |

### ⚠️ 我们 progress_fn 的问题（顺便修）

`train_quad_pg.py:74-77`：

```python
def progress(step, metrics):
    if step == 0:
        times.append(time.time())
    else:
        times.append(time.time())
```

`if/else` 两个分支干一样的事（append time），这个 if 没意义。还有 `elapsed` 只在 step 0 之后才算，初始 eval 那次 elapsed=0 看不出东西。**这个 progress_fn 本身没 bug，但它依赖的 braks 回调频率太低，所以你看不到中间过程**。

---

## 7. 调参旋钮速查表（本篇涉及的）

| 旋钮 | 位置 | 作用 | 调大 | 调小 |
|---|---|---|---|---|
| `num_evals` | train.py:194 | eval + progress_fn 次数 | 日志更密、eval 开销更大 | 日志更稀 |
| `num_resets_per_eval` | train.py:213 | 每 eval 周期 reset 次数 | 每 epoch 训练步更多 | 每 epoch 训练步更少 |
| `log_training_metrics` | train.py:231 | 开训练 loss 上报 | 能看 loss 曲线 | 看不到训练 loss |
| `training_metrics_steps` | train.py:232 | 训练 loss 上报间隔 | 日志更稀 | 日志更密 |

---

## 8. Debug 决策树（本篇涉及的症状）

```
症状：训练启动后几分钟没有任何日志输出
├─ 可能原因 1：JIT 编译期（正常，1-2 分钟无输出）
│   └─ 验证：top/htop 看 GPU 利用率，或看进程 CPU 100%
│   └─ 解决：等。或开 absl logging 看 "starting iteration" 心跳
│
├─ 可能原因 2：progress_fn 频率太低（num_evals 太小）
│   └─ 验证：checkpoint 目录有没有 tmp 文件 → 有就是训练在跑
│   └─ 解决：调大 num_evals 或开 log_training_metrics
│
└─ 可能原因 3：真的卡在编译（AGENTS.md 提到的 PPO 编译 100+ 秒）
    └─ 验证：进程 CPU 100% 但 GPU 0%，且持续 5 分钟以上
    └─ 解决：见笔记 5 的编译慢 debug 章节
```

---

## 9. 本篇要点回顾

1. **PPO 训练是三层嵌套循环**：epoch → training_step（rollout+SGD）→ minibatch_step（loss+grad）
2. **`progress_fn` 整个训练只回调 `num_evals` 次**（默认 1 次，我们配置 5 次）—— 这是"看不见日志"的根因
3. **我们昨天训练没卡，是跑完一个 epoch（1.6M 步）才打一次日志**，编译期 + 第一个 epoch 加起来 2-3 分钟没输出被误判为卡住
4. **三种让自己能看见日志的方法**：调大 num_evals / 开 log_training_metrics / 开 absl logging
5. **checkpoint 文件名 = 步数**，看到 tmp 后缀的 checkpoint 说明训练在跑

---

## 下一篇

笔记 2：PPO loss 与超参 —— 进入 `losses.py`，逐项拆解 PPO loss（policy clip / value / entropy / 无 KL penalty），讲清每个超参调什么。

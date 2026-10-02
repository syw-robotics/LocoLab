# Multi Reward Group：最小更新计划

LocoLab 和 z_rl 目前都不支持多 reward group。最小更新只在 LocoLab 换掉 reward manager：各组单独计算、单独记日志，enabled 的组再加总成现有的 `(num_envs,)` 标量。Gym `step`、z_rl 的 storage、PPO 和 logger 保持不变。

多 critic（每组一条 value / advantage）不在这次范围内。

## 现状

LocoLab 的环境配置只有一个 `rewards` 字段，例如 `rewards: FlatRewardsCfg = FlatRewardsCfg()`。字段里是扁平的 `RewTerm`。

Isaac Lab 2.3 的 `RewardManager`：

- 只接受 `RewardTermCfg`。嵌套的 group configclass 会在 `_prepare_terms` 里 `TypeError`。
- 每个 term 乘 `weight * dt` 后加总。
- `compute(dt)` 返回 `(num_envs,)`。
- `reset()` 写入扁平日志 `Episode_Reward/<term>`。

`ManagerBasedRLEnv` 把调用点写死了：

- `step()`：`self.reward_buf = self.reward_manager.compute(dt=self.step_dt)`
- `_reset_idx()`：`self.reward_manager.reset(env_ids)`，结果合并进 `extras["log"]`

z_rl 同样假定一条标量：

- `VecEnv.step` 的 rewards 形状是 `(num_envs,)`。
- `ZRlVecEnvWrapper` 原样转发 Isaac Lab 的 `rew`。
- rollout 把 reward、value、return、advantage 存成 `(T, num_envs, 1)`，写入时用 `rewards.view(-1, 1)`。
- PPO 只有一个 critic。timeout bootstrap 把 value 收成一维再加到 reward 上。
- logger 按这个标量累计 episode return。

Active Adaptation 的 wrapper 认得 reward group，但 `_aggregate_rewards` 会立刻 `sum(dim=-1)`。分组在进入 PPO 之前就消失了。

## 目标

和 AA 的 reward group 对齐的是环境侧组织，不是多目标学习：

- 一组是若干现有 `RewTerm`，组内仍由 Isaac Lab `RewardManager` 计算。
- 组可以单独 `enabled`。
- 日志按 `Episode_Reward/<group>/<term>` 分开。
- 交给策略的 reward 仍是 enabled 组之和，形状 `(num_envs,)`。
- 现有 `FlatRewardsCfg` / `RoughRewardsCfg` 不改也能跑：字段本身是 `RewardTermCfg` 时退回单个 `RewardManager`。

## 改哪里

只加 LocoLab 里的一个薄 manager，并在环境 `load_managers` 里换上它。不复制 `step`，不改 Isaac Lab，不改 z_rl。

`step` 和 `_reset_idx` 已经只依赖 `reward_manager.compute(dt)` 和 `reward_manager.reset(env_ids)`。新对象实现这两个方法，以及 visualizer 用到的 `active_terms`、`get_active_iterable_terms`，现有调用点就可以继续用。

### `RewardGroupManager`

- 配置的每个字段是一组 rewards。组内仍然是 `RewTerm`。
- 每组内部直接建一个 Isaac Lab `RewardManager`。dt、weight、episode sum、class term 的 reset 都复用，不重写 term 计算。
- 组上只加 `enabled`。`compute(dt)` 对 enabled 的组求和，返回 `(num_envs,)`。
- `reset()` 把子 manager 的日志键改成 `Episode_Reward/<group>/<term>`。组 return 是这些 term 的和，不再维护第二份 buffer。
- `active_terms` 和 `get_active_iterable_terms` 给 term 名加上 group 前缀，live visualizer 才能继续用。
- 如果字段本身就是 `RewardTermCfg`，退回单个 `RewardManager`，行为与现在一致。

### 配置

现有任务保持：

```python
rewards: FlatRewardsCfg = FlatRewardsCfg()
```

需要分组的任务改成：

```python
@configclass
class RewardGroupCfg:
    enabled: bool = True
    terms: FlatRewardsCfg = FlatRewardsCfg()

@configclass
class RewardGroupsCfg:
    locomotion: RewardGroupCfg = RewardGroupCfg()
    # manipulation: RewardGroupCfg = RewardGroupCfg(terms=EeRewardsCfg())
```

环境里把：

```python
self.reward_manager = RewardManager(self.cfg.rewards, self)
```

换成：

```python
self.reward_manager = RewardGroupManager(self.cfg.rewards, self)
```

`setup_manager_visualizers` 继续把这个对象交给 visualizer。

`enabled` 不要塞进 Isaac Lab 的 `RewardManager` 配置。它会把未知字段当成 term。组开关放在外层 `RewardGroupCfg`。

## z_rl

不改 wrapper、storage、PPO、logger。分组结果走现有的 `extras["log"]`，logger 已经会记 `Episode_Reward/...`。

## 这次不做

下面这些才需要改 z_rl，不属于这次最小更新：

- 把 `(num_envs, num_groups)` 写进 rollout。
- 每组一个 critic、一条 advantage。
- timeout bootstrap、advantage normalization、reward normalization、logger 的 `[:, 0]` 按列处理。

那一步要动的点是：`RolloutStorage` 的最后一维、`add_transition` 的 `view(-1, 1)`、PPO 的 timeout 加法，以及 logger 里对 reward 的标量累计。

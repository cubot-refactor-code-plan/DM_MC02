# module_cfg

> 代码：`module_cfg.hpp` / `module_cfg.cpp`
> 作用：**模块层的特化点**。限位、速度、到位窗口由这里传入；几何参数因为只支持 ZDT 电机，
> 固定写在 `CorexyT` 的私有 `constexpr` 里。

## 1. 实例绑定

| 项 | 本工程取值 | 说明 |
| --- | --- | --- |
| 电机组 | `emm_group`（`device_cfg.cpp`） | 一路 RS485 挂多台从站，本模块用其中两台 |
| 电机地址 | `1`、`2` | 与驱动器面板设置一致 |
| 理论限位 | `(0,0) ~ (190,200)` | 机械真实行程，运动中越界即判碰撞 |
| 安全限位 | `(1,1) ~ (188,198)` | 允许作为目标的范围；x1/y1 距理论边 2 mm，留作减速余量 |
| 默认速度 | `300 RPM` / `acc 230` | 实机调试确定 |
| 到位窗口 | `0.20 mm` | 与轮询周期、减速过程匹配 |

```cpp
CorexyT<decltype(emm_group)> corexy({
  .group      = &emm_group,
  .addr1      = 1U,
  .addr2      = 2U,
  .theory     = {0.0f, 0.0f, 190.0f, 200.0f},   // 理论限位
  .safe       = {1.0f, 1.0f, 188.0f, 198.0f},   // 安全限位
  .rpm        = 300U,
  .acc        = 230U,
  .arrive_tol = 0.20f,                          // 到位窗口
});
```

## 2. 几何参数（类内常量）

| 量 | 值 | 来源 |
| --- | --- | --- |
| 一圈行程 | 40 mm | 2GT 节距 2 mm × 20 齿 |
| 指令脉冲 | 3200 脉冲/圈（80 脉冲/mm） | 驱动器 16 细分 |
| 反馈 counts | 65536 counts/圈（1638.4 counts/mm） | 编码器分辨率 |
| 换算比 | 1 脉冲 = 20.48 counts | 65536 / 3200 |

这三个值写在 `corexy.hpp` 私有区的 `MM_PER_REV` / `PULSES_PER_REV` / `COUNTS_PER_REV`，
两个比例 `PULSES_PER_MM` / `COUNTS_PER_MM` / `COUNTS_PER_PULSE` 在编译期算出。
换非 ZDT 驱动器或改细分时要同步改这三个常量：

- `PULSES_PER_REV` 偏了 → 走错距离（按比例缩放）；
- `COUNTS_PER_REV` 偏了 → 到位判定永远不满足，表现为一直停在 `Moving`、`pos` 与目标总差一截。

## 3. 对外别名

```cpp
using CorexyModule = CorexyT<decltype(emm_group)>;
extern CorexyModule corexy;
```

模块层其余使用者（`corexy_protocol.hpp`、`corexy_io_task.cpp`、`api_main.cpp`）只引用
`module_cfg.hpp` 和 `CorexyModule`，不直接依赖具体串口类型。

## 4. 初始化

```cpp
void module_init(void)      // api_main.cpp 的 all_init() 里调用
{
  configASSERT(corexy.init() == Status::OK);   // 只校验配置，不发命令
}
```

清零 / 使能 / 原点复核都在 `corexy_io_task` 启动阶段完成（调度器起来之后），
原因见 `Docs/plan/CoreXY模块层方案.md` §14。

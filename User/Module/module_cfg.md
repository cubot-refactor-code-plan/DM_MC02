# module_cfg

> 代码：`module_cfg.hpp` / `module_cfg.cpp`
> 作用：**模块层的特化点**。`corexy.*` 本身不写死任何项目数值，全部由这里传入。

## 1. 实例绑定

| 项 | 本工程取值 | 说明 |
| --- | --- | --- |
| 电机组 | `emm_group`（`device_cfg.cpp`） | 一路 RS485 挂多台从站，本模块用其中两台 |
| 电机地址 | `1`、`2` | 与驱动器面板设置一致 |
| 理论限位 | `(0,0) ~ (190,200)` | 机械真实行程，运动中越界即判碰撞 |
| 安全限位 | `(1,1) ~ (189,199)` | 允许作为目标的范围，四周各内收 1 mm 作减速余量 |
| 默认速度 | `300 RPM` / `acc 230` | 实机调试确定 |
| 到位窗口 | `0.20 mm` | 与轮询周期、减速过程匹配 |

```cpp
CorexyT<decltype(emm_group)> corexy({&emm_group, 1U, 2U,
                                     {0.0f, 0.0f, 190.0f, 200.0f},   // 理论限位
                                     {1.0f, 1.0f, 189.0f, 199.0f},   // 安全限位
                                     40.0f, 3200U, 65536U});         // 几何
```

## 2. 几何参数来历

| 量 | 值 | 来源 |
| --- | --- | --- |
| 一圈行程 | 40 mm | 2GT 节距 2 mm × 20 齿 |
| 指令脉冲 | 3200 脉冲/圈（80 脉冲/mm） | 驱动器 16 细分 |
| 反馈 counts | 65536 counts/圈（1638.4 counts/mm） | 编码器分辨率 |
| 换算比 | 1 脉冲 = 20.48 counts | 65536 / 3200 |

三者必须与驱动器实际设置一致：

- `pulses_per_rev` 偏了 → 走错距离（按比例缩放）；
- `counts_per_rev` 偏了 → 到位判定永远不满足，表现为 `FAULT TIMEOUT`。

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

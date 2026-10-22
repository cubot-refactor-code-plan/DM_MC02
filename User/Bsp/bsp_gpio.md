# bsp_gpio

> 代码：`bsp_gpio.hpp`（仅头文件）　实例：`bsp_cfg.cpp`

## 0. 如何快速完整的使用（不需要了解实际实现）

| 步骤 | 文件 | 写什么 |
| --- | --- | --- |
| 1. 头文件 | 使用处 | `#include "bsp_gpio.hpp"`（或 `bsp_cfg.hpp`） |
| 2. 全局实例 | `bsp_cfg.cpp` | `BspGpio xxx({GPIOx, GPIO_PIN_y});` |
| 3. 声明 | `bsp_cfg.hpp` | `extern BspGpio xxx;` |
| 4. 初始化 | `Core/Src/main.c` | 无需驱动初始化；方向/上下拉由 `MX_GPIO_Init()` 配 |

无模板实例化、无中断、无 `init()`。

**调用示例**

```cpp
#include "bsp_cfg.hpp"

power_24v_1.set();                        // 高电平
power_24v_1.reset();                      // 低电平
btb_pa5.toggle();                         // 翻转
const bool level = gyro_acc_cs.get_level();
```

构造绑定 `{port, pin}`；方向与上下拉由 `MX_GPIO_Init()` 配置，本类不做初始化。

接口：`set()` / `reset()` / `toggle()` / `set_level(bool)` / `get_level()`。

现有实例与引脚见 `bsp_cfg.cpp` 与 `Docs/hardware/引脚分配与冲突.md`。

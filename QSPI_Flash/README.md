# DM-MC-Board02 外置 Flash 驱动

本目录是当前 `STM32H723VG + W25Q64JV` 工程的外置 NOR Flash 实现。目录名沿用
`QSPI_Flash`，底层实际使用 STM32H723 的 **OCTOSPI2 Port 1 Quad**。

正式运行方案不是“只用一种模式”：初始化、擦除和编程使用间接模式；USB 等低时延敏感度
代码的稳态执行使用 Memory-Mapped/XIP。进入映射前由间接模式完成器件初始化，擦写前必须
退出映射模式。

## 当前硬件配置

| 项目 | 当前工程配置 |
| --- | --- |
| MCU | STM32H723VGTx |
| 外置 Flash | Winbond W25Q64JV，JEDEC ID `EF 40 17` |
| 容量 | 8 MiB，内部偏移 `0x000000~0x7FFFFF` |
| 映射窗口 | `0x70000000~0x707FFFFF` |
| 页/最小擦除 | 256 B / 4 KiB |
| 地址阶段 | 24 位 |
| OCTOSPI 时钟 | D1HCLK 275 MHz，Prescaler=3，约 91.7 MHz |
| Memory-Mapped 读 | `0x6B` Quad Output Fast Read，8 dummy cycles |
| Memory-Mapped 写配置 | `0x32` Quad Page Program |

引脚与工程 `.ioc` 一致：

| 信号 | 引脚 | AF |
| --- | --- | --- |
| IO0 | PD11 | AF9 OCTOSPIM_P1 |
| IO1 | PB0 | AF4 OCTOSPIM_P1 |
| IO2 | PA3 | AF6 OCTOSPIM_P1 |
| IO3 | PA1 | AF9 OCTOSPIM_P1 |
| CLK | PB2 | AF9 OCTOSPIM_P1 |
| NCS | PE11 | AF11 OCTOSPIM_P1 |

CubeMX 在 `Core/Src/octospi.c` 中创建 `hospi2`。本驱动只引用该句柄，不修改
`Core/` 生成代码。

## 文件职责

| 文件 | 作用 |
| --- | --- |
| `flash_port_stm32h7.*` | `HAL_OSPI` 平台适配、MPU 保护及映射切换 |
| `flash_device.*` | W25Q64JV 指令、JEDEC 校验、状态寄存器和超时 |
| `external_flash.*` | 应用公共 API，包含跨页拆分和边界检查 |
| `external_flash_test.*` | 最后两个扇区的破坏性数据测试 |
| `external_flash_xip.*` | 将测试函数写入外置 Flash 并实际取指执行 |
| `ExternalLoader/` | 面向烧录工具的 STM32H723/W25Q64JV Loader 源码 |

## 公共 API

间接读写示例：

```c
ext_flash_info_t info;
uint8_t data[32] = {0};

if (ext_flash_init(&info) == EXT_FLASH_OK) {
    ext_flash_erase_sector(0U);
    ext_flash_write(0U, data, sizeof(data));
    ext_flash_read(0U, data, sizeof(data));
}
```

`ext_flash_write()` 会按 256 B 页边界自动拆分，但不会自动擦除。所有公共 API 的地址都是
器件内部偏移，不是 `0x70000000` 起的绝对地址。

映射数据访问或 XIP 必须严格配对：

```c
if (ext_flash_memory_mapped_enable() == EXT_FLASH_OK &&
    ext_flash_memory_mapped_access_enable(0) == EXT_FLASH_OK) {
    const volatile uint8_t *p =
        (const volatile uint8_t *)(FLASH_DEVICE_MAPPED_BASE + flash_offset);
    uint8_t value = *p;
    (void)value;

    ext_flash_memory_mapped_access_disable();
    ext_flash_memory_mapped_disable();
}
```

传给 `ext_flash_memory_mapped_access_enable()` 的参数为非零时，MPU 允许从该区域取指；仅
读取数据时应传 0。映射期间不要调用间接读、擦除或编程接口。

当前 CubeMX 配置没有为 `hospi2` 绑定 MDMA 句柄，因此 `ext_flash_read_mdma()` 和
`ext_flash_write_mdma()` 会安全回退到轮询传输。以后启用 MDMA 后，平台层会启用
`OCTOSPI2_IRQn`，调用者还需正确处理 D-Cache 一致性。

## MPU 和 Cache

`flash_port_prepare()` 建立 256 MiB、禁止访问且禁止执行的 MPU Region 6，防止映射尚未
建立时 Cortex-M7 的推测访问触发总线故障。进入 Memory-Mapped 后，Region 7 以更高优先级
开放实际 8 MiB 窗口，并按调用参数决定是否允许执行。Region 6 和 7 因此由本驱动保留。

映射区域配置为可缓存、只读。XIP 测试在调用外置函数前失效 I-Cache；若应用在映射期间
自行修改同一区域，需要先退出映射，完成编程，再做相应 Cache 维护后重新进入映射。

## 工程集成和 XIP 链接

`User/CMakeLists.txt` 已将驱动及测试源文件加入 `User_lib`，并公开本目录头文件路径。
`STM32H723xG_flash.ld` 当前声明：

```ld
OSPI2 (rx) : ORIGIN = 0x70000000, LENGTH = 8M
```

任务 5 的 `.qspi_text` 是自编程测试布局：运行地址为 `0x70100000`，但通过 `AT>FLASH`
额外在内部 Flash 保留一份加载镜像，测试任务再把它写到 W25Q64JV。这只能证明 XIP 链路，
**不会节省内部 Flash**。

正式 USB 迁移使用独立的 `.usb_xip` 段，运行和存储地址均为 `0x70110000`，没有内部
Flash LMA。构建后生成内部镜像 `DM_MC02_internal.hex` 和外置镜像
`DM_MC02_usb_xip.bin`，由 `Flash/OpenOCD_flash.sh` 依次烧录。USB 初始化会先完成
`ext_flash_init()`、Memory-Mapped 和可执行 MPU 配置，再校验当前 CDC/HID 类别对应的载荷
签名；签名不匹配时不会执行外置代码，而是切换成 OpenOCD 可更新的单线映射。

USB 类通过 CMake 缓存变量选择，TinyUSB 编译期只保留其中一个类驱动：

当前工程默认选择 HID；只有明确需要虚拟串口兼容时才切换为 CDC。

```bash
cmake -S . -B build/Debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DDM_MC02_USB_CLASS=CDC

cmake -S . -B build/HID-Debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DDM_MC02_USB_CLASS=HID
```

链接脚本把内部 `FLASH` 区域硬限制为 128 KiB，超过时构建会直接失败。当前 Debug 实测 CDC
内部镜像占 99,528 B、外置段 284 B；HID 内部镜像占 98,608 B、外置段 308 B。

## 测试

实机测试代码位于 `User/App/test/qspi_flash/qspi_flash_test.cpp`，由
`APP_TEST_QSPI_FLASH_ENABLED` 控制。测试覆盖：

- JEDEC ID 和三个状态寄存器读取；
- 擦除校验、固定/递增/全零/全 FF/伪随机数据；
- 非对齐长度、跨页和跨扇区写入；
- `0x70000000` 映射窗口逐字节读取；
- 从 `0x70100000` 实际执行 40 B 的 `.qspi_text` 测试函数。

警告：测试会覆盖外置 Flash 最后两个扇区 `0x007FE000~0x007FFFFF`，并擦写
`0x00100000` 附近的 XIP 测试扇区。完成后应把宏设回 0，避免每次启动重复擦写。

本板实测读出的 JEDEC ID 为 `EF 40 17`，7 个间接读写用例、映射读取和 XIP 均通过。

## External Loader

`ExternalLoader/` 已按当前板卡移植：24 MHz HSE、550 MHz SYSCLK、275 MHz HCLK、
OCTOSPI2 引脚、8 MiB 容量和 `0x70000000` 基址都与主工程一致。它导出 `Init`、`Read`、
`Write`、`SectorErase`、`MassErase` 和 `Verify`。

Loader 使用独立链接脚本 `ExternalLoader/external_loader.ld`，全部装入 128 KiB DTCM RAM，
不依赖应用启动代码。当前 Linux 正式烧录流程使用 OpenOCD 的 `stmqspi` bank 和目标固件完成
OCTOSPI2 初始化，不依赖 `.stldr`；该 Loader 保留给 STM32CubeProgrammer 等工具集成。

## 使用约束

- 不要在未确认数据可丢失时调用 `ext_flash_erase_chip()`。
- Page Program 前必须擦除目标扇区，NOR Flash 编程只能将位从 1 改为 0。
- 擦写期间必须退出 Memory-Mapped，执行映射期间不得擦写。
- 修改 CubeMX 的 OCTOSPI2 时钟、引脚、DeviceSize 或 MPU 分配后，要同步检查本目录。
- W25Q64JV 使用 24 位地址，不要重新引入原 W25Q256 移植版的 32 位地址命令。

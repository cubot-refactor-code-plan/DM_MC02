# 烧录脚本

两个 Linux 脚本均可从任意目录调用，无需先切入工程根目录。

| 探针 | Linux | Windows | 连接配置 |
| --- | --- | --- | --- |
| CMSIS-DAP | `OpenOCD_flash.sh` | `OpenOCD_flash.bat` | `daplink.cfg` |
| SEGGER J-Link | `JLink_flash.sh` | `JLink_flash.bat` | `jlink.cfg`（OpenOCD 的 `jlink` 适配器） |

- `OpenOCD_flash.sh`：定位 `build/Debug/<工程名>.elf`（找不到时兵底搜索 `build/` 下任意 `.elf`），
  经 `daplink.cfg` 执行 `program <elf> verify reset exit`。
- `JLink_flash.sh`：参数 `[BUILD_TYPE]`（默认 `Debug`），经 OpenOCD 的 `jlink` 适配器（libjaylink）
  驱动 J-Link，以 SWD 4 MHz 下载 `build/<BUILD_TYPE>/<工程名>.elf` 并复位运行。
- 两个脚本都依赖 `openocd`；J-Link 版本还要求 OpenOCD 带 `jlink` 适配器（可用
  `openocd -c "adapter list" -c "shutdown"` 确认）。`.sh` 文件需要可执行权限。
- 两个脚本都用 `program … verify reset exit` 一次完成下载、校验与复位，并禁用 OpenOCD 的
  GDB / Telnet / TCL 服务端口（烧录不需要，避免占用 3333 / 4444 / 6666）。

## 用法

```bash
bash Flash/OpenOCD_flash.sh
bash Flash/JLink_flash.sh          # 默认 build/Debug
```

Windows 对应命令：

```bat
Flash\OpenOCD_flash.bat
Flash\JLink_flash.bat Debug
```

## 本目录其他文件

| 文件 | 用途 |
| --- | --- |
| `daplink.cfg` | OpenOCD 的 DAPLink / CMSIS-DAP 配置 |
| `jlink.cfg` | OpenOCD 的 SEGGER J-Link 配置（`adapter driver jlink` + SWD 4 MHz） |
| `STM32H723.svd` | STM32H723 外设寄存器描述，供 Cortex-Debug 查看外设寄存器 |
| `linux.jdebug`、`linux.jdebug.user` | Ozone 工程示例；内含创建者机器的绝对路径、探针序列号与 Ozone 版本信息，换机器后必须在 Ozone 中重新选择 ELF 与探针 |

相关配置：`.vscode/tasks.json`（烧录任务 `OpenOCD_flash_linux` / `JLink_flash_linux`）、
`.vscode/launch.json`（调试配置）、`Docs/guide/开发环境与烧录调试.md`。

# 烧录脚本

四个脚本均支持当前工程的内部 Flash + W25Q64JV 双镜像烧录：

| 探针 | Linux | Windows | OpenOCD 配置 |
| --- | --- | --- | --- |
| CMSIS-DAP | `OpenOCD_flash.sh` | `OpenOCD_flash.bat` | `daplink.cfg` |
| SEGGER J-Link | `JLink_flash.sh` | `JLink_flash.bat` | `jlink.cfg` |

所有脚本都依赖 `openocd`。J-Link 版本通过 OpenOCD 的 `jlink`/libjaylink 适配器使用物理
J-Link 探针，不调用 J-Link Commander；可用 `openocd -c "adapter list" -c "shutdown"`
确认当前 OpenOCD 是否包含 `jlink` 适配器。`.sh` 文件需要具有可执行权限。

## 烧录当前工程

当前固件由两个独立产物组成：

- `DM_MC02_internal.hex`：STM32 内部 Flash 镜像；
- `DM_MC02_usb_xip.bin`：写入 W25Q64JV `0x70110000` 的 USB XIP 镜像。

构建完成后必须使用双镜像脚本，不能只下载 ELF。各脚本默认使用 `build/Debug`：

```bash
bash Flash/OpenOCD_flash.sh Debug
bash Flash/JLink_flash.sh Debug
```

Windows 对应命令为：

```bat
Flash\OpenOCD_flash.bat Debug
Flash\JLink_flash.bat Debug
```

可选参数是 `build/` 下的目录名。例如 HID 验证构建位于 `build/HID-Debug` 时使用：

```bash
bash Flash/OpenOCD_flash.sh HID-Debug
```

脚本先下载并校验内部镜像，再通过一次性 D3 SRAM 邮箱让固件把 OCTOSPI2 切换到 OpenOCD
可识别的单线映射，随后擦写、校验外置镜像并复位运行。CDC/HID 跨模式更新由类别相关的
XIP 签名保护；旧类别载荷不会被新内部固件执行。

Linux 若把 HID 节点创建成 `0600 root:root`，可将 `Flash/99-dm-mc02-usb.rules` 安装到
`/etc/udev/rules.d/` 后重载规则并重新插拔设备。规则仅匹配本工程的 `cafe:4001` CDC 和
`cafe:4002` HID，不会影响独立的 CMSIS-DAP 虚拟串口。

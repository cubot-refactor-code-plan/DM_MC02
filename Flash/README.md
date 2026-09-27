这里面为烧录脚本

个人测试可用

- `.bat`为win下面的烧录脚本
- `.sh` 为linux下面的烧录脚本
  - .sh文件 需要提前`chmod +x` 但是只需要一次就可以一直授权
- `.cfg`为openocd用到的配置内容
- 带ozone的为ozone的相关配置内容，应该每个人都不同

需要自行安装+配置好openocd jlink的环境变量

对应 VS Code 任务按钮（配置在 `.vscode/tasks.json`）：

| 按钮 | 任务名 | 实际执行 |
| --- | --- | --- |
| `[🔨 编译 F7]` | `Build` | `cmake --build --preset Debug` |
| `[🧹 重新编译]` | `Clean_Rebuild` | 删 `build/Debug` 后重新配置 + 编译 |
| `[🗑️ 清理]` | `Clean` | 删除 `build/Debug` |
| `[⚡ DAP烧录Linux]` | `OpenOCD_flash_linux` | `Flash/OpenOCD_flash.sh` |
| `[🔌 JLink烧录Linux]` | `JLink_flash_linux` | `Flash/JLink_flash.sh` |
| `[⚡ DAP烧录Win]` | `OpenOCD_flash_win` | `Flash/OpenOCD_flash.bat` |
| `[🔌 JLink烧录Win]` | `JLink_flash_win` | `Flash/JLink_flash.bat` |

> `.sh` 的可执行权限已设置；重新 clone 或换机器后需再执行一次 `chmod +x Flash/*.sh`。


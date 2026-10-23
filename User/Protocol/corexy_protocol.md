# corexy_protocol

> 代码：`corexy_protocol.hpp` / `corexy_protocol.cpp`
> 上位机 ↔ CoreXY 模块的二进制串口协议：帧格式、命令表、应答/事件，以及可直接跑的测试案例。

> **当前默认关闭**：`app_task.hpp` 里 `APP_COREXY_PROTOCOL = 0`，USART1 交给命令码终端
> `corexy_console_task`（手工测试用，命令表见 `User/App/task/corexy_console_task.cpp` 文件头）。
> 改成 1 就切到本协议，两者互斥、共用同一路串口。

## 1. 物理层

| 项 | 值 |
| --- | --- |
| 外设 | USART1（代码里是 `bsp_uart1`） |
| 引脚 | PA9 = TX，PA10 = RX |
| 参数 | 115200 / 8N1 / 无硬件流控 |
| 接线 | USB-TTL 的 RX → PA9，TX → PA10，GND 共地（3.3 V） |

> 这一路上还会周期性地出现 ASCII 心跳（`[sys] up=...`），解析器必须能靠 `A5 5A` 重同步。

## 2. 帧格式

```
A5 5A | CMD | LEN | PAYLOAD[LEN] | SUM
SUM = CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[LEN-1]
```

- 多字节**大端**；
- 坐标单位 **0.01 mm**（i16，±327.67 mm）。190 mm = `0x4A38`；
- 请求帧长 = 5 + LEN；`LEN > 24` 或校验错的帧整帧丢弃；
- 帧头错位会自动重同步，不需要额外同步字节。

## 3. 命令表

| CMD | 名称 | 请求负载 | 说明 |
| --- | --- | --- | --- |
| `0x01` | 使能/失能 | `on:u8` | 两台一起，1 = 使能 |
| `0x02` | 绝对位置 | `x:i16 y:i16` | 世界坐标，受安全限位校验 |
| `0x03` | 相对位置 | `dx:i16 dy:i16` | 世界坐标增量 |
| `0x04` | 归零 | — | 当前位置记为 `(0,0)`，需后续读数复核 |
| `0x05` | STOP | — | 立即停止并清故障 |
| `0x06` | 速度/加速度 | `rpm:u16 acc:u8` | 运行期生效 |
| `0x07` | 快速位置参数 | `addr:u8 rpm:u16 acc:u8 mode:u8` | `mode` 0 = 相对 1 = 绝对 |
| `0x08` | 快速位置 | `addr:u8 pulses:i32` | 相对/绝对由 `0x07` 决定 |
| `0x09` | 查询限位 | — | 返回理论 + 安全两块矩形 |
| `0x0A` | 分段下发模式 | `mode:u8` | 0 = 位置帧(`0xFD`)，1 = 快速位置(`0xF1`+`0xFC`) |

## 4. 应答与事件

### 4.1 通用应答（每条命令都有）

```
A5 5A | CMD|0x80 | 05 | status | x:i16 | y:i16 | SUM
```

共 10 字节。`status` 就是 `Status` 枚举值：

| 值 | 含义 |
| --- | --- |
| 0 | OK |
| 1 | BUSY（分段进行中，不接受改模式） |
| 2 | TIMEOUT |
| 3 | FULL |
| 4 | IO_ERROR |
| 5 | BAD_ARG（越界 / 处于 Fault 未 STOP） |
| 6 | NOT_INIT（原点未复核 / 设备未初始化） |
| 7 | NOT_SUPPORTED |

### 4.2 限位应答（`0x09` 专用）

```
A5 5A | 89 | 11 | status | 理论 x0,y0,x1,y1（4×i16） | 安全 x0,y0,x1,y1（4×i16） | SUM
```

共 22 字节。

### 4.3 事件（MCU 主动上报）

```
A5 5A | 7F | 06 | evt | status | x:i16 | y:i16 | SUM
```

共 11 字节。

| evt | 含义 | status |
| --- | --- | --- |
| `0x01` | 上电自检完成 | 0 = 原点已复核；6 = 未复核（拒绝运动） |
| `0x02` | 一段移动完成 | 0 |
| `0x03` | 进入 Fault | 具体原因（2/4/5） |

> 移动命令的应答只表示"已开始走"，**走完要看 `EVT_DONE`**。

## 5. 测试案例

前置：烧录后上电，串口打开 115200 8N1，用 **HEX 发送**。

| # | 用例 | 发送（hex） | 期望 | 判据 |
| --- | --- | --- | --- | --- |
| T1 | 上电事件 | *（MCU 主动）* | `A5 5A 7F 06 01 00 00 00 00 00 78` | status = 00 才能运动；06 说明原点未复核 |
| T2 | 查询限位 | `A5 5A 09 00 09` | `A5 5A 89 11 00 00 00 00 00 4A 38 4E 20 00 64 00 64 49 D4 4D BC E8` | 理论 (0,0,190,200)、安全 (1,1,189,199) mm |
| T3 | 使能 | `A5 5A 01 01 01 01` | `A5 5A 81 05 00 xx xx yy yy SUM` | status = 00 |
| T4 | 设速度 | `A5 5A 06 03 01 2C E6 CE` | `A5 5A 86 05 00 ...` | status = 00（300 rpm / acc 230） |
| T5 | 归零 | `A5 5A 04 00 04` | `A5 5A 84 05 00 ...` | status = 00；2 拍后坐标 ≈ (0,0) |
| T6 | 相对 +10 mm X | `A5 5A 03 04 03 E8 00 00 EC` | 立即回 `A5 5A 83 05 00 ...`；随后事件 `A5 5A 7F 06 02 00 03 E8 00 00 90` | 先 status = 00，再收到 evt = 02 且 x ≈ 10.00 |
| T7 | 超安全限位 | `A5 5A 02 04 4E 20 00 00 68` | `A5 5A 82 05 05 ...` | status = **05 BAD_ARG**（目标 200 mm 超出 189） |
| T8 | STOP | `A5 5A 05 00 05` | `A5 5A 85 05 00 ...` | status = 00，Fault 被清 |
| T9 | 切快速位置模式 | `A5 5A 0A 01 01 0A` | `A5 5A 8A 05 00 ...` | status = 00 |
| T10 | 快速模式下移动 | 同 T6 | 同 T6 | 验证 `0xFC` 的**符号**：负增量必须真的是反向 |
| T11 | 坏校验 | 把 T2 最后一字节改掉 | **无应答** | 整帧被丢弃，串口安静 |
| T12 | 未知命令 | `A5 5A 55 00 55` | **无应答** | 未知 CMD 静默丢弃 |

补充约定：

- 命令之间建议间隔 ≥ 20 ms（固件里使能帧之间要等 20 ms、发位置前有 20 ms 静默）；
- 应答里的 `x / y` 是**当前实测坐标**，T3/T4 这类用例只需校验 `status`；
- T6 的完整流程：发命令 → 等到 `status=00` → 等 `evt=02`（最长 5 s）→ 读坐标。

## 6. 发送测试脚本

把下面存成 `cxy_test.py`，`pip install pyserial` 后运行：

```bash
python3 cxy_test.py /dev/ttyUSB0
```

```python
#!/usr/bin/env python3
"""CoreXY 协议发送测试：逐条下发并校验应答。"""
import sys, time, serial

HEAD = b'\xA5\x5A'


def frame(cmd, payload=b''):
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return HEAD + bytes([cmd, len(payload)]) + payload + bytes([chk])


def i16(v):
    v &= 0xFFFF
    return bytes([v >> 8, v & 0xFF])


def i16v(b):
    v = (b[0] << 8) | b[1]
    return v - 0x10000 if v & 0x8000 else v


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.buf = b''

    def pop(self):
        i = self.buf.find(HEAD)
        if i < 0:
            self.buf = self.buf[-1:]
            return None
        self.buf = self.buf[i:]
        if len(self.buf) < 4 or len(self.buf) < self.buf[3] + 5:
            return None
        n = self.buf[3]
        f, self.buf = self.buf[:n + 5], self.buf[n + 5:]
        chk = f[2] ^ f[3]
        for b in f[4:-1]:
            chk ^= b
        return f if chk == f[-1] else None

    def pump(self):
        """读一帧，返回 (cmd, payload) 或 None。"""
        self.buf += self.s.read(64)
        f = self.pop()
        return None if f is None else (f[2], f[4:-1])

    def cmd(self, code, payload=b'', timeout=2.0):
        """发一条命令，返回 (status, x_mm, y_mm) 或 None。"""
        self.s.write(frame(code, payload))
        end = time.time() + timeout
        while time.time() < end:
            r = self.pump()
            if r is None:
                continue
            c, p = r
            if c == 0x7F:                                  # 事件：打印后继续等应答
                print(f'    <event evt={p[0]:#04x} status={p[1]}>')
                continue
            if c == (code | 0x80):
                return p[0], i16v(p[1:3]) / 100.0, i16v(p[3:5]) / 100.0
        return None

    def wait_event(self, evt, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            r = self.pump()
            if r is None:
                continue
            c, p = r
            if c == 0x7F and p[0] == evt:
                return p[1], i16v(p[2:4]) / 100.0, i16v(p[4:6]) / 100.0
        return None


PASS = FAIL = 0


def check(name, ok, detail=''):
    global PASS, FAIL
    PASS, FAIL = PASS + ok, FAIL + (not ok)
    print(f'[{"PASS" if ok else "FAIL"}] {name}  {detail}')


def main():
    lk = Link(sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0')
    time.sleep(0.3)

    # T2 查询限位
    lk.s.write(frame(0x09))
    end, lim = time.time() + 2, None
    while time.time() < end and lim is None:
        r = lk.pump()
        if r and r[0] == 0x89:
            lim = r[1]
    if lim:
        th = [i16v(lim[1 + 2 * i:3 + 2 * i]) / 100 for i in range(4)]
        sf = [i16v(lim[9 + 2 * i:11 + 2 * i]) / 100 for i in range(4)]
        print(f'    理论 {th}  安全 {sf}')
        check('T2 限位', th == [0, 0, 190, 200] and sf == [1, 1, 189, 199])
    else:
        check('T2 限位', False, '无应答')

    # T3 使能 / T4 速度 / T5 归零
    r = lk.cmd(0x01, bytes([1]))
    check('T3 使能', r is not None and r[0] == 0, f'{r}')

    r = lk.cmd(0x06, bytes([0x01, 0x2C, 0xE6]))
    check('T4 速度', r is not None and r[0] == 0, f'{r}')

    r = lk.cmd(0x04)
    check('T5 归零', r is not None and r[0] == 0, f'{r}')
    time.sleep(0.3)

    # T7 越界必须被拒
    r = lk.cmd(0x02, i16(20000) + i16(0))
    check('T7 越界拒绝', r is not None and r[0] == 5, f'{r}')

    # T6 相对 +10 mm，等 EVT_DONE
    r = lk.cmd(0x03, i16(1000) + i16(0))
    check('T6 下达', r is not None and r[0] == 0, f'{r}')
    ev = lk.wait_event(0x02, 6.0)
    check('T6 到位', ev is not None and abs(ev[1] - 10.0) < 0.5, f'{ev}')

    # T8 STOP
    r = lk.cmd(0x05)
    check('T8 STOP', r is not None and r[0] == 0, f'{r}')

    # T11 / T12：坏帧与未知命令都应当没有应答
    lk.s.write(b'\xA5\x5A\x09\x00\x00')          # 校验错
    lk.s.write(frame(0x55))                       # 未知命令
    time.sleep(0.5)
    got = lk.pump()
    check('T11/T12 静默丢弃', got is None, f'{got}')

    print(f'\n结果：{PASS} 通过 / {FAIL} 失败')
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
```

## 7. 排错

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 完全没反应 | 接线反了 / 波特率不对 / 没共地 | 先确认能否收到 BOOT 事件帧 |
| 有回帧但 `status=6` | 原点未复核（清零后读数没回到 0 附近） | 复位重新上电；确认上电时机构静止在限位内 |
| `status=5` 且目标是合法坐标 | 处于 Fault 未清 | 先发 `0x05` STOP |
| 移动命令回 0 但不动 | 电机未使能 / 驱动掉线 | 发 `0x01 01`；查驱动电源与总线接线 |
| 偶发解析失败 | 串口里混了 ASCII 心跳 | 解析器按 `A5 5A` 重同步（脚本已处理） |
| 一次连发多条只应了部分 | 命令队列深度 8，满了会静默丢 | 一条一条发，等到应答再发下一条 |

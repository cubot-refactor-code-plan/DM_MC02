# motor_definition

> 代码：`motor_definition.hpp`（仅头文件）

只存数据：不含协议、不含硬件、不含函数。**结构已定稿，不要改。**

```cpp
struct MotorData
{
  struct Radian {
    float  angle_single_round;  // 单圈机械角 (rad)
    double angle_multi_round;   // 累计多圈机械角 (rad)
    float  velocity;            // 角速度 (rad/s)
    float  acceleration;        // 角加速度 (rad/s^2)
  } radian_data;

  struct Param {
    float offset;  // 设备轴机械零位 (rad)
    float ratio;   // 设备轴转速 / 输出轴转速，必须 > 0
  } param;
};
```

- 约定：`radian_data` 一律表示**最终机构输出轴**；设备先把协议原始值换算到设备轴，再按
  `(设备轴角度 − offset) / ratio` 换算到输出轴。
- 角加速度是**跨帧量**：`DjiMotorGroupT` 不填（恒 0），需要时由上层自己差分 + 滤波。
- `LuenbergerMotorData` 是可选观测器输出，目前无引用。

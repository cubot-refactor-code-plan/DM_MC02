/**
 * @file bsp_usb.hpp
 * @author ChoseB (ChoseB@cumt.edu.cn)
 * @brief TinyUSB 的 BSP 封装接口。
 * @version 0.1
 * @date 2026-08-13
 *
 * @copyright Copyright (c) 2026
 *
 * @details
 *
 * 该文件将 USB 设备抽象为一个类对象，向上层提供：
 * - 初始化与周期任务驱动
 * - 构建期互斥的 CDC 或 Generic HID 收发接口
 * - 连接状态查询
 * - 当前传输类接收回调注册
 */

#ifndef __BSP_USB_HPP__
#define __BSP_USB_HPP__

#include <stdbool.h>
#include <stdint.h>


/**
 * @class BspUsb
 * @brief USB BSP 单例类，负责 TinyUSB CDC/HID 的初始化与数据收发。
 */
class BspUsb
{
public:
  /** @brief 当前固件唯一编译进来的 USB 设备类。 */
  enum class DeviceClass : uint8_t
  {
    CDC,
    HID
  };

  static constexpr uint32_t HID_REPORT_SIZE = 64U;

  /**
   * @brief 当前 USB 传输类的接收回调函数类型。
   * @param data 本次接收的数据指针。
   * @param len 本次接收的数据长度（字节）。
   * @param user_ctx 用户上下文指针，由 set_rx_callback 传入。
   */
  typedef void (*RxCallback)(const uint8_t* data, uint32_t len, void* user_ctx);

  /**
   * @brief 获取 USB BSP 单例对象。
   * @return BspUsb& 单例引用。
   */
  static BspUsb& instance();

  /**
   * @brief 初始化 TinyUSB Device 栈。
   *
   * @note 建议在 RTOS 启动后、USB 任务开始前调用一次。
   */
  void init();

  /**
   * @brief USB 周期任务函数。
   *
   * @details
   * 该函数应在独立任务中高频调用（例如 1ms 周期），用于：
   * - 驱动 TinyUSB 状态机
   * - 轮询当前 CDC/HID 类的接收数据
   * - 将接收数据分发给已注册回调
   */
  void task();

  /** @brief 返回当前构建选择的唯一 USB 设备类。 */
  DeviceClass active_class() const;

  /** @brief 查询外置 XIP 和 TinyUSB 是否均初始化成功。 */
  bool initialized() const;

  /**
   * @brief 查询当前构建选择的 USB 类是否就绪可通信。
   * @return true 已挂载且满足发送门控条件；false 否则。
   */
  bool is_ready() const;

  /**
   * @brief 设置发送前是否要求 CDC DTR 置位。
   * @param enable true 要求 DTR；false 不要求 DTR，仅要求已挂载。
   *
   * @note
   * 某些串口调试助手不会主动拉高 DTR。若遇到已枚举但收不到数据，
   * 建议关闭该限制（默认已关闭）。
   */
  void set_require_dtr(bool enable);

  /**
   * @brief 查询设备是否已被主机枚举挂载。
   * @return true 已挂载；false 未挂载。
   */
  bool mounted() const;

  /**
   * @brief 通过 CDC 发送数据。
   * @param data 待发送数据指针。
   * @param len 待发送长度（字节）。
   * @return true 全部发送成功；false 未连接或未全部写入。
   */
  bool cdc_write(const uint8_t* data, uint32_t len);

  /**
   * @brief 通过 CDC 读取数据。
   * @param data 输出缓冲区指针。
   * @param len 最多读取长度（字节）。
   * @return uint32_t 实际读取字节数。
   */
  uint32_t cdc_read(uint8_t* data, uint32_t len);

  /**
   * @brief 获取 CDC 接收缓冲区内可读字节数。
   * @return uint32_t 可读字节数。
   */
  uint32_t cdc_available() const;

  /**
   * @brief 发送一个 Generic HID Input Report。
   * @param data 有效载荷；不足 64 字节时自动补零。
   * @param len 有效载荷长度，范围 1~64 字节。
   * @return true 已提交完整 64 字节报告；false 未就绪或参数非法。
   */
  bool hid_write(const uint8_t* data, uint32_t len);

  /** @brief 从 HID Output Report 接收队列读取数据。 */
  uint32_t hid_read(uint8_t* data, uint32_t len);

  /** @brief 返回 HID Output Report 接收队列的可读字节数。 */
  uint32_t hid_available() const;

  /** @brief TinyUSB HID OUT 回调入口；应用代码不应直接调用。 */
  void accept_hid_report(const uint8_t* data, uint32_t len);

  /**
   * @brief 注册当前 USB 传输类的接收回调。
   * @param cb 回调函数指针，传入 nullptr 可注销回调。
   * @param user_ctx 用户上下文指针，将原样传回回调。
   */
  void set_rx_callback(RxCallback cb, void* user_ctx);

private:
  /**
   * @brief 处理当前 USB 类的接收轮询并分发回调。
   */
  void _process_rx();

  BspUsb() = default;

  RxCallback _rx_callback = nullptr;
  void*      _rx_user_ctx = nullptr;
  bool       _require_dtr = false;
  bool       _initialized = false;

  static constexpr uint16_t HID_RX_BUFFER_SIZE = 256U;
  uint8_t           _hid_rx_buffer[HID_RX_BUFFER_SIZE] = {};
  volatile uint16_t _hid_rx_head = 0U;
  volatile uint16_t _hid_rx_tail = 0U;
};

#endif // __BSP_USB_HPP__

/**
 * @file data_pack.hpp
 * @author ChoseB (ChoseB@cumt.edu.cn)
 * @brief DataPack 数据打包模块 —— 变量地址注册、单帧打包/解包、USB CDC 传输
 * @version 0.1
 * @date 2026-08-13
 *
 * @copyright Copyright (c) 2026
 *
 * @details 打包、解包与 USB 传输实现位于 data_pack.cpp。
 */

#ifndef __DATA_PACK_HPP__
#define __DATA_PACK_HPP__

#include "status.hpp" // 统一状态码

/** @brief 单帧数据区最大字节数。 */
#define DATA_PACK_MAX_LENGTH 100

/**
 * @brief 数据打包格式。
 */
enum class DataFormat
{
  HEX, /**< 以二进制字节流方式打包与发送。 */
  STR, /**< 以字符串方式打包与发送（当前未实现）。 */
};

/**
 * @brief 单帧数据包。
 *
 * @details
 * DataPack 负责管理变量地址、打包与解包，以及通过 USB CDC 收发单帧数据。
 */
class DataPack
{
public:
  /**
   * @brief 数据包配置结构体（可匿名按序传入）。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）。
     */
    Config(uint8_t header = 0xAA, DataFormat format = DataFormat::HEX)
      : header(header),
        format(format)
    {
    }

    uint8_t   header; ///< 帧头字节。
    DataFormat format; ///< 打包格式。
  };

  /**
   * @brief 构造函数。
   * @param cfg 数据包配置（帧头/格式，可匿名按序传入）。
   */
  DataPack(const Config &cfg);

private:
  /** @brief 数据缓存区。 */
  uint8_t _data[DATA_PACK_MAX_LENGTH];
  /** @brief 当前缓存区有效字节数。 */
  uint32_t _data_length;

  /**
   * @brief 支持的数据类型标签。
   */
  enum class VarType
  {
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    INT8,
    INT16,
    INT32,
    INT64,
    FLOAT,
    DOUBLE,
    STRING,
  };

  /**
   * @brief 单个数据源条目。
   */
  struct VarEntry
  {
    void*   addr; /**< 变量地址。 */
    VarType type; /**< 变量类型。 */
  };

  /** @brief 固定帧头（由构造函数指定）。 */
  uint8_t _header;

  /** @brief 本包绑定的数据源数组。 */
  VarEntry _data_source[DATA_PACK_MAX_LENGTH];

  /** @brief 已绑定的数据源数量。 */
  uint32_t _data_source_length;

  /** @brief 关联的数据包源数组。 */
  DataPack* _data_pack_source[DATA_PACK_MAX_LENGTH];

  /** @brief 已关联的数据包源数量。 */
  uint8_t _data_pack_source_length;

  /** @brief 全局打包格式。 */
  static DataFormat _data_format;

  /** @brief 清空缓存数据并复位长度。 */
  void _clear_data();

  /**
   * @brief 添加一个数据源条目。
   * @param addr 变量地址。
   * @param type 变量类型。
   * @return Status 添加结果。
   */
  Status _link_data_entry(void* addr, VarType type);

public:
  /** @brief 析构函数。 */
  ~DataPack();

  /** @brief 绑定 uint8_t 变量地址。 */
  Status link_data(uint8_t* data_source);
  /** @brief 绑定 uint16_t 变量地址。 */
  Status link_data(uint16_t* data_source);
  /** @brief 绑定 uint32_t 变量地址。 */
  Status link_data(uint32_t* data_source);
  /** @brief 绑定 uint64_t 变量地址。 */
  Status link_data(uint64_t* data_source);
  /** @brief 绑定 int8_t 变量地址。 */
  Status link_data(int8_t* data_source);
  /** @brief 绑定 int16_t 变量地址。 */
  Status link_data(int16_t* data_source);
  /** @brief 绑定 int32_t 变量地址。 */
  Status link_data(int32_t* data_source);
  /** @brief 绑定 int64_t 变量地址。 */
  Status link_data(int64_t* data_source);
  /** @brief 绑定 float 变量地址。 */
  Status link_data(float* data_source);
  /** @brief 绑定 double 变量地址。 */
  Status link_data(double* data_source);

  /**
   * @brief 绑定字符串地址。
   * @param str 以 '\0' 结束的字符串地址。
   * @return Status 绑定结果。
   */
  Status link_data(const char* str);

  /**
   * @brief 关联一个 DataPack，并复制其数据源条目到当前对象。
   * @param pack_source 源包对象。
   * @return Status 关联结果。
   */
  Status link_data_pack(DataPack* pack_source);

  /**
   * @brief 从已绑定变量读取值并打包到内部缓存。
   * @return Status 打包结果。
   */
  Status get_data();

  /**
   * @brief 将内部缓存按绑定顺序解包并写回变量。
   * @return Status 解包结果。
   */
  Status distribute_data();

  /**
   * @brief 打包并通过 USB CDC 发送数据。
   *
   * @details
   * 发送帧格式：`[header][payload][tail_hash]`，其中 `tail_hash` 为 payload 的哈希值。
   * @return Status 发送结果。
   */
  Status send_data();

  /**
   * @brief 从 USB CDC 接收数据并分发到已绑定变量。
   *
   * @details
   * 接收后按 `[header][payload][tail_hash]` 校验：
   * 1) 帧头匹配；
   * 2) 尾部哈希与 payload 计算值一致；
   * 3) STR 模式下字符串字段内容一致。
   * @return Status 接收与分发结果。
   */
  Status receive_data();
};

#endif // __DATA_PACK_HPP__
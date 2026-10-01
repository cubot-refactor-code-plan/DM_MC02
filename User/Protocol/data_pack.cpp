#include "data_pack.hpp"
#include "bsp_cfg.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

DataFormat DataPack::_data_format = DataFormat::HEX;

namespace
{
uint8_t calc_payload_hash(const uint8_t* payload, uint32_t len)
{
  uint8_t hash = 0x5Au;
  for (uint32_t i = 0; i < len; ++i)
  {
    hash = static_cast<uint8_t>((hash * 131u) ^ payload[i]);
  }
  return hash;
}

template <typename T>
Status append_value(uint8_t* buffer, uint32_t capacity, uint32_t* used, const T* value)
{
  if ((buffer == nullptr) || (used == nullptr) || (value == nullptr))
  {
    return Status::BAD_ARG;
  }

  const uint32_t size = static_cast<uint32_t>(sizeof(T));
  if ((*used + size) > capacity)
  {
    return Status::FULL;
  }

  memcpy(&buffer[*used], value, size);
  *used += size;
  return Status::OK;
}

template <typename T>
Status extract_value(const uint8_t* buffer, uint32_t length, uint32_t* offset, T* out)
{
  if ((buffer == nullptr) || (offset == nullptr) || (out == nullptr))
  {
    return Status::BAD_ARG;
  }

  const uint32_t size = static_cast<uint32_t>(sizeof(T));
  if ((*offset + size) > length)
  {
    return Status::FULL;
  }

  memcpy(out, &buffer[*offset], size);
  *offset += size;
  return Status::OK;
}
} // namespace

DataPack::DataPack(const Config& cfg) :
  _data(),         // 构造函数只做赋值：数组清零
  _data_length(1), // 帧头占 1 字节
  _header(cfg.header),
  _data_source_length(0),
  _data_pack_source_length(0)
{
  _data_format = cfg.format; // 静态成员赋值
  _data[0]     = cfg.header;
}

/**
 * @brief 析构函数。
 */
DataPack::~DataPack()
{
  _clear_data();
  _data_source_length      = 0;
  _data_pack_source_length = 0;
}

/**
 * @brief 清空内部缓存数据。
 */
void DataPack::_clear_data()
{
  memset(_data, 0, sizeof(_data));
  _data_length          = 0;
  _data[_data_length++] = _header;
}

/**
 * @brief 添加一个数据源条目。
 * @param addr 变量地址。
 * @param type 变量类型。
 * @return Status 添加结果。
 */
Status DataPack::_link_data_entry(void* addr, VarType type)
{
  if (addr == nullptr)
  {
    return Status::BAD_ARG;
  }

  if (_data_source_length >= DATA_PACK_MAX_LENGTH)
  {
    return Status::FULL;
  }

  this->_data_source[_data_source_length++] = {addr, type};
  return Status::OK;
}

#define DATA_PACK_LINK_IMPL(_ctype, _vartype)      \
  Status DataPack::link_data(_ctype* data_source)  \
  {                                                \
    return _link_data_entry(data_source, _vartype); \
  }

DATA_PACK_LINK_IMPL(uint8_t, VarType::UINT8)
DATA_PACK_LINK_IMPL(uint16_t, VarType::UINT16)
DATA_PACK_LINK_IMPL(uint32_t, VarType::UINT32)
DATA_PACK_LINK_IMPL(uint64_t, VarType::UINT64)
DATA_PACK_LINK_IMPL(int8_t, VarType::INT8)
DATA_PACK_LINK_IMPL(int16_t, VarType::INT16)
DATA_PACK_LINK_IMPL(int32_t, VarType::INT32)
DATA_PACK_LINK_IMPL(int64_t, VarType::INT64)
DATA_PACK_LINK_IMPL(float, VarType::FLOAT)
DATA_PACK_LINK_IMPL(double, VarType::DOUBLE)

#undef DATA_PACK_LINK_IMPL

Status DataPack::link_data(const char* str)
{
  return _link_data_entry(const_cast<char*>(str), VarType::STRING);
}

/**
 * @brief 关联一个数据包并复制其数据源条目。
 * @param pack_source 源数据包。
 * @return Status 关联结果。
 */
Status DataPack::link_data_pack(DataPack* pack_source)
{
  if (pack_source == nullptr)
  {
    return Status::BAD_ARG;
  }

  if (_data_pack_source_length >= DATA_PACK_MAX_LENGTH)
  {
    return Status::FULL;
  }

  if ((_data_source_length + pack_source->_data_source_length) > DATA_PACK_MAX_LENGTH)
  {
    return Status::FULL;
  }

  _data_pack_source[_data_pack_source_length++] = pack_source;

  for (uint32_t i = 0; i < pack_source->_data_source_length; ++i)
  {
    _data_source[_data_source_length++] = pack_source->_data_source[i];
  }

  return Status::OK;
}

/**
 * @brief 从绑定变量打包到内部缓存。
 * @return Status 打包结果。
 */
Status DataPack::get_data()
{
  _clear_data();

  for (uint32_t i = 0; i < _data_source_length; ++i)
  {
    const VarEntry& entry  = _data_source[i];
    Status          status = Status::IO_ERROR;

    if (_data_format == DataFormat::STR)
    {
      char token[64];
      token[0] = '\0';

      switch (entry.type)
      {
        case VarType::UINT8:
          (void)snprintf(token, sizeof(token), "%u", static_cast<unsigned>(*static_cast<uint8_t*>(entry.addr)));
          break;
        case VarType::UINT16:
          (void)snprintf(token, sizeof(token), "%u", static_cast<unsigned>(*static_cast<uint16_t*>(entry.addr)));
          break;
        case VarType::UINT32:
          (void)snprintf(token, sizeof(token), "%lu", static_cast<unsigned long>(*static_cast<uint32_t*>(entry.addr)));
          break;
        case VarType::UINT64:
          (void)snprintf(token, sizeof(token), "%llu", static_cast<unsigned long long>(*static_cast<uint64_t*>(entry.addr)));
          break;
        case VarType::INT8:
          (void)snprintf(token, sizeof(token), "%d", static_cast<int>(*static_cast<int8_t*>(entry.addr)));
          break;
        case VarType::INT16:
          (void)snprintf(token, sizeof(token), "%d", static_cast<int>(*static_cast<int16_t*>(entry.addr)));
          break;
        case VarType::INT32:
          (void)snprintf(token, sizeof(token), "%ld", static_cast<long>(*static_cast<int32_t*>(entry.addr)));
          break;
        case VarType::INT64:
          (void)snprintf(token, sizeof(token), "%lld", static_cast<long long>(*static_cast<int64_t*>(entry.addr)));
          break;
        case VarType::FLOAT:
          (void)snprintf(token, sizeof(token), "%.6g", static_cast<double>(*static_cast<float*>(entry.addr)));
          break;
        case VarType::DOUBLE:
          (void)snprintf(token, sizeof(token), "%.12g", *static_cast<double*>(entry.addr));
          break;
        case VarType::STRING:
        {
          if (entry.addr == nullptr)
          {
            return Status::BAD_ARG;
          }
          const char* str = static_cast<const char*>(entry.addr);
          uint32_t    len = static_cast<uint32_t>(strlen(str));
          if ((_data_length + len + ((i + 1u < _data_source_length) ? 1u : 0u)) > DATA_PACK_MAX_LENGTH)
          {
            return Status::FULL;
          }
          memcpy(&_data[_data_length], str, len);
          _data_length += len;
          if (i + 1u < _data_source_length)
          {
            _data[_data_length++] = ',';
          }
          status = Status::OK;
          break;
        }
        default:
          return Status::IO_ERROR;
      }

      if (entry.type != VarType::STRING)
      {
        uint32_t len = static_cast<uint32_t>(strlen(token));
        if ((_data_length + len + ((i + 1u < _data_source_length) ? 1u : 0u)) > DATA_PACK_MAX_LENGTH)
        {
          return Status::FULL;
        }
        memcpy(&_data[_data_length], token, len);
        _data_length += len;
        if (i + 1u < _data_source_length)
        {
          _data[_data_length++] = ',';
        }
        status = Status::OK;
      }

      if (status != Status::OK)
      {
        return status;
      }

      continue;
    }

    switch (entry.type)
    {
      case VarType::UINT8:
        status = append_value<uint8_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<uint8_t*>(entry.addr));
        break;
      case VarType::UINT16:
        status = append_value<uint16_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<uint16_t*>(entry.addr));
        break;
      case VarType::UINT32:
        status = append_value<uint32_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<uint32_t*>(entry.addr));
        break;
      case VarType::UINT64:
        status = append_value<uint64_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<uint64_t*>(entry.addr));
        break;
      case VarType::INT8:
        status = append_value<int8_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<int8_t*>(entry.addr));
        break;
      case VarType::INT16:
        status = append_value<int16_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<int16_t*>(entry.addr));
        break;
      case VarType::INT32:
        status = append_value<int32_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<int32_t*>(entry.addr));
        break;
      case VarType::INT64:
        status = append_value<int64_t>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<int64_t*>(entry.addr));
        break;
      case VarType::FLOAT:
        status = append_value<float>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<float*>(entry.addr));
        break;
      case VarType::DOUBLE:
        status = append_value<double>(_data, DATA_PACK_MAX_LENGTH, &_data_length, static_cast<double*>(entry.addr));
        break;
      case VarType::STRING:
      {
        if (entry.addr == nullptr)
        {
          status = Status::BAD_ARG;
          break;
        }

        const char* str = static_cast<const char*>(entry.addr);
        uint32_t    len = static_cast<uint32_t>(strlen(str)) + 1u;
        if ((_data_length + len) > DATA_PACK_MAX_LENGTH)
        {
          status = Status::FULL;
          break;
        }
        memcpy(&_data[_data_length], str, len);
        _data_length += len;
        status = Status::OK;
        break;
      }
      default:
        status = Status::IO_ERROR;
        break;
    }

    if (status != Status::OK)
    {
      return status;
    }
  }

  return Status::OK;
}

Status DataPack::distribute_data()
{
  if (_data_format == DataFormat::STR)
  {
    uint32_t offset = 0;

    for (uint32_t i = 0; i < _data_source_length; ++i)
    {
      const VarEntry& entry = _data_source[i];
      uint32_t        start = offset;

      while ((offset < _data_length) && (_data[offset] != ','))
      {
        offset++;
      }

      uint32_t token_len = offset - start;
      char     token[DATA_PACK_MAX_LENGTH + 1];
      if (token_len > DATA_PACK_MAX_LENGTH)
      {
        return Status::FULL;
      }

      memcpy(token, &_data[start], token_len);
      token[token_len] = '\0';

      char* endptr = nullptr;
      switch (entry.type)
      {
        case VarType::UINT8:
          *static_cast<uint8_t*>(entry.addr) = static_cast<uint8_t>(strtoul(token, &endptr, 10));
          break;
        case VarType::UINT16:
          *static_cast<uint16_t*>(entry.addr) = static_cast<uint16_t>(strtoul(token, &endptr, 10));
          break;
        case VarType::UINT32:
          *static_cast<uint32_t*>(entry.addr) = static_cast<uint32_t>(strtoul(token, &endptr, 10));
          break;
        case VarType::UINT64:
          *static_cast<uint64_t*>(entry.addr) = static_cast<uint64_t>(strtoull(token, &endptr, 10));
          break;
        case VarType::INT8:
          *static_cast<int8_t*>(entry.addr) = static_cast<int8_t>(strtol(token, &endptr, 10));
          break;
        case VarType::INT16:
          *static_cast<int16_t*>(entry.addr) = static_cast<int16_t>(strtol(token, &endptr, 10));
          break;
        case VarType::INT32:
          *static_cast<int32_t*>(entry.addr) = static_cast<int32_t>(strtol(token, &endptr, 10));
          break;
        case VarType::INT64:
          *static_cast<int64_t*>(entry.addr) = static_cast<int64_t>(strtoll(token, &endptr, 10));
          break;
        case VarType::FLOAT:
          *static_cast<float*>(entry.addr) = strtof(token, &endptr);
          break;
        case VarType::DOUBLE:
          *static_cast<double*>(entry.addr) = strtod(token, &endptr);
          break;
        case VarType::STRING:
        {
          // STR 模式字符串校验：接收内容必须与本地绑定字符串一致。
          const char* expected = static_cast<const char*>(entry.addr);
          if ((expected == nullptr) || (strcmp(expected, token) != 0))
          {
            return Status::IO_ERROR;
          }
          endptr = token + token_len;
          break;
        }
        default:
          return Status::IO_ERROR;
      }

      if ((entry.type != VarType::STRING) && ((endptr == nullptr) || (*endptr != '\0')))
      {
        return Status::IO_ERROR;
      }

      if (offset < _data_length)
      {
        offset++; // skip comma
      }
    }

    if (offset < _data_length)
    {
      return Status::IO_ERROR;
    }

    return Status::OK;
  }

  uint32_t offset = 0;

  for (uint32_t i = 0; i < _data_source_length; ++i)
  {
    const VarEntry& entry  = _data_source[i];
    Status          status = Status::IO_ERROR;

    switch (entry.type)
    {
      case VarType::UINT8:
        status = extract_value<uint8_t>(_data, _data_length, &offset, static_cast<uint8_t*>(entry.addr));
        break;
      case VarType::UINT16:
        status = extract_value<uint16_t>(_data, _data_length, &offset, static_cast<uint16_t*>(entry.addr));
        break;
      case VarType::UINT32:
        status = extract_value<uint32_t>(_data, _data_length, &offset, static_cast<uint32_t*>(entry.addr));
        break;
      case VarType::UINT64:
        status = extract_value<uint64_t>(_data, _data_length, &offset, static_cast<uint64_t*>(entry.addr));
        break;
      case VarType::INT8:
        status = extract_value<int8_t>(_data, _data_length, &offset, static_cast<int8_t*>(entry.addr));
        break;
      case VarType::INT16:
        status = extract_value<int16_t>(_data, _data_length, &offset, static_cast<int16_t*>(entry.addr));
        break;
      case VarType::INT32:
        status = extract_value<int32_t>(_data, _data_length, &offset, static_cast<int32_t*>(entry.addr));
        break;
      case VarType::INT64:
        status = extract_value<int64_t>(_data, _data_length, &offset, static_cast<int64_t*>(entry.addr));
        break;
      case VarType::FLOAT:
        status = extract_value<float>(_data, _data_length, &offset, static_cast<float*>(entry.addr));
        break;
      case VarType::DOUBLE:
        status = extract_value<double>(_data, _data_length, &offset, static_cast<double*>(entry.addr));
        break;
      case VarType::STRING:
      {
        if (offset >= _data_length)
        {
          return Status::FULL;
        }

        while ((offset < _data_length) && (_data[offset] != '\0'))
        {
          offset++;
        }
        if (offset >= _data_length)
        {
          return Status::FULL;
        }
        offset++; // 跳过 '\0'
        status = Status::OK;
        break;
      }
      default:
        status = Status::IO_ERROR;
        break;
    }

    if (status != Status::OK)
    {
      return status;
    }
  }

  return Status::OK;
}

/**
 * @brief 打包后发送到 USB CDC。
 * @return Status 发送结果。
 */
Status DataPack::send_data()
{
  Status status = get_data();
  if (status != Status::OK)
  {
    return status;
  }

  if (_data_length == 0u)
  {
    return Status::FULL;
  }

  const uint8_t tail_hash = calc_payload_hash(_data, _data_length);

  _data[_data_length++] = tail_hash;

  if (!bsp_usb.cdc_write(_data, _data_length))
  {
    return Status::IO_ERROR;
  }

  return Status::OK;
}

/**
 * @brief 从 USB CDC 接收后分发到绑定变量。
 * @return Status 接收与分发结果。
 */
Status DataPack::receive_data()
{
  uint32_t available = bsp_usb.cdc_available();
  if (available == 0u)
  {
    return Status::FULL;
  }

  if (available > (DATA_PACK_MAX_LENGTH))
  {
    available = DATA_PACK_MAX_LENGTH;
  }

  uint8_t  frame[DATA_PACK_MAX_LENGTH];
  uint32_t read_len = bsp_usb.cdc_read(frame, available);
  if (read_len == 0u)
  {
    return Status::FULL;
  }

  if (read_len < 3u)
  {
    return Status::IO_ERROR;
  }

  uint32_t start = 0;
  while ((start < read_len) && (frame[start] != _header))
  {
    start++;
  }
  if (start >= read_len)
  {
    return Status::IO_ERROR;
  }

  // 协议固定为：[header][payload][tail_hash]
  const uint32_t end = read_len - 1u;
  if (end <= start)
  {
    return Status::IO_ERROR;
  }

  uint32_t payload_len = end - start - 0u;
  payload_len -= 1u;
  if (payload_len > DATA_PACK_MAX_LENGTH)
  {
    return Status::FULL;
  }

  memcpy(_data, &frame[start + 1u], payload_len);

  const uint8_t recv_tail_hash = frame[end];
  const uint8_t calc_tail_hash = calc_payload_hash(_data, payload_len);
  if (recv_tail_hash != calc_tail_hash)
  {
    return Status::IO_ERROR;
  }

  _data_length = payload_len;

  // 通信校验：帧头与哈希合法后，再通过 STR 模式字符串一致性校验。
  return distribute_data();
}
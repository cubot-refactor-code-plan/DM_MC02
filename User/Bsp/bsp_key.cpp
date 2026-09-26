#include "bsp_key.hpp"

// ---------------- 初始化 ----------------

/**
 * @brief 绑定引脚并重置消抖状态
 * @param cfg 引脚 + 消抖/长按参数
 *
 * @note 读取 GPIO 初始电平作为 _last_stable，
 *       避免启动时误触发按下/释放事件。
 *
 * @return Status OK=绑定成功，BAD_ARG=端口/引脚非法或计数值为 0
 */
Status BspKey::init(const Config &cfg)
{
  if (cfg.port == nullptr || cfg.pin == 0U)
  {
    return Status::BAD_ARG; // 端口/引脚非法
  }
  if (cfg.debounce_cnt == 0U || cfg.long_press_cnt == 0U)
  {
    return Status::BAD_ARG; // 计数为 0 会导致消抖/长按行为未定义
  }

  _port           = cfg.port;
  _pin            = cfg.pin;
  _active_low     = cfg.active_low;
  _debounce_cnt   = cfg.debounce_cnt;
  _long_press_cnt = cfg.long_press_cnt;
  _cnt            = 0U;
  _hold_cnt       = 0U;
  _long_fired     = false;

  /* 以当前电平作为初始稳定状态 */
  bool raw     = (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
  _last_stable = _active_low ? !raw : raw;

  return Status::OK;
}

// ----------------

// ---------------- 消抖轮询 ----------------

/**
 * @brief 单次消抖轮询（裸机非阻塞，立即返回）
 *
 * 事件产生流程:
 * @code
 *   按下 ──(消抖确认)──► PRESS ──┬──(阈值内松手)────► SHORT ──► 松手到NONE
 *                              └──(按住到阈值)────► LONG  ──► 松手到NONE
 * @endcode
 *
 * 长按计时起点: 在确认按下(PRESS)时，把已累计的消抖次数作为 _hold_cnt 初值，
 *               使长按时间从"电平跳变那一刻"起算，即 long_press_cnt × 轮询周期。
 *
 * @return 通过返回值来确定按钮状态。本次触发的事件，大部分情况返回 NONE
 */
BspKey::Event BspKey::poll()
{
  /* 未 init() / init() 失败：安全返回，避免空指针解引用导致 HardFault */
  if (_port == nullptr)
  {
    return Event::NONE;
  }

  bool raw    = (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
  bool active = _active_low ? !raw : raw; /* true = 按键按下 */

  /* 阶段一: 电平与上次一致 → 稳定 */
  if (active == _last_stable)
  {
    _cnt = 0U; /* 清零抖动计数 */
    if (active)
    {
      if (_hold_cnt < UINT16_MAX)
      {
        _hold_cnt++; /* 按下保持中（防溢出回绕） */
      }
      if (!_long_fired && _hold_cnt >= _long_press_cnt)
      {
        _long_fired = true; /* 标记已触发，本次按下不再重复 */
        return Event::LONG;
      }
    }
    return Event::NONE;
  }

  /* 阶段二: 电平与上次不同 → 可能是抖动 */
  _cnt++;
  if (_cnt < _debounce_cnt)
  {
    return Event::NONE; /* 消抖未完成，继续等待 */
  }

  /* 阶段三: 消抖确认，状态切换 */
  uint8_t confirm_cnt = _cnt; /* 已累计的消抖次数 = 电平跳变后经过的轮询次数 */
  _cnt                = 0U;

  if (active && !_last_stable)
  {
    /* 确认按下：长按从电平跳变那一刻起算，故用消抖次数作为保持计数初值 */
    _last_stable = true;
    _hold_cnt    = confirm_cnt;
    _long_fired  = false;
    return Event::PRESS;
  }
  else if (!active && _last_stable)
  {
    /* 确认释放 */
    _last_stable = false;
    _hold_cnt    = 0U;
    if (_long_fired)
    {
      /* 长按已触发过：本次松手不产生 SHORT（避免被上层误判为一次短按） */
      _long_fired = false;
      return Event::NONE;
    }
    return Event::SHORT; /* 阈值内松手 → 短按 */
  }

  return Event::NONE;
}

// ----------------

// ---------------- 辅助接口 ----------------

/**
 * @brief 读取引脚原始电平（未消抖、未换算 active_low）
 *
 * @note 未 init() 时安全返回 false，避免空指针解引用。
 *       注意与 is_pressed() 语义不同：低有效按键松开时本函数返回 true。
 */
bool BspKey::read_raw() const
{
  if (_port == nullptr)
  {
    return false;
  }
  return (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
}

// ----------------

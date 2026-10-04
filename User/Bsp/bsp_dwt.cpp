#include "bsp_dwt.hpp"


// ---------------- 公共接口 ----------------


/**
 * @brief 使能 CYCCNT 周期计数（幂等，可重复调用）
 *
 * @return Status OK=可用；IO_ERROR=本内核无 CYCCNT（CTRL.NOCYCCNT=1）或使能未生效
 *
 * @note 顺序：先开 DEMCR.TRCENA，再写 LAR 解锁，然后清零 → 使能 →
 *       等约 8 个周期计数稳定 → 再清零（否则第一次读数不可靠）。
 */
Status BspDwt::init()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 1. 总开关（脱机运行时必须自己置位）
  DWT->LAR    = 0xC5ACCE55UL;                     // 2. Cortex-M7 的软件锁解锁
  DWT->CYCCNT = 0U;                               // 3. 清零
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 4. 使能计数
  for (volatile uint32_t i = 0U; i < 16U; i++)    // 5. 使能后约 8 周期计数才稳定
  {
  }
  DWT->CYCCNT = 0U; // 6. 稳定后再清零

  _cpu_hz   = cpu_clock_hz();
  _round_s  = 4294967296.0 / (double)_cpu_hz; // 2^32 / CPU 时钟
  _base_s   = 0.0;
  _last_cnt = 0U;

  // 复核：NOCYCCNT=1（本内核无 CYCCNT）或使能位没写进去 ⇒ 计时不可信
  _inited = ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U) && ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);

  return _inited ? Status::OK : Status::IO_ERROR;
}

/** @brief 与上次调用的时间差（秒）；无符号差值，回绕安全 */
double BspDwt::delta_s(uint32_t *last) const
{
  const uint32_t now = DWT->CYCCNT;
  const double   dt  = (double)(uint32_t)(now - *last) / (double)_cpu_hz; // 无符号差值 ⇒ 回绕安全
  *last              = now;
  return dt;
}

/** @brief 绝对时间（秒，自 init() 起算） */
double BspDwt::get_time_s()
{
  // 必须先调用 update_timeline()（它会累加 _base_s 的副作用），再取 _base_s。
  // 若写成 `_base_s + update_timeline()/...`，则 "+" 两侧求值顺序未定义：
  // 编译器可能先取旧的 _base_s，导致回绕那一次调用少一整圈（时间倒退 7.809 s）。
  const uint32_t now = update_timeline();
  return _base_s + (double)now / (double)_cpu_hz;
}

/** @brief 绝对时间（毫秒） */
double BspDwt::get_time_ms()
{
  return get_time_s() * 1000.0;
}

/** @brief 绝对时间（微秒） */
double BspDwt::get_time_us()
{
  return get_time_s() * 1000000.0;
}

/** @brief 忙等指定纳秒，ns的值只能是1.818的倍数，这个太短了 */
void BspDwt::delay_ns(double ns) const
{
  wait_cycles(ns * 1e-9 * (double)_cpu_hz);
}

/** @brief 忙等指定微秒 */
void BspDwt::delay_us(double us) const
{
  wait_cycles(us * 1e-6 * (double)_cpu_hz);
}

/** @brief 忙等指定毫秒 */
void BspDwt::delay_ms(double ms) const
{
  wait_cycles(ms * 1e-3 * (double)_cpu_hz);
}

/** @brief 忙等指定秒 */
void BspDwt::delay_s(double s) const
{
  wait_cycles(s * (double)_cpu_hz);
}


// ----------------
// ---------------- 私有实现 ----------------


/** @brief 四个 delay_* 的公共实现：忙等 cycles 个 CPU 周期 */
void BspDwt::wait_cycles(double cycles) const
{
  if (cycles <= 0.0 || !_inited) // 未初始化时计数频率未知，无法换算
  {
    return;
  }

  if (cycles > 4294967295.0) // 32 位计数器上限（本工程 ≈7.809 s @550 MHz）
  {
    cycles = 4294967295.0;
  }

  const uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < (uint32_t)cycles)
  {
  }
}

/** @brief 回绕检测 + 累加，返回本次读到的 CYCCNT（绝对时间轴唯一的状态更新点） */
uint32_t BspDwt::update_timeline()
{
  const uint32_t now = DWT->CYCCNT;

  if (now < _last_cnt) // 计数器回绕：补上一圈
  {
    _base_s += _round_s;
  }
  _last_cnt = now;

  return now;
}

/**
 * @brief 算出 CPU 时钟（Hz）= SYSCLK / D1CPRE
 *
 * @note D1CPRE 的编码与 AHB 预分频 HPRE 相同：0xx=/1，1000=/2 … 1111=/512。
 */
uint32_t BspDwt::cpu_clock_hz()
{
  // D1CPRE 的编码与 AHB 预分频 HPRE 相同：0xx=/1，1000=/2 … 1111=/512
  static const uint16_t div_tbl[8] = {2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  const uint32_t        d1cpre     = (RCC->D1CFGR & RCC_D1CFGR_D1CPRE_Msk) >> RCC_D1CFGR_D1CPRE_Pos;

  const uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  return (d1cpre < 8U) ? sysclk : (sysclk / div_tbl[d1cpre - 8U]);
}

// ----------------

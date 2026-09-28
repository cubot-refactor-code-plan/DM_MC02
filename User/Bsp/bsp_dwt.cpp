#include "bsp_dwt.hpp"


// ---------------- 公共接口 ----------------


Status BspDwt::init()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 1. 总开关（脱机运行时必须自己置位）
  DWT->LAR = 0xC5ACCE55UL;                        // 2. Cortex-M7 的软件锁解锁
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

bool BspDwt::available() const
{
  return _inited;
}

uint32_t BspDwt::cpu_hz() const
{
  return _cpu_hz;
}

double BspDwt::delta_s(uint32_t *last) const
{
  const uint32_t now = DWT->CYCCNT;
  const double   dt  = (double)(uint32_t)(now - *last) / (double)_cpu_hz; // 无符号差值 ⇒ 回绕安全
  *last = now;
  return dt;
}

double BspDwt::time_s()
{
  // 必须先调用 update_timeline()（它会累加 _base_s 的副作用），再取 _base_s。
  // 若写成 `_base_s + update_timeline()/...`，则 "+" 两侧求值顺序未定义：
  // 编译器可能先取旧的 _base_s，导致回绕那一次调用少一整圈（时间倒退 7.809 s）。
  const uint32_t now = update_timeline();
  return _base_s + (double)now / (double)_cpu_hz;
}

double BspDwt::time_ms()
{
  return time_s() * 1000.0;
}

double BspDwt::time_us()
{
  return time_s() * 1000000.0;
}

void BspDwt::delay(double seconds) const
{
  if (seconds <= 0.0)
  {
    return;
  }

  double cycles = seconds * (double)_cpu_hz;
  if (cycles > 4294967295.0) // 32 位上限（本工程 ≈7.809 s @550 MHz）
  {
    cycles = 4294967295.0;
  }

  const uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < (uint32_t)cycles)
  {
  }
}


// ----------------
// ---------------- 私有实现 ----------------


///< 回绕检测 + 累加（绝对时间轴唯一的状态更新点）
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

///< 按 RCC 算 CPU 时钟（Hz）= SYSCLK / D1CPRE 分频
uint32_t BspDwt::cpu_clock_hz()
{
  // D1CPRE 的编码与 AHB 预分频 HPRE 相同：0xx=/1，1000=/2 … 1111=/512
  static const uint16_t div_tbl[8] = {2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  const uint32_t        d1cpre     = (RCC->D1CFGR & RCC_D1CFGR_D1CPRE_Msk) >> RCC_D1CFGR_D1CPRE_Pos;

  const uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  return (d1cpre < 8U) ? sysclk : (sysclk / div_tbl[d1cpre - 8U]);
}

// ----------------

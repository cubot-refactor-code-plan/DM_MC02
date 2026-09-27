#include "app_test.hpp"

#if APP_TEST_CAN_ENABLED

#  include "bsp_cfg.hpp"
#  include "fdcan.h"    // IWYU pragma: keep (DLC/ID/帧类型校验宏)
#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "status.hpp"
#  include "task.h"


// ---------------- 测试参数 ----------------

///< 帧 ID：CAN1 发 0x123 → CAN3 收；CAN3 发 0x321 → CAN1 收（避开 IMU 的 0x58/0x59）
#  define CAN_TEST_ID_FROM_CAN1 0x123U
#  define CAN_TEST_ID_FROM_CAN3 0x321U

///< 帧内来源标识字节（防止把本路自发帧误当作对方帧）
#  define CAN_TEST_TAG_C1 0xC1U
#  define CAN_TEST_TAG_C3 0xC3U

#  define CAN_TEST_PERIOD_MS 1U      ///< 发送周期（1 kHz tick 下 10 ms = 100 帧/s）
#  define CAN_TEST_FRAMES 10000U      ///< 每个方向发送的帧数
#  define CAN_TEST_IDLE_END_MS 500U   ///< 发完后静默这么久仍无新帧，即认为测试收尾
#  define CAN_TEST_REPORT_MS 1000U    ///< 测试中的进度打印周期
#  define CAN_TEST_HEARTBEAT_MS 5000U ///< 收尾后的诊断心跳周期（观察 tec/rec/bus_off）

// ----------------


// ---------------- 统计量（执行期由任务更新，调试器/Live Watch 可直接观察） ----------------

typedef struct
{
  volatile uint32_t tx_cnt;      ///< 本路成功入发送缓冲的帧数
  volatile uint32_t tx_fail_cnt; ///< 本路 send() 未返回 OK 的次数（缓冲满等）
  volatile uint32_t rx_cnt;      ///< 本路收到且校验通过的帧数
  volatile uint32_t rx_bad_cnt;  ///< 本路收到但 ID/长度/内容不符的帧数
} CanTestStat;

volatile CanTestStat can_test_stat1    = {0U, 0U, 0U, 0U}; ///< CAN1 侧统计
volatile CanTestStat can_test_stat3    = {0U, 0U, 0U, 0U}; ///< CAN3 侧统计
volatile uint32_t    can_test_finished = 0U;               ///< 1=已发满并打印结论

// ----------------


// ---------------- 帧编解码 ----------------

/**
 * @brief 组装 8 字节测试帧：seq(4B, 小端) + tag + 校验字节
 */
static void can_build_frame(uint8_t *data, uint32_t seq, uint8_t tag)
{
  data[0] = (uint8_t)(seq & 0xFFU);
  data[1] = (uint8_t)((seq >> 8) & 0xFFU);
  data[2] = (uint8_t)((seq >> 16) & 0xFFU);
  data[3] = (uint8_t)((seq >> 24) & 0xFFU);
  data[4] = tag;
  data[5] = (uint8_t)(~data[0]);
  data[6] = (uint8_t)(~data[1]);
  data[7] = (uint8_t)(tag ^ 0x5AU);
}

/**
 * @brief 校验一帧是否为本测试期望的帧（ID/类型/长度/内容）
 * @return true=通过；false=不符（调用方计入 rx_bad_cnt）
 */
static bool can_verify_frame(const CanRxMsg &msg, uint32_t expect_id, uint8_t expect_tag)
{
  if (msg.header.IdType != FDCAN_STANDARD_ID || msg.header.RxFrameType != FDCAN_DATA_FRAME)
  {
    return false;
  }
  if (msg.header.Identifier != expect_id || msg.header.DataLength != FDCAN_DLC_BYTES_8)
  {
    return false;
  }
  if (msg.data[4] != expect_tag)
  {
    return false;
  }
  if (msg.data[5] != (uint8_t)(~msg.data[0]) || msg.data[6] != (uint8_t)(~msg.data[1]))
  {
    return false;
  }
  return msg.data[7] == (uint8_t)(expect_tag ^ 0x5AU);
}

// ----------------


// ---------------- 收发辅助 ----------------

/**
 * @brief 构造并送出一帧（只入软件缓冲，硬件发送由中断接力）
 */
static void can_send_one(BspCan &bus, uint32_t std_id, uint8_t tag, uint32_t seq, volatile CanTestStat &stat)
{
  uint8_t data[8];
  can_build_frame(data, seq, tag);

  if (bus.send(std_id, data) == Status::OK)
  {
    stat.tx_cnt++;
  }
  else
  {
    stat.tx_fail_cnt++; // 缓冲满/未初始化：测试期间不应出现
  }
}

/**
 * @brief 非阻塞排空一路接收缓冲并逐帧校验（一次调用尽量清空积压）
 */
static void can_drain_rx(BspCan &bus, volatile CanTestStat &stat, uint32_t expect_id, uint8_t expect_tag)
{
  CanRxMsg msg;

  while (bus.receive(&msg, 0U) == Status::OK)
  {
    if (can_verify_frame(msg, expect_id, expect_tag))
    {
      stat.rx_cnt++;
    }
    else
    {
      stat.rx_bad_cnt++; // ID/类型/长度/内容不符
    }
  }
}

// ----------------


// ---------------- 打印（USART1, 115200；单次调用控制在流缓冲 128B 内） ----------------

static void can_print_progress(uint32_t elapsed_ms)
{
  // C1tx/C1rx = CAN1 发出/收到，C3tx/C3rx = CAN3 发出/收到；bad/fail 均为 (C1侧/C3侧)
  bsp_uart1.printf("[CAN] t=%u.%us C1tx=%u C1rx=%u C3tx=%u C3rx=%u bad=%u/%u fail=%u/%u\r\n",
                   elapsed_ms / 1000U,
                   (elapsed_ms % 1000U) / 100U,
                   (uint32_t)can_test_stat1.tx_cnt,
                   (uint32_t)can_test_stat1.rx_cnt,
                   (uint32_t)can_test_stat3.tx_cnt,
                   (uint32_t)can_test_stat3.rx_cnt,
                   (uint32_t)can_test_stat1.rx_bad_cnt,
                   (uint32_t)can_test_stat3.rx_bad_cnt,
                   (uint32_t)can_test_stat1.tx_fail_cnt,
                   (uint32_t)can_test_stat3.tx_fail_cnt);
}

/**
 * @brief 打印一路的收发/总线诊断（1 s 或 5 s 调用一次，非周期热路径）
 */
static void can_print_diag(const char *name, BspCan &bus)
{
  const BspCan::RxDiag    rx = bus.rx_diag();
  const BspCan::TxDiag    tx = bus.tx_diag();
  const BspCan::BusStatus bs = bus.bus_status();

  bsp_uart1.printf("[CAN] %s rx_drop=%u/%u/%u tx=%u/%u/%u/%u tec/rec=%u/%u bo=%u\r\n",
                   name,
                   rx.sw_drop_cnt,
                   rx.lost_cnt,
                   rx.len_drop_cnt,
                   tx.drop_cnt,
                   tx.fifo_fail_cnt,
                   tx.stall_recover_cnt,
                   tx.it_fail_cnt,
                   bs.tec,
                   bs.rec,
                   bs.bus_off_cnt);
}

/**
 * @brief 打印最终结论（含 PASS/FAIL 判据与两路诊断快照）
 */
static void can_print_result(uint32_t elapsed_ms)
{
  const uint32_t s1 = can_test_stat1.tx_cnt;
  const uint32_t r1 = can_test_stat1.rx_cnt;
  const uint32_t s3 = can_test_stat3.tx_cnt;
  const uint32_t r3 = can_test_stat3.rx_cnt;

  const BspCan::RxDiag    rx1 = bsp_can1.rx_diag();
  const BspCan::TxDiag    tx1 = bsp_can1.tx_diag();
  const BspCan::BusStatus bs1 = bsp_can1.bus_status();
  const BspCan::RxDiag    rx3 = bsp_can3.rx_diag();
  const BspCan::TxDiag    tx3 = bsp_can3.tx_diag();
  const BspCan::BusStatus bs3 = bsp_can3.bus_status();

  // 判据拆开写：出结论时哪一项不满足一目了然
  const bool sent_ok = (s1 == CAN_TEST_FRAMES) && (s3 == CAN_TEST_FRAMES);                                                                                                                                                                                                                   ///< 两侧都发满
  const bool recv_ok = (r1 == s3) && (r3 == s1);                                                                                                                                                                                                                                             ///< 收帧数 = 对方发帧数
  const bool data_ok = (can_test_stat1.rx_bad_cnt == 0U) && (can_test_stat3.rx_bad_cnt == 0U);                                                                                                                                                                                               ///< 无坏帧
  const bool txq_ok  = (can_test_stat1.tx_fail_cnt == 0U) && (can_test_stat3.tx_fail_cnt == 0U);                                                                                                                                                                                             ///< 无入队失败
  const bool drv_ok  = (rx1.sw_drop_cnt == 0U) && (rx1.lost_cnt == 0U) && (rx1.len_drop_cnt == 0U) && (rx3.sw_drop_cnt == 0U) && (rx3.lost_cnt == 0U) && (rx3.len_drop_cnt == 0U) && (tx1.drop_cnt == 0U) && (tx1.fifo_fail_cnt == 0U) && (tx3.drop_cnt == 0U) && (tx3.fifo_fail_cnt == 0U); ///< 驱动诊断全 0
  const bool bus_ok  = (bs1.bus_off_cnt == 0U) && (bs3.bus_off_cnt == 0U);                                                                                                                                                                                                                   ///< 未发生 Bus-Off
  const bool pass    = sent_ok && recv_ok && data_ok && txq_ok && drv_ok && bus_ok;

  bsp_uart1.printf("\r\n[CAN] ======== done in %u ms ========\r\n", elapsed_ms);
  bsp_uart1.printf("[CAN] CAN1->CAN3 sent=%u recv=%u bad=%u fail=%u\r\n",
                   s1,
                   r3,
                   (uint32_t)can_test_stat3.rx_bad_cnt,
                   (uint32_t)can_test_stat1.tx_fail_cnt);
  bsp_uart1.printf("[CAN] CAN3->CAN1 sent=%u recv=%u bad=%u fail=%u\r\n",
                   s3,
                   r1,
                   (uint32_t)can_test_stat1.rx_bad_cnt,
                   (uint32_t)can_test_stat3.tx_fail_cnt);
  can_print_diag("C1", bsp_can1);
  can_print_diag("C3", bsp_can3);
  bsp_uart1.printf("[CAN] chk sent=%u recv=%u data=%u txq=%u drv=%u bus=%u\r\n",
                   (uint32_t)sent_ok,
                   (uint32_t)recv_ok,
                   (uint32_t)data_ok,
                   (uint32_t)txq_ok,
                   (uint32_t)drv_ok,
                   (uint32_t)bus_ok);
  bsp_uart1.printf("[CAN] VERDICT: %s\r\n", pass ? "PASS" : "FAIL");
}

// ----------------


// ---------------- 任务入口 ----------------

extern "C" void can_test_task(void *argument)
{
  (void)argument; // 任务不需要外部参数

  const TickType_t period     = pdMS_TO_TICKS(CAN_TEST_PERIOD_MS);
  const TickType_t start_tick = xTaskGetTickCount();
  TickType_t       wake       = start_tick;

  uint32_t   seq1           = 0U; ///< CAN1 待发帧序号
  uint32_t   seq3           = 0U; ///< CAN3 待发帧序号
  TickType_t last_send_tick = start_tick;
  TickType_t last_rep_tick  = start_tick;
  TickType_t last_diag_tick = start_tick;
  bool       finished       = false;

  for (;;)
  {
    const TickType_t now = xTaskGetTickCount();

    // 阶段一：两侧各按周期发一帧（发满即停；排空接收放在发送之后，避免积压）
    if (seq1 < CAN_TEST_FRAMES)
    {
      can_send_one(bsp_can1, CAN_TEST_ID_FROM_CAN1, CAN_TEST_TAG_C1, seq1, can_test_stat1);
      seq1++;
      last_send_tick = now;
    }
    if (seq3 < CAN_TEST_FRAMES)
    {
      can_send_one(bsp_can3, CAN_TEST_ID_FROM_CAN3, CAN_TEST_TAG_C3, seq3, can_test_stat3);
      seq3++;
      last_send_tick = now;
    }

    // 阶段二：排空两路接收缓冲（CAN1 上应只出现 0x321，CAN3 上只出现 0x123）
    can_drain_rx(bsp_can1, can_test_stat1, CAN_TEST_ID_FROM_CAN3, CAN_TEST_TAG_C3);
    can_drain_rx(bsp_can3, can_test_stat3, CAN_TEST_ID_FROM_CAN1, CAN_TEST_TAG_C1);

    // 阶段三：进度打印 / 收尾判定 / 收尾后的诊断心跳
    if (!finished)
    {
      if ((now - last_rep_tick) >= pdMS_TO_TICKS(CAN_TEST_REPORT_MS))
      {
        last_rep_tick = now;
        can_print_progress((uint32_t)(now - start_tick));
      }

      // 两侧都发满，且静默超过 IDLE_END_MS 仍无新帧 → 剩在路上的帧已收完，出结论
      if ((seq1 == CAN_TEST_FRAMES) && (seq3 == CAN_TEST_FRAMES) && ((now - last_send_tick) >= pdMS_TO_TICKS(CAN_TEST_IDLE_END_MS)))
      {
        finished          = true;
        can_test_finished = 1U;
        last_diag_tick    = now;
        can_print_result((uint32_t)(now - start_tick));
      }
    }
    else if ((now - last_diag_tick) >= pdMS_TO_TICKS(CAN_TEST_HEARTBEAT_MS))
    {
      // 收尾后不再发帧，只做诊断心跳：拔线/接线时可直接看 tec/rec/bus_off 变化
      last_diag_tick = now;
      can_print_diag("C1", bsp_can1);
      can_print_diag("C3", bsp_can3);
    }

    vTaskDelayUntil(&wake, period);
  }
}

// ----------------

#endif // APP_TEST_CAN_ENABLED

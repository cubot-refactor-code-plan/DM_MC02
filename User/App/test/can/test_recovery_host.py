#!/usr/bin/env python3
"""Compile the production Bus-Off recovery state machine against fake registers/RTOS.

The state machine lives in User/Bsp/bsp_can.cpp (service_recovery/_tx_available);
this script extracts those two function bodies and links them with a fake BspCan
class, so the production source is never modified and no HAL is needed.

Run: python3 User/App/test/can/test_recovery_host.py
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
source = (root / 'User/Bsp/bsp_can.cpp').read_text()


def extract(signature):
    """切出 signature 开始的函数体，直到第 0 列的那个右花括号。"""
    start = source.index(signature)
    end = source.index('\n}\n', start) + len('\n}\n')
    return source[start:end]


header = r'''
#pragma once
#include <cstdint>
#include <cassert>
using TickType_t = uint32_t;
extern TickType_t tick;
inline TickType_t xTaskGetTickCount() { return tick; }
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() do {} while(0)
#define taskEXIT_CRITICAL() do {} while(0)
#define CLEAR_BIT(r,b) ((r) &= ~(b))
constexpr uint32_t FDCAN_PSR_BO=128, FDCAN_CCCR_INIT=1;
constexpr int HAL_FDCAN_STATE_BUSY=2;
enum class Status {OK,BUSY,NOT_INIT};
struct Regs {uint32_t PSR=0,CCCR=0,TXBCR=0,TXBRP=0;};
struct Handle {Regs *Instance; int State=HAL_FDCAN_STATE_BUSY;};
struct Buffer {int resets=0;};
inline void xMessageBufferReset(Buffer *b) {++b->resets;}
class BspCan {
public:
 Handle *_hfdcan;
 Buffer *_tx_message_buffer, *_rx_message_buffer;
 struct {uint32_t bus_off_events=0,recovery_attempts=0,recovery_successes=0,recovery_max_ticks=0;bool recovering=false;} diagnostics;
 TickType_t _recovery_started=0,_recovery_attempted=0;
 bool _tx_available() const;
 Status service_recovery();
};
'''
test = r'''
#include "bsp_can.hpp"
#include <iostream>
TickType_t tick=0;
int main() {
 Regs r;Handle h;h.Instance=&r;Buffer tx,rx;BspCan c;
 c._hfdcan=&h;c._tx_message_buffer=&tx;c._rx_message_buffer=&rx;
 assert(c._tx_available() && c.service_recovery()==Status::OK);
 r.PSR=FDCAN_PSR_BO;r.CCCR=1;r.TXBRP=3;
 assert(!c._tx_available());
 assert(c.service_recovery()==Status::BUSY && r.CCCR==0 && r.TXBCR==3);
 assert(tx.resets==1 && rx.resets==1 && c.diagnostics.recovery_attempts==1);
 // Still observing recessive bits: do not repeatedly restart recovery.
 tick=99;c.service_recovery();assert(c.diagnostics.recovery_attempts==1);
 r.CCCR=1;tick=100;c.service_recovery();assert(r.CCCR==0 && c.diagnostics.recovery_attempts==2);
 // BO clear is insufficient until stale hardware requests are cancelled.
 r.PSR=0;tick=102;assert(c.service_recovery()==Status::BUSY && !c._tx_available());
 r.TXBRP=0;tick=103;assert(c.service_recovery()==Status::OK && c._tx_available());
 assert(c.diagnostics.recovery_successes==1 && c.diagnostics.recovery_max_ticks==103);
 // Repeated Bus-Off cycles do not allocate or leave the sender latched off.
 for (unsigned i=0;i<1000;++i) {
   tick+=10;r.PSR=128;r.CCCR=1;r.TXBRP=1;
   assert(c.service_recovery()==Status::BUSY && r.TXBCR==1);
   r.PSR=0;r.TXBRP=0;tick+=2;
   assert(c.service_recovery()==Status::OK && c._tx_available());
 }
 assert(c.diagnostics.bus_off_events==1001 && c.diagnostics.recovery_successes==1001);
 // Tick rollover and sustained fault rate limiting.
 tick=0xfffffff0;r.PSR=128;r.CCCR=1;c.service_recovery();
 const auto attempts=c.diagnostics.recovery_attempts;
 r.CCCR=1;tick=20;c.service_recovery();assert(c.diagnostics.recovery_attempts==attempts);
 tick=100;c.service_recovery();assert(c.diagnostics.recovery_attempts==attempts+1);
 r.PSR=0;r.TXBRP=0;c.service_recovery();assert(c._tx_available());
 h.State=0;assert(c.service_recovery()==Status::NOT_INIT && !c._tx_available());
 std::cout << "PASS: 1001 recovery cycles, sustained faults, cancellation gate, retry delay, tick rollover, uninitialized state\n";
}
'''
with tempfile.TemporaryDirectory(prefix='can-recovery-') as directory:
    path = Path(directory)
    fragment = ('#include "bsp_can.hpp"\n\n'
                + extract('bool BspCan::_tx_available() const')
                + '\n'
                + extract('Status BspCan::service_recovery()'))
    (path / 'bsp_can.hpp').write_text(header)
    (path / 'recovery.cpp').write_text(fragment)
    (path / 'test.cpp').write_text(test)
    subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    str(path / 'recovery.cpp'), str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)

#pragma once
#include <Arduino.h>
#include "../modem/modem_types.h"

// 来电事件处理器。
// 线程说明：handleRING/handleCLIP 在 SIM reader 任务上运行；tick() 在 loop() 上运行。
// ESP32-C3 单核 + 全部采用 millis() 超时比较 → 无需显式互斥锁。
// 业务逻辑：
//   1) RING 到达 → 记录起始时间，启动 CLIP 等待窗口
//   2) +CLIP 或未经请求的 +CLCC 到达 → 提取号码，取消等待窗口后推送
//   3) 等待超时 → 主动 AT+CLCC 查询底代；仍无果则以 "未知号码" 推送
//   4) 通话结束（NO CARRIER 等）→ 立即用已知号码推送，不再等满窗口
//   5) DEDUP_MS 窗口内同号重复来电只推送一次
//
// 实测本模组（ML307A + 中国电信）不发 +CLIP:，而是主动报 +CLCC:，因此第 2 步的
// +CLCC 分支不是冗余保险，而是这块硬件上唯一的即时号码来源。
class Call {
public:
  // 同号码去重窗口（毫秒），避免运营商重拨重复推送。
  static constexpr unsigned long DEDUP_MS      = 10000;
  // RING 后等待 +CLIP 的最大时长（毫秒），超时后以 "未知号码" 推送。
  static constexpr unsigned long CLIP_WAIT_MS  = 10000;
  // RING 后多久主动发送 AT+CLCC 查询来电详情（毫秒）。
  static constexpr unsigned long CLCC_DELAY_MS = 3000;

  static void init();

  // SIM reader 任务检测到 RING URC 时调用。
  static void handleRING(ModemId modemId);
  static void handleRING() { handleRING(MODEM_PRIMARY); }

  // SIM reader 任务检测到 +CLIP URC 时调用。
  static void handleCLIP(ModemId modemId, const String& line);

  // SIM reader 任务检测到未经请求的 +CLCC URC 时调用。
  static void handleCLCC(ModemId modemId, const String& line);

  // SIM reader 任务检测到通话结束上报时调用。
  static void handleCallEnd(ModemId modemId, const String& line);

  // 每轮 loop() 调用：处理 CLIP 超时、推送派发、AT+CLCC 底代查询。
  static void tick(ModemId modemId);
  static void tick() { tick(MODEM_PRIMARY); }

private:
  static volatile bool          s_pending[MODEM_COUNT];
  static volatile bool          s_dispatchPending[MODEM_COUNT];
  static String                 s_callerNumber[MODEM_COUNT];
  static volatile unsigned long s_clipWaitUntilMs[MODEM_COUNT];
  static unsigned long          s_lastNotifyMs[MODEM_COUNT];
  static bool                   s_clccAttempted[MODEM_COUNT];
  static unsigned long          s_clccAttemptMs[MODEM_COUNT];

  static void   dispatch(ModemId modemId, const String& callerNum);
  static String parseCLCC(const String& resp);        // 从 +CLCC 响应中提取主叫号
};

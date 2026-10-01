#include "call.h"
#include "push/push.h"
#include "sms/phone_utils.h"
#include "sim/sim_dispatcher.h"
#include "call_events.h"
#include "time/time_sync.h"
#include "../logger/logger.h"

volatile bool          Call::s_pending[MODEM_COUNT]          = {false, false};
volatile bool          Call::s_dispatchPending[MODEM_COUNT] = {false, false};
String                 Call::s_callerNumber[MODEM_COUNT]     = {"未知号码", "未知号码"};
volatile unsigned long Call::s_clipWaitUntilMs[MODEM_COUNT]  = {0, 0};
unsigned long          Call::s_lastNotifyMs[MODEM_COUNT]     = {0, 0};
bool                   Call::s_clccAttempted[MODEM_COUNT]    = {false, false};
unsigned long          Call::s_clccAttemptMs[MODEM_COUNT]    = {0, 0};

void Call::dispatch(ModemId modemId, const String& callerNum) {
  if (phoneMatchesBlacklist(callerNum)) {
    LOG("CALL", "黑名单拦截来电，号码: %s", callerNum.c_str());
    return;
  }
  // 记录在推送之前、黑名单之后:与固件自身的推送严格同源,不会出现「推送了但外部
  // 系统查不到」或反之。黑名单拦截的来电两边都不出现。
  CallEvents::record(callerNum);
  String ts = TimeSync::dateStr();
  Push::send(callerNum, "来电号码: " + callerNum + "\n时间: " + ts, ts, MsgTypeInfo(MSG_TYPE_CALL), modemId);
  s_lastNotifyMs[modemId] = millis();
  LOG("CALL", "来电通知已发送，号码: %s", callerNum.c_str());
}

void Call::init() {
  CallEvents::init();
  for (ModemId i = 0; i < MODEM_COUNT; i++) {
    s_pending[i]         = false;
    s_dispatchPending[i] = false;
    s_callerNumber[i]    = "未知号码";
    s_clipWaitUntilMs[i] = 0;
    s_lastNotifyMs[i]    = 0;
    s_clccAttempted[i]   = false;
    s_clccAttemptMs[i]   = 0;
  }
}

void Call::handleRING(ModemId modemId) {
  if (millis() - s_lastNotifyMs[modemId] < DEDUP_MS) {
    LOG("CALL", "防抖：忽略 RING（%lu ms 内已通知）", DEDUP_MS);
    return;
  }
  s_pending[modemId]         = true;
  s_dispatchPending[modemId] = false;
  s_callerNumber[modemId]    = "未知号码";
  s_clipWaitUntilMs[modemId] = millis() + CLIP_WAIT_MS;
  s_clccAttempted[modemId]   = false;
  s_clccAttemptMs[modemId]   = millis() + CLCC_DELAY_MS;
  LOG("CALL", "RING 检测，等待 +CLIP（%lu ms），%lu ms 后主动查询 AT+CLCC", CLIP_WAIT_MS, CLCC_DELAY_MS);
}

void Call::handleCLIP(ModemId modemId, const String& line) {
  String num;
  int q1 = line.indexOf('"');
  int q2 = (q1 >= 0) ? line.indexOf('"', q1 + 1) : -1;
  if (q1 >= 0 && q2 > q1) {
    num = line.substring(q1 + 1, q2);
  }

  // +CLIP: 在没有先导 RING 时曾被整条丢弃,于是一次响铃指示没被认出就丢掉整通来电。
  // 而 +CLIP: 本身已经带齐了一条通知需要的全部信息(谁、什么时候),所以这里把它
  // 当作来电本身,而不是丢掉。
  //
  // 模组每个响铃周期都会重发 RING 与 +CLIP:,所以必须套用与 RING 相同的去抖窗口,
  // 否则一通电话会变成好几条记录。
  if (!s_pending[modemId]) {
    if (millis() - s_lastNotifyMs[modemId] < DEDUP_MS) {
      return;
    }
    s_callerNumber[modemId]    = num.length() > 0 ? num : String("号码保密");
    s_dispatchPending[modemId] = true;
    LOG("CALL", "收到 SIM%u +CLIP 但此前无 RING，按来电处理，号码: %s", modemId + 1, s_callerNumber[modemId].c_str());
    return;
  }

  if (num.length() > 0 || s_callerNumber[modemId].length() == 0) {
    s_callerNumber[modemId] = num.length() > 0 ? num : String("号码保密");
  }
  s_pending[modemId]         = false;
  s_dispatchPending[modemId] = true;
}

// handleCLCC 接收未经请求的 +CLCC:。
//
// 实测本模组不发 +CLIP:,而是在 RING 之后主动报一条 +CLCC:,所以这里是这块硬件上
// 唯一的即时号码来源 —— 没有它就只能等 3 秒后自己去查,短促来电会查空。
void Call::handleCLCC(ModemId modemId, const String& line) {
  String num = parseCLCC(line);
  if (num.length() == 0) {
    return;
  }
  // 与 handleCLIP 同样的去抖:模组每个响铃周期都会重发,一通电话不能变成好几条。
  if (!s_pending[modemId]) {
    if (millis() - s_lastNotifyMs[modemId] < DEDUP_MS) {
      return;
    }
    s_callerNumber[modemId]    = num;
    s_dispatchPending[modemId] = true;
    LOG("CALL", "收到主动 +CLCC 但此前无 RING，按来电处理，号码: %s", num.c_str());
    return;
  }
  s_callerNumber[modemId]    = num;
  s_pending[modemId]         = false;
  s_dispatchPending[modemId] = true;
  LOG("CALL", "从主动 +CLCC 获取来电号码: %s", num.c_str());
}

// handleCallEnd 在通话结束上报到达时收口。
//
// 没有它,一通两秒就挂断的来电要等满 10 秒窗口才被记录 —— 号码其实早就拿到了。
// 已经有号码就立即派发;没有的话也不再等,主叫方已经走了,再等也不会有 +CLCC。
void Call::handleCallEnd(ModemId modemId, const String& line) {
  if (!s_pending[modemId]) {
    return;
  }
  s_pending[modemId]         = false;
  s_dispatchPending[modemId] = true;
  LOG("CALL", "SIM%u通话结束（%s），立即以已知号码派发: %s", modemId + 1, line.c_str(), s_callerNumber[modemId].c_str());
}

String Call::parseCLCC(const String& resp) {
  int pos = 0;
  while (true) {
    int clccIdx = resp.indexOf("+CLCC:", pos);
    if (clccIdx < 0) {
      break;
    }
    int commaAfterIdx = resp.indexOf(',', clccIdx + 6);
    if (commaAfterIdx < 0) {
      break;
    }
    int commaAfterDir = resp.indexOf(',', commaAfterIdx + 1);
    if (commaAfterDir < 0) {
      break;
    }
    int q1 = resp.indexOf('"', commaAfterDir);
    int q2 = (q1 >= 0) ? resp.indexOf('"', q1 + 1) : -1;
    if (q1 >= 0 && q2 > q1) {
      String num = resp.substring(q1 + 1, q2);
      if (num.length() > 0) {
        return num;
      }
    }
    pos = clccIdx + 6;
  }
  return "";
}

void Call::tick(ModemId modemId) {
  if (s_dispatchPending[modemId]) {
    s_dispatchPending[modemId] = false;
    dispatch(modemId, s_callerNumber[modemId]);
    return;
  }
  if (!s_pending[modemId]) {
    return;
  }
  unsigned long now = millis();

  if (!s_clccAttempted[modemId] && now >= s_clccAttemptMs[modemId]) {
    s_clccAttempted[modemId] = true;
    LOG("CALL", "主动发送 AT+CLCC 查询来电号码");
    String resp;
    SimDispatcher::sendCommand(modemId, "AT+CLCC", 3000, &resp, true);
    LOG("CALL", "AT+CLCC 响应: %s", resp.c_str());
    String num = parseCLCC(resp);
    if (num.length() > 0) {
      LOG("CALL", "AT+CLCC 成功获取来电号码: %s", num.c_str());
      s_callerNumber[modemId] = num;
      s_pending[modemId] = false;
      dispatch(modemId, s_callerNumber[modemId]);
      return;
    }
    LOG("CALL", "AT+CLCC 未获取到号码（可能已挂断或格式不符），继续等待 +CLIP");
  }

  if (now >= s_clipWaitUntilMs[modemId]) {
    s_pending[modemId] = false;
    dispatch(modemId, s_callerNumber[modemId]);
  }
}

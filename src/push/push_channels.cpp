#include "push_channels.h"
#include "../logger/logger.h"
#include "sms/sms.h"
#include "../sim/sim_dispatcher.h"
#include "../utils/http.h"
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <mbedtls/md.h>
#include <base64.h>

// ---------- helpers ----------

static String urlEncode(const String& str) {
  String encoded;
  for (unsigned int i = 0; i < str.length(); i++) {
    unsigned char c = (unsigned char)str.charAt(i);
    if (c == ' ') {
      encoded += '+';
    } else if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += (char)c;
    } else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", c);
      encoded += buf;
    }
  }
  return encoded;
}

static String computeHmacSha256Base64(const String& key, const String& data) {
  uint8_t hmacResult[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key.c_str(), key.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)data.c_str(), data.length());
  mbedtls_md_hmac_finish(&ctx, hmacResult);
  mbedtls_md_free(&ctx);
  return base64::encode(hmacResult, 32);
}

static String dingtalkSign(const String& secret, int64_t timestamp) {
  return urlEncode(computeHmacSha256Base64(secret, String(timestamp) + "\n" + secret));
}

static int64_t getUtcMillis() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) == 0) {
    return (int64_t)tv.tv_sec * 1000LL + tv.tv_usec / 1000;
  }
  return (int64_t)time(nullptr) * 1000LL;
}

// 上次 HTTP 请求完成的时刻（millis()）；0 表示尚未发送过任何请求。
static unsigned long s_lastHttpEndMs = 0;

// 等待冷却间隔后创建 HttpSession，给 TCP/TLS 栈足够的资源释放时间。
// 失败（连接无法建立）时返回 nullptr。
static std::unique_ptr<HttpSession> request(const String& url) {
  unsigned long now = millis();
  if (s_lastHttpEndMs > 0 && now - s_lastHttpEndMs < HTTP_COOLDOWN_MS) {
    delay(HTTP_COOLDOWN_MS - (now - s_lastHttpEndMs));
  }
  return std::unique_ptr<HttpSession>(HttpSession::request(url));
}

// 记录响应结果并更新冷却时间戳；失败时额外打印首 120 字节响应 body，便于定位原因。
static bool isResponseSuccessful(HttpSession* session, int code) {
  if (code >= 200 && code < 300) {
    LOG("PUSHCH", "响应码: %d 成功", code);
  } else {
    String resp = session->http()->getString();
    if (resp.length() > 120) resp = resp.substring(0, 120) + "...";
    LOG("PUSHCH", "响应码: %d 失败，响应体: %s", code, resp.c_str());
  }
  s_lastHttpEndMs = millis();
  return code >= 200 && code < 300;
}

// ---------- ML307 4G HTTP transport ----------

static bool isCellularHttpError(const String& line) {
  return line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR");
}

static const int CELLULAR_HTTP_INSTANCE_ID = 0;
// 4G HTTP 是 WiFi 不可用时的旁路能力，不能让一次网络异常长时间阻塞主循环。
// 该时限覆盖数据上下文、reader 独占和 MHTTP 事务；超时后仍会执行短时清理。
static const unsigned long CELLULAR_HTTP_TOTAL_TIMEOUT_MS = 15000;
static const unsigned long CELLULAR_HTTP_COMMAND_TIMEOUT_MS = 3000;
static const unsigned long CELLULAR_HTTP_PAYLOAD_TIMEOUT_MS = 5000;
static const unsigned long CELLULAR_HTTP_REQUEST_TIMEOUT_MS = 10000;
static const unsigned long CELLULAR_HTTP_CLEANUP_TIMEOUT_MS = 2000;
static const unsigned long CELLULAR_HTTP_PAUSE_TIMEOUT_MS = 3000;

static unsigned long remainingCellularTimeout(unsigned long deadlineMs, unsigned long maxMs) {
  long remaining = (long)(deadlineMs - millis());
  if (remaining <= 0) return 0;
  return (unsigned long)remaining < maxMs ? (unsigned long)remaining : maxMs;
}

static String encodeMhttpPath(const String& value) {
  static const char hex[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 2);
  for (unsigned int i = 0; i < value.length(); i++) {
    unsigned char c = (unsigned char)value.charAt(i);
    encoded += hex[(c >> 4) & 0x0F];
    encoded += hex[c & 0x0F];
  }
  return encoded;
}

static bool splitMhttpUrl(const String& url, String* protocol, String* authority, String* path) {
  int schemeEnd = url.indexOf("://");
  if (schemeEnd <= 0) return false;

  *protocol = url.substring(0, schemeEnd);
  protocol->toLowerCase();
  if (*protocol != "http" && *protocol != "https") return false;

  int authorityStart = schemeEnd + 3;
  int pathStart = url.indexOf('/', authorityStart);
  int queryStart = url.indexOf('?', authorityStart);
  if (pathStart < 0 || (queryStart >= 0 && queryStart < pathStart)) pathStart = queryStart;

  if (pathStart < 0) {
    *authority = url.substring(authorityStart);
    *path = "/";
  } else {
    *authority = url.substring(authorityStart, pathStart);
    *path = url.substring(pathStart);
    if ((*path).charAt(0) == '?') *path = "/" + *path;
  }
  return authority->length() > 0 && path->length() > 0;
}

// Reader 暂停期间仍要把已经进入 UART FIFO 的短信/来电 URC 转交给业务层，不能直接
// 清空，否则 4G HTTP 请求恰好与短信到达重叠时会丢消息。
static void drainCellularHttpInput(ModemId modemId) {
  HardwareSerial& serial = SimDispatcher::serial(modemId);
  String lineBuf;
  unsigned long lastDataMs = millis();
  while (millis() - lastDataMs < 20) {
    bool received = false;
    while (serial.available()) {
      received = true;
      lastDataMs = millis();
      char c = (char)serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        lineBuf.trim();
        if (lineBuf.length() > 0) SimDispatcher::routeIfUrc(modemId, lineBuf);
        lineBuf = "";
      } else {
        lineBuf += c;
        if (lineBuf.length() > SIM_LINE_BUF_MAX) lineBuf = "";
      }
    }
    if (!received) delay(1);
  }
}

// MHTTP 命令都在同一个独占事务中执行，避免异步 +MHTTPURC 被普通 reader
// 当成下一条 AT 命令的响应。createdId 仅用于 MHTTPCREATE，其他命令传 nullptr。
static bool sendMhttpCommandLocked(ModemId modemId, const String& command,
                                    unsigned long timeoutMs, int* createdId = nullptr,
                                    bool logFailure = true, unsigned long deadlineMs = 0,
                                    String* outResp = nullptr) {
  HardwareSerial& serial = SimDispatcher::serial(modemId);

  bool ok = false;
  bool terminal = false;
  bool createSeen = false;
  unsigned long okAt = 0;
  String lineBuf;
  unsigned long start = millis();
  unsigned long effectiveTimeout = timeoutMs;
  if (deadlineMs != 0) {
    effectiveTimeout = remainingCellularTimeout(deadlineMs, timeoutMs);
    if (effectiveTimeout == 0) {
      if (logFailure) LOG("PUSH4G", "SIM%u ML307命令因总时限到期而丢弃: %s", modemId + 1, command.c_str());
      return false;
    }
  }
  serial.println(command);
  while (millis() - start < effectiveTimeout && !terminal) {
    esp_task_wdt_reset();
    while (serial.available()) {
      char c = (char)serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        lineBuf.trim();
        if (lineBuf.length() > 0) {
          if (SimDispatcher::routeIfUrc(modemId, lineBuf)) {
            lineBuf = "";
            continue;
          }
          if (outResp != nullptr) {
            *outResp += lineBuf;
            *outResp += '\n';
          }
          if (lineBuf.startsWith("+MHTTPCREATE:")) {
            int id = -1;
            if (sscanf(lineBuf.c_str(), "+MHTTPCREATE: %d", &id) == 1 && id >= 0) {
              createSeen = true;
              if (createdId != nullptr) *createdId = id;
              if (ok) terminal = true;
            }
          } else if (lineBuf == "OK") {
            ok = true;
            okAt = millis();
            if (createdId == nullptr || createSeen) terminal = true;
          } else if (isCellularHttpError(lineBuf)) {
            terminal = true;
          }
        }
        lineBuf = "";
      } else {
        lineBuf += c;
        if (lineBuf.length() > 4096) lineBuf = "";
      }
      if (terminal) break;
    }
    if (!terminal && ok && createdId != nullptr && !createSeen && millis() - okAt >= 200) {
      terminal = true;
    }
    if (!terminal) delay(5);
  }

  if (!terminal && logFailure) {
    LOG("PUSH4G", "SIM%u ML307命令超时: %s", modemId + 1, command.c_str());
  } else if (!ok && logFailure) {
    LOG("PUSH4G", "SIM%u ML307命令失败: %s", modemId + 1, command.c_str());
  }
  return terminal && ok;
}

// AT+MHTTPCONTENT 是“命令 → > 提示符 → 原始数据”的交互，不能通过普通
// sendCommand() 完成。调用方必须已经暂停 reader，且结束后由调用方恢复。
static bool sendMhttpPayloadLocked(ModemId modemId, const String& command, const String& payload,
                                   unsigned long deadlineMs = 0) {
  HardwareSerial& serial = SimDispatcher::serial(modemId);

  bool gotPrompt = false;
  bool earlyError = false;
  String lineBuf;
  unsigned long start = millis();
  unsigned long promptTimeout = deadlineMs == 0
                              ? CELLULAR_HTTP_COMMAND_TIMEOUT_MS
                              : remainingCellularTimeout(deadlineMs, CELLULAR_HTTP_COMMAND_TIMEOUT_MS);
  if (promptTimeout == 0) {
    LOG("PUSH4G", "SIM%u MHTTP 数据写入因总时限到期而丢弃", modemId + 1);
    return false;
  }
  serial.println(command);
  while (millis() - start < promptTimeout && !gotPrompt && !earlyError) {
    esp_task_wdt_reset();
    while (serial.available()) {
      char c = (char)serial.read();
      if (c == '>') {
        gotPrompt = true;
        break;
      }
      if (c == '\r') continue;
      if (c == '\n') {
        lineBuf.trim();
        if (lineBuf.length() > 0) {
          if (SimDispatcher::routeIfUrc(modemId, lineBuf)) {
            lineBuf = "";
            continue;
          }
          if (isCellularHttpError(lineBuf)) earlyError = true;
        }
        lineBuf = "";
      } else {
        lineBuf += c;
        if (lineBuf.length() > SIM_LINE_BUF_MAX) lineBuf = "";
      }
      if (gotPrompt || earlyError) break;
    }
    if (!gotPrompt && !earlyError) delay(5);
  }

  if (!gotPrompt) {
    LOG("PUSH4G", "SIM%u 未收到 MHTTP 数据提示符%s", modemId + 1,
        earlyError ? "（模组返回错误）" : "（超时）");
    return false;
  }

  serial.print(payload);
  serial.flush();

  bool ok = false;
  bool terminal = false;
  lineBuf = "";
  start = millis();
  unsigned long resultTimeout = deadlineMs == 0
                              ? CELLULAR_HTTP_PAYLOAD_TIMEOUT_MS
                              : remainingCellularTimeout(deadlineMs, CELLULAR_HTTP_PAYLOAD_TIMEOUT_MS);
  while (millis() - start < resultTimeout && !terminal) {
    esp_task_wdt_reset();
    while (serial.available()) {
      char c = (char)serial.read();
      if (c == '\r') continue;
      if (c != '\n') {
        lineBuf += c;
        if (lineBuf.length() > SIM_LINE_BUF_MAX) lineBuf = "";
        continue;
      }
      String line = lineBuf;
      lineBuf = "";
      line.trim();
      if (line.length() == 0) continue;
      if (SimDispatcher::routeIfUrc(modemId, line)) continue;
      if (line == "OK") {
        ok = true;
        terminal = true;
      } else if (isCellularHttpError(line)) {
        terminal = true;
      }
    }
    if (!terminal) delay(5);
  }
  if (!ok) LOG("PUSH4G", "SIM%u MHTTP 数据写入失败或超时", modemId + 1);
  return ok;
}

static bool runMhttpRequestLocked(ModemId modemId, const String& command, int httpId, int* httpCode,
                                  unsigned long deadlineMs = 0) {
  HardwareSerial& serial = SimDispatcher::serial(modemId);

  bool commandAck = false;
  bool terminal = false;
  bool headerSeen = false;
  int statusCode = -1;
  int errorCode = -1;
  String lineBuf;
  unsigned long start = millis();
  unsigned long requestTimeout = deadlineMs == 0
                              ? CELLULAR_HTTP_REQUEST_TIMEOUT_MS
                              : remainingCellularTimeout(deadlineMs, CELLULAR_HTTP_REQUEST_TIMEOUT_MS);
  if (requestTimeout == 0) {
    LOG("PUSH4G", "SIM%u MHTTP 请求因总时限到期而丢弃", modemId + 1);
    return false;
  }
  serial.println(command);
  while (millis() - start < requestTimeout && !terminal) {
    esp_task_wdt_reset();
    while (serial.available()) {
      char c = (char)serial.read();
      if (c == '\r') continue;
      if (c != '\n') {
        lineBuf += c;
        // MHTTP header URC may contain a long response header. Keep the
        // complete line so the status code near its beginning is not lost.
        if (lineBuf.length() > 4096) lineBuf = "";
        continue;
      }
      String line = lineBuf;
      lineBuf = "";
      line.trim();
      if (line.length() == 0) continue;
      if (line.startsWith("+MHTTPURC: \"header\"")) {
        int responseId = -1;
        int responseLength = 0;
        if (sscanf(line.c_str(), "+MHTTPURC: \"header\",%d,%d,%d", &responseId,
                   &statusCode, &responseLength) >= 3 && responseId == httpId) {
          headerSeen = true;
          // 204/304 responses have no response body. For other statuses,
          // keep reading until the content URC is fully drained before
          // deleting the MHTTP instance.
          if (statusCode == 204 || statusCode == 304) terminal = true;
        }
      } else if (line.startsWith("+MHTTPURC: \"content\"")) {
        int responseId = -1;
        int contentLength = -1;
        int sumLength = 0;
        int currentLength = 0;
        if (sscanf(line.c_str(), "+MHTTPURC: \"content\",%d,%d,%d,%d", &responseId,
                   &contentLength, &sumLength, &currentLength) >= 4 && responseId == httpId) {
          // 本实现未开启 response chunked 模式；非分块响应必须等累计长度
          // 达到 contentLength 后才能删除 MHTTP 实例。不能仅凭 currentLength
          // 为 0 判断结束，否则会提前关闭仍在发送正文的会话。
          if (contentLength <= 0 || sumLength >= contentLength) {
            terminal = true;
          }
        }
      } else if (line.startsWith("+MHTTPURC: \"err\"")) {
        int responseId = -1;
        if (sscanf(line.c_str(), "+MHTTPURC: \"err\",%d,%d", &responseId, &errorCode) >= 2 &&
            responseId == httpId) {
          terminal = true;
        }
      } else if (line == "OK") {
        commandAck = true;
      } else if (isCellularHttpError(line)) {
        terminal = true;
      } else if (SimDispatcher::routeIfUrc(modemId, line)) {
        continue;
      }
    }

    if (!terminal) delay(5);
  }

  if (httpCode != nullptr) *httpCode = statusCode;
  if (statusCode < 0 || !headerSeen || !terminal) {
    LOG("PUSH4G", "SIM%u 未收到完整 MHTTP 响应（命令确认=%s，错误码=%d）", modemId + 1,
        commandAck ? "是" : "否", errorCode);
    return false;
  }
  LOG("PUSH4G", "SIM%u MHTTP %s 响应码: %d", modemId + 1,
      statusCode >= 200 && statusCode < 300 ? "成功" : "失败", statusCode);
  return statusCode >= 200 && statusCode < 300;
}

static bool activateCellularDataLocked(ModemId modemId, unsigned long deadlineMs) {
  unsigned long activateTimeout = remainingCellularTimeout(deadlineMs, 8000);
  if (activateTimeout > 0 && sendMhttpCommandLocked(modemId, "AT+CGACT=1,1", activateTimeout,
                                                     nullptr, false, deadlineMs)) {
    return true;
  }
  String resp;
  unsigned long queryTimeout = remainingCellularTimeout(deadlineMs, 2000);
  if (queryTimeout > 0 && sendMhttpCommandLocked(modemId, "AT+CGACT?", queryTimeout,
                                                 nullptr, false, deadlineMs, &resp) &&
      resp.indexOf("+CGACT: 1,1") >= 0) {
    LOG("PUSH4G", "SIM%u 数据上下文已处于激活状态", modemId + 1);
    return true;
  }
  LOG("PUSH4G", "SIM%u 数据上下文激活失败", modemId + 1);
  return false;
}

static bool sendCellularHttpRequest(ModemId modemId, int method, const String& url,
                                    const String& contentType, const String& body) {
  if (!SimDispatcher::running(modemId)) {
    LOG("PUSH4G", "SIM%u 通讯任务未启动，跳过 4G HTTP", modemId + 1);
    return false;
  }
  if (url.length() == 0 || url.indexOf('"') >= 0 || url.indexOf('\r') >= 0 || url.indexOf('\n') >= 0) {
    LOG("PUSH4G", "SIM%u HTTP URL 无效或包含非法字符", modemId + 1);
    return false;
  }
  unsigned long deadlineMs = millis() + CELLULAR_HTTP_TOTAL_TIMEOUT_MS;

  String protocol;
  String authority;
  String path;
  if (!splitMhttpUrl(url, &protocol, &authority, &path)) {
    LOG("PUSH4G", "SIM%u 不支持的 HTTP URL: %s", modemId + 1, url.c_str());
    return false;
  }

  unsigned long pauseTimeout = remainingCellularTimeout(deadlineMs, CELLULAR_HTTP_PAUSE_TIMEOUT_MS);
  if (pauseTimeout == 0 || !SimDispatcher::pauseReader(modemId, pauseTimeout)) {
    LOG("PUSH4G", "SIM%u reader 忙，无法执行 ML307 HTTP", modemId + 1);
    return false;
  }

  bool ok = false;
  int httpId = CELLULAR_HTTP_INSTANCE_ID;
  drainCellularHttpInput(modemId);

  auto cleanupMhttp = [&]() {
    bool cleanupOk = sendMhttpCommandLocked(modemId, String("AT+MHTTPDEL=") + httpId,
                                             CELLULAR_HTTP_CLEANUP_TIMEOUT_MS, nullptr, false);
    if (!cleanupOk) {
      LOG("PUSH4G", "SIM%u MHTTP 实例清理失败，已继续恢复 reader", modemId + 1);
    }
    // 清掉清理命令之后已经进入 FIFO 的残留字节；延迟 MHTTPURC 即使在这之后
    // 到达，也会由 SimDispatcher 按 URC 隔离，不会污染后续 AT 响应。
    drainCellularHttpInput(modemId);
    SimDispatcher::resumeReader(modemId);
  };

  if (!activateCellularDataLocked(modemId, deadlineMs)) {
    LOG("PUSH4G", "SIM%u 数据上下文激活失败，丢弃本次 4G 推送", modemId + 1);
    cleanupMhttp();
    return false;
  }

  // ML307 使用 MHTTP 接口，不是 SIMCom 的 HTTPINIT/HTTPACTION 接口。
  // 先删除固定实例，避免上一次异常中断留下活动实例导致 MHTTPCREATE 失败。
  sendMhttpCommandLocked(modemId, String("AT+MHTTPDEL=") + CELLULAR_HTTP_INSTANCE_ID,
                         CELLULAR_HTTP_CLEANUP_TIMEOUT_MS, nullptr, false);

  String createCmd = "AT+MHTTPCREATE=\"" + protocol + "://" + authority + "\"";
  if (!sendMhttpCommandLocked(modemId, createCmd, CELLULAR_HTTP_COMMAND_TIMEOUT_MS, &httpId,
                              true, deadlineMs)) {
    LOG("PUSH4G", "SIM%u MHTTP 实例创建失败", modemId + 1);
    cleanupMhttp();
    return false;
  }

  String prefix = String("AT+MHTTPCFG=\"encoding\",") + httpId + ",0,0";
  if (protocol == "https") {
    ok = sendMhttpCommandLocked(modemId, String("AT+MHTTPCFG=\"ssl\",") + httpId + ",1,0",
                                CELLULAR_HTTP_COMMAND_TIMEOUT_MS, nullptr, true, deadlineMs);
  }
  if (ok || protocol == "http") {
    ok = sendMhttpCommandLocked(modemId, prefix, CELLULAR_HTTP_COMMAND_TIMEOUT_MS,
                                nullptr, true, deadlineMs);
  }

  if (ok && method == 1) {
    String header = "Content-Type: " + contentType;
    if (header.indexOf('"') >= 0 || header.indexOf('\r') >= 0 || header.indexOf('\n') >= 0) {
      ok = false;
    } else {
      String headerCmd = String("AT+MHTTPHEADER=") + httpId + ",0," + header.length() + ",\"" + header + "\"";
      ok = sendMhttpCommandLocked(modemId, headerCmd, CELLULAR_HTTP_COMMAND_TIMEOUT_MS,
                                  nullptr, true, deadlineMs);
    }
    if (ok && body.length() > 0) {
      String dataCmd = String("AT+MHTTPCONTENT=") + httpId + ",0," + body.length();
      ok = sendMhttpPayloadLocked(modemId, dataCmd, body, deadlineMs);
    }
  }

  // MHTTPCONTENT 使用明文写入；发送请求前必须恢复 HEX 编码，
  // 否则 MHTTPREQUEST 的路径和查询参数会被模组按错误格式解析。
  if (ok) {
    String encodingOn = String("AT+MHTTPCFG=\"encoding\",") + httpId + ",1,1";
    ok = sendMhttpCommandLocked(modemId, encodingOn, CELLULAR_HTTP_COMMAND_TIMEOUT_MS,
                                nullptr, true, deadlineMs);
  }

  int httpCode = -1;
  if (ok) {
    int mhttpMethod = method == 0 ? 1 : 2;  // ML307: GET=1, POST=2
    String requestCmd = String("AT+MHTTPREQUEST=") + httpId + "," + mhttpMethod + ",0," + encodeMhttpPath(path);
    ok = runMhttpRequestLocked(modemId, requestCmd, httpId, &httpCode, deadlineMs);
  }

  cleanupMhttp();
  return ok;
}

static bool buildCellularHttpRequest(const PushChannel& ch, const String& sender,
                                    const PushBody& message, const String& timestamp,
                                    int* method, String* url, String* contentType, String* body) {
  *method = 1;
  *url = ch.url;
  *contentType = "application/json";
  body->remove(0);

  switch (ch.type) {
    case PUSH_TYPE_POST_JSON: {
      if (message.type == PUSH_BODY_CUSTOM) {
        *body = message.content;
      } else {
        JsonDocument doc;
        doc["sender"] = sender;
        doc["message"] = message.content;
        doc["timestamp"] = timestamp;
        serializeJson(doc, *body);
      }
      return true;
    }
    case PUSH_TYPE_BARK: {
      JsonDocument doc;
      doc["title"] = ch.key1.length() > 0 ? ch.key1 : sender;
      doc["body"] = message.content;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_GET:
      *method = 0;
      *contentType = "";
      *url += (*url).indexOf('?') == -1 ? "?" : "&";
      *url += "sender=" + urlEncode(sender);
      *url += "&message=" + urlEncode(message.content);
      *url += "&timestamp=" + urlEncode(timestamp);
      return true;
    case PUSH_TYPE_DINGTALK: {
      if (ch.key1.length() > 0) {
        int64_t ts = getUtcMillis();
        char tsBuf[21];
        snprintf(tsBuf, sizeof(tsBuf), "%lld", ts);
        *url += (*url).indexOf('?') == -1 ? "?" : "&";
        *url += "timestamp=" + String(tsBuf) + "&sign=" + dingtalkSign(ch.key1, ts);
      }
      String content = message.type == PUSH_BODY_CUSTOM
                     ? message.content
                     : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
      JsonDocument doc;
      doc["msgtype"] = "text";
      doc["text"]["content"] = content;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_PUSHPLUS: {
      if (ch.url.length() == 0) *url = "http://www.pushplus.plus/send";
      String channelValue = "wechat";
      if (ch.key2 == "wechat" || ch.key2 == "extension" || ch.key2 == "app") channelValue = ch.key2;
      String content = message.type == PUSH_BODY_CUSTOM
                     ? message.content
                     : ("<b>发送者:</b> " + sender + "<br><b>时间:</b> " + timestamp + "<br><b>内容:</b><br>" + message.content);
      JsonDocument doc;
      doc["token"] = ch.key1;
      doc["title"] = "短信来自: " + sender;
      doc["content"] = content;
      doc["channel"] = channelValue;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_SERVERCHAN: {
      if (ch.url.length() == 0) *url = "https://sctapi.ftqq.com/" + ch.key1 + ".send";
      *contentType = "application/x-www-form-urlencoded";
      String desp = message.type == PUSH_BODY_CUSTOM
                  ? message.content
                  : ("**发送者:** " + sender + "\n\n**时间:** " + timestamp + "\n\n**内容:**\n\n" + message.content);
      *body = "title=" + urlEncode("短信来自: " + sender) + "&desp=" + urlEncode(desp);
      return true;
    }
    case PUSH_TYPE_CUSTOM:
      *body = message.content;
      return true;
    case PUSH_TYPE_FEISHU: {
      JsonDocument doc;
      if (ch.key1.length() > 0) {
        int64_t ts = time(nullptr);
        doc["timestamp"] = String(ts);
        doc["sign"] = computeHmacSha256Base64(ch.key1, String(ts) + "\n" + ch.key1);
      }
      String text = message.type == PUSH_BODY_CUSTOM
                  ? message.content
                  : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
      doc["msg_type"] = "text";
      doc["content"]["text"] = text;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_GOTIFY: {
      if (!(*url).endsWith("/")) *url += "/";
      *url += "message?token=" + ch.key1;
      String msg = message.type == PUSH_BODY_CUSTOM ? message.content : (message.content + "\n\n时间: " + timestamp);
      JsonDocument doc;
      doc["title"] = "短信来自: " + sender;
      doc["message"] = msg;
      doc["priority"] = 5;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_TELEGRAM: {
      if (url->length() == 0) *url = "https://api.telegram.org";
      if (url->endsWith("/")) url->remove(url->length() - 1);
      *url += "/bot" + ch.key2 + "/sendMessage";
      String text = message.type == PUSH_BODY_CUSTOM
                  ? message.content
                  : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
      JsonDocument doc;
      doc["chat_id"] = ch.key1;
      doc["text"] = text;
      serializeJson(doc, *body);
      return true;
    }
    case PUSH_TYPE_WECHAT_WORK: {
      if (ch.key1.length() > 0) {
        int64_t ts = getUtcMillis();
        char tsBuf[21];
        snprintf(tsBuf, sizeof(tsBuf), "%lld", ts);
        *url += (*url).indexOf('?') == -1 ? "?" : "&";
        *url += "timestamp=" + String(tsBuf) + "&sign=" + urlEncode(computeHmacSha256Base64(ch.key1, String(tsBuf) + "\n" + ch.key1));
      }
      String content = message.type == PUSH_BODY_CUSTOM
                     ? message.content
                     : ("📱短信通知\n发件人: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
      JsonDocument doc;
      doc["msgtype"] = "text";
      doc["text"]["content"] = content;
      serializeJson(doc, *body);
      return true;
    }
    default:
      return false;
  }
}

bool PushChannels::sendCellularHttp(const PushChannel& ch, const String& sender,
                                    const PushBody& message, const String& timestamp,
                                    ModemId modemId) {
  int method = 1;
  String url;
  String contentType;
  String body;
  if (!buildCellularHttpRequest(ch, sender, message, timestamp, &method, &url, &contentType, &body)) {
    return false;
  }
  LOG("PUSH4G", "SIM%u 发送 4G HTTP 通道: %s，方法=%s，body=%u 字节",
      modemId + 1, ch.name.c_str(), method == 0 ? "GET" : "POST", (unsigned)body.length());
  return sendCellularHttpRequest(modemId, method, url, contentType, body);
}

// ---------- channel implementations ----------

bool PushChannels::sendPostJson(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  auto session = request(ch.url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String body;
  if (message.type == PUSH_BODY_CUSTOM) {
    body = message.content;
  } else {
    JsonDocument doc;
    doc["sender"]    = sender;
    doc["message"]   = message.content;
    doc["timestamp"] = timestamp;
    serializeJson(doc, body);
  }

  LOG("PUSHCH", "POST JSON to %s: %s", ch.url.c_str(), body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendBark(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  auto session = request(ch.url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  JsonDocument doc;
  // key1 已由调用方渲染，非空时作为自定义标题，否则回退到发件人号码
  String title = ch.key1.length() > 0 ? ch.key1 : sender;
  doc["title"] = title;
  doc["body"]  = message.content;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "Bark to %s: %s", ch.url.c_str(), body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendGet(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String url = ch.url;
  url += (url.indexOf('?') == -1) ? "?" : "&";
  url += "sender=" + urlEncode(sender);
  url += "&message=" + urlEncode(message.content);
  url += "&timestamp=" + urlEncode(timestamp);

  LOG("PUSHCH", "GET %s", url.c_str());
  auto session = request(url);
  if (!session) return false;
  int code = session->http()->GET();
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendDingtalk(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String webhookUrl = ch.url;

  if (ch.key1.length() > 0) {
    int64_t ts = getUtcMillis();
    String sign = dingtalkSign(ch.key1, ts);
    webhookUrl += (webhookUrl.indexOf('?') == -1) ? "?" : "&";
    char tsBuf[21];
    snprintf(tsBuf, sizeof(tsBuf), "%lld", ts);
    webhookUrl += "timestamp=" + String(tsBuf) + "&sign=" + sign;
  }

  auto session = request(webhookUrl);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String content = message.type == PUSH_BODY_CUSTOM ? message.content : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);

  JsonDocument doc;
  doc["msgtype"] = "text";
  doc["text"]["content"] = content;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "DingTalk: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendPushPlus(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String url = ch.url.length() > 0 ? ch.url : "http://www.pushplus.plus/send";
  auto session = request(url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String channelValue = "wechat";
  if (ch.key2.length() > 0) {
    if (ch.key2 == "wechat" || ch.key2 == "extension" || ch.key2 == "app") {
      channelValue = ch.key2;
    } else {
      LOG("PUSHCH", "Invalid PushPlus channel '%s'. Using default 'wechat'.", ch.key2.c_str());
    }
  }

  String content = message.type == PUSH_BODY_CUSTOM ? message.content : ("<b>发送者:</b> " + sender + "<br><b>时间:</b> " + timestamp + "<br><b>内容:</b><br>" + message.content);

  JsonDocument doc;
  doc["token"]   = ch.key1;
  doc["title"]   = "短信来自: " + sender;
  doc["content"] = content;
  doc["channel"] = channelValue;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "PushPlus: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendServerChan(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String url = ch.url.length() > 0 ? ch.url : ("https://sctapi.ftqq.com/" + ch.key1 + ".send");
  auto session = request(url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/x-www-form-urlencoded");

  String desp = message.type == PUSH_BODY_CUSTOM ? message.content : ("**发送者:** " + sender + "\n\n**时间:** " + timestamp + "\n\n**内容:**\n\n" + message.content);
  String postData = "title=" + urlEncode("短信来自: " + sender);
  postData += "&desp=" + urlEncode(desp);

  LOG("PUSHCH", "Server酱: %s", postData.c_str());
  int code = session->http()->POST(postData);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendCustom(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  // 类型7（POST请求）：使用 message.content（可为空，FR-008: 留空时发送空 POST body）
  auto session = request(ch.url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String body = message.content;
  LOG("PUSHCH", "POST请求: %s，body长度: %d", ch.url.c_str(), body.length());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendFeishu(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  auto session = request(ch.url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  JsonDocument doc;

  if (ch.key1.length() > 0) {
    int64_t ts = time(nullptr);
    doc["timestamp"] = String(ts);
    doc["sign"]      = computeHmacSha256Base64(ch.key1, String(ts) + "\n" + ch.key1);
  }

  String text = message.type == PUSH_BODY_CUSTOM ? message.content : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
  doc["msg_type"]        = "text";
  doc["content"]["text"] = text;

  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "飞书: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendGotify(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String url = ch.url;
  if (!url.endsWith("/")) url += "/";
  url += "message?token=" + ch.key1;
  auto session = request(url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String msg = message.type == PUSH_BODY_CUSTOM ? message.content : (message.content + "\n\n时间: " + timestamp);
  JsonDocument doc;
  doc["title"]    = "短信来自: " + sender;
  doc["message"]  = msg;
  doc["priority"] = 5;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "Gotify: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendTelegram(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String baseUrl = ch.url.length() > 0 ? ch.url : "https://api.telegram.org";
  if (baseUrl.endsWith("/")) baseUrl.remove(baseUrl.length() - 1);
  String url = baseUrl + "/bot" + ch.key2 + "/sendMessage";
  auto session = request(url);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String text = message.type == PUSH_BODY_CUSTOM ? message.content : ("📱短信通知\n发送者: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
  JsonDocument doc;
  doc["chat_id"] = ch.key1;
  doc["text"]    = text;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "Telegram: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendWechatWork(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp) {
  String webhookUrl = ch.url;

  if (ch.key1.length() > 0) {
    int64_t ts = getUtcMillis();
    char tsBuf[21];
    snprintf(tsBuf, sizeof(tsBuf), "%lld", ts);
    webhookUrl += (webhookUrl.indexOf('?') == -1) ? "?" : "&";
    webhookUrl += "timestamp=" + String(tsBuf) + "&sign=" + urlEncode(computeHmacSha256Base64(ch.key1, String(tsBuf) + "\n" + ch.key1));
  }

  auto session = request(webhookUrl);
  if (!session) return false;
  session->http()->addHeader("Content-Type", "application/json");

  String content = message.type == PUSH_BODY_CUSTOM ? message.content : ("📱短信通知\n发件人: " + sender + "\n内容: " + message.content + "\n时间: " + timestamp);
  JsonDocument doc;
  doc["msgtype"] = "text";
  doc["text"]["content"] = content;
  String body;
  serializeJson(doc, body);

  LOG("PUSHCH", "企业微信: %s", body.c_str());
  int code = session->http()->POST(body);
  return isResponseSuccessful(session.get(), code);
}

bool PushChannels::sendSmsPush(const PushChannel& ch, const String& sender, const PushBody& message, const String& timestamp, ModemId modemId) {
  String content = message.type == PUSH_BODY_CUSTOM ? message.content : ("[转发]发件人: " + sender + "\n内容: " + message.content);
  // Sms::sendPDU 内部自动处理长短信拆分，无需手动截断
  LOG("PUSHCH", "SMS备份推送到: %s", ch.url.c_str());
  bool ok = Sms::sendPDU(modemId, ch.url.c_str(), content.c_str());
  if (!ok) LOG("PUSHCH", "SMS备份推送失败");
  return ok;
}

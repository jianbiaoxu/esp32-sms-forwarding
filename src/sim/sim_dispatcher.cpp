#include "sim_dispatcher.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <strings.h>
#include <new>
#include "../logger/logger.h"

// ---------- 文件级私有状态与辅助函数（匿名命名空间） ----------

namespace {

struct DispatcherContext {
    QueueHandle_t     queue             = nullptr;
    TaskHandle_t      task              = nullptr;
    SemaphoreHandle_t directTxnMutex    = nullptr;
    SimUrcCallback    urcCb             = nullptr;
    SimCmdSlot*       activeCmd         = nullptr;
    unsigned long     cmdStartMs        = 0;
    volatile bool     pauseRequested    = false;
    volatile bool     readerPaused      = false;
    bool              drainAfterTimeout = false;
    unsigned long     lastRxMs          = 0;
    bool              waitingPdu        = false;
};

DispatcherContext s_ctx[MODEM_COUNT];
uint8_t s_taskIds[MODEM_COUNT] = {0, 1};

static DispatcherContext& context(ModemId modemId) {
    return s_ctx[isValidModemId(modemId) ? modemId : MODEM_PRIMARY];
}

static HardwareSerial& modemSerial(ModemId modemId) {
    return modemId == 0 ? Serial1 : Serial0;
}

bool isFinalOkLine(const String& line) {
    String s = line;
    s.trim();
    return s.equals("OK");
}

bool isFinalErrorLine(const String& line) {
    String s = line;
    s.trim();
    return s.equals("ERROR") || s.startsWith("+CME ERROR") || s.startsWith("+CMS ERROR");
}

// ---------- 内部 URC 识别 ----------

bool isUrcLine(ModemId modemId, const String& line) {
    if (line.equals("RING"))                   return true;
    // AT+CRC=1 会把来电指示从裸 RING 换成 +CRING: <type>(TS 27.007 §6.11)。这个
    // 设置和 CNMI 一样不进模组 NVM,模组自己重启后可能不是我们下发的那个值;不在此
    // 识别的话,它在某条命令在途时会被并入该命令的响应,并且来电会彻底丢失 ——
    // 后续的 +CLIP: 虽然能被识别,却因为没有先导 RING 而被丢弃。
    if (line.startsWith("+CRING:"))            return true;
    // 实测(ML307A + 中国电信):模组**不发** +CLIP:,而是在 RING 之后主动报一条
    // +CLCC: 1,1,4,0,0,"158…",129,"",0,0。不在此识别有两个后果:主叫号码只能靠
    // 3 秒后主动查询 AT+CLCC 拿到(短促来电会拿不到),更要紧的是它若在某条命令
    // 在途时到达,会被并入该命令的响应 —— 一通来电就能污染 AT+CMGL 的短信列举。
    // 本固件自己发的 AT+CLCC 不受影响:solicitedInfoLine() 会把它认作该命令的
    // 信息响应而不是主动上报。
    if (line.startsWith("+CLCC:"))             return true;
    // 通话结束上报。本固件从不拨号也不挂机(全仓无 ATD/ATA/ATH/AT+CHUP),所以这
    // 三行只可能是未经请求的,不会是任何命令的最终结果码,并入响应只会造成污染。
    if (line.equals("NO CARRIER"))             return true;
    if (line.equals("BUSY"))                   return true;
    if (line.equals("NO ANSWER"))              return true;
    if (line.startsWith("+CLIP:"))             return true;
    if (line.startsWith("+CMT:"))              return true;
    // 瘦模式下短信落存储并给出 +CMTI: 指示。必须在此识别为主动上报：否则它在
    // 某条命令在途时会被并入该命令的响应，直接污染 AT+CMGL 等结果。
    if (line.startsWith("+CMTI:"))             return true;
    if (context(modemId).waitingPdu)          return true;
    if (line.indexOf("+CPIN:") >= 0)           return true;
    if (line.startsWith("+SIMCARD:"))          return true;
    if (line.startsWith("+CUSD:"))             return true;
    // 模组自身重启后的就绪上报。它没有冒号，也不对应任何被下发的命令，因此如果
    // 不在此识别，就会在某条命令在途时被并入该命令的响应并使其解析失败——实测
    // 表现为 AT+CPMS? 返回 "+MATREADY\n+CPMS: ..."，上层直接判为非法响应。
    if (line.startsWith("+MATREADY"))           return true;
    // ML307 MHTTP 的响应通过异步 +MHTTPURC 上报。4G 请求超时或清理稍晚时，残留
    // 上报可能在 reader 恢复后才到达；必须隔离它，不能污染下一条普通 AT 命令。
    if (line.startsWith("+MHTTPURC:"))          return true;
    return false;
}

// ---------- 已发出命令的「自身信息响应」识别 ----------

// 从命令推导它自己的信息响应前缀：AT+CPIN? → "+CPIN:"，AT+CGDCONT=1 → "+CGDCONT:"。
// 只做纯语法推导，不含任何命令知识表。
void expectedInfoPrefix(const char* cmd, char* out, size_t outSize) {
    if (out == nullptr || outSize < 3) return;
    out[0] = '\0';
    if (cmd == nullptr) return;
    const char* p = cmd;
    while (*p == ' ') p++;
    if (strncasecmp(p, "AT", 2) != 0) return;
    p += 2;
    if (*p != '+' && *p != '#' && *p != '$' && *p != '^' && *p != '&') return;
    size_t i = 0;
    out[i++] = *p++;
    while (*p != '\0' && *p != '=' && *p != '?' && *p != ';' && i + 2 < outSize) {
        out[i++] = *p++;
    }
    out[i++] = ':';
    out[i]   = '\0';
}

// 活跃命令的信息响应，是否被 isUrcLine() 误判为主动上报。
//
// 背景：+CPIN: 既是主动上报（插卡就绪）也是 AT+CPIN? 的信息响应。isUrcLine() 只看
// 行本身，因此在 AT+CPIN? 在途时会把它自己的应答当成 URC 吞掉，调用方只拿到 "OK"，
// SIM 插卡状态查询因此永远得不到结论。判据用「该行前缀是否等于本命令自身的响应
// 前缀」——这是唯一无需命令知识表就能区分的信号。
//
// 等待 PDU 数据行时一律不认定为信息响应：那一行属于上一条 +CMT: 上报。
bool solicitedInfoLine(const SimCmdSlot* slot, const String& line, bool waitingPdu) {
    if (slot == nullptr || waitingPdu) return false;
    char prefix[40];
    expectedInfoPrefix(slot->cmd, prefix, sizeof(prefix));
    if (prefix[0] == '\0') return false;
    return line.startsWith(prefix);
}

// ---------- 内部 URC 路由 ----------

void routeURC(ModemId modemId, const String& line) {
    SimUrcCallback cb = context(modemId).urcCb;
    if (cb == nullptr) return;

    if (line.equals("RING") || line.startsWith("+CRING:")) {
        cb(modemId, SimUrcType::RING, line);
        return;
    }
    if (line.startsWith("+CMTI:")) {
        cb(modemId, SimUrcType::CMTI, line);
        return;
    }
    if (line.startsWith("+MATREADY")) {
        cb(modemId, SimUrcType::MATREADY, line);
        return;
    }
    if (line.startsWith("+CLIP:")) {
        cb(modemId, SimUrcType::CLIP, line);
        return;
    }
    if (line.startsWith("+CMT:")) {
        context(modemId).waitingPdu = true;
        cb(modemId, SimUrcType::CMT, line);
        return;
    }
    if (context(modemId).waitingPdu) {
        context(modemId).waitingPdu = false;
        cb(modemId, SimUrcType::CMT_PDU, line);
        return;
    }
    if (line.indexOf("+CPIN: READY") >= 0) {
        cb(modemId, SimUrcType::CPIN_READY, line);
        return;
    }
    if (line.indexOf("+CPIN: NOT INSERTED") >= 0 || line.startsWith("+SIMCARD:0")) {
        cb(modemId, SimUrcType::SIM_REMOVE, line);
        return;
    }
    if (line.startsWith("+CUSD:")) {
        cb(modemId, SimUrcType::CUSD, line);
        return;
    }
    if (line.startsWith("+CLCC:")) {
        cb(modemId, SimUrcType::CLCC, line);
        return;
    }
    if (line.equals("NO CARRIER") || line.equals("BUSY") || line.equals("NO ANSWER")) {
        cb(modemId, SimUrcType::CALL_END, line);
        return;
    }
    // 走到这里说明模组主动报了一条我们不认识的东西。此前是无声丢弃:模组说了话、
    // 固件什么都没做、日志里一个字都没有,唯一症状是「某类通知从来不出现」,而两侧
    // 看起来都正常 —— 与 +CMT: 那处注释描述的是同一类故障。
    //
    // 空闲时每一行都会走到这里(不经过 isUrcLine),所以这条日志也是唯一能看见
    // 「模组究竟发了什么」的地方。
    LOG("SIMDSP", "[URC-unrecognized] %s", line.c_str());
}

void appendResponseLine(SimCmdSlot* slot, const String& line) {
    if (slot->respBuf == nullptr || slot->respCap == 0) return;
    size_t existing = strnlen(slot->respBuf, slot->respCap);
    if (existing >= slot->respCap - 1) return;

    size_t remaining = (slot->respCap - 1) - existing;
    size_t copyLen = line.length();
    if (copyLen > remaining) copyLen = remaining;
    if (copyLen > 0) {
        memcpy(slot->respBuf + existing, line.c_str(), copyLen);
        existing += copyLen;
        slot->respBuf[existing] = '\0';
    }

    if (existing < slot->respCap - 1) {
        slot->respBuf[existing++] = '\n';
        slot->respBuf[existing] = '\0';
    }
}

// ---------- SIM reader task ----------

void simReaderTaskWithId(void* arg) {
    ModemId modemId = arg == nullptr ? MODEM_PRIMARY : *(static_cast<uint8_t*>(arg));
    DispatcherContext& state = context(modemId);
    HardwareSerial& serial = modemSerial(modemId);
    String lineBuf;
    // 预分配：SMS PDU hex 串典型约 340 字符；AT+CSIM 的长响应行可达约 530 字符，
    // 按 SIM_LINE_BUF_MAX 预留避免反复扩容
    lineBuf.reserve(SIM_LINE_BUF_MAX);

    for (;;) {
        if (state.pauseRequested && state.activeCmd == nullptr) {
            state.readerPaused = true;
            while (state.pauseRequested) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            state.readerPaused = false;
        }

        // 读取对应硬件串口字符，按行处理
        while (serial.available()) {
            char c = (char)serial.read();
            state.lastRxMs = millis();
            if (c == '\n') {
                String line = lineBuf;
                lineBuf = "";

                if (line.length() == 0) continue;

                if (state.activeCmd != nullptr) {
                    // T015: 有活跃指令时先检查是否为 URC 行；但本命令自身的信息
                    // 响应不能被当作 URC 吞掉（见 solicitedInfoLine 注释）。
                    if (isUrcLine(modemId, line) && !solicitedInfoLine(state.activeCmd, line, state.waitingPdu)) {
                        LOG("SIMDSP", "[URC-during-cmd] %s", line.c_str());
                        routeURC(modemId, line);
                    } else {
                        appendResponseLine(state.activeCmd, line);

                        if (isFinalOkLine(line)) {
                            state.activeCmd->isOk = true;
                            xSemaphoreGive(state.activeCmd->doneSem);
                            state.activeCmd = nullptr;
                        } else if (isFinalErrorLine(line)) {
                            state.activeCmd->isOk = false;
                            xSemaphoreGive(state.activeCmd->doneSem);
                            state.activeCmd = nullptr;
                        }
                    }
                } else {
                    routeURC(modemId, line);
                }
            } else if (c != '\r') {
                lineBuf += c;
                if (lineBuf.length() > SIM_LINE_BUF_MAX) {
                    LOG("SIMDSP", "串口行超过 %u 字节，已丢弃", (unsigned)SIM_LINE_BUF_MAX);
                    lineBuf = "";
                    state.waitingPdu = false;
                }
            }
        }

        // 取下一条命令（若当前无活跃命令）
        if (state.activeCmd == nullptr && !state.pauseRequested) {
            if (state.drainAfterTimeout) {
                if (millis() - state.lastRxMs < SIM_TIMEOUT_DRAIN_QUIET_MS) {
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;
                }
                state.drainAfterTimeout = false;
            }
            SimCmdSlot* ptr = nullptr;
            if (xQueueReceive(state.queue, &ptr, 0) == pdTRUE && ptr != nullptr) {
                state.activeCmd   = ptr;
                state.cmdStartMs  = millis();
                serial.println(state.activeCmd->cmd);
            }
        }

        // 超时检测
        if (state.activeCmd != nullptr &&
            millis() - state.cmdStartMs > state.activeCmd->timeoutMs) {
            LOG("SIMDSP", "AT 指令超时: %.96s", state.activeCmd->cmd);
            state.activeCmd->isOk = false;
            xSemaphoreGive(state.activeCmd->doneSem);
            state.activeCmd = nullptr;
            state.drainAfterTimeout = true;
            state.lastRxMs = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

}  // namespace

// ---------- 公共 API 实现 ----------

void SimDispatcher::registerUrcCallback(ModemId modemId, SimUrcCallback cb) {
    context(modemId).urcCb = cb;
}

void SimDispatcher::start(ModemId modemId) {
    if (!isValidModemId(modemId)) return;
    DispatcherContext& state = context(modemId);
    if (state.queue != nullptr) return;
    state.queue = xQueueCreate(SIM_CMD_QUEUE_SIZE, sizeof(SimCmdSlot*));
    if (state.queue == nullptr) {
        LOG("SIMDSP", "SimDispatcher::start: 队列创建失败");
        return;
    }
    state.directTxnMutex = xSemaphoreCreateMutex();
    if (state.directTxnMutex == nullptr) {
        LOG("SIMDSP", "SimDispatcher::start: 直接事务互斥锁创建失败");
        vQueueDelete(state.queue);
        state.queue = nullptr;
        return;
    }
    char taskName[16];
    snprintf(taskName, sizeof(taskName), "sim_reader_%u", (unsigned)modemId);
    if (xTaskCreate(simReaderTaskWithId, taskName, SIM_READER_TASK_STACK,
                    &s_taskIds[modemId], SIM_READER_TASK_PRIORITY, &state.task) != pdPASS) {
        vSemaphoreDelete(state.directTxnMutex);
        state.directTxnMutex = nullptr;
        vQueueDelete(state.queue);
        state.queue = nullptr;
        LOG("SIMDSP", "SimDispatcher::start: reader task 创建失败");
    }
}

bool SimDispatcher::sendCommand(ModemId modemId, const char* cmd, unsigned long timeoutMs,
                    String* outResp, bool prio, size_t respCap) {
    DispatcherContext& state = context(modemId);
    if (state.queue == nullptr) return false;
    if (cmd == nullptr) return false;

    size_t cmdLen = strlen(cmd);
    if (cmdLen >= SIM_CMD_BUF_SIZE) {
        // AT 指令超长会被静默截断 → 模组返回 ERROR 难以排查；改为直接拒绝
        LOG("SIMDSP", "AT 指令超长（%u ≥ %u），拒绝执行: %.64s...",
            (unsigned)cmdLen, (unsigned)SIM_CMD_BUF_SIZE, cmd);
        return false;
    }

    // 堆分配：缓冲放大到能容纳完整 APDU 后 SimCmdSlot 约 1.2KB，
    // 继续放在调用方栈上会给 async_tcp / loopTask 带来近 1KB 的额外栈压力。
    if (respCap < SIM_RESP_BUF_SIZE) respCap = SIM_RESP_BUF_SIZE;
    if (respCap > SIM_RESP_LARGE_BUF_SIZE) respCap = SIM_RESP_LARGE_BUF_SIZE;

    SimCmdSlot* slot = new (std::nothrow) SimCmdSlot();
    if (slot == nullptr) {
        LOG("SIMDSP", "AT 指令槽分配失败（堆不足）: %.64s", cmd);
        return false;
    }
    slot->respBuf = new (std::nothrow) char[respCap];
    if (slot->respBuf == nullptr) {
        LOG("SIMDSP", "AT 响应缓冲分配失败（需要 %u 字节）: %.64s", (unsigned)respCap, cmd);
        delete slot;
        return false;
    }
    slot->respCap = respCap;

    memcpy(slot->cmd, cmd, cmdLen);
    slot->cmd[cmdLen] = '\0';
    slot->timeoutMs   = timeoutMs;
    slot->respBuf[0]  = '\0';
    slot->isOk        = false;
    slot->priority    = prio;

    slot->doneSem = xSemaphoreCreateBinary();
    if (slot->doneSem == nullptr) {
        delete slot;
        return false;
    }

    BaseType_t sent;
    if (prio) {
        sent = xQueueSendToFront(state.queue, &slot, pdMS_TO_TICKS(100));
    } else {
        sent = xQueueSendToBack(state.queue, &slot, pdMS_TO_TICKS(100));
    }

    if (sent != pdTRUE) {
        vSemaphoreDelete(slot->doneSem);
        delete slot;
        return false;
    }

    // 必须一直等到 reader task 给信号量，不可自行超时：
    // 若 SimDispatcher::sendCommand 提前返回并 delete 掉 slot，
    // reader task 之后再 xSemaphoreGive(s_activeCmd->doneSem) 将访问
    // 悬空指针，导致崩溃。reader task 内部已有 timeoutMs 超时机制，
    // 最终一定会 Give 信号量（OK / ERROR / 超时三路均有 Give）。
    // reader task 在三条路径上都是「先解引用、再 Give、最后把 s_activeCmd 置空」，
    // Give 之后不再解引用，因此本函数被唤醒后 delete 是安全的。
    //
    // 但不能用 portMAX_DELAY 一次性长睡：本函数会在 async_tcp 任务上下文中被
    // HTTP 控制器调用（/at、/ping、/flight），而 async_tcp 已订阅 TWDT，
    // Arduino-ESP32 的 TWDT 默认超时为 5s。一条 5000ms 超时的 AT 指令
    // （如 AT+COPS=? 全网扫描）会让该任务整整 5s 不喂狗，直接触发
    // "Task watchdog got triggered" 而 abort 重启。
    // 因此改为分段等待：每 200ms 醒一次喂狗，但**永不提前返回**。
    // 注：未订阅 TWDT 的任务（如 sim_reader）调用 esp_task_wdt_reset()
    // 会返回 ESP_ERR_NOT_FOUND，无副作用，故忽略返回值。
    while (xSemaphoreTake(slot->doneSem, pdMS_TO_TICKS(200)) != pdTRUE) {
        esp_task_wdt_reset();
    }
    vSemaphoreDelete(slot->doneSem);

    bool ok = slot->isOk;
    if (outResp != nullptr) {
        *outResp = String(slot->respBuf);
    }
    delete slot;
    return ok;
}

bool SimDispatcher::pauseReader(ModemId modemId, unsigned long timeoutMs) {
    DispatcherContext& state = context(modemId);
    // Reader task 不存在时一律拒绝独占。
    // 旧实现在此返回 true（语义是「没什么要暂停的，可以直接用串口」），但在
    // USB AT 透传模式下 SimDispatcher 根本不会启动，调用方拿到 true 后会裸写
    // Serial1（/ping 的 AT+MPING、短信发送的 AT+CMGS），从而与透传任务抢串口、
    // 污染送给 USB 主机的 AT 流。
    // 正常流程中 startReaderTask() 之后 s_task 必然非空，且在此之前没有任何
    // 调用点，因此改为返回 false 不影响既有路径。
    if (state.task == nullptr) return false;
    if (state.directTxnMutex == nullptr) return false;
    if (xSemaphoreTake(state.directTxnMutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
        LOG("SIMDSP", "等待直接串口事务锁超时");
        return false;
    }
    state.pauseRequested = true;
    unsigned long start = millis();
    while (!state.readerPaused) {
        if (millis() - start >= timeoutMs) {
            state.pauseRequested = false;
            xSemaphoreGive(state.directTxnMutex);
            LOG("SIMDSP", "等待 reader 暂停超时");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

void SimDispatcher::resumeReader(ModemId modemId) {
    DispatcherContext& state = context(modemId);
    state.pauseRequested = false;
    // 等 reader 真正退出暂停态后再释放独占锁，避免下一次直接事务或普通
    // AT 命令在 reader 仍处于暂停态时开始，造成串口响应丢失。
    unsigned long start = millis();
    while (state.readerPaused && millis() - start < 1000) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (state.directTxnMutex != nullptr) {
        xSemaphoreGive(state.directTxnMutex);
    }
}

bool SimDispatcher::routeIfUrc(ModemId modemId, const String& line) {
    if (line.length() == 0) return false;
    if (!isUrcLine(modemId, line)) return false;
    LOG("SIMDSP", "[URC-during-raw] %s", line.c_str());
    routeURC(modemId, line);
    return true;
}

bool SimDispatcher::running(ModemId modemId) {
    return context(modemId).queue != nullptr;
}

HardwareSerial& SimDispatcher::serial(ModemId modemId) {
    return modemSerial(isValidModemId(modemId) ? modemId : MODEM_PRIMARY);
}

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <esp_task_wdt.h>

#include "wifi/wifi_manager.h"
#include "logger/logger.h"
#include "config/config.h"
#include "sim/sim.h"
#include "sim/at_bridge.h"
#include "call/call.h"
#include "time/time_sync.h"
#include "sms/sms.h"
#include "push/push.h"
#include "push/push_retry.h"
#include "push/push_queue.h"
#include "http/http_server.h"
#include "ota/ota_manager.h"
#include "coredump/coredump.h"
#include <time.h>

AsyncWebServer server(80);

// 记录各模组信息是否已抓取（在对应 SIM_READY 后执行一次）
static bool s_simInfoFetched[MODEM_COUNT] = {false, false};
static bool s_modemStarted[MODEM_COUNT]  = {false, false};
static int  s_startupModem              = -1;
static bool s_startupFinished           = false;

// 开机推送崩溃快照（在安排推送时捕获，防止 RTC 被后续更新覆写）
static bool   s_cachedHasCrash      = false;
static time_t s_cachedCrashTime     = 0;
static String s_cachedCrashVersion  = "";

// 开机推送：检测到 WiFi 初始化完成后延迟 BOOT_PUSH_DELAY_MS 触发
static bool          s_bootPushPending    = false;
static unsigned long s_bootPushAfterMs   = 0;
static bool          s_wifiInitWasSeen   = false;  // 已观测到初始化完成，防止重复触发

// WiFi 初始化完成（STA 连上或进入 AP 模式）后，等待此时长再发送开机推送通知
constexpr unsigned long BOOT_PUSH_DELAY_MS = 3000;

// ---------- helpers ----------

static void blinkShort(unsigned long gap = 500) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(gap);
}

static void delayWithWdt(unsigned long ms) {
  unsigned long elapsed = 0;
  while (elapsed < ms) {
    unsigned long step = min((unsigned long)500, ms - elapsed);
    delay(step);
    esp_task_wdt_reset();
    elapsed += step;
  }
}

static void modemPowerOff(ModemId modemId) {
  pinMode(config.modems[modemId].enPin, OUTPUT);
  digitalWrite(config.modems[modemId].enPin, LOW);
}

static void modemPowerCycle(ModemId modemId) {
  pinMode(config.modems[modemId].enPin, OUTPUT);
  LOG("MAIN", "SIM%u EN 拉低：关闭模组", modemId + 1);
  digitalWrite(config.modems[modemId].enPin, LOW);
  delayWithWdt(1200);
  LOG("MAIN", "SIM%u EN 拉高：开启模组", modemId + 1);
  digitalWrite(config.modems[modemId].enPin, HIGH);
  // 这里只做最小稳定延时，不再盲等固定时长：
  // 拉高之后到 Sim::ensureFreshModemSession() 之间没有任何代码访问 Serial1
  // （WiFi / NTP / LittleFS / HTTP 初始化都不碰模组），模组可以在那段时间里
  // 并行启动；真正的「等 AT 就绪」由 ensureFreshModemSession 轮询完成。
  delayWithWdt(500);
}

static bool startNextConfiguredModem() {
  while (s_startupModem + 1 < MODEM_COUNT) {
    ModemId modemId = (ModemId)(s_startupModem + 1);
    s_startupModem = modemId;
    if (!config.modems[modemId].enabled) {
      LOG("MAIN", "SIM%u 已禁用，跳过启动", modemId + 1);
      continue;
    }

    LOG("MAIN", "按顺序启动 SIM%u（上一模组已进入终态）", modemId + 1);
    modemPowerCycle(modemId);
    Sim::ensureFreshModemSession(modemId);
    Sim::init(modemId);
    Sim::startReaderTask(modemId);
    s_modemStarted[modemId] = true;
    return true;
  }
  s_startupFinished = true;
  LOG("MAIN", "所有已启用 ML307 模组均已完成顺序启动");
  return false;
}

static String enabledPhoneSummary() {
  String summary;
  int enabledCount = 0;
  for (ModemId modemId = 0; modemId < MODEM_COUNT; modemId++) {
    if (!config.modems[modemId].enabled) continue;

    if (enabledCount > 0) summary += ", ";
    summary += "SIM";
    summary += String(modemId + 1);
    summary += "=";
    String number = Sim::phoneNum(modemId);
    summary += number.length() > 0 ? number : "未知";
    enabledCount++;
  }
  return enabledCount > 0 ? summary : "无启用SIM";
}

static void serviceModemStartup() {
  if (config.atBridgeEnabled) return;

  for (ModemId i = 0; i < MODEM_COUNT; i++) {
    if (s_modemStarted[i]) {
      Sim::tick(i);
      Call::tick(i);
    }
  }

  if (s_startupFinished || s_startupModem < 0) return;
  ModemId current = (ModemId)s_startupModem;
  SimState state = Sim::state(current);
  if (state == SIM_READY || state == SIM_INIT_FAILED || state == SIM_NOT_INSERTED) {
    startNextConfiguredModem();
  }
}

// ---------- Arduino entry points ----------

void setup() {
  // 立即喂狗：框架初始化可能已消耗部分 TWDT 窗口
  esp_task_wdt_reset();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delayWithWdt(1500);  // 替换裸 delay：此时尚未有任何输出，必须喂狗

  // 先加载配置，再按配置初始化两路 UART 和 EN。这样 UART 引脚可由网页配置，
  // 且上电阶段可以保证两路模组不会同时启动。
  TimeSync::init();
  ConfigStore::load();
  Sms::initConcatBuffer();
  Coredump::init();
  ConfigStore::loadReboot(rebootSchedule);

  // RX 缓冲需容纳最长的一整行响应：AT+CSIM 透传完整 APDU 时
  // "+CSIM: 516,\"<516 hex>\"" 约 530 字节，再加后续的 "OK"。
  // 原值 500 会在 reader task 稍有延迟时溢出丢字节。
  for (ModemId modemId = 0; modemId < MODEM_COUNT; modemId++) {
    HardwareSerial& serial = SimDispatcher::serial(modemId);
    serial.setRxBufferSize(2048);
    serial.begin(115200, SERIAL_8N1,
                 config.modems[modemId].rxPin,
                 config.modems[modemId].txPin);
    while (serial.available()) serial.read();
    modemPowerOff(modemId);
  }
  esp_task_wdt_reset();

  // WiFi
  // 必须排在 Sim::init() 之前：SIM 初始化（CPIN/CFUN/CNUM/模组配置）是同步阻塞的，
  // 放在前面会把拿到 IP 的时间整段推后。SIM 的网络注册等待已改为非阻塞状态机
  // （见 Sim::tick），所以这里换序后开机到拿到 IP 只剩模组上电 + WiFi 自身耗时。
  WifiManager::setReconnectCallback([]{ TimeSync::syncNTP(); });
  WifiManager::init();
  esp_task_wdt_reset();

  // 时间同步：STA 模式下优先使用 NTP；AP 模式下等 SIM 就绪后从 NITZ 同步
  if (WifiManager::mode() == WIFI_MODE_STA_CONNECTED) {
    TimeSync::syncNTP();
    if (TimeSync::isSynced()) {
      LOG("MAIN", "NTP时间同步成功，UTC: %ld", (long)time(nullptr));
    } else {
      LOG("MAIN", "NTP时间同步失败，将在SIM就绪后通过NITZ同步");
    }
  } else {
    LOG("MAIN", "AP模式，跳过NTP同步。请访问 %s 配置WiFi", WifiManager::deviceUrl().c_str());
  }

  // LittleFS — formatOnFail=true 保证首次烧录或分区损坏时自动格式化
  // 格式化操作可能耗时数秒，需在前后喂狗
  esp_task_wdt_reset();
  if (!LittleFS.begin(true)) {
    LOG("MAIN", "LittleFS 挂载失败，HTML 页面不可用");
  } else {
    LOG("MAIN", "LittleFS 挂载成功");
    // 不写入文件的模块（仍输出到串口和内存缓冲）；如需全量记录传 nullptr。
    // 可用模块名：Push / PushQ / Retry / SMS / SIM / Call / WiFi / HTTP / OTA / BLE / Time / Cfg
    static const char* kLogFileSkip[] = { nullptr };
    Logger::init(kLogFileSkip);
    Logger::setStorageEnabled(config.logFileEnabled);
  }
  esp_task_wdt_reset();

  HttpServer::setup(server);
  Ota::init();

  Call::init();
  PushRetry::init();
  PushQueue::init();

  if (config.atBridgeEnabled) {
    // USB AT 透传模式：不启动 SimDispatcher，固件之后不再访问 Serial1，
    // 把模组的 AT 接口原样交给 USB 主机（原因详见 sim/at_bridge.h）。
    LOG("MAIN", "USB AT 透传模式已启用，跳过 SIM 初始化");
    modemPowerCycle(MODEM_PRIMARY);
    AtBridge::start();
  } else {
    // 先启动 SIM1；只有 SIM1 进入 READY/失败/未插卡终态后，loop 中才会启动 SIM2。
    // 失败不阻断后续模组，满足逐路启动和故障隔离要求。
    startNextConfiguredModem();
    esp_task_wdt_reset();
  }

  digitalWrite(LED_BUILTIN, LOW);
  // 开机推送在 loop() 中检测 WifiManager::isInitDone() 上升沿后自动安排
}

void loop() {
  {
    static unsigned long lastUrlPrint = 0;
    if (millis() - lastUrlPrint >= 3000) {
      lastUrlPrint = millis();
      String phoneSummary = enabledPhoneSummary();
      if (WifiManager::mode() == WIFI_MODE_AP_ACTIVE) {
        LOG("MAIN", "启用SIM本机号码: %s；请访问 %s 配置WiFi，或通过 BluFi BLE 配网（设备名: %s）", phoneSummary.c_str(), WifiManager::deviceUrl().c_str(), WifiManager::deviceName().c_str());
      } else {
        LOG("MAIN", "启用SIM本机号码: %s；请访问 %s 进行配置", phoneSummary.c_str(), WifiManager::deviceUrl().c_str());
      }
    }
  }

  // 开机推送：首次检测到 WiFi 初始化完成（STA 获取到 IP 或进入 AP 模式）后
  // 延迟 BOOT_PUSH_DELAY_MS 再触发，确保网络栈已就绪
  if (!s_wifiInitWasSeen && WifiManager::isInitDone()) {
    s_wifiInitWasSeen = true;
    if (ConfigStore::isValid()) {
      s_cachedHasCrash     = Coredump::hasData();
      s_cachedCrashTime    = Coredump::crashTime();
      s_cachedCrashVersion = Coredump::crashVersion();
      s_bootPushPending = true;
      s_bootPushAfterMs = millis() + BOOT_PUSH_DELAY_MS;
      LOG("MAIN", "WiFi初始化完成，%lu ms 后触发开机推送", BOOT_PUSH_DELAY_MS);
    }
  }

  // 开机推送：WiFi 初始化后 3 秒延迟触发，不依赖 WiFi 连接状态
  if (s_bootPushPending && millis() >= s_bootPushAfterMs) {
    s_bootPushPending = false;
    LOG("MAIN", "触发开机推送...");
    {
      String bootMsg = String("🚀 设备已启动") +
        "\n🌐 设备地址: " + WifiManager::deviceUrl() +
        "\n📶 MAC: " + WiFi.macAddress() +
        "\n📦 固件版本: " + Ota::version();
      if (s_cachedHasCrash) {
        time_t ct = s_cachedCrashTime;
        if (ct > 0) {
          char timeBuf[20];
          struct tm tmInfo;
          gmtime_r(&ct, &tmInfo);
          strftime(timeBuf, sizeof(timeBuf), "%Y%m%dT%H%M%S", &tmInfo);
          bootMsg += String("\n⚠️ 崩溃记录: 上次崩溃时间 ") + timeBuf + "（近似）";
        } else {
          bootMsg += "\n⚠️ 崩溃记录: 检测到上次崩溃（时间未知）";
        }
        if (s_cachedCrashVersion.length() > 0) {
          bootMsg += String("，崩溃版本: ") + s_cachedCrashVersion;
        }
        bootMsg += "，请前往工具箱导出 coredump";
      }
      Push::send("设备", bootMsg, TimeSync::dateStr(), MsgTypeInfo(MSG_TYPE_SIM));
    }
  }

  Sms::checkConcatTimeout();
  serviceModemStartup();
  PushQueue::tick();
  PushRetry::tick();
  TimeSync::tick();
  WifiManager::tick();

  // RTC 最后已知时间更新（每 10 秒，仅时间已同步时）
  {
    static unsigned long s_lastRtcUpdate = 0;
    if (TimeSync::isSynced() && millis() - s_lastRtcUpdate >= 10000) {
      s_lastRtcUpdate = millis();
      Coredump::updateLastKnownTime(time(nullptr));
    }
  }

  // SIM 就绪后抓取运营商/信号，并在 NTP 未同步时从 SIM NITZ 同步时间
  for (ModemId modemId = 0; modemId < MODEM_COUNT; modemId++) {
    if (s_modemStarted[modemId] && !s_simInfoFetched[modemId] && Sim::state(modemId) == SIM_READY) {
      s_simInfoFetched[modemId] = true;
      Sim::fetchInfo(modemId);
      if (!TimeSync::isSynced()) {
        TimeSync::syncFromSIM(modemId);
      }
    }
  }

  ConfigStore::rebootTick();
}

#include <HardwareSerial.h>
#include <STM32FreeRTOS.h>
extern "C" {
#include "droneController.h"
}

HardwareSerial crsfSerial(PD_9, PD_8);
HardwareSerial nrfSerial(PA_10, PA_9);

static const double ANCHOR_X[] = {0.0, 20.0, 0.0};
static const double ANCHOR_Y[] = {0.0, 0.0, 20.0};

/* ── Sensor packet ───────────────────────────────────────────────────────
 * Distances parsed from the nRF Zephyr LOG_INF text line, in metres.    */
struct SensorPacket {
  float ifft_m;
  float phase_m;
  float rtt_m;
};

struct FlightCommand {
  float roll;
  float pitch;
  uint16_t throttle;
};

static QueueHandle_t sensorQueue;
static QueueHandle_t commandQueue;
static volatile bool armed = false;

/* ── UART text format (Zephyr LOG_INF) ───────────────────────────────────
 * The nRF board outputs one log line per ranging event on uart21 TX (P1.04):
 *
 *   "I: path=<n>  ifft=<f>m  phase_slope=<f>m  rtt=<f>m\r\n"
 *
 * Values are in metres with 3 decimal places.
 * nRF UART: uart21, TX = P1.04, 115200 baud
 * STM32 RX pin: PA10 (USART1)
 * ─────────────────────────────────────────────────────────────────────── */

#define LINE_BUF_LEN 160 /* longer than the longest expected log line */

/* ── CRC / CRSF ──────────────────────────────────────────────────────── */
uint8_t crc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++)
      crc = crc & 0x80 ? (crc << 1) ^ 0xD5 : crc << 1;
  }
  return crc;
}

uint16_t stickToCRSF(float value) { return 992 + (uint16_t)(value * 150.0f); }

void sendCRSF(const FlightCommand &cmd) {
  uint8_t packet[26];
  packet[0] = 0xC8;
  packet[1] = 24;
  packet[2] = 0x16;

  uint16_t channels[16];
  channels[0] = stickToCRSF(cmd.roll);
  channels[1] = stickToCRSF(cmd.pitch);
  channels[2] = cmd.throttle;
  channels[3] = 992;
  channels[4] = armed ? 1811 : 992;
  channels[5] = armed ? 1811 : 992;
  for (int i = 6; i < 16; i++)
    channels[i] = 992;

  uint32_t bitBuffer = 0;
  int bitCount = 0;
  int index = 3;
  for (int i = 0; i < 16; i++) {
    bitBuffer |= (uint32_t)channels[i] << bitCount;
    bitCount += 11;
    while (bitCount >= 8) {
      packet[index++] = bitBuffer & 0xFF;
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }
  packet[25] = crc8(&packet[2], 23);
  crsfSerial.write(packet, 26);
}

/* ── Line parser ─────────────────────────────────────────────────────────
 * Searches for "ifft=", "phase_slope=", "rtt=" key=value pairs anywhere
 * in the line.  strtof() stops cleanly at the trailing 'm' unit suffix. */
static bool parseLine(const char *line, SensorPacket &pkt) {
  const char *p;

  p = strstr(line, "ifft=");
  if (!p)
    return false;
  pkt.ifft_m = strtof(p + 5, NULL);

  p = strstr(line, "phase_slope=");
  if (!p)
    return false;
  pkt.phase_m = strtof(p + 12, NULL);

  p = strstr(line, "rtt=");
  if (!p)
    return false;
  pkt.rtt_m = strtof(p + 4, NULL);

  return true;
}

/* ── FreeRTOS tasks ──────────────────────────────────────────────────── */
void taskUartRead(void *pvParameters) {
  char lineBuf[LINE_BUF_LEN];
  uint8_t lineLen = 0;
  SensorPacket pkt;
  static uint32_t packetCount = 0;
  static uint32_t byteCount = 0;
  static uint32_t lastReport = 0;

  Serial.println("[UART] Task started (text parser)");

  for (;;) {
    uint32_t now = xTaskGetTickCount();
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.print("[UART] Bytes: ");
      Serial.print(byteCount);
      Serial.print("  Packets: ");
      Serial.println(packetCount);
    }

    while (nrfSerial.available()) {
      char c = (char)nrfSerial.read();
      byteCount++;

      if (c == '\n') {
        lineBuf[lineLen] = '\0';

        if (parseLine(lineBuf, pkt)) {
          packetCount++;
          xQueueOverwrite(sensorQueue, &pkt);

          if (packetCount % 5 == 1) {
            Serial.print("[UART] #");
            Serial.print(packetCount);
            Serial.print("  ifft=");
            Serial.print(pkt.ifft_m, 3);
            Serial.print("m  phase=");
            Serial.print(pkt.phase_m, 3);
            Serial.print("m  rtt=");
            Serial.print(pkt.rtt_m, 3);
            Serial.println("m");
          }
        }
        lineLen = 0;

      } else if (c != '\r') {
        if (lineLen < LINE_BUF_LEN - 1) {
          lineBuf[lineLen++] = c;
        } else {
          lineLen = 0; /* line too long — discard and resync */
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void taskControl(void *pvParameters) {
  SensorPacket pkt;
  FlightCommand cmd;
  static uint32_t stepCount = 0;

  Serial.println("[CTL ] Task started");

  droneController_U.x1 = ANCHOR_X[0];
  droneController_U.y1 = ANCHOR_Y[0];
  droneController_U.x2 = ANCHOR_X[1];
  droneController_U.y2 = ANCHOR_Y[1];
  droneController_U.x3 = ANCHOR_X[2];
  droneController_U.y3 = ANCHOR_Y[2];

  for (;;) {
    if (xQueueReceive(sensorQueue, &pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
      /* Values already in metres — pass directly to controller */
      droneController_U.In2 = (double)pkt.ifft_m;
      droneController_U.In4 = (double)pkt.phase_m;
      droneController_U.In5 = (double)pkt.rtt_m;
      droneController_step();
      stepCount++;

      /* TODO: replace with proper velocity-to-attitude conversion */
      cmd.roll = (float)constrain(droneController_Y.vx, -1.0, 1.0);
      cmd.pitch = (float)constrain(droneController_Y.vy, -1.0, 1.0);
      cmd.throttle = (uint16_t)constrain(
          (int)(992.0 + droneController_Y.vz * 819.0), 172, 1811);
      xQueueOverwrite(commandQueue, &cmd);

      if (stepCount % 5 == 1) {
        Serial.print("[CTL ] Step #");
        Serial.println(stepCount);
        Serial.print("       ifft=");
        Serial.print(droneController_U.In2, 3);
        Serial.print("m  phase=");
        Serial.print(droneController_U.In4, 3);
        Serial.print("m  rtt=");
        Serial.println(droneController_U.In5, 3);
        Serial.print("       vx=");
        Serial.print(droneController_Y.vx, 3);
        Serial.print("  vy=");
        Serial.print(droneController_Y.vy, 3);
        Serial.print("  vz=");
        Serial.print(droneController_Y.vz, 3);
        Serial.print("  enableZ=");
        Serial.println(droneController_Y.enableZ);
        Serial.print("       roll=");
        Serial.print(cmd.roll);
        Serial.print("  pitch=");
        Serial.print(cmd.pitch);
        Serial.print("  throttle=");
        Serial.println(cmd.throttle);
      }
    }
  }
}

void taskCrsfSend(void *pvParameters) {
  FlightCommand cmd = {0.0f, 0.0f, 172};
  static uint32_t sendCount = 0;
  const uint32_t ARM_DELAY_MS = 5000;
  uint32_t startTick = xTaskGetTickCount();

  Serial.println("[CRSF] Task started — will auto-arm in 5s");

  for (;;) {
    if (!armed &&
        (xTaskGetTickCount() - startTick) >= pdMS_TO_TICKS(ARM_DELAY_MS)) {
      armed = true;
      cmd.throttle = 172;
      Serial.println("[CRSF] AUTO-ARMED — throttle at minimum");
    }

    bool gotCmd = xQueueReceive(commandQueue, &cmd, 0) == pdTRUE;
    if (!armed)
      cmd.throttle = 172;
    // TODO: remove hardcoded throttle
    // cmd.throttle = 500;
    sendCRSF(cmd);
    sendCount++;

    if (sendCount % 50 == 1) {
      Serial.print("[CRSF] Packet #");
      Serial.print(sendCount);
      Serial.print(armed ? "  [ARMED]" : "  [DISARMED]");
      Serial.print(gotCmd ? "  (live)" : "  (stale)");
      Serial.print("  roll=");
      Serial.print(stickToCRSF(cmd.roll));
      Serial.print("  pitch=");
      Serial.print(stickToCRSF(cmd.pitch));
      Serial.print("  thr=");
      Serial.println(cmd.throttle);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ── Setup ───────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("[INIT] Booting...");

  crsfSerial.begin(420000);
  Serial.println("[INIT] CRSF serial ready (PD9/PD8 @ 420000)");

  nrfSerial.begin(115200);
  Serial.println("[INIT] nRF UART ready (PA10 RX @ 115200)");

  Serial.println("[INIT] ESC init delay 3s...");
  delay(3000);

  droneController_initialize();
  Serial.println("[INIT] Controller initialized");

  sensorQueue = xQueueCreate(1, sizeof(SensorPacket));
  commandQueue = xQueueCreate(1, sizeof(FlightCommand));
  Serial.println("[INIT] Queues created");

  xTaskCreate(taskUartRead, "UART", 512, NULL, 3, NULL);
  xTaskCreate(taskControl, "CTL", 512, NULL, 2, NULL);
  xTaskCreate(taskCrsfSend, "CRSF", 256, NULL, 1, NULL);
  Serial.println("[INIT] Tasks created — starting scheduler");

  vTaskStartScheduler();
}

void loop() {}

#include <HardwareSerial.h>
#include <STM32FreeRTOS.h>
extern "C" {
#include "droneController.h"
}

HardwareSerial crsfSerial(PD_9, PD_8);
HardwareSerial nrfSerial(PA_10, PA_9);

static const double ANCHOR_X[] = {0.0, 20.0, 0.0};
static const double ANCHOR_Y[] = {0.0, 0.0, 20.0};

struct SensorPacket {
  uint8_t stationId[3];
  int32_t distance[3];
};

struct FlightCommand {
  float roll;
  float pitch;
  uint16_t throttle;
};

static QueueHandle_t sensorQueue;
static QueueHandle_t commandQueue;

/* ── UART frame format (17 bytes) ────────────────────────────────────────
 *   [0]      0xAA  start marker
 *   [1..15]  15-byte payload:
 *              byte 0       station ID  (board 1)
 *              bytes 1..4   distance cm (board 1, big-endian int32)
 *              byte 5       station ID  (board 2)
 *              bytes 6..9   distance cm (board 2, big-endian int32)
 *              byte 10      station ID  (board 3)
 *              bytes 11..14 distance cm (board 3, big-endian int32)
 *   [16]     0x55  end marker
 *
 * nRF UART: uart20, 115200 baud
 * STM32 RX pin: PA10 (USART1)
 * ─────────────────────────────────────────────────────────────────────── */

#define UART_FRAME_LEN  17
#define PAYLOAD_LEN     15
#define FRAME_START     0xAA
#define FRAME_END       0x55

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

uint16_t stickToCRSF(float value) { return 992 + (uint16_t)(value * 500.0f); }

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
  channels[4] = 992; /* TODO: arming logic */
  channels[5] = 992;
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

/* ── Frame parser ────────────────────────────────────────────────────── */
static bool parseFrame(const uint8_t *raw, SensorPacket &pkt) {
  if (raw[0] != FRAME_START || raw[UART_FRAME_LEN - 1] != FRAME_END) {
    Serial.print("[UART] Bad markers — got 0x");
    Serial.print(raw[0], HEX);
    Serial.print(" / 0x");
    Serial.println(raw[UART_FRAME_LEN - 1], HEX);
    return false;
  }
  const uint8_t *p = &raw[1];
  for (int i = 0; i < 3; i++) {
    pkt.stationId[i] = p[i * 5];
    pkt.distance[i] =
        (int32_t)((uint32_t)p[i * 5 + 1] << 24 | (uint32_t)p[i * 5 + 2] << 16 |
                  (uint32_t)p[i * 5 + 3] << 8  | (uint32_t)p[i * 5 + 4]);
  }
  return true;
}

/* ── FreeRTOS tasks ──────────────────────────────────────────────────── */
void taskUartRead(void *pvParameters) {
  enum { WAIT_HEADER, READ_PAYLOAD, READ_FOOTER } state = WAIT_HEADER;
  uint8_t buf[UART_FRAME_LEN];
  uint8_t idx = 0;
  SensorPacket pkt;
  static uint32_t frameCount   = 0;
  static uint32_t badFrameCount = 0;

  Serial.println("[UART] Task started");

  static uint32_t byteCount = 0;
  static uint32_t lastReport = 0;

  for (;;) {
    uint32_t now = xTaskGetTickCount();
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.print("[UART] Raw bytes received so far: "); Serial.println(byteCount);
    }

    while (nrfSerial.available()) {
      uint8_t b = (uint8_t)nrfSerial.read();
      byteCount++;

      switch (state) {
        case WAIT_HEADER:
          if (b == FRAME_START) {
            buf[0] = b;
            idx    = 1;
            state  = READ_PAYLOAD;
          } else {
            Serial.print("[UART] Unexpected byte while seeking header: 0x");
            Serial.println(b, HEX);
          }
          break;

        case READ_PAYLOAD:
          buf[idx++] = b;
          if (idx == UART_FRAME_LEN - 1)
            state = READ_FOOTER;
          break;

        case READ_FOOTER:
          buf[idx] = b;
          state    = WAIT_HEADER;

          if (parseFrame(buf, pkt)) {
            frameCount++;
            xQueueOverwrite(sensorQueue, &pkt);

            if (frameCount % 10 == 1) {
              Serial.print("[UART] Frame #"); Serial.print(frameCount);
              Serial.print("  IDs: ");
              Serial.print(pkt.stationId[0]); Serial.print(" ");
              Serial.print(pkt.stationId[1]); Serial.print(" ");
              Serial.println(pkt.stationId[2]);
              Serial.print("       Distances (cm): ");
              Serial.print(pkt.distance[0]); Serial.print("  ");
              Serial.print(pkt.distance[1]); Serial.print("  ");
              Serial.println(pkt.distance[2]);
              Serial.print("       Raw hex: ");
              for (int i = 0; i < UART_FRAME_LEN; i++) {
                if (buf[i] < 0x10) Serial.print("0");
                Serial.print(buf[i], HEX); Serial.print(" ");
              }
              Serial.println();
            }
          } else {
            badFrameCount++;
            Serial.print("[UART] Bad frame #"); Serial.println(badFrameCount);
          }
          break;
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
      /* TODO: verify station ID mapping */
      droneController_U.In2 = (double)pkt.distance[0];
      droneController_U.In4 = (double)pkt.distance[1];
      droneController_U.In5 = (double)pkt.distance[2];
      droneController_step();
      stepCount++;

      /* TODO: replace with proper velocity-to-attitude conversion */
      cmd.roll = (float)constrain(droneController_Y.vx, -1.0, 1.0);
      cmd.pitch = (float)constrain(droneController_Y.vy, -1.0, 1.0);
      cmd.throttle = (uint16_t)constrain(
          (int)(992.0 + droneController_Y.vz * 819.0), 172, 1811);
      xQueueOverwrite(commandQueue, &cmd);

      if (stepCount % 5 == 1) {
        Serial.print("[CTL ] Step #"); Serial.println(stepCount);
        Serial.print("       In2="); Serial.print(droneController_U.In2);
        Serial.print("  In4="); Serial.print(droneController_U.In4);
        Serial.print("  In5="); Serial.println(droneController_U.In5);
        Serial.print("       vx="); Serial.print(droneController_Y.vx);
        Serial.print("  vy="); Serial.print(droneController_Y.vy);
        Serial.print("  vz="); Serial.print(droneController_Y.vz);
        Serial.print("  enableZ="); Serial.println(droneController_Y.enableZ);
        Serial.print("       roll="); Serial.print(cmd.roll);
        Serial.print("  pitch="); Serial.print(cmd.pitch);
        Serial.print("  throttle="); Serial.println(cmd.throttle);
      }
    }
  }
}

void taskCrsfSend(void *pvParameters) {
  FlightCommand cmd = {0.0f, 0.0f, 172};
  static uint32_t sendCount = 0;

  Serial.println("[CRSF] Task started");

  for (;;) {
    bool gotCmd = xQueueReceive(commandQueue, &cmd, 0) == pdTRUE;
    sendCRSF(cmd);
    sendCount++;

    if (sendCount % 50 == 1) {
      Serial.print("[CRSF] Packet #"); Serial.print(sendCount);
      Serial.print(gotCmd ? "  (live)" : "  (stale)");
      Serial.print("  roll="); Serial.print(stickToCRSF(cmd.roll));
      Serial.print("  pitch="); Serial.print(stickToCRSF(cmd.pitch));
      Serial.print("  thr="); Serial.println(cmd.throttle);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ── Setup ───────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("[INIT] Booting...");

  crsfSerial.begin(420000);
  Serial.println("[INIT] CRSF serial ready (PD9/PD8 @ 420000)");

  nrfSerial.begin(115200);
  Serial.println("[INIT] nRF UART ready (PA10 RX @ 115200)");

  Serial.println("[INIT] ESC init delay 3s...");
  delay(3000);

  droneController_initialize();
  Serial.println("[INIT] Controller initialized");

  sensorQueue  = xQueueCreate(1, sizeof(SensorPacket));
  commandQueue = xQueueCreate(1, sizeof(FlightCommand));
  Serial.println("[INIT] Queues created");

  xTaskCreate(taskUartRead,  "UART", 512, NULL, 3, NULL);
  xTaskCreate(taskControl,   "CTL",  512, NULL, 2, NULL);
  xTaskCreate(taskCrsfSend,  "CRSF", 256, NULL, 1, NULL);
  Serial.println("[INIT] Tasks created — starting scheduler");

  vTaskStartScheduler();
}

void loop() {}

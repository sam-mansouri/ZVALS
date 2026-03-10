#include <HardwareSerial.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>
extern "C" {
#include "droneController.h"
}

HardwareSerial crsfSerial(PD_9, PD_8);

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

/* ── SPI slave receive ───────────────────────────────────────────────────
 *
 * nRF54L15 frame format (16 bytes):
 *   [0]     0xAA  start marker
 *   [1..15] 15-byte payload:
 *             byte 0      station ID  (board 1)
 *             bytes 1..4  distance cm (board 1, big-endian int32)
 *             byte 5      station ID  (board 2)
 *             bytes 6..9  distance cm (board 2, big-endian int32)
 *             byte 10     station ID  (board 3)
 *             bytes 11..14 distance cm (board 3, big-endian int32)
 *   [15]    0x55  end marker
 *
 * STM32 is SPI SLAVE — nRF drives SCK and CS.
 * Pins (SPI1):  SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4
 * The nRF CS pin (P2.07) connects to STM32 PA4.
 *
 * We use a raw SPI slave via HAL rather than Arduino SPI.transfer()
 * (which is master-only on STM32duino). The ISR fires on CS falling
 * edge, collects 16 bytes via DMA, validates markers, then posts to
 * the FreeRTOS queue.
 * ─────────────────────────────────────────────────────────────────────── */

#define SPI_FRAME_LEN 16
#define FRAME_START 0xAA
#define FRAME_END 0x55

static SPI_HandleTypeDef hspi1;
static DMA_HandleTypeDef hdma_spi1_rx;
static uint8_t spiRxBuf[SPI_FRAME_LEN];
static volatile bool frameReady = false;
static volatile uint32_t csPulseCount = 0;

extern "C" void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    frameReady = true;
  }
}

static void spi_slave_init(void) {
  Serial.println("[INIT] SPI1 slave init...");

  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);

  hdma_spi1_rx.Instance                 = DMA2_Stream0;
  hdma_spi1_rx.Init.Channel             = DMA_CHANNEL_3;
  hdma_spi1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_spi1_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_spi1_rx.Init.Mode                = DMA_NORMAL;
  hdma_spi1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_spi1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  HAL_StatusTypeDef dmaStatus = HAL_DMA_Init(&hdma_spi1_rx);
  Serial.print("[INIT] DMA init: "); Serial.println(dmaStatus == HAL_OK ? "OK" : "FAILED");

  __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  HAL_StatusTypeDef spiStatus = HAL_SPI_Init(&hspi1);
  Serial.print("[INIT] SPI init: "); Serial.println(spiStatus == HAL_OK ? "OK" : "FAILED");

  HAL_StatusTypeDef rxStatus = HAL_SPI_Receive_DMA(&hspi1, spiRxBuf, SPI_FRAME_LEN);
  Serial.print("[INIT] DMA RX arm: "); Serial.println(rxStatus == HAL_OK ? "OK" : "FAILED");

  Serial.println("[INIT] SPI1 slave ready");
}

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

/* ── Frame parser ────────────────────────────────────────────────────────
 * raw points to the 16-byte DMA buffer.
 * Payload layout (bytes 1..15, big-endian int32 distances):
 *   [0]     board ID 1
 *   [1..4]  distance 1
 *   [5]     board ID 2
 *   [6..9]  distance 2
 *   [10]    board ID 3
 *   [11..14] distance 3
 * ─────────────────────────────────────────────────────────────────────── */
static bool parseFrame(const uint8_t *raw, SensorPacket &pkt) {
  if (raw[0] != FRAME_START || raw[15] != FRAME_END) {
    Serial.print("[SPI] Bad markers — got 0x");
    Serial.print(raw[0], HEX);
    Serial.print(" / 0x");
    Serial.println(raw[15], HEX);
    return false;
  }
  const uint8_t *p = &raw[1];
  for (int i = 0; i < 3; i++) {
    pkt.stationId[i] = p[i * 5];
    pkt.distance[i] =
        (int32_t)((uint32_t)p[i * 5 + 1] << 24 | (uint32_t)p[i * 5 + 2] << 16 |
                  (uint32_t)p[i * 5 + 3] << 8 | (uint32_t)p[i * 5 + 4]);
  }
  return true;
}

/* ── FreeRTOS tasks ──────────────────────────────────────────────────── */
void taskSpiRead(void *pvParameters) {
  SensorPacket pkt;
  static uint32_t frameCount = 0;
  static uint32_t badFrameCount = 0;

  Serial.println("[SPI ] Task started");

  GPIO_PinState lastCs = GPIO_PIN_SET;

  for (;;) {
    GPIO_PinState cs = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
    if (lastCs == GPIO_PIN_SET && cs == GPIO_PIN_RESET) {
      csPulseCount++;
      Serial.print("[SPI ] CS fell — total pulses: "); Serial.println(csPulseCount);
    }
    lastCs = cs;

    if (frameReady) {
      frameReady = false;

      if (parseFrame(spiRxBuf, pkt)) {
        frameCount++;
        xQueueOverwrite(sensorQueue, &pkt);

        if (frameCount % 100 == 1) {
          Serial.print("[SPI ] Frame #"); Serial.print(frameCount);
          Serial.print("  IDs: ");
          Serial.print(pkt.stationId[0]); Serial.print(" ");
          Serial.print(pkt.stationId[1]); Serial.print(" ");
          Serial.println(pkt.stationId[2]);
          Serial.print("       Distances (cm): ");
          Serial.print(pkt.distance[0]); Serial.print("  ");
          Serial.print(pkt.distance[1]); Serial.print("  ");
          Serial.println(pkt.distance[2]);
          Serial.print("       Raw hex: ");
          for (int i = 0; i < SPI_FRAME_LEN; i++) {
            if (spiRxBuf[i] < 0x10) Serial.print("0");
            Serial.print(spiRxBuf[i], HEX); Serial.print(" ");
          }
          Serial.println();
        }
      } else {
        badFrameCount++;
        Serial.print("[SPI ] Bad frame #"); Serial.println(badFrameCount);
      }

      HAL_SPI_Receive_DMA(&hspi1, spiRxBuf, SPI_FRAME_LEN);
    }
    static uint32_t lastCsPrint = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - lastCsPrint >= 2000) {
      lastCsPrint = now;
      Serial.print("[SPI ] CS pulses detected: "); Serial.println(csPulseCount);
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

      if (stepCount % 50 == 1) {
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
    } else {
      Serial.println("[CTL ] Queue timeout — no sensor data");
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

  Serial.println("[INIT] ESC init delay 3s...");
  delay(3000);

  spi_slave_init();

  droneController_initialize();
  Serial.println("[INIT] Controller initialized");

  sensorQueue  = xQueueCreate(1, sizeof(SensorPacket));
  commandQueue = xQueueCreate(1, sizeof(FlightCommand));
  Serial.println("[INIT] Queues created");

  xTaskCreate(taskSpiRead,  "SPI",  256, NULL, 3, NULL);
  xTaskCreate(taskControl,  "CTL",  512, NULL, 2, NULL);
  xTaskCreate(taskCrsfSend, "CRSF", 256, NULL, 1, NULL);
  Serial.println("[INIT] Tasks created — starting scheduler");

  vTaskStartScheduler();
}

void loop() {}

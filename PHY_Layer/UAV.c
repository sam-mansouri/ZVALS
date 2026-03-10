/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Channel Sounding Reflector + GATT Write Server + SPI Output
 * ============================================================
 *
 *   nRF Board #4  = BLE PERIPHERAL only
 *                   - Advertises as "Nordic CS Reflector"
 *                   - CS Reflector for Initiator boards (unchanged)
 *                   - Hosts a GATT Write characteristic (15 bytes)
 *                     that the RPi4 writes to at 10 Hz
 *                   - On each write, outputs 16-byte SPI frame
 *
 *   RPi4          = BLE CENTRAL
 *                   - Scans, connects, discovers, writes at 10 Hz
 *
 * Connection identification: RPi4 is identified by its MAC address
 * (RPI_ADDR_STR). All other connections are treated as CS initiators.
 * This makes connect order irrelevant.
 *
 * KEY FIX: advertising is explicitly restarted after every connection
 * and after RPi disconnect, because BT_LE_ADV_CONN_FAST_2 stops
 * automatically once a connection is established on NCS/Zephyr.
 *
 * SPI frame: [0xAA | 15 payload bytes | 0x55] = 16 bytes
 *
 * GATT Service:  12345678-0000-1000-8000-00805f9b34fb
 * GATT Char:     12345678-0001-1000-8000-00805f9b34fb  WRITE_WITHOUT_RESPONSE
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/ras.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/spi.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

/* ── LEDs ───────────────────────────────────────────────────────────────── */
#define CON_STATUS_LED  DK_LED1   /* CS initiator connected  */
#define RPI_STATUS_LED  DK_LED2   /* RPi4 connected          */

/* ── RPi4 identity ──────────────────────────────────────────────────────── */
#define RPI_ADDR_STR "DC:A6:32:38:58:E3"

/* ── GATT UUIDs ─────────────────────────────────────────────────────────── */
#define RANGING_SVC_UUID \
	BT_UUID_128_ENCODE(0x12345678, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define RANGING_CHR_UUID \
	BT_UUID_128_ENCODE(0x12345678, 0x0001, 0x1000, 0x8000, 0x00805f9b34fb)

static struct bt_uuid_128 ranging_svc_uuid = BT_UUID_INIT_128(RANGING_SVC_UUID);
static struct bt_uuid_128 ranging_chr_uuid = BT_UUID_INIT_128(RANGING_CHR_UUID);

/* ── SPI ────────────────────────────────────────────────────────────────── */
#define SPI_NODE      DT_NODELABEL(spi21)
#define SPI_FRAME_LEN 16

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static const struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave     = 0,
};

static void spi_send_frame(const void *payload)
{
	static uint8_t frame[SPI_FRAME_LEN];

	frame[0] = 0xAA;
	memcpy(&frame[1], payload, 15);
	frame[15] = 0x55;

	struct spi_buf     tx_buf = { .buf = frame, .len = SPI_FRAME_LEN };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	int rc = spi_write(spi_dev, &spi_cfg, &tx_set);

	if (rc) {
		LOG_WRN("SPI write failed: %d", rc);
	} else {
		LOG_INF("SPI TX OK");
	}
}

/* ── GATT write handler ─────────────────────────────────────────────────── */
static ssize_t ranging_write_cb(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len,
				uint16_t offset, uint8_t flags)
{
	if (len != 15) {
		LOG_WRN("Unexpected write length: %u (expected 15)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *p = (const uint8_t *)buf;
	int32_t d1 = (int32_t)sys_get_be32(&p[1]);
	int32_t d2 = (int32_t)sys_get_be32(&p[6]);
	int32_t d3 = (int32_t)sys_get_be32(&p[11]);

	LOG_INF("WRITE B%u=%dcm  B%u=%dcm  B%u=%dcm",
		p[0], d1, p[5], d2, p[10], d3);

	spi_send_frame(buf);
	return len;
}

/* ── GATT service definition ────────────────────────────────────────────── */
BT_GATT_SERVICE_DEFINE(ranging_svc,
	BT_GATT_PRIMARY_SERVICE(&ranging_svc_uuid),
	BT_GATT_CHARACTERISTIC(&ranging_chr_uuid.uuid,
				BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_WRITE,
				NULL, ranging_write_cb, NULL),
);

/* ── CS semaphores ──────────────────────────────────────────────────────── */
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_config,    0, 1);

static struct bt_conn *cs_conn;
static struct bt_conn *rpi_conn;

/* ── Advertising data (forward declared for restart_advertising) ─────────── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── Advertising restart ─────────────────────────────────────────────────── */
/*
 * Connectable advertising stops automatically after the first connection
 * on Zephyr/NCS. We must restart it so a second peer can connect.
 * Called after every connection and after RPi disconnect.
 */
static void restart_advertising(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err == -EALREADY) {
		LOG_INF("Advertising already active");
	} else if (err) {
		LOG_ERR("Failed to restart advertising (err %d)", err);
	} else {
		LOG_INF("Advertising restarted");
	}
}

/* ── Helper: check if a connection is from the RPi4 ─────────────────────── */
static bool is_rpi(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	/* bt_addr_le_to_str: "XX:XX:XX:XX:XX:XX (type)" — compare first 17 chars */
	return strncmp(addr, RPI_ADDR_STR, 17) == 0;
}

/* ── BLE connection callbacks ────────────────────────────────────────────── */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connection failed (err 0x%02X)", err);
		bt_conn_unref(conn);
		return;
	}

	LOG_INF("Connected: %s", addr);

	if (is_rpi(conn)) {
		if (rpi_conn != NULL) {
			LOG_WRN("RPi4 already tracked – ignoring duplicate");
			bt_conn_unref(conn);
			return;
		}
		rpi_conn = bt_conn_ref(conn);
		dk_set_led_on(RPI_STATUS_LED);
		LOG_INF("RPi4 connected – GATT write chr ready");
	} else {
		if (cs_conn != NULL) {
			LOG_WRN("CS initiator already tracked – ignoring duplicate");
			bt_conn_unref(conn);
			return;
		}
		cs_conn = bt_conn_ref(conn);
		k_sem_give(&sem_connected);
		dk_set_led_on(CON_STATUS_LED);
		LOG_INF("CS Initiator connected: %s", addr);
	}

	/* Restart advertising so the other peer can still connect */
	restart_advertising();
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Disconnected %s (reason 0x%02X)", addr, reason);

	if (conn == rpi_conn) {
		bt_conn_unref(rpi_conn);
		rpi_conn = NULL;
		dk_set_led_off(RPI_STATUS_LED);
		LOG_INF("RPi4 disconnected – restarting advertising for reconnect");
		restart_advertising();
	} else if (conn == cs_conn) {
		bt_conn_unref(cs_conn);
		cs_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
		LOG_INF("CS Initiator disconnected – rebooting");
		sys_reboot(SYS_REBOOT_COLD);
	}
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				    struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn); ARG_UNUSED(params);
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
	} else {
		LOG_WRN("CS capability exchange failed (0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			      struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS config creation complete.");
		k_sem_give(&sem_config);
	} else {
		LOG_WRN("CS config creation failed (0x%02x)", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
	} else {
		LOG_WRN("CS security enable failed (0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS procedures %s.", params->state == 1 ? "enabled" : "disabled");
	} else {
		LOG_WRN("CS procedures enable failed (0x%02x)", status);
	}
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected                               = connected_cb,
	.disconnected                            = disconnected_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete                   = config_create_cb,
	.le_cs_security_enable_complete          = security_enable_cb,
	.le_cs_procedure_enable_complete         = procedure_enable_cb,
};

/* ── Main ───────────────────────────────────────────────────────────────── */
int main(void)
{
	int err;

	LOG_INF("Starting CS Reflector + GATT Write Server + SPI Output");

	dk_leds_init();

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}
	LOG_INF("SPI ready");

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}
	LOG_INF("Advertising as 'Nordic CS Reflector'");
	LOG_INF("Waiting for RPi4 and CS Initiator connections...");

	/* ── CS reflector loop ── */
	while (true) {
		k_sem_take(&sem_connected, K_FOREVER);

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = false,
			.enable_reflector_role = true,
			.cs_sync_antenna_selection =
				BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};

		err = bt_le_cs_set_default_settings(cs_conn, &default_settings);
		if (err) {
			LOG_ERR("Failed to configure CS settings (err %d)", err);
		}

		k_sem_take(&sem_config, K_FOREVER);

		const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
			.config_id               = 0,
			.max_procedure_len       = 1000,
			.min_procedure_interval  = 1,
			.max_procedure_interval  = 100,
			.max_procedure_count     = 0,
			.min_subevent_len        = 10000,
			.max_subevent_len        = 75000,
			.tone_antenna_config_selection =
				BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
			.phy                     = BT_LE_CS_PROCEDURE_PHY_2M,
			.tx_power_delta          = 0x80,
			.preferred_peer_antenna  =
				BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
			.snr_control_initiator   = BT_LE_CS_SNR_CONTROL_NOT_USED,
			.snr_control_reflector   = BT_LE_CS_SNR_CONTROL_NOT_USED,
		};

		err = bt_le_cs_set_procedure_parameters(cs_conn, &procedure_params);
		if (err) {
			LOG_ERR("Failed to set procedure parameters (err %d)", err);
			return 0;
		}
	}

	return 0;
}

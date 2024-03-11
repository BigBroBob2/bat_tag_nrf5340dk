
#ifndef BT_H
#define BT_H 1
#endif

#include "icm42688.h"
#include "pdm_microphone.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <nrfx_log.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>



#define STACKSIZE 1024 // Stack size used in each of the two threads
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

/// LED
#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

/// BUTTON
#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

/// bt buffer
#define BT_BUF_SIZE 40 // Size of the payload buffer in each RX and TX FIFO element



static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

//
// Callback for MTU exchange, for updating max data length. The MTU must be
// negotiated between the peripheral and central by sending packets back and
// forth (which takes time), and this function gets called after that
// negotiation process completes.
//
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	printk("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");

	printk("   NEW Max transmission unit (#chars): %d  (%d)\n",
		bt_nus_get_mtu(conn), bt_gatt_get_mtu(conn));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected %s\n", addr);

	// MUST be static, since it must be available after the function exits.
	static struct bt_gatt_exchange_params exchange_params = {.func = exchange_func};
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	printk("exchange_mtu err: %d\n", err);

	printk("Connected!! Max transmission unit (#chars): %d  (%d)\n",
		bt_nus_get_mtu(conn), bt_gatt_get_mtu(conn));

	// current_conn = bt_conn_ref(conn);
	current_conn = conn;

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason %u)\n", addr, reason);

	if (auth_conn) {
		// bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		// bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr,
			level, err);
	}
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected
	// .security_changed = security_changed,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = conn;
	// auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
	printk("Press Button 1 to confirm, Button 2 to reject.\n");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

///////////////////////// nus_callback
// This is apparently not in the C library so we make a simple one here. Find
// first "sep"erator character in the string, and put a 0 there, and return
// first part of string before the sep character.
char *strtok(char *str, char sep)
{
	char *inc;

	// Search for sep character in string.
	for(inc=str; *inc && *inc!=sep; ++inc)
	   ;
	
	*inc = 0;    // Terminate string at sep character.
	return str;  // Return beginning of string.
}


char spbuf[100];
void BLE_printf(char const *Format, ...)
{
  va_list ap;
  va_start(ap, Format);
  int len = vsnprintf(spbuf, sizeof(spbuf), Format, ap);
  va_end(ap);
  bt_nus_send(NULL, spbuf, len); // Send string across BLE. NULL sends to ALL connections.
}

// If wanted, can interpret sent strings as "commands", and perform some task.
#define STR_MAX 300
char cmdstr[STR_MAX+1];

// trial_start to start a recording trial
static bool trial_start = false;

int NReceived = 0;
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int error;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	NReceived++;

	// printk ble address
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	printk("Received data from: %s\n", addr);

	// Copy BLE data bytes to a null-terminated string.
	if(len > STR_MAX) len = STR_MAX;
	memcpy(cmdstr, data, len);
	cmdstr[len] = 0;

	//printk("Got BLE UART data (%d bytes): %.*s\n", len, len, data);
	printk("Got BLE UART data (%d bytes): %s\n", len, cmdstr);

	// Can come up with any command scheme you want. Here, we take an initial
	// '*' to mean that we should look for a command, then we search for a
	// matching command name.
	if(cmdstr[0] == '*') {
		char *cmd = strtok(cmdstr+1, ' '); // Get the first word after *
		int cmdlen = strlen(cmd);
		char *args = cmdstr+cmdlen+2; // remainder of string, after command token and space.

		printk("Found command: '%s' (%d chars), and args '%s'\n", cmd, cmdlen, args);

		// If we recognize the command, do something with it.
		if(strcmp(cmd, "setp") == 0) {
			// get var name and value to set
			char *var = strtok(args, ' ');
			args = args+strlen(var)+1;
			char *value_str = strtok(args, ' ');

			// check legal 
			if (strcmp(var,"")==0 || strcmp(value_str,"")==0){
				printk("Invalid command\n");
				error = -1;
				return error;
			}

			int value = atoi(value_str);

			printk("%d\n",value);

			if (strcmp(var, "imu_fs") == 0) {
				if (value==1000) {rate_idx=6;}
				else if (value==200) {rate_idx=7;}
				else if (value==100) {rate_idx=8;}
				else if (value==500) {rate_idx=15;}
				else {
					printk("Invalid imu_fs value, set default value 1000 Hz\n");
					rate_idx=6;
				}
			}
			else if (strcmp(var, "mic_fs") == 0) {
				if (value==4000000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8;}
				else if (value==3200000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10;}
				else if (value==2000000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16;}
				else if (value==1600000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21;}
				else if (value==1000000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32;}
				else if (value== 800000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV42;}
				else if (value== 500000) {i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63;}
				else {
					printk("Invalid mic_fs value, set default value 1000000 Hz\n");
					i2s_mckfreq=I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32;
				}
			}
			else {
				printk("Invalid command\n");
				error = -1;
				return error;
			}

			printk("Successfully set %s\n",var);
		}
		else if (strcmp(cmd, "start") == 0) {
			// start recording
			trial_start = true;
			printk("Trial start\n");
		}
		else if (strcmp(cmd, "stop") == 0) {
			// stop recording
			trial_start = false;
			printk("Trial stop\n");
		}
		else {
			printk("Invalid command\n");
			error = -1;
			return error;
		}
	}

	k_msleep(100);
}

// Callback to count number of sent notifications.
int NSent = 0;
void bt_sent_cb(struct bt_conn *conn)
{
	++NSent;
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
};




// static void num_comp_reply(bool accept)
// {
// 	if (accept) {
// 		bt_conn_auth_passkey_confirm(auth_conn);
// 		printk("Numeric Match, conn %p\n", (void *)auth_conn);

// 	} else {
// 		bt_conn_auth_cancel(auth_conn);
// 		printk("Numeric Reject, conn %p\n", (void *)auth_conn);

// 		// bt_conn_unref(auth_conn);
// 		auth_conn = NULL;
// }
// 	}

	

// button0: true, button1: false
// void button_changed(uint32_t button_state, uint32_t has_changed)
// {
// 	uint32_t buttons = button_state & has_changed;

// 	if (auth_conn) {
// 		if (buttons & KEY_PASSKEY_ACCEPT) {
// 			num_comp_reply(true);
// 		}

// 		if (buttons & KEY_PASSKEY_REJECT) {
// 			num_comp_reply(false);
// 		}
// 	}
// }



// void led_show_error(void)
// {
// 	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

// 	while (true) {
// 		/* Spin for ever */
// 		k_sleep(K_MSEC(1000));
// 	}
// }


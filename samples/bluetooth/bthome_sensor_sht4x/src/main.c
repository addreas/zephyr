/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#include <zephyr/drivers/sensor/sht4x.h>

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sht4x)
#error "No sensirion,sht4x compatible node found in the device tree"
#endif

#define SERVICE_DATA_LEN        9
#define SERVICE_UUID            0xfcd2      /* BTHome service UUID */
#define IDX_TEMPL               4           /* Index of lo byte of temp in service data*/
#define IDX_TEMPH               5           /* Index of hi byte of temp in service data*/
#define IDX_HUMDL               7           /* Index of lo byte of temp in service data*/
#define IDX_HUMDH               8           /* Index of hi byte of temp in service data*/

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
				  BT_GAP_ADV_SLOW_INT_MIN, \
				  BT_GAP_ADV_SLOW_INT_MAX, NULL)


static uint8_t service_data[SERVICE_DATA_LEN] = {
	BT_UUID_16_ENCODE(SERVICE_UUID),
	0x40,
	0x02,	/* Temperature */
	0xc4,	/* Low byte */
	0x00,   /* High byte */
	0x03,	/* Humidity */
	0xbf,	/* 50.55%  low byte*/
	0x13,   /* 50.55%  high byte*/
};

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
}

int main(void)
{
	int err;
	const struct device *const sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
	struct sensor_value temp, hum;

	if (!device_is_ready(sht)) {
		printf("Device %s is not ready.\n", sht->name);
		return 0;
	}


	printk("Starting BTHome sensor template\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	for (;;) {
		if (sensor_sample_fetch(sht)) {
			printf("Failed to fetch sample from SHT4X device\n");
			return 0;
		}

		sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		int16_t temp_val = sensor_value_to_milli(&temp) / 10;
		service_data[IDX_TEMPH] = temp_val >> 8;
		service_data[IDX_TEMPL] = temp_val & 0xff;

		sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &hum);
		uint16_t humd_val = sensor_value_to_milli(&hum) / 10;
		service_data[IDX_HUMDH] = humd_val >> 8;
		service_data[IDX_HUMDL] = humd_val & 0xff;

		printf("SHT4X: %.2f Temp. [C] ; %0.2f RH [%%]\n",
			sensor_value_to_double(&temp),
			sensor_value_to_double(&hum));

		err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			printk("Failed to update advertising data (err %d)\n", err);
		}
		k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
	}
	return 0;
}

/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver//gpio.h"
#include "sdkconfig.h"
#include "controller.h"
#include "nvs_flash.h"

#include "esp_wifi.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 5

static int8_t rssi_threshold = -80;

typedef struct __attribute__((packed)) ble_adv_data
{
	uint32_t pressure;
	int16_t temperature;
	uint16_t humidity;
	uint32_t visible_lux;
	uint32_t infrared_lux;
}ble_adv_data_t;

ble_adv_data_t scan_parse_sensor_data(esp_ble_gap_cb_param_t *scan_rst)
{
	ble_adv_data_t adv_data;
	memcpy(&adv_data, scan_rst->scan_rst.ble_adv + 7, sizeof(ble_adv_data_t));

	printf("\nPressure = %d\n",  adv_data.pressure);
	printf("Temperature = %d", adv_data.temperature);
	printf(" | Humidity: %d\n", adv_data.humidity);
	printf("Vis LUX = %d", adv_data.visible_lux);
	printf(" | IR LUX = %d\n", adv_data.infrared_lux);
	printf("---------------\n\n");
	return adv_data;
}

bool msd_nordic_filter(esp_ble_gap_cb_param_t *scan_rst)
{
	static uint8_t ref_data[3] = {0xFF, 0x59, 0x00};
	int32_t result = memcmp(&ref_data[0],
			scan_rst->scan_rst.ble_adv + 4, 3);

	if(result != 0)	{
		printf("Packet filtered.\n----------------\n\n");
		return false;
	}

	printf("Packet matches. Device MAC:");
	for (uint8_t i = 0; i < 6; i++)	{
		printf("%02X", scan_rst->scan_rst.bda[i]);
		if(i < 5)printf(":");
		}
	return true;
}


void set_rssi_filter(int8_t rssi)
{
	rssi_threshold = rssi;
}

bool scan_rssi_filter(esp_ble_gap_cb_param_t *scan_rst)
{
	if(scan_rst->scan_rst.rssi > rssi_threshold)
	{
		return true;
	}
	return false;
}

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static esp_ble_scan_params_t ble_scan_params = {
		.scan_type 			= BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type 		= BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval		= 0x50,
		.scan_window 		= 0x50
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch(event)
	{
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		esp_ble_gap_start_scanning(0);
		break;
#if 1

	case ESP_GAP_BLE_SCAN_RESULT_EVT:
		switch(param->scan_rst.search_evt)
		{
		case ESP_GAP_SEARCH_INQ_RES_EVT:
			if(scan_rssi_filter(param) && msd_nordic_filter(param))
			{
				scan_parse_sensor_data(param);
				printf("\n");
			}
			break;

#else

		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			switch(param->scan_rst.search_evt)
			{
			case ESP_GAP_SEARCH_INQ_RES_EVT:
				printf("\n\n\n-------------------\n");
				printf("ADVERTISING FOUND!\nDevice MAC: ");

				for (uint8_t i = 0; i < 6; i++)	{
					printf("%02X", param->scan_rst.bda[i]);
					if(i < 5)printf(":");
				}
				printf(" - RSSI = %d\n", param->scan_rst.rssi);
				printf("Advertising data size: %d\nAdvertising data:\n",
					    param->scan_rst.adv_data_len);
				for (uint8_t i = 0; i < param->scan_rst.adv_data_len; i++){
					printf("%02X", param->scan_rst.ble_adv[i]);
				}
				break;
#endif

		default:
			break;
		}

		break;

	default:
		break;
	}
}

static void ble_init()
{
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	esp_bt_controller_enable(ESP_BT_MODE_BLE);

	esp_bluedroid_init();
	esp_bluedroid_enable();

	esp_ble_gap_register_callback(esp_gap_cb);

	esp_ble_gap_set_scan_params(&ble_scan_params);
}

void app_main()
{
	ble_init();
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

}

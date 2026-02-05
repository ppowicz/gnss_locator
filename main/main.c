#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "led_strip.h"

#define LED_STRIP_GPIO 10
#define LED_STRIP_LED_COUNT 1
#define LED_STEP_DELAY_MS 30

#define SIM_UART_PORT UART_NUM_1
#define SIM_UART_TX_GPIO 4
#define SIM_UART_RX_GPIO 5
#define SIM_UART_BAUD 9600
#define SIM_UART_BUF_SIZE 256
#define SIM_CMD_INTERVAL_MS 1000

static const char *TAG = "hello_led";
static led_strip_handle_t led_strip;

static void hue_to_rgb(uint16_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
	hue %= 360;
	uint16_t sector = hue / 60;
	uint8_t offset = (uint8_t)((hue % 60) * 255 / 60);
	uint8_t inverse = (uint8_t)(255 - offset);

	switch (sector) {
	case 0:
		*r = 255;
		*g = offset;
		*b = 0;
		break;
	case 1:
		*r = inverse;
		*g = 255;
		*b = 0;
		break;
	case 2:
		*r = 0;
		*g = 255;
		*b = offset;
		break;
	case 3:
		*r = 0;
		*g = inverse;
		*b = 255;
		break;
	case 4:
		*r = offset;
		*g = 0;
		*b = 255;
		break;
	default:
		*r = 255;
		*g = 0;
		*b = inverse;
		break;
	}
}

static void init_led_strip(void)
{
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED_STRIP_GPIO,
		.max_leds = LED_STRIP_LED_COUNT,
		.led_pixel_format = LED_PIXEL_FORMAT_GRB,
		.led_model = LED_MODEL_WS2812,
		.flags.invert_out = false,
	};

	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10 * 1000 * 1000,
		.mem_block_symbols = 64,
		.flags.with_dma = false,
	};

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
	ESP_ERROR_CHECK(led_strip_clear(led_strip));
	ESP_LOGI(TAG, "LED strip ready on GPIO %d", LED_STRIP_GPIO);
}

static void init_sim_uart(void)
{
	uart_config_t uart_config = {
		.baud_rate = SIM_UART_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_ERROR_CHECK(uart_param_config(SIM_UART_PORT, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(SIM_UART_PORT, SIM_UART_TX_GPIO, SIM_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_ERROR_CHECK(uart_driver_install(SIM_UART_PORT, SIM_UART_BUF_SIZE, SIM_UART_BUF_SIZE, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_flush_input(SIM_UART_PORT));
	ESP_LOGI(TAG, "SIM UART ready (TX=%d, RX=%d)", SIM_UART_TX_GPIO, SIM_UART_RX_GPIO);
}

typedef bool (*sim_response_check_fn)(const char *response);

typedef struct {
	const char *command;
	const char *label;
	uint32_t timeout_ms;
	uint8_t retries;
	uint32_t retry_delay_ms;
	sim_response_check_fn check_fn;
} sim_command_step_t;

static void sim_log_command(const char *label, const char *command)
{
	char printable[64];
	size_t idx = 0;
	for (size_t i = 0; command[i] != '\0' && idx < sizeof(printable) - 1; ++i) {
		if (command[i] == '\r' || command[i] == '\n') {
			continue;
		}
		printable[idx++] = command[i];
	}
	printable[idx] = '\0';
	ESP_LOGI(TAG, "[SIM][%s] >> %s", label, printable);
}

static void sim_log_response(const char *label, const char *response)
{
	const char *body = (response && response[0] != '\0') ? response : "<empty>";
	ESP_LOGI(TAG, "[SIM][%s] << %s", label, body);
}

static int sim_collect_response(uint8_t *buffer, size_t buffer_len, uint32_t timeout_ms)
{
	if (buffer_len == 0) {
		return 0;
	}
	size_t total = 0;
	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
	if (timeout_ticks == 0) {
		timeout_ticks = 1;
	}
	do {
		size_t remaining = buffer_len - 1 - total;
		if (remaining == 0) {
			break;
		}
		int len = uart_read_bytes(SIM_UART_PORT, buffer + total, remaining, pdMS_TO_TICKS(100));
		if (len > 0) {
			total += len;
			buffer[total] = '\0';
			if (strstr((char *)buffer, "\r\nOK\r\n") || strstr((char *)buffer, "\r\nERROR\r\n")) {
				break;
			}
		}
	} while ((xTaskGetTickCount() - start) < timeout_ticks);
	buffer[total] = '\0';
	return (int)total;
}

static bool sim_response_has_token(const char *response, const char *token)
{
	return response && token && strstr(response, token);
}

static bool sim_parse_creg_status(const char *response, int *mode, int *status)
{
	if (!response) {
		return false;
	}
	const char *creg = strstr(response, "+CREG:");
	if (!creg) {
		return false;
	}
	int local_mode = 0;
	int local_status = 0;
	if (sscanf(creg, "+CREG: %d,%d", &local_mode, &local_status) != 2) {
		return false;
	}
	if (mode) {
		*mode = local_mode;
	}
	if (status) {
		*status = local_status;
	}
	return true;
}

static bool sim_check_ok(const char *response)
{
	return sim_response_has_token(response, "OK");
}

static bool sim_check_cpin_ready(const char *response)
{
	if (sim_response_has_token(response, "READY")) {
		return true;
	}
	if (sim_response_has_token(response, "SIM PIN")) {
		ESP_LOGW(TAG, "[SIM] SIM card requires a PIN");
	}
	return false;
}

static bool sim_check_csq_usable(const char *response)
{
	if (!response) {
		return false;
	}
	int rssi = -1;
	int ber = -1;
	const char *csq = strstr(response, "+CSQ:");
	if (csq && sscanf(csq, "+CSQ: %d,%d", &rssi, &ber) == 2) {
		ESP_LOGI(TAG, "[SIM] Signal quality RSSI=%d BER=%d", rssi, ber);
		return rssi >= 0 && rssi <= 31 && rssi != 99;
	}
	return sim_response_has_token(response, "OK");
}

static bool sim_check_creg_registered(const char *response)
{
	int mode = -1;
	int status = -1;
	if (!sim_parse_creg_status(response, &mode, &status)) {
		return false;
	}
	bool registered = (status == 1 || status == 5);
	ESP_LOGI(TAG, "[SIM] +CREG: mode=%d status=%d (%s)", mode, status, registered ? "registered" : "searching");
	return registered;
}

static bool sim_check_cgatt_attached(const char *response)
{
	if (!response) {
		return false;
	}
	int attached = -1;
	const char *cgatt = strstr(response, "+CGATT:");
	if (cgatt && sscanf(cgatt, "+CGATT: %d", &attached) == 1) {
		ESP_LOGI(TAG, "[SIM] Packet service attach=%d", attached);
		return attached == 1;
	}
	return sim_response_has_token(response, "+CGATT: 1");
}

static bool sim_send_step(const sim_command_step_t *step)
{
	if (!step || !step->command || !step->label || step->retries == 0) {
		return false;
	}
	uint8_t rx_buffer[SIM_UART_BUF_SIZE];
	for (uint8_t attempt = 1; attempt <= step->retries; ++attempt) {
		if (attempt > 1) {
			esp_err_t flush_err = uart_flush_input(SIM_UART_PORT);
			if (flush_err != ESP_OK) {
				ESP_LOGW(TAG, "[SIM][%s] flush failed: %s", step->label, esp_err_to_name(flush_err));
			}
		}
		sim_log_command(step->label, step->command);
		int written = uart_write_bytes(SIM_UART_PORT, step->command, strlen(step->command));
		if (written < 0) {
			ESP_LOGE(TAG, "[SIM][%s] failed to write command", step->label);
		} else {
			ESP_LOGD(TAG, "[SIM][%s] %d bytes written", step->label, written);
		}
		int len = sim_collect_response(rx_buffer, sizeof(rx_buffer), step->timeout_ms);
		sim_log_response(step->label, (char *)rx_buffer);
		bool has_error = sim_response_has_token((char *)rx_buffer, "ERROR");
		bool passed = !has_error;
		if (step->check_fn) {
			passed = step->check_fn((char *)rx_buffer);
		}
		if (passed && !has_error) {
			return true;
		}
		if (attempt < step->retries) {
			ESP_LOGW(TAG, "[SIM][%s] attempt %d/%d failed, retrying in %u ms", step->label, attempt, step->retries, step->retry_delay_ms);
			if (step->retry_delay_ms > 0) {
				vTaskDelay(pdMS_TO_TICKS(step->retry_delay_ms));
			}
		}
	}
	ESP_LOGE(TAG, "[SIM][%s] command failed after %d attempts", step->label, step->retries);
	return false;
}

static bool sim_register_to_network(void)
{
	const sim_command_step_t init_steps[] = {
		{"AT\r\n", "Basic Check", 500, 5, 200, sim_check_ok},
		{"ATE0\r\n", "Echo Disable", 500, 3, 200, sim_check_ok},
		{"AT+CFUN=1\r\n", "Full Function", 2000, 3, 300, sim_check_ok},
		{"AT+CPIN?\r\n", "SIM Status", 1000, 3, 500, sim_check_cpin_ready},
		{"AT+CSQ\r\n", "Signal Quality", 1000, 3, 500, sim_check_csq_usable},
	};
	for (size_t i = 0; i < (sizeof(init_steps) / sizeof(init_steps[0])); ++i) {
		if (!sim_send_step(&init_steps[i])) {
			return false;
		}
	}
	const sim_command_step_t creg_step = {
		.command = "AT+CREG?\r\n",
		.label = "Network Registration",
		.timeout_ms = 1500,
		.retries = 1,
		.retry_delay_ms = 0,
		.check_fn = sim_check_creg_registered,
	};
	bool registered = false;
	for (uint8_t attempt = 1; attempt <= 30; ++attempt) {
		if (sim_send_step(&creg_step)) {
			registered = true;
			break;
		}
		ESP_LOGW(TAG, "[SIM] Waiting for network (%d/30)", attempt);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	if (!registered) {
		ESP_LOGE(TAG, "[SIM] Timed out waiting for registration");
		return false;
	}
	const sim_command_step_t cgatt_step = {
		.command = "AT+CGATT?\r\n",
		.label = "Packet Attach",
		.timeout_ms = 2000,
		.retries = 3,
		.retry_delay_ms = 500,
		.check_fn = sim_check_cgatt_attached,
	};
	if (!sim_send_step(&cgatt_step)) {
		return false;
	}
	ESP_LOGI(TAG, "[SIM] Modem attached to network");
	return true;
}

static void sim_uart_task(void *pv_parameters)
{
	const sim_command_step_t monitor_creg_step = {
		.command = "AT+CREG?\r\n",
		.label = "Registration Monitor",
		.timeout_ms = 1500,
		.retries = 1,
		.retry_delay_ms = 0,
		.check_fn = sim_check_creg_registered,
	};
	const sim_command_step_t monitor_csq_step = {
		.command = "AT+CSQ\r\n",
		.label = "Signal Monitor",
		.timeout_ms = 1000,
		.retries = 2,
		.retry_delay_ms = 200,
		.check_fn = sim_check_csq_usable,
	};

	while (true) {
		if (!sim_register_to_network()) {
			ESP_LOGE(TAG, "[SIM] Registration sequence failed, retrying in 5 seconds");
			vTaskDelay(pdMS_TO_TICKS(5000));
			continue;
		}
		ESP_LOGI(TAG, "[SIM] Registration complete, entering monitoring mode");
		while (true) {
			if (!sim_send_step(&monitor_creg_step)) {
				ESP_LOGW(TAG, "[SIM] Lost network registration, restarting workflow");
				break;
			}
			(void)sim_send_step(&monitor_csq_step);
			vTaskDelay(pdMS_TO_TICKS(SIM_CMD_INTERVAL_MS));
		}
	}
}

void app_main(void)
{
	init_led_strip();
	init_sim_uart();
	if (xTaskCreate(sim_uart_task, "sim_uart_task", 4096, NULL, 5, NULL) != pdPASS) {
		ESP_LOGE(TAG, "Failed to start SIM UART task");
	}
	uint16_t hue = 0;
	TickType_t last_print = 0;
	const TickType_t print_interval = pdMS_TO_TICKS(1000);

	while (true) {
		uint8_t r = 0, g = 0, b = 0;
		hue_to_rgb(hue, &r, &g, &b);
		ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
		ESP_ERROR_CHECK(led_strip_refresh(led_strip));

		TickType_t now = xTaskGetTickCount();
		if (now - last_print >= print_interval) {
			printf("Hello, world!\n");
			last_print = now;
		}

		hue = (hue + 3) % 360;
		vTaskDelay(pdMS_TO_TICKS(LED_STEP_DELAY_MS));
	}
}

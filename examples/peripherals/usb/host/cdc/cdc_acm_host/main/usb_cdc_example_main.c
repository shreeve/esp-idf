#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"

#define USB_HOST_PRIORITY (20)
#define USB_DEVICE_VID    (0x0403) // FTDI
#define USB_DEVICE_PID    (0xcd18) // FT232: Abaxis Piccolo Xpress

static const char *TAG = "TRUST";

static SemaphoreHandle_t device_disconnected_sem;

// enable power from the USB Dev port to the USB Host port
static void enable_usb_host_power(void) {
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_12, 1); // DEV_VBUS_EN
  gpio_set_level(GPIO_NUM_17, 1); // IDEV_LIMIT_EN
  gpio_set_level(GPIO_NUM_18, 1); // USB_SEL
}

// usb library task
static void usb_lib_task(void *arg) {
  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB: All devices freed");
    }
  }
}

// setup usb host
static void setup_usb_host(void) {

  // install usb host (once per application)
  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .intr_flags     = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  // setup usb library task
  BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096,
    xTaskGetCurrentTaskHandle(), USB_HOST_PRIORITY, NULL
  );
  assert(task_created == pdTRUE);

  // install CDC-ACM driver
  ESP_LOGI(TAG, "Installing CDC-ACM driver");
  ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
}

// create semaphore to handle device disconnection
static void create_disconnect_sem(void) {
  device_disconnected_sem = xSemaphoreCreateBinary();
  assert(device_disconnected_sem);
}

// data received callback
static bool data_cb(const uint8_t *data, size_t size, void *arg) {
  if (size > 2) {
    ESP_LOG_BUFFER_HEXDUMP(TAG, data + 2, size - 2, ESP_LOG_INFO);
  }
  return true;
}

// device event callback
static void event_cb(const cdc_acm_host_dev_event_data_t *event, void *arg) {
  if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
    ESP_LOGI(TAG, "Device disconnected");
    ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
    xSemaphoreGive(device_disconnected_sem);
  } else {
    ESP_LOGW(TAG, "Unsupported CDC event: %i", event->type);
  }
}

void app_main(void) {

  // prepare environment
  enable_usb_host_power();
  setup_usb_host();
  create_disconnect_sem();

  // device config
  const cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 1000,
    .out_buffer_size       = 512,
    .in_buffer_size        = 512,
    .user_arg              = NULL,
    .event_cb              = event_cb,
    .data_cb               = data_cb,
  };

  while (true) {

    // attempt to open device
    cdc_acm_dev_hdl_t cdc_dev = NULL;
    esp_err_t err = cdc_acm_host_open_vendor_specific(
      USB_DEVICE_VID, USB_DEVICE_PID, 0, &dev_config, &cdc_dev
    );

    // handle error
    if (ESP_OK != err) {
      ESP_LOGI(TAG, "Unable to open device, trying again");
      continue;
    }

    // celebrate success
    ESP_LOGI(TAG, "Connected to the Abaxis Piccolo Xpress");
    cdc_acm_host_desc_print(cdc_dev);
    vTaskDelay(pdMS_TO_TICKS(100));

    // reset the connection
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 0,      0, 1, 0, 0); // reset both
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 0,      2, 1, 0, 0); // reset TX
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 0,      1, 1, 0, 0); // reset RX
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 2,      0, 1, 0, 0); // set flow control (none)
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 9,     16, 1, 0, 0); // set latency to 16ms
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 3, 0x4138, 1, 0, 0); // 9600 baud (3MHz/312.5)
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 1, 0x0303, 1, 0, 0); // enable DTR/RTS
    cdc_acm_host_send_custom_request(cdc_dev, 0x40, 2, 0x1311, 1, 0, 0); // set flow control (XON=0x11, XOFF=0x13)
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)"\x06", 1, 1000); // ACK see if we get some info from the Piccolo
    vTaskDelay(pdMS_TO_TICKS(3000));

    // wait for the device to disconnect, then start over
    xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
  }
}

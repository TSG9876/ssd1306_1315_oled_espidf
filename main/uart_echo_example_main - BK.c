/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "nvs_flash.h"
#include "esp_mac.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include <driver/i2c_master.h>
// #include <driver/i2c.h>
//  #include "driver_ssd1306_basic.h"
//  #include "driver_ssd1306_interface.h"
#include "u8g2.h"

extern const uint8_t u8g2_font_helvB10_tr[] U8G2_FONT_SECTION("u8g2_font_helvB10_tr");
extern const uint8_t u8g2_font_8x13_mf[] U8X8_FONT_SECTION("u8g2_font_8x13_mf");

// extern const uint8_t

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

// #define ECHO_TEST_TXD (gpio_num_t)(4)
// #define ECHO_TEST_RXD (gpio_num_t)(5)

#define ECHO_TEST_TXD (gpio_num_t)(12)
#define ECHO_TEST_RXD (gpio_num_t)(11)

#define ECHO_TEST_TXD1 (gpio_num_t)(12)
#define ECHO_TEST_RXD1 (gpio_num_t)(11)

#define CONFIG_PWR_PIN (gpio_num_t)(10) // yellow
#define CONFIG_SLP_PIN (gpio_num_t)(11) // yellow

#define TXD_PIN (gpio_num_t)(10)
#define RXD_PIN (gpio_num_t)(9)

#define CONFIG_I2C_0_PORT I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO (gpio_num_t)(5) // blue
#define CONFIG_I2C_0_SCL_IO (gpio_num_t)(6)

#define CONFIG_I2C_0_MASTER_DEFAULT {  \
    .clk_source = I2C_CLK_SRC_DEFAULT, \
    .i2c_port = CONFIG_I2C_0_PORT,     \
    .scl_io_num = CONFIG_I2C_0_SCL_IO, \
    .sda_io_num = CONFIG_I2C_0_SDA_IO, \
    .glitch_ignore_cnt = 7,            \
    .flags.enable_internal_pullup = true}

i2c_master_bus_handle_t i2c0_bus_hdl;
i2c_master_bus_handle_t bus_hdl;
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

i2c_master_dev_handle_t i2c_dev_handle;

i2c_master_bus_config_t i2c0_master_cfg = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_1,
    .scl_io_num = (gpio_num_t)(10),
    .sda_io_num = (gpio_num_t)(9),
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true};

i2c_device_config_t i2c_dev_conf = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x3C,
    .scl_speed_hz = 400000,
    // .scl_wait_us = 100000,

};

#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM 1
#define ECHO_UART_PORT_NUM1 2

#define ECHO_UART_BAUD_RATE 9600
#define ECHO_TASK_STACK_SIZE 4592
#define CTRL(x) (#x[0] - 'a' + 1)

static const char *TAG = "UART TEST";
static const char *TAG1 = "BMI323";

#define BUF_SIZE (1024)

#define EXAMPLE_ESP_WIFI_SSID "HUAT888"
#define EXAMPLE_ESP_WIFI_PASS "TP59836361"
#define EXAMPLE_ESP_MAXIMUM_RETRY 7

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define CONNECTED_BIT BIT0
#define FAIL_BIT BIT1
static httpd_handle_t web_server = NULL;
static EventGroupHandle_t s_wifi_event_group;
EventGroupHandle_t s_bmi_event_group;
#define CONFIG_APP_TAG "UART [APP]"

static int s_retry_num = 0;
EventBits_t uxBits;
static esp_netif_t *ptr_netif = NULL;
char *mdata = NULL;
char *token = NULL;
char str_data[400];
char ydata[180];
char Sdata[180];
char *out_data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static void data_draw();
static inline void vTaskDelayMs(const uint ms);

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void i2c_master_init();

void oled_clear();

static void clear_oled();
// char hu_msg[80];
// httpd_req_t *req=NULL;
/* Manufacturer: AirM2M
Model: Air780EG
Revision: AirM2M_780EG_V1174_LTE_AT
HWver: A12
Buildtime: Dec 17 2024 19:42:24
IMEI: 866597076034311
ICCID: 8960153011731687850F
IMSI: 502153010668985 */

//<meta http-equiv="refresh" content="10"/>

uint8_t ssd1315[27][3] = {
    {0xAE, 0x33, 0x33},

    {0x00, 0x33, 0x33},
    {0x10, 0x33, 0x33},
    {0x20, 0x10, 0x33},
    //  {0x21, 0x03, 0x82}, // Set column address: start = 3, end = 130 (0x82 = 130)

    {0x21, 0x00, 0x7F},
    {0x22, 0x00, 0x07}, // Set page address: start = 0, end = 7

    {0x40, 0x33, 0x33},
    {0xB0, 0x33, 0x33},
    {0x23, 0x00, 0x33}, // disable fade out
    {0x2E, 0x33, 0x33},
    {0xD6, 0x00, 0x33},
    {0x81, 0x30, 0x33}, // cf to 30
    {0xA1, 0x33, 0x33},
    {0xC8, 0x33, 0x33}, // 0xc8 to c0

    {0xA8, 0x3F, 0x33},
    {0xD3, 0x00, 0x33},
    {0xD5, 0x80, 0x33}, // chg 90 to 80
    {0xD9, 0xF1, 0x33},
    {0xDA, 0x12, 0x33},
    {0xDB, 0x30, 0x33}, // 20 t0 30 to 40
    {0x8D, 0x14, 0x33},
    {0xA4, 0x33, 0x33},
    {0xA6, 0x33, 0x33},
    {0xAF, 0x33, 0x33},

    // {0x00, 0x33, 0x33},
    // {0x10, 0x33, 0x33},

};

static void SSD1315_INIT()
{

    uint8_t cmd = UINT8_C(0x00);

    uint8_t cmd1 = UINT8_C(0x00);
    for (int i = 0; i < (24); i++)
    {

        int array_size = 3;

        uint8_t x_2 = UINT8_C(ssd1315[i][1]);
        uint8_t x_3 = UINT8_C(ssd1315[i][2]);

        //  printf("x2--- > %02x   x3---->%02x \n ", x_2, x_3);

        if (x_2 == 0x33 && x_3 == 0x33)
        {

            array_size = 1;
        }

        else if (x_2 != 0x33 && x_3 == 0x33)
        {

            array_size = 2;
        }

        // printf("Array size len  %i\n", array_size);

        uint8_t *data1 = malloc(4);

        esp_err_t ret = -1;

        if (array_size == 1)
        {
            data1[0] = cmd;
            data1[1] = UINT8_C(ssd1315[i][0]);

            if (dev_handle != NULL)
                ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
            vTaskDelay(50 / portTICK_PERIOD_MS);

            //  printf("ssd1315 write    %i\n", ret);
        }

        if (array_size == 2)

        {
            data1[0] = cmd1;
            data1[1] = UINT8_C(ssd1315[i][0]);
            //  ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
            //  vTaskDelay( 100 / portTICK_PERIOD_MS);

            data1[2] = UINT8_C(ssd1315[i][1]);

            if (dev_handle != NULL)
                ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
            vTaskDelay(50 / portTICK_PERIOD_MS);

            //  printf("ssd1315 two byte write  %02x    %i\n", data1[0], ret);
        }

        if (array_size == 3)

        {
            data1[0] = cmd1;
            data1[1] = UINT8_C(ssd1315[i][0]);
            //  ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
            //  vTaskDelay( 100 / portTICK_PERIOD_MS);

            data1[2] = UINT8_C(ssd1315[i][1]);

            data1[3] = UINT8_C(ssd1315[i][2]);
            if (dev_handle != NULL)
                ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
            vTaskDelay(50 / portTICK_PERIOD_MS);

            // printf("ssd1315 three byte write  %02x    %i\n", data1[0], ret);
        }
    }
}

static void clear_oled()
{
    uint8_t page_cmd[4];
    //  uint8_t *data_buffer;
    esp_err_t ret = -1;

    // Allocate buffer for 128 pixels + control byte
    // data_buffer = (uint8_t *)malloc(129);
    /*   if (!data_buffer)
      {
          ESP_LOGE("OLED", "Memory allocation failed");
          return;
      } */
    uint8_t data_buffer[129] = {0x40};  // Data control byte
    memset(data_buffer + 1, 0x00, 128); // Clear pattern

    // uint8_t col_cmd[] = {0x00, 0x21, 0x03, 0x82}; // 0x00=command, 0x21=set column, start=3, end=130
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, col_cmd, sizeof(col_cmd), -1));

    for (uint8_t page = 0; page < 8; page++)
    {
        // Set page address (0xB0 - 0xB7)
        page_cmd[0] = 0x00; // Command control byte
        page_cmd[1] = 0xB0 | page;
        page_cmd[2] = 0x00; // Column low nibble
        page_cmd[3] = 0x10; // Column high nibble

        // Send page setup command
        if (dev_handle != NULL)
            ret = i2c_master_transmit(dev_handle,
                                      page_cmd,
                                      4,
                                      -1);
        if (ret != ESP_OK)
        {
            ESP_LOGE("OLED", "Page command failed: %d", ret);
            //  free(data_buffer);
            return;
        }

        // Send 128 zeros for this page

        if (dev_handle != NULL)
            ret = i2c_master_transmit(dev_handle,
                                      data_buffer,
                                      130,
                                      -1);
        if (ret != ESP_OK)
        {
            ESP_LOGE("OLED", "Data write failed: %d", ret);
        }

        // Short delay for display stability
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // free(data_buffer);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t *data1 = malloc(2);
    /*
        data1[0] = UINT8_C(0x00);
        data1[1] = UINT8_C(0xA4);
        ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
        vTaskDelay(50 / portTICK_PERIOD_MS);

         data1[0] = UINT8_C(0x00);
        data1[1] = UINT8_C(0xAF);
        ret = i2c_master_transmit(dev_handle, (uint8_t *)data1, sizeof(data1), -1);
        vTaskDelay(50 / portTICK_PERIOD_MS); */
}

void test_clear()
{
    // Fill screen with pattern
    uint8_t pattern[129] = {0x40};
    for (int i = 1; i < 129; i++)
        pattern[i] = 0xFF;

    for (uint8_t page = 0; page < 8; page++)
    {
        // uint8_t page_cmd[] = {0x00, 0xB0 | page};
        uint8_t page_cmd[4];

        page_cmd[0] = 0x00; // Command control byte
        page_cmd[1] = 0xB0 | page;
        // page_cmd[2] = 0x00; // Column low nibble
        // page_cmd[3] = 0x10; // Column high nibble

        i2c_master_transmit(dev_handle, page_cmd, sizeof(page_cmd), -1);
        i2c_master_transmit(dev_handle, pattern, sizeof(pattern), -1);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Clear and verify
    //  clear_ssd1315(oled);
    ESP_LOGI("TEST", "Screen should be completely blank now");
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret = -1;
    s_wifi_event_group = xEventGroupCreate();

    ret = esp_netif_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize TCP/IP network stack");
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create default event loop");
        return ret;
    }

    ret = esp_wifi_set_default_wifi_sta_handlers();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set default handlers");
        return ret;
    }
    ptr_netif = esp_netif_create_default_wifi_sta();
    if (ptr_netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return ESP_FAIL;
    }

    //	esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            //  .scan_method=WIFI_ALL_CHANNEL_SCAN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
            .failure_retry_cnt = 7,
            //.sort_method=WIFI_CONNECT_AP_BY_SIGNAL,
            .channel = 0,

        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // default is WIFI_PS_MIN_MODEM
    // ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // default is WIFI_STORAGE_FLASH
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP_LOGI(TAG, "wifi_init_sta finished.");

    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "UNEXPECTED EVENT");
    return ESP_FAIL;
}

/* esp_err_t Alert_get_handler(httpd_req_t *req)
{
    char hu_msg[50];
    sprintf(hu_msg, "%s", data_reading.sta_fall);
    // printf("Ht handler --> %s\n", ht_msg);
    httpd_resp_send(req, hu_msg, strlen(hu_msg));

    return ESP_OK;
}

static const httpd_uri_t AlertH = {
    .uri = "/AlertCnt",
    .method = HTTP_GET,
    .handler = Alert_get_handler,
     Let's pass response string in user
     context to demonstrate it's usage
    .user_ctx = "alert_"
} */

esp_err_t get_handler(httpd_req_t *req)
{
    /* Send a simple response */
    const char resp[] = "URI GET Response";
    const char resp1[] = "";
    // client.print("Click <a href=\"/H\">here</a> to turn the LED on.<br>");
    // client.print("Click <a href=\"/L\">here</a> to turn the LED off.<br>");
    // https://accounts.google.com/o/oauth2/auth?client_id=1065235734199-t7sp92l13cvae37tmi9sumks0tgvg6c8.apps.googleusercontent.com&response_type=code&redirect_uri=
    //  1U5h5k2qoUUxsx3BZSY2KXYKFVSI-xMCS4rm-5m7KcxmsVhX08pfeHSgK/usercallback
    //"HTTP/1.1 200 OK\r\nContent-type:text/html\r\n\n"
    // https://script.google.com/macros/s/AKfycbwrUsU4jcohImlVQ00WT5TAEh06lt_MrYARK5QgRhId68-W0Cvp/exec
    //"Link <a href=\"https://accounts.google.com/o/oauth2/auth?client_id=1065235734199-t7sp92l13cvae37tmi9sumks0tgvg6c8.apps.googleusercontent.com&response_type=code&redirect_uri=https://script.google.com/macros/d/1U5h5k2qoUUxsx3BZSY2KXYKFVSI-xMCS4rm-5m7KcxmsVhX08pfeHSgK/usercallback"
    //             "&scope=https://www.googleapis.com/auth/spreadsheets https://www.googleapis.com/auth/script.external_request&access_type=offline\">GAS link here</a>\n"

    static char header[] =
        "<!DOCTYPE html>"
        "<html>\n"
        "Link <a href=\"https://script.google.com/macros/s/AKfycbwrUsU4jcohImlVQ00WT5TAEh06lt_MrYARK5QgRhId68-W0Cvp/exec?LightIntensity=9876\">"
        "GAS link here</a>\n"
        "</html>\n";

    // extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    // extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");

    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[] asm("_binary_index_html_end");

    // extern const char index_html_start[] asm("_binary_index_html_start");
    // extern const char index_html_end[] asm("_binary_index_html_end");

    char *buf = malloc(req->content_len + 1);
    size_t off = 0;
    int ret;

    /* Search for Custom header field */
    char *req_hdr = 0;
    size_t hdr_len = httpd_req_get_hdr_value_len(req, "/ACC_X");
    char qry_len = (char)req->uri;

    if (hdr_len)
    {
        /* Read Custom header value */
        req_hdr = malloc(hdr_len + 1);
        httpd_req_get_hdr_value_str(req, "ACC_X", req_hdr, hdr_len + 1);
        printf("req hdr for temp=== %s", req_hdr);
    }
    printf("index req %s\n", req->uri);

    httpd_resp_send(req, index_html_start, (index_html_end - index_html_start));

    // httpd_resp_send(req, header, strlen(header));
    // http_auth_basic_redirect();
    return ESP_OK;
}

esp_err_t AccX_get_handler(httpd_req_t *req)
{
    char hu_msg[450];

    httpd_resp_send(req, str_data, strlen(str_data));

    return ESP_OK;
}

static const httpd_uri_t ACCX = {
    .uri = "/Acc_X",
    .method = HTTP_GET,
    .handler = AccX_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "_AccX"

};

esp_err_t AccY_get_handler(httpd_req_t *req)
{
    char hu_msg[120];
    // printf("Ht handler --> %s\n", ht_msg);
    httpd_resp_send(req, ydata, strlen(ydata));

    return ESP_OK;
}

static const httpd_uri_t ACCY = {
    .uri = "/Acc_Y",
    .method = HTTP_GET,
    .handler = AccY_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "_AccY"

};

esp_err_t Sensor_get_handler(httpd_req_t *req)
{

    // printf("Ht handler --> %s\n", ht_msg);
    httpd_resp_send(req, Sdata, strlen(Sdata));

    return ESP_OK;
}

static const httpd_uri_t Sens = {
    .uri = "/Val_1",
    .method = HTTP_GET,
    .handler = Sensor_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "_Sens"

};

esp_err_t get_index_handler(httpd_req_t *req)
{
    // httpd_resp_send(req, temp_msg, strlen(temp_msg));

    return ESP_OK;
}

static const httpd_uri_t ws = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    // .is_websocket = true,
    .user_ctx = "Hello World"

};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6500;
    config.max_uri_handlers = 10;
    config.max_open_sockets = 3;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&web_server, &config) == ESP_OK)
    {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(web_server, &ws);
        httpd_register_uri_handler(web_server, &ACCX);
        httpd_register_uri_handler(web_server, &ACCY);
        httpd_register_uri_handler(web_server, &Sens);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
/* esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // Write out data
            printf("%.*s", evt->data_len, (char *)evt->data);
        }
        else
        {
            printf("chunked data -->%s\n", (char *)evt->data);
            //cJson_Parse((char *)evt->data);
        }

        break;

    case HTTP_EVENT_REDIRECT:
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAGHTTP, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}
 */

int sendData(const char *logName, char *data);

static inline void vTaskDelayMs(const uint ms)
{
    const TickType_t xDelay = (ms / portTICK_PERIOD_MS);
    vTaskDelay(xDelay);
}

int sendData(const char *logName, char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(ECHO_UART_PORT_NUM, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void readData()
{

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE * 1);

    int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);
    // Write data back to the UART
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
    if (len > 0)
    {
        data[len] = '\0';
        //  mdata = (char *)data;
        // printf("DATA %s \n",mdata);

        ESP_LOGI(TAG, "READ DATA str: %s \n", (char *)data);
    }
}

static int Srh_str(char *instr)
{

    //+UGNSINF: 1,1,20250519120934,3.046186,101.640335,69.800,0.48,0.00,3,,2.11,1.03,4.00,,21,14,,,32,,
    char srtr[] = "+UGNSINF:";

    int pos = -1;

    if (instr != NULL)
    {
        char *chk_str = strstr(instr, srtr);
        if (chk_str != NULL)
        {
            pos = chk_str - instr;
        }
    }

    return pos;
}

static void tokenizer()
{

    char delimiter[] = "\r";

    // Declare empty string to store token
    // char* token;

    // printf("Initial String: %s", str);
    // printf("\nAfter Tokenization: \n");
    // Get the first token
    token = strtok(mdata, delimiter);

    httpd_req_t *req = NULL;
    // using loop to get the rest of the token
    while (token)
    {
        printf("token = %s\n", token);

        strncpy(str_data, token, 350);
        AccX_get_handler(req);

        token = strtok(NULL, delimiter);
        int chk_pos = Srh_str(token);

        // vTaskDelayMs(15);

        if (chk_pos > 0)
        {

            int xlen = 100;

            if (strlen(token) <= 110)
            {

                xlen = strlen(token);
            }

            strncpy(ydata, token + chk_pos + 10, xlen - 10);

            AccY_get_handler(req);
        }
    }
}

static void tokenizer1(char *instr)
{

    char delimiter[] = ",";
    char delimiter1[] = "\0";

    // Declare empty string to store token
    // char* token;

    // printf("Initial String: %s", str);
    // printf("\nAfter Tokenization: \n");
    // Get the first token

    char *token;

    token = strtok(instr, delimiter);
    //  printf("token = %s\n", token);

    out_data[0] = (char *)token;
    int cnt = 1;

    // httpd_req_t *req = NULL;
    // using loop to get the rest of the token
    while (token)
    {
        //   printf("token = %s\n", token);

        // strncpy(str_data, token, 350);
        token = strtok(NULL, delimiter);

        out_data[cnt] = (char *)token;

        // printf("out_data [%i]  %s \n",cnt,out_data[cnt]);
        // int chk_pos = Srh_str(token);

        // vTaskDelayMs(15);

        cnt++;
    }
    // xEventGroupClearBits(s_bmi_event_group, CONNECTED_BIT);

    xEventGroupSetBits(s_bmi_event_group, CONNECTED_BIT);

    // data_draw();

    // vTaskDelayMs(10);
}

static void data_draw()
{

    printf("Data Draw Call \n");
    // ssd1306_basic_clear();
    uint8_t offset = UINT8_C(0);
    uint8_t xoffset = UINT8_C(5);
    // ssd1306_basic_clear();

    uint8_t ret = -1;
    // max x =127-font/2, max y=63-font

    char ostr[10];
    //// strcpy(ostr, " ");
    //  sprintf(ostr, "%s", out_data[0]);
    // ret = ssd1306_basic_string(11, 1, " =", 2, 1, SSD1306_FONT_12);

    /*   ret = ssd1306_basic_string(10 + offset, 0, out_data[0], 7, 1, SSD1306_FONT_12);

      if (ret != 0)
      {
          ssd1306_interface_debug_print("ssd1306: show string failed.\n");
          (void)ssd1306_basic_deinit();
      }

      printf("ret 0 = %i ", ret);
      vTaskDelayMs(1000);
      // vTaskDelayMs(20);
      //   strcpy(ostr, " ");
      //   sprintf(ostr, "%s", out_data[1]);

      ret = ssd1306_basic_string(10 + xoffset, 11 + offset, out_data[1], 10, 1, SSD1306_FONT_12);

      printf("ret 1 = %i ", ret);
      vTaskDelayMs(1000);

      // strcpy(ostr, " ");
      //  sprintf(ostr, "%s", out_data[2]);

      ret = ssd1306_basic_string(10 + xoffset, 22 + offset, out_data[2], 10, 1, SSD1306_FONT_12);

      printf("ret 2 = %i ", ret);
      vTaskDelayMs(1000);

      ret = ssd1306_basic_string(10 + xoffset, 33 + offset, out_data[3], 10, 1, SSD1306_FONT_12);

      printf("ret 3 = %i ", ret);
      vTaskDelayMs(1000);

      ret = ssd1306_basic_string(10 + xoffset, 44 + offset, out_data[4], 10, 1, SSD1306_FONT_12);

      printf("ret 4 = %i ", ret);
      vTaskDelayMs(1000);
      ret = ssd1306_basic_string(10 + xoffset * 8, 48 + offset, out_data[5], 10, 1, SSD1306_FONT_12);

      if (ret != 0)
      {
          ssd1306_interface_debug_print("ssd1306: show string failed.\n");
          (void)ssd1306_basic_deinit();
      }
      printf("ret 5 = %i \n", ret); */

    vTaskDelayMs(1000);
    if (ret == 0)
    {
        xEventGroupClearBits(s_bmi_event_group, CONNECTED_BIT);
    }
    else
    {
        xEventGroupClearBits(s_bmi_event_group, CONNECTED_BIT);

        xEventGroupSetBits(s_bmi_event_group, FAIL_BIT);
    }
}

static void init_uart()
{

    uart_config_t uart_config = {

        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}

static void power_a7670()
{

    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask = (1ULL << CONFIG_PWR_PIN);

    // disable pull-up mode
    io_conf.pull_up_en = 0;

    // ENable pull-down mode
    io_conf.pull_down_en = 0;

    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_direction(CONFIG_PWR_PIN, GPIO_MODE_OUTPUT);

    // gpio_reset_pin(CONFIG_PWR_PIN);
    // vTaskDelayMs(30);

    // gpio_set_level(CONFIG_PWR_PIN, 0);
    // vTaskDelayMs(1000);

    gpio_set_level(CONFIG_PWR_PIN, 1);
    vTaskDelayMs(1000);

    // gpio_set_level(CONFIG_PWR_PIN, 1);
    // vTaskDelayMs(2000);

    // gpio_set_level(CONFIG_PWR_PIN, 0);
    // vTaskDelayMs(1000);

    int level3 = gpio_get_level(CONFIG_PWR_PIN);
    printf("gpio's High level is: %d\n", level3);
}

static void Sleep_a7670()
{

    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask = (1ULL << CONFIG_SLP_PIN);

    // disable pull-up mode
    io_conf.pull_up_en = 0;

    // ENable pull-down mode
    io_conf.pull_down_en = 0;

    // configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_direction(CONFIG_SLP_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(CONFIG_SLP_PIN);
    vTaskDelayMs(30);

    gpio_set_level(CONFIG_SLP_PIN, 0);
    vTaskDelayMs(1000);
}

static void echo_AIR780_task(void *arg)
{
    // init_uart();

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {

        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 1, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM1, BUF_SIZE * 1, 0, 0, NULL, intr_alloc_flags));
    // ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM1, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM1, ECHO_TEST_TXD1, ECHO_TEST_RXD1, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE * 1);

    uint8_t *data1 = (uint8_t *)malloc(BUF_SIZE * 1);
    sendData("UART", "AT\r\n");
    //  vTaskDelayMs(30);

    sendData("UART", "ATE1\r\n");
    //  vTaskDelayMs(30);

    sendData("UART", "AT*I\r\n");
    //  vTaskDelayMs(100);

    sendData("UART", "AT+CGMM\r\n");
    // vTaskDelayMs(100);

    readData();
    // sendData("UART", "+++\r");

    vTaskDelayMs(1000);

    sendData("UART", "AT+CFUN=1\r\n");
    vTaskDelayMs(30);
    sendData("UART", "AT+CGNSPWR?\r\n");
    vTaskDelayMs(30);
    readData();
    sendData("UART", "AT+CGNSPWR=1\r\n");
    vTaskDelayMs(30);
    sendData("AGPS", "AT+CGNSAID=31,1,1,1");

    //  vTaskDelayMs(50);

    /*  sendData("SIGNAL", "AT+CSQ\r");
     vTaskDelayMs(30);

     readData();
     vTaskDelayMs(1000);



     sendData("UART", "AT+ICCID\r\n");
     vTaskDelayMs(30);
     //  sendData("UART", "AT+CSCA=\"+60163239688\",145\r\n");

     readData();
     vTaskDelayMs(50);

     sendData("PSM", "AT+CPMS=\"SM\"\r");
     vTaskDelayMs(30);

     sendData("UART", "AT+CSCA=\"+601110499999\",145\r\n");
     vTaskDelayMs(50);

     sendData("APN", "AT+CGDCONT=1,\"IP\",\"unifi\"\r");
     vTaskDelayMs(50);

     sendData("PIN", "AT+CSCA?\r");
     vTaskDelayMs(30);
     sendData("REQ", "AT+CREG?\r");
     vTaskDelayMs(30);
     sendData("PIN", "AT+CPIN?\r");

     vTaskDelayMs(30);

     sendData("UART", "AT+CSCA?\r\n");
     vTaskDelayMs(30);

     sendData("UART", "AT+CGATT?\r\n");
     vTaskDelayMs(30);

     sendData("SMS", "AT+CSMS?\r");
     vTaskDelayMs(30);

     sendData("SMS", "AT+CSMS=1\r");
     vTaskDelayMs(30);

     sendData("SMS", "AT+CSMP?\r");
     vTaskDelayMs(30);

     // sendData("LSMS", "AT+CMGL=\"ALL\"\r");
     // vTaskDelayMs(50);

     sendData("SMS", "AT+CMGF=1\r");
     vTaskDelayMs(50); */

    char phone[] = "+60173168026";

    char sms[] = "Air780E";

    int tot_len = 12 + strlen(phone) + 2 + strlen(sms) + 1;

    sendData("SMS", "AT+CSCS?\r");
    vTaskDelayMs(50);

    readData();

    char at_command[] = "AT+CMGS=\"60173168026\"\r\n";

    char at_command1[] = "Air780ESIM\x1A";

    char at_command2[] = "x1A";

    //  sendData("VOICE", "ATD60173168026\r");
    // vTaskDelayMs(5000);

    // vTaskDelayMs(30);

    /*  const int txBytes = uart_write_bytes(ECHO_UART_PORT_NUM, at_command, strlen(at_command));
     vTaskDelayMs(30);
     ESP_LOGI("SMS", "Wrote %d bytes", txBytes);
     printf("AT CMD== %s\n", at_command);

     const int txBytes1 = uart_write_bytes(ECHO_UART_PORT_NUM, at_command1, strlen(at_command1));
     vTaskDelayMs(100);
     ESP_LOGI("SMS", "Wrote %d bytes", txBytes1);
     printf("AT CMD1== %s\n", at_command1);

     vTaskDelayMs(1000);

     sendData("VOICE", "ATD0173168026\r");
     vTaskDelayMs(1000);
  */
    // sendData("VOICE", "AT+CVOICE?\r");
    // vTaskDelayMs(50);

    //+UGNSINF: 1,1,20250519120934,3.046186,101.640335,69.800,0.48,0.00,3,,2.11,1.03,4.00,,21,14,,,32,,
    sendData("UART", "AT+CGNSPWR=1\r\n");
    vTaskDelayMs(30);
    sendData("GPS1", "AT+CGNSURC=2\r\n");
    vTaskDelayMs(50);
    sendData("GPS", "AT+CGNSINF\r\n");
    vTaskDelayMs(50);

    sendData("NMEA", "AT+CGNSTST=1\r");
    vTaskDelayMs(50);

    /*  sendData("GPSM", "AT+CGNSSMODE=1,1");
     vTaskDelayMs(50);
     sendData("GPSM", "AT+CGNSSMODE=1,2");
     vTaskDelayMs(50);
     sendData("GPSM", "AT+CGNSSMODE=1,4");
     vTaskDelayMs(50); */

    while (1)
    {
        // Read data from the UART

        // sendData("UART","AT+CGMI\r");

        // sendData("GPS", "AT+CGNSINF\r\n");

        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);
        // int len1 = uart_read_bytes(ECHO_UART_PORT_NUM1, data1, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);

        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
        //  uart_write_bytes(ECHO_UART_PORT_NUM1, (const char *)data1, len1);

        if (len > 0)
        {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s \n", (char *)data);
            printf("DATA %s \n", (char *)data);
            mdata = (char *)data;
            tokenizer();
            printf("DATA %s \n", mdata);

            //  snprintf(hu_msg, 80, "%s", mdata);
            //  AccX_get_handler(req);

            int chk_pos = Srh_str(mdata);

            if (chk_pos > 0)
            {
                httpd_req_t *req = NULL;

                strncpy(ydata, mdata + chk_pos + 10, 130);

                AccY_get_handler(req);
            }

            //  char hu_msg1[80];
            // snprintf(hu_msg, 80, "%s", mdata);
            // printf("Ht handler --> %s\n", ht_msg);
            // httpd_resp_send(req, hu_msg, strlen(hu_msg));
            // ESP_LOGI(TAG, "Recv str1 : %s \n", (char *)mdata);
            // ESP_LOG_BUFFER_HEXDUMP("Recv str: ", data, len, ESP_LOG_INFO);
        }

        /*  if (len1 > 0)
         {

             data1[len1] = '\0';
             ESP_LOGI(TAG1, "Recv str1 : %s \n", (char *)data1);
             httpd_req_t *req1 = NULL;
             if (len1 > 10)
             {
                 strncpy(Sdata, (char *)data1, 180);

                 Sensor_get_handler(req1);
             }
         } */
        vTaskDelayMs(100);
    }
}

static void echo_task1(void *arg)
{

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 1, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    //  sendData("BRATE", "AT+IPREX=115200\r\n");

    sendData("AT", "AT\r\n");
    readData();
    sendData("UART", "AT+CSCLK=0\r\n");
    vTaskDelayMs(30);
    readData();
    readData();
    sendData("SIGNAL", "AT+CSQ\r\n");
    readData();

    vTaskDelayMs(30);
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE * 2);

    while (1)
    {

        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);
        // int len1 = uart_read_bytes(ECHO_UART_PORT_NUM1, data1, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);

        // Write data back to the UART

        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);

        //  data[len] = '\0';
        printf("DATAET = %s \n", (char *)data);
        ESP_LOGI(TAG, "Recv strET: %s \n", (char *)data);

        vTaskDelayMs(100);
    }
}

static void uart_sensor_task(void *arg)
{

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, TXD_PIN, RXD_PIN, ECHO_TEST_RTS, ECHO_TEST_CTS));

    //  sendData("BRATE", "AT+IPREX=115200\r\n");

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE * 2);

    while (1)
    {

        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE * 2 - 1), 100 / portTICK_PERIOD_MS);
        // int len1 = uart_read_bytes(ECHO_UART_PORT_NUM1, data1, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);

        // Write data back to the UART

        if (len > 0)
        {

            data[len] = '\0';
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
            // ESP_LOGI(TAG, "Recv strET: %s \n", (char *)data);

            tokenizer1((const char *)data);
            //  vTaskDelayMs(20);

            //  data[len] = '\0';
            //  ssd1306_basic_string(2,15 , (char*)data, 2000, 1, SSD1306_FONT_12);
            // printf("DATAET = %s \n", (char *)data);

            //  printf("data len = %i \n", sizeof((char *)data));
        }

        vTaskDelayMs(1000);
    }
}

static void draw_task(void *arg)
{

    EventBits_t bits;
    int d = 0;

    while (1)
    {
        char arr[100];
        bits = xEventGroupWaitBits(s_bmi_event_group,
                                   CONNECTED_BIT | FAIL_BIT,
                                   pdFALSE,
                                   pdFALSE,
                                   portMAX_DELAY);

        printf("BIT =  %02lx \n", bits);

        sprintf(arr, "%d", d);

        if (bits & CONNECTED_BIT)
        {

            //  ssd1306_basic_string(1, 11 , "D=", 3, 1, SSD1306_FONT_12);
            //  ssd1306_basic_string(20, 11, arr, 10, 1, SSD1306_FONT_16);
            vTaskDelayMs(5000);
            //   ssd1306_basic_clear();

            //  data_draw();
            //  ssd1306_basic_clear();
            xEventGroupClearBits(s_bmi_event_group, CONNECTED_BIT);
        }

        xEventGroupSetBits(s_bmi_event_group, CONNECTED_BIT);
        d++;
    }
}

static void draw_task1()
{

    EventBits_t bits;
    int d = 0;

    uint8_t chk = -1;
    int initf = -1;
    uint8_t res = -1;
    char arr[15];
    while (1)
    {

        sprintf(arr, "%02d", d);

        printf("arr =========  %s \n", arr);
        //  ssd1306_basic_string(1, 11 , "D=", 3, 1, SSD1306_FONT_12);

        //  ssd1306_basic_string(20, 11, "0000", 4, 0, SSD1306_FONT_12);
        //
        ESP_LOGI("MEMORY", "Free memory: %lu bytes", esp_get_free_heap_size());

        if (esp_get_free_heap_size() <= 28000)
        {

            //   res = ssd1306_basic_deinit();
            vTaskDelay(10000 / portTICK_PERIOD_MS);

            printf("deinit status ----> %i \n", res);

            ESP_LOGI("MEMORY", "Free memory ater DEINIT : %lu bytes", esp_get_free_heap_size());

            if (res == 0)
            {
                initf = 0;
                //  ssd1306_interface_iic_deinit1();

                ESP_LOGI("MEMORY", "Free memory ater DEINIT 1: %lu bytes", esp_get_free_heap_size());
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }

            if (initf == 0)
            {
                uint8_t res = -1;
                ESP_LOGI("MEMORY", "Free memory B4 REINIT: %lu bytes", esp_get_free_heap_size());

                // res = ssd1306_basic_init(SSD1306_INTERFACE_IIC, SSD1306_ADDR_SA0_0);
                printf("Reinit ------->  %i \n", res);
                ESP_LOGI("MEMORY", "Free memory AFTER RE INIT: %lu bytes", esp_get_free_heap_size());

                // vTaskDelay(5000 / portTICK_PERIOD_MS);
                //   ssd1306_basic_clear();
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                initf = -1;
            }
        }
        //   if (esp_get_free_heap_size() > 29000)
        // {

        //    chk = ssd1306_basic_string(20, 11, arr, strlen(arr), 2, SSD1306_FONT_12);

        printf("draw string status  ----> %i \n", chk);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // }

        // ESP_LOGI("MEMORY", "Free heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        // ESP_LOGI("MEMORY", "Free DRAM  heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
        ESP_LOGI("MEMORY", "Free memory: %lu bytes", esp_get_free_heap_size());
        //  ssd1306_basic_clear();

        /*   if (esp_get_free_heap_size() <= 30000 || chk != 0)
          {

              res = ssd1306_basic_deinit();
              vTaskDelay(10000 / portTICK_PERIOD_MS);

              printf("deinit status ----> %i \n", res);

              if (res == 0)
              {
                  initf = 0;
                  ssd1306_interface_iic_deinit1();
              }
          }

          if (initf == 0)
          {
              uint8_t res = -1;

              // ssd1306_interface_iic_init();
                //  vTaskDelay(3000 / portTICK_PERIOD_MS);

              res = ssd1306_basic_init(SSD1306_INTERFACE_IIC, SSD1306_ADDR_SA0_0);
              printf("Reinit ------->  %i \n", res);

              // vTaskDelay(5000 / portTICK_PERIOD_MS);
              //   ssd1306_basic_clear();
              vTaskDelay(5000 / portTICK_PERIOD_MS);
              initf = -1;
          } */
        d++;
        //  data_draw();
        //  ssd1306_basic_clear();
    }
}

esp_err_t i2c_master_bus_detect_devices1(i2c_master_bus_handle_t handle)
{
    const uint16_t probe_timeout_ms = 50; // timeout in milliseconds
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);

        for (int j = 0; j < 16; j++)
        {
            fflush(stdout);

            address = i + j;

            esp_err_t ret = i2c_master_probe(handle, address, probe_timeout_ms);

            if (ret == ESP_OK)
            {
                printf("%02x ", address);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                printf("UU ");
            }
            else
            {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return ESP_OK;
}

static void draw_frame(uint8_t pt_x, uint8_t pt_y, uint8_t length, uint8_t width)
{

    if (length >= 127 || pt_x + length >= 127)
    {

        length = 127 - pt_x;
    }

    if (width >= 63 || pt_y + width >= 63)
    {

        width = 63 - pt_y;
    }

    // ssd1306_basic_rect(pt_x, pt_y, pt_x + length, pt_y, 1);

    // ssd1306_basic_rect(pt_x, pt_y, pt_x, pt_y + width, 1);
    // ssd1306_basic_rect(pt_x + length, pt_y, pt_x + length, pt_y + width, 1);
    // ssd1306_basic_rect(pt_x, pt_y + width, pt_x + length, pt_y + width, 1);
}

static void i2c_task(void *arg)
{

    /*   i2c_master_bus_config_t i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
         i2c_master_bus_config_t itesr;

         // i2c0_master_cfg.flags.allow_pd=true;

         i2c_master_bus_handle_t i2c0_bus_hdl;
          i2c_master_dev_handle_t i2c_dev_handle;
         //
         // initialize bmi3 i2c device configuration

     i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
     i2c_master_bus_detect_devices1(i2c0_bus_hdl);
         if (i2c0_bus_hdl == NULL)
         {
             ESP_LOGE(CONFIG_APP_TAG, "i2c address not found , i2c_bus_create handle init failed");

             vTaskDelete(NULL);
             vTaskSuspendAll();
         }


     i2c_device_config_t i2c_dev_conf = {
             .dev_addr_length = I2C_ADDR_BIT_LEN_7,
             .device_address = 0x3c,
             .scl_speed_hz = 100000,
             .scl_wait_us=100000,

             //.scl_speed_hz = I2C_BMI270_DATA_RATE_HZ,
         };
         esp_err_t ret=-1;

      ret=i2c_master_bus_add_device(i2c0_bus_hdl ,&i2c_dev_conf,&i2c_dev_handle);

     printf("i2c check %i\n",ret); */

    // ssd1306_handle_t oled_hdl;
    // oled_hdl.i2c_bus_handle=&i2c0_bus_hd;
    // oled_hdl.i2c_dev_handle=&i2c_dev_handle;

    uint8_t res = -1;
    uint8_t offset = UINT8_C(0);

    //  res = ssd1306_basic_init(SSD1306_INTERFACE_IIC, SSD1306_ADDR_SA0_0);

    printf("oled check %i\n", res);
    //  res = ssd1306_basic_clear();
    ESP_LOGI("MEMORY", "Free memory AFTER 1st INIT: %lu bytes", esp_get_free_heap_size());

    // ssd1306_basic_string(10, 10, "X", 3, 1, SSD1306_FONT_12);
    ESP_LOGI("MEMORY", "Free memory AFTER 1st Draw: %lu bytes", esp_get_free_heap_size());

    // max x =127-font/2, max y=63-font
    /* ssd1306_basic_string(0, 0, "ax", 3, 1, SSD1306_FONT_12);


     ssd1306_basic_deinit();

     res = ssd1306_basic_init(SSD1306_INTERFACE_IIC, SSD1306_ADDR_SA0_0);
      printf("oled init check %i\n", res);


     ssd1306_basic_string(0, 0, "ax", 3, 1, SSD1306_FONT_16);


    */

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    /*    printf("oled clear check %i\n", res);
     res = ssd1306_basic_string(0, 0, "ax", 3, 1, SSD1306_FONT_12);

       res = ssd1306_basic_string(0, 11 + offset, "ay", 3, 1, SSD1306_FONT_12);

      res = ssd1306_basic_string(0, 22 + offset, "az", 3, 1, SSD1306_FONT_12);

      res = ssd1306_basic_string(0, 33 + offset, "gx", 3, 1, SSD1306_FONT_12);

      // res = ssd1306_basic_string(0, 44 + offset, "gy", 3, 1, SSD1306_FONT_12);

       res = ssd1306_basic_string(0, 50 + offset, "gz", 3, 1, SSD1306_FONT_12);
       vTaskDelayMs(1000);
       // ssd1306_basic_string(4, 14, "ESP32-C3 ", 10, 1, SSD1306_FONT_12);

       // draw_frame(1, 28, 128, 64);

       printf("oled draw %i\n", res); */
}

void i2c_master_init()
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .scl_io_num = (gpio_num_t)(10), // Your SCL GPIO
        .sda_io_num = (gpio_num_t)(9),  // Your SDA GPIO
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,

    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    i2c_master_bus_detect_devices1(bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3c,
        .scl_speed_hz = 400000,
        // .flags.disable_ack_check=true
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

int idx = 0;
uint8_t *fdata1 = NULL;

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{

    // printf("MSG ===  %i\n", msg);

    fdata1 = (uint8_t *)malloc(arg_int + 1);

    switch (msg)
    {

    case U8X8_MSG_BYTE_INIT:

        printf("INIT CALL\n");
        /*  if (dev_handle == NULL)
         {
             i2c_device_config_t dev_cfg = {
                 .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                 .device_address = 0x3c,
                 .scl_speed_hz = 400000,
                 .flags.disable_ack_check=true
             };
             ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
         } */
        break;

    case U8X8_MSG_BYTE_SEND:

        for (int i = 0; i <= arg_int; i++)
        {
            fdata1[idx] = *(uint8_t *)(arg_ptr) + i;
            //   data[i] = *((uint8_t *)(arg_ptr) + i);
            // printf(" data1 [%i]--> ,%02x \t ", idx, (uint8_t)fdata1[idx]);
            // fprintf(stderr, "    %d  %d:  %0x\n", i, idx, data[idx]);
            idx++;
        }

        esp_err_t ret = -1;
        ret = i2c_master_transmit(dev_handle, fdata1, UINT16_C(idx), -1);
        vTaskDelay(pdMS_TO_TICKS(20));
        if (ret != ESP_OK)
        {

            printf("I2c Write Failed %i \n", ret);
        }

        //   printf("idx cnt %i\n",idx);
        //   printf("\n");

        break;

    case U8X8_MSG_BYTE_START_TRANSFER:

        idx = 0;

        break;

    case U8X8_MSG_BYTE_END_TRANSFER:

        // printf("END TFER  CALL\n");
        // i2c_master_transmit(dev_handle, (uint8_t *)arg_ptr, sizeof(arg_ptr), -1);

        // No action needed

        break;

    default:
        return 0; // Failure
    }
    return 1; // Success
}

// 4. GPIO/Delay callback
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        break;
        // Add other GPIO handling if needed
    }
    return 1;
}

/* static i2c_cmd_handle_t current_cmd = NULL; // Active command handle

uint8_t u8x8_byte_espidf_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static i2c_master_dev_handle_t dev_handle = NULL;
    static uint8_t i2c_address = 0;
    esp_err_t ret;

    switch (msg)
    {
    case U8X8_MSG_BYTE_INIT:
        if (dev_handle == NULL)
        {
            i2c_address = u8x8_GetI2CAddress(u8x8) >> 1;

            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = i2c_address,
                .scl_speed_hz = 400000,
            };
            ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
            if (ret != ESP_OK)
            {
                ESP_LOGE("U8G2", "Failed to add I2C device: %s", esp_err_to_name(ret));
                return 0;
            }
        }
        break;

    case U8X8_MSG_BYTE_START_TRANSFER:
        // Create new command link
        current_cmd = i2c_cmd_link_create();
        if (current_cmd == NULL)
        {
            ESP_LOGE("U8G2", "Failed to create I2C command link");
            return 0;
        }

        // Start condition
        ret = i2c_master_start(current_cmd);
        if (ret != ESP_OK)
        {
            ESP_LOGE("U8G2", "Failed to add start condition: %s", esp_err_to_name(ret));
            i2c_cmd_link_delete(current_cmd);
            current_cmd = NULL;
            return 0;
        }

        // Device address + write bit
        ret = i2c_master_write_byte(current_cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
        if (ret != ESP_OK)
        {
            ESP_LOGE("U8G2", "Failed to add address: %s", esp_err_to_name(ret));
            i2c_cmd_link_delete(current_cmd);
            current_cmd = NULL;
            return 0;
        }
        break;

    case U8X8_MSG_BYTE_SEND:
    {
        if (current_cmd == NULL)
        {
            ESP_LOGE("U8G2", "Send without active transfer");
            return 0;
        }

        uint8_t *data = (uint8_t *)arg_ptr;
        ret = i2c_master_write(current_cmd, data, arg_int, true);
        if (ret != ESP_OK)
        {
            ESP_LOGE("U8G2", "Failed to write data: %s", esp_err_to_name(ret));
            i2c_cmd_link_delete(current_cmd);
            current_cmd = NULL;
            return 0;
        }
        break;
    }

    case U8X8_MSG_BYTE_END_TRANSFER:
        if (current_cmd == NULL)
        {
            return 0; // No active transfer
        }

        // Stop condition
        ret = i2c_master_stop(current_cmd);
        if (ret != ESP_OK)
        {
            ESP_LOGE("U8G2", "Failed to add stop condition: %s", esp_err_to_name(ret));
            i2c_cmd_link_delete(current_cmd);
            current_cmd = NULL;
            return 0;
        }

        // Execute transaction
        ret = i2c_master_cmd_begin(I2C_NUM_0, current_cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(current_cmd);
        current_cmd = NULL;

        if (ret != ESP_OK)
        {
            ESP_LOGE("U8G2", "I2C transfer failed: %s", esp_err_to_name(ret));
            return 0;
        }
        break;

    case U8X8_MSG_BYTE_SET_DC:
        // Not used for I2C
        break;

    default:
        return 0; // Unknown message
    }
    return 1;
} */

void oled_clear()
{
    // Reset memory addressing
    uint8_t reset_cmd[] = {0x00, 0x20, 0x01}; // Vertical addressing
    i2c_master_transmit(dev_handle, (uint8_t *)reset_cmd, sizeof(reset_cmd), -1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Clear GDDRAM in vertical mode
    uint8_t clear_buffer[129] = {UINT8_C(0x40)};  // Data header
    memset(clear_buffer + 1, UINT8_C(0x00), 128); // Fill with zeros

    for (int page = 0; page < 8; page++)
    {
        // Set page address
        uint8_t page_cmd[] = {0x00, 0xB0 | page};
        i2c_master_transmit(dev_handle, (uint8_t *)page_cmd, sizeof(page_cmd), -1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        uint8_t col_cmd[] = {0x00, 0x21, 0x03, 0x82};
        i2c_master_transmit(dev_handle, (uint8_t *)col_cmd, sizeof(col_cmd), -1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Write 128 zeros to entire page
        i2c_master_transmit(dev_handle, (uint8_t *)clear_buffer, sizeof(clear_buffer), -1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Additional full-display reset
    uint8_t full_reset[] = {0x00, 0xA4}; // Resume to RAM content
    i2c_master_transmit(dev_handle, (uint8_t *)full_reset, sizeof(full_reset), -1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /*     uint8_t full_reset1[] = {0x00, 0xA6}; // Resume to RAM content
     i2c_master_transmit(dev_handle, (uint8_t *)full_reset1, sizeof(full_reset1), -1);
     vTaskDelay(100 / portTICK_PERIOD_MS); */

    uint8_t on_cmd[] = {0x00, 0xAF};
    i2c_master_transmit(dev_handle, (uint8_t *)on_cmd, sizeof(on_cmd), -1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void dump_buffer(u8g2_t *u8g2)
{

    for (int y = 0; y < 8; y++)
    {

        for (int x = 0; x < 16; x++)
        {

            printf("%02x ", u8g2->tile_buf_ptr[y * 16 + x]);
        }
        printf("\n");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Startup...............");
    ESP_LOGI(TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //   wifi_init_sta();
    //   web_server = start_webserver();

    // vTaskDelayMs(5000);
    s_bmi_event_group = xEventGroupCreate();

    //  i2c_task(NULL);
    i2c_master_init();
    uint8_t init_cmds[] = {

        // Display setup
        0xAE, // Display OFF (sleep mode)
        0x00,
        0x10,

        // Fundamental commands
        // 0xD5, 0x80, // Display clock divide ratio/oscillator: default ratio 0x80
        0xD5, 0x80,

        // 0x08, 0x00, // chg
        //  Hardware configuration
        0xA8, 0x3F, // Multiplex ratio: 64 lines (0x3F = 64-1)
        0xD3, 0x00, // Display offset: no vertical shift (0x00)
        0x40,       // Set display start line: 0

        // Power management
        0x8D, 0x14, // Charge pump: ENABLE (0x14) - CRITICAL FOR STABILITY
        0x20, 0x02, // Memory addressing mode: Horizontal

        // Display mapping
        // 0xA1, // Segment remap: column 127 mapped to SEG0 (horizontal flip)
        0xA1,

        // Pin configuration
        0xDA, 0x12, // COM pins hardware: Sequential, disable remap (0x12)

        // Display optimization
        //  0x81, 0x8F, // Contrast control: 0x8F (143) - optimal for 3.3V
        0x81, 0x7F,

        // 0xD9, 0x22, // Pre-charge period: Phase1 = 15, Phase2 = 1
        0xD9, 0xF1,

        //  0xD9, 0x0F,

        // 0x01,0x0F,

        0xDB, 0x20, // VCOMH deselect level: ~0.77 x VCC //chg to 0x20

        // Display behavior
        0xA4, // Entire display ON: resume to RAM content //chr 0xA5
        0xA6, // Normal display (not inverted) //chr to 0xa7

        // 0xC8, // COM output scan direction: remapped mode (vertical flip) // chr 0xc0
        0xC8,

        // Fix 3-pixel horizontal offset (SSD1315 specific)
        0x21, 0x03, 0x82, // Set column address: start = 3, end = 130 (0x82 = 130)
        0x22, 0x00, 0x07, // Set page address: start = 0, end = 7

        // Enable display
        0xAF, // Display ON

        //   0xAE,// DISPLAY OFF

    };

    uint8_t power_seq[] = {0x00, 0x8D, 0x10};

    const uint8_t OLED_Init_CMD[] =
        {0x00,
         0xae, 0x00, 0x10, 0x40, 0x81, 0xcf, 0xa1, 0xc8, 0xa6, 0xa8,
         0x3f, 0xd3, 0x00, 0xd5, 0x80, 0xd9, 0xf1, 0xda, 0x12, 0xdb,
         0x40, 0x20, 0x00, 0x8d, 0x14, 0xa4, 0xa6, 0xaf};

    const uint8_t OLED_Init_CMD1[] =

        {

            0xAE, // Display OFF
            0x00,
            0x10,
            0x40, // Start line at 0
            // 0x8D, 0x10, // disable charge pump
            0xB0,
            0x81,
            0xCF, // Contrast
            0xA1, // Segment re-map

            0xA8,
            0x3F, // Multiplex ratio (64 rows)
            0xC8, // Scan direction
            0xD3,
            0x00, // Offset
            0xD5,
            0x80, // Clock
            0xD9,
            0x22,
            // 0xF1, // Pre-charge
            0xDA,
            0x12,
            // 0x02, // COM hardware config

            0xDB,
            0x30, // VCOMH deselect
            0x8D,
            0x14, // Charge pump
            0x20,
            0x02, // Horizontal addressing mode
                  // 0x21, 0x03, 0x82, // Column address range (shift +3)
                  //  0x22, 0x00, 0x07, // Page address range (0-7)

            0xAF, // Display ON
            // 0xA4, // Entire display ON
            // 0xA6, // Normal display

        };

    const uint8_t init_cmdsx[] = {

        0xAE,
        0x8D,
        0x10,
        0xA4,
        0xA6,
        0x40,
        0xD9,
        0xF1,
        0x81,
        0x8F,
        0xDB,
        0x40,
        0x21,
        0x03,
        0x82, // Set column address: start = 3, end = 130 (0x82 = 130)
        0x22,
        0x00,
        0x07, // Set page address: start = 0, end = 7
        0x8D,
        0x14,
        0xAF,
        0xAE,

    };

    const uint8_t magic_byte[] = {

        0x33, 0x00, 0x90, 0xA5, 0x15, 0x00, 0x3F, 0x75, 0x01, 0x12};

    const uint8_t ad_hoc[] = {

        0x00,
        // 0x33,0x00,0x90,0xA5,0x15,0x00,0x3F,0x75,0x01, 0x12, // magic bytes
        0xA4,
        0xA6,
        0xAE,
        0x2E,

    };

    uint8_t unlock[] = {0x00, 0xFD, 0x12, 0xAE, 0x00, 0x00, 0x00, 0x10, 0x40};

    //  oled_clear();

    //  u8g2_SetI2CAddress(&u8g2, 0x3C * 2);

    //  esp_err_t ret0 = i2c_master_transmit(dev_handle, (uint8_t *)OLED_Init_CMD1, sizeof(OLED_Init_CMD1), -1);
    // vTaskDelay(50 / portTICK_PERIOD_MS);

    //  esp_err_t ret1 = i2c_master_transmit(dev_handle, (uint8_t *)init_cmdsx, sizeof(init_cmdsx), -1);

    //   esp_err_t ret1 = i2c_master_transmit(dev_handle, (uint8_t *)init_cmds, sizeof(init_cmds), -1);
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    //  esp_err_t ret2 = i2c_master_transmit(dev_handle, (uint8_t *)magic_byte, sizeof(magic_byte), -1);

    // esp_err_t ret2 = i2c_master_transmit(dev_handle, (uint8_t *)magic_byte, sizeof(magic_byte), -1);

    // i2c_master_transmit(dev_handle, (uint8_t *)ad_hoc, sizeof(ad_hoc), -1);
    //  vTaskDelay(100 / portTICK_PERIOD_MS);

    //  u8g2_SendF(&u8g2,"ca",0x8D,0x14);

    // u8g2_SendF(&u8g2, "c", 0xAF);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    //   printf(" i2c oled init write %i \n", ret1);
    //    i2c_master_transmit(dev_handle, (uint8_t *)unlock, sizeof(unlock), -1);

    u8g2_t u8g2;
    /*   u8g2_Setup_ssd1315_i2c_128x64_noname_1(
          &u8g2,
          U8G2_R0,
          u8x8_byte_espidf_hw_i2c, // Use our custom callback
          u8x8_gpio_and_delay); */
    // u8g2_SetI2CAddress(u8g2->u8x8, 0x3C * 2);

    u8g2_Setup_ssd1315_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_hw_i2c, // Use our custom callback
        u8x8_gpio_and_delay);


    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    u8g2_SetI2CAddress(&u8g2.u8x8, 0x3C * 2);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    clear_oled();
    u8g2_ClearBuffer(&u8g2);

    SSD1315_INIT();

    clear_oled();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    dump_buffer(&u8g2);
    printf("dump buffer after manual INIT\n");

    //   SSD1315_INIT();
    //  clear_oled();

    /*   for (int i = 5; i < 120; i++)
      {
          u8g2_DrawPixel(&u8g2, i, 60);
          vTaskDelay(2 / portTICK_PERIOD_MS);
      }
   */
    //
    // u8x8_t u8g2;

    // u8x8_Setup(&u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_byte_hw_i2c, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);

    // //

    // // vTaskDelay(100 / portTICK_PERIOD_MS);
    // //  u8g2_ClearDisplay(&u8g2);
    // u8g2_SetContrast(&u8g2, 255);

    //  vTaskDelay(1000 / portTICK_PERIOD_MS);

    /*  u8x8_InitDisplay(&u8g2);
     u8x8_SetPowerSave(&u8g2, 0);

     u8x8_ClearDisplay(&u8g2);

     u8x8_SetFont(&u8g2, u8x8_font_amstrad_cpc_extended_f);
     u8x8_SetFlipMode(&u8g2, 1);
     u8x8_DrawGlyph(&u8g2, 2, 1, 'A');
     u8x8_DrawString(&u8g2, 2, 3, "TEST");

     u8x8_RefreshDisplay(&u8g2); */

    //  u8g2_SetFont(&u8g2, u8g2_font_8x13_mf);

    char *ostr = "1 2 3 4 5 6 7 8 9  ";

    char *ostr1 = "T E S T 1";

    char *ostr2 = "T E S T 2";

    uint8_t colour = 1;

    // u8g2_DrawHLine(&u8g2, 0, 20, 120);
    /*  u8g2_SetDrawColor(&u8g2, colour);

     // u8g2_SetFont(&u8g2, u8g2_font_7x13O_tf);
     u8g2_SetFontRefHeightText(&u8g2);

     u8g2_SetFontDirection(&u8g2, 0);
     u8g2_SetFontMode(&u8g2, 0); */
    // u8g2_DrawBox(&u8g2, 4, 4, 120, 16); // Background
    //  u8g2_DrawHLine(&u8g2, 40, 10, 100);
    //  u8g2_DrawCircle(&u8g2, 60, 30, 40, U8G2_DRAW_ALL);
    //  vTaskDelay(20 / portTICK_PERIOD_MS);
    //  u8g2_DrawUTF8(&u8g2, 3, 15, (char *)ostr);

    //  u8g2_DrawStr(&u8g2, 30, 35, ostr);
    // u8g2_DrawHLine(&u8g2, 40, 20, 30);

    //   u8g2_DrawStr(&u8g2, 5, 45, (char *)ostr2);

    printf("Display Height --->%i \t", u8g2_GetDisplayHeight(&u8g2));
    printf("Display Width --->%i \n", u8g2_GetDisplayWidth(&u8g2));

    /*    for (int y = 0; y < u8g2_GetDisplayHeight(&u8g2); y += 8)
       {
           for (int x = 0; x < u8g2_GetDisplayWidth(&u8g2); x += 8)
           {
               u8g2_DrawBox(&u8g2, x, y, 4, 4);
           }
       }  */

    ESP_LOGI("MEMORY", "Free memory: %lu bytes", esp_get_free_heap_size());

    // u8g2_UpdateDisplay(&u8g2);

    //

    /*

        u8g2_FirstPage(&u8g2);
        do
        {
            u8g2_SetFont(&u8g2, u8g2_font_5x7_mn);
            u8g2_SetFontDirection(&u8g2, 0);
            u8g2_SetFontMode(&u8g2, 0);
            // Draw grid pattern
            for (int y = 0; y < u8g2_GetDisplayHeight(&u8g2); y += 8)
            {
                for (int x = 0; x < u8g2_GetDisplayWidth(&u8g2); x += 8)
                {
                    u8g2_DrawBox(&u8g2, x, y, 4, 4);
                }
            }

            // Draw border
            u8g2_DrawFrame(&u8g2, 0, 0,
                           u8g2_GetDisplayWidth(&u8g2),
                           u8g2_GetDisplayHeight(&u8g2));

            // Draw text

            u8g2_DrawStr(&u8g2, 2, 10, "OLED TEST");
        } while (u8g2_NextPage(&u8g2));
*/
    /*     u8g2_FirstPage(&u8g2);
        do
        {
            u8g2_SetDrawColor(&u8g2, 1);

    u8g2_DrawHLine(&u8g2, 0, 30, 120);


             for (int y = 0; y < 64; y++)
           {

               for (int x = 0; x < 128; x++)
               {
                   // printf("X =%i, Y= %i \t",x,y);
                   u8g2_DrawPixel(&u8g2, x, y);
               }
            }
               for (int y = 0; y < u8g2_GetDisplayHeight(&u8g2); y += 8)
              {
                  for (int x = 0; x < u8g2_GetDisplayWidth(&u8g2); x += 8)
                  {
                      u8g2_DrawBox(&u8g2, x, y, 4, 4);
                  }
              }

        } while (u8g2_NextPage(&u8g2)); */

    u8g2_SendBuffer(&u8g2);
    dump_buffer(&u8g2);
    printf("\n");

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //   ESP_LOGI("MAIN", "Display updated successfully");

    //   while (1)
    //       vTaskDelay(portMAX_DELAY);

    //  ESP_LOGI("MEMORY", "Free memory B4 Draw Task1 : %lu bytes", esp_get_free_heap_size());
    // draw_task1();
    // xEventGroupSetBits(s_bmi_event_group, CONNECTED_BIT);

    // xTaskCreate(echo_task1, "uart_7670C_task", ECHO_TASK_STACK_SIZE, NULL, 1, NULL);

    //  xTaskCreate(echo_AIR780_task, "uart_780_task", ECHO_TASK_STACK_SIZE, NULL, 1, NULL);

    // xTaskCreate(uart_sensor_task, "uart_sensor_task", ECHO_TASK_STACK_SIZE + 1024 * 2, NULL, 1, NULL);

    // draw_task1();
    // xTaskCreate(draw_task, "uart_draw_task", ECHO_TASK_STACK_SIZE + 1024 * 3, NULL, 2, NULL);

    /*  init_uart();
     // power_a7670();
     //   Sleep_a7670();
     sendData("UART", "AT\r\n");
     //  vTaskDelayMs(30);

     // sendData("RST", "AT+CRESET\r\n");
     // vTaskDelayMs(1000);

     sendData("UART", "ATE1\r\n");
     //  vTaskDelayMs(30);
     sendData("UART", "AT+CGMM\r\n");

     sendData("UART", "ATI\r\n");
     //   vTaskDelayMs(100);

     // readData();
     sendData("UART", "AT+CGNSPWR?\r\n");
     //  vTaskDelayMs(30);
     //   readData();
     sendData("SIGNAL", "AT+CSQ\r\n");
     //   vTaskDelayMs(30);

     //   readData();
     while (1)
     {

         uint8_t *data = (uint8_t *)malloc(BUF_SIZE * 1);
         int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);
         // int len1 = uart_read_bytes(ECHO_UART_PORT_NUM1, data1, (BUF_SIZE * 1 - 1), 100 / portTICK_PERIOD_MS);

         // Write data back to the UART
         uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
         //  uart_write_bytes(ECHO_UART_PORT_NUM1, (const char *)data1, len1);

         if (len > 0)
         {
             data[len] = '\0';
             ESP_LOGI(TAG, "Recv str: %s \n", (char *)data);
             printf("DATA %s \n", (char *)data);
         }

         vTaskDelayMs(100);
     } */
}

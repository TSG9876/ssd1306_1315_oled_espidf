/* U8g2 Oled  Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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


#include <driver/i2c_master.h>
#include "u8g2.h"

extern const uint8_t u8g2_font_helvB10_tr[] U8G2_FONT_SECTION("u8g2_font_helvB10_tr");
extern const uint8_t u8g2_font_8x13_tf[] U8X8_FONT_SECTION(" u8g2_font_8x13_tf");

u8g2_t u8g2;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;



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



void i2c_master_init()
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = (gpio_num_t)(12), // Your SCL GPIO
        .sda_io_num = (gpio_num_t)(11), // Your SDA GPIO
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,

    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    i2c_master_bus_detect_devices1(bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3c, // Your OLED I2C address
        .scl_speed_hz = 400000,
       
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

int idx = 0;
uint8_t fdata1[55] = {0x00};

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32]; /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buf_ndx;
    uint8_t *data;

    switch (msg)
    {

    case U8X8_MSG_BYTE_INIT:

     
        break;

    case U8X8_MSG_BYTE_SEND:
    {
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_ndx++] = *data;
            data++;
            arg_int--;
        }
        break;
    }
    case U8X8_MSG_BYTE_START_TRANSFER:

        buf_ndx = 0;
     

        break;

    case U8X8_MSG_BYTE_END_TRANSFER:

    {
        esp_err_t ret = -1;
    
        ret = i2c_master_transmit(dev_handle, buffer, buf_ndx, -1);
        vTaskDelay(pdMS_TO_TICKS(20));
        idx = 0;
        if (ret != ESP_OK)
        {

            printf("I2c Write Failed %i \n", ret);
        }

        // No action needed

        break;
    }
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



void u8g2_task()
{

    u8g2_Setup_ssd1315_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_hw_i2c, // Use our custom callback
        u8x8_dummy_cb);
    u8g2_SetI2CAddress(&u8g2, 0x3C * 2);
   

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

 
 
    u8g2_ClearBuffer(&u8g2);

    vTaskDelay(30 / portTICK_PERIOD_MS);

    for (int y = 0; y < u8g2_GetDisplayHeight(&u8g2) / 2; y += 2)
    {

        u8g2_DrawFrame(&u8g2, y, y, u8g2_GetDisplayWidth(&u8g2) - 2 * y, u8g2_GetDisplayHeight(&u8g2) - 2 * y);
    }
  u8g2_SendBuffer(&u8g2);
  vTaskDelay(50 / portTICK_PERIOD_MS);

    u8g2_SetDrawColor(&u8g2, 0);

    for (int y = 0; y < u8g2_GetDisplayHeight(&u8g2) / 2; y += 3)
    {

        u8g2_DrawFrame(&u8g2, y, y, u8g2_GetDisplayWidth(&u8g2) - 2 * y, u8g2_GetDisplayHeight(&u8g2) - 2 * y);
    }

    u8g2_SendBuffer(&u8g2);
    dump_buffer(&u8g2);
    vTaskDelay(8000 / portTICK_PERIOD_MS);
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
i2c_master_init();
   u8g2_task();
}

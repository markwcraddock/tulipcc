#include "esp_lcd_touch_gt911.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_PORT       I2C_NUM_0
#define I2C_FREQ_HZ    100000
#define I2C_SDA        18
#define I2C_SCL        8

#define TOUCH_RST      2
#define TOUCH_INT      16
#define TOUCH_ADDR     0x14

#define SCREEN_WIDTH   320
#define SCREEN_HEIGHT  240

esp_lcd_touch_handle_t tp;

static void gt911_gpio_reset_sequence() {
    gpio_config_t rst_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(TOUCH_RST)
    };
    gpio_config(&rst_cfg);

    gpio_config_t int_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(TOUCH_INT)
    };
    gpio_config(&int_cfg);

    gpio_set_level(TOUCH_INT, 0);     // LOW = 0x14
    gpio_set_level(TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ets_delay_us(100);
    gpio_set_level(TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(6));

    gpio_config_t int_in = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(TOUCH_INT)
    };
    gpio_config(&int_in);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t touch_init() {
    // --- I2C Setup ---
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_conf.mode, 0, 0, 0));

    // --- Touch Reset ---
    gt911_gpio_reset_sequence();

    // --- Touch IO Setup ---
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT, &io_config, &io_handle));

    // --- GT911 Config ---
    esp_lcd_touch_io_gt911_config_t gt911_cfg = {
        .dev_addr = TOUCH_ADDR
    };

    esp_lcd_touch_config_t touch_cfg = {
        .x_max = SCREEN_WIDTH,
        .y_max = SCREEN_HEIGHT,
        .rst_gpio_num = TOUCH_RST,
        .int_gpio_num = TOUCH_INT,
        .levels = {
            .reset = 1,
            .interrupt = 0
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 0
        },
        .driver_data = &gt911_cfg
    };

    return esp_lcd_touch_new_i2c_gt911(io_handle, &touch_cfg, &tp);
}

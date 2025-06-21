// Modified gt911 driver for Tdeck, created by ChatGPT and Mark Craddock
#include "display.h"
#include "gt911_touchscreen.h"
#include "pins.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h" // for ets_delay_us

#ifndef TDECK
int16_t touch_x_delta = -2;
int16_t touch_y_delta = -12;
float touch_y_scale = 0.8f;
#else
int16_t touch_x_delta = 0;
int16_t touch_y_delta = 0;
float touch_y_scale = 1.0f;
#endif

esp_lcd_touch_handle_t tp;

static void gt911_reset(uint8_t reset_level, uint8_t int_level) {
    gpio_config_t rst_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TOUCH_RST)
    };
    gpio_config_t int_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TOUCH_INT)
    };
    gpio_config(&rst_cfg);
    gpio_config(&int_cfg);

    gpio_set_level(TOUCH_INT, int_level);
    gpio_set_level(TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ets_delay_us(100);
    gpio_set_level(TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(6));

    gpio_config_t int_in = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << TOUCH_INT)
    };
    gpio_config(&int_in);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t touch_init(uint8_t alternate) {
    tp = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_CLK_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    uint8_t dev_addr = alternate ? 0x14 : 0x5D;

    gt911_reset(1, alternate ? 0 : 1);  // 0x14 = INT LOW; 0x5D = INT HIGH

    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = dev_addr,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM, &io_config, &io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = H_RES,
        .y_max = V_RES,
        .rst_gpio_num = TOUCH_RST,
        .int_gpio_num = TOUCH_INT,
        .levels = {
#if defined(TDECK) || defined(MATOUCH7) || defined(TULIP4_R11)
            .reset = 1,
#else
            .reset = 0,
#endif
            .interrupt = 0,
        },
        .flags = {
#ifdef TDECK
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 0,
#else
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
#endif
        },
        .driver_data = &tp_gt911_config,
    };

    return esp_lcd_touch_new_i2c_gt911(io_handle, &tp_cfg, &tp);
}

uint8_t gt911_held = 0;
extern void set_pin(uint16_t pin, uint8_t value);

void run_gt911(void *param) {
    uint16_t touch_x[3];
    uint16_t touch_y[3];
    uint16_t touch_strength[3];
    uint8_t touch_cnt = 0;
    uint8_t touchscreen_ok = 0;

#ifndef TDECK
    delay_ms(2000);
#endif

    if (touch_init(0) == ESP_OK) {
        fprintf(stderr, "touchscreen OK at 0x5D\n");
        touchscreen_ok = 1;
    } else {
        fprintf(stderr, "attempting fallback to 0x14\n");
        delay_ms(500);
        if (touch_init(1) == ESP_OK) {
            fprintf(stderr, "touchscreen OK at 0x14\n");
            touchscreen_ok = 1;
        } else {
            fprintf(stderr, "Touchscreen failed on both 0x5D and 0x14\n");
        }
    }

    while (1) {
        if (touchscreen_ok) {
            esp_lcd_touch_read_data(tp);
            if (esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 3)) {
                for (uint8_t i = 0; i < touch_cnt; i++) {
                    last_touch_x[i] = touch_x[i] + touch_x_delta;
#ifdef TDECK
                    last_touch_y[i] = ((V_RES - touch_y[i]) + touch_y_delta) * touch_y_scale;
#else
                    last_touch_y[i] = (touch_y[i] + touch_y_delta) * touch_y_scale;
#endif
                }
                for (uint8_t i = touch_cnt; i < 3; i++) {
                    last_touch_x[i] = -1;
                    last_touch_y[i] = -1;
                }
                send_touch_to_micropython(last_touch_x[0], last_touch_y[0], 0);
                gt911_held = 1;
            } else {
                if (gt911_held) {
                    send_touch_to_micropython(last_touch_x[0], last_touch_y[0], 1);
                    gt911_held = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

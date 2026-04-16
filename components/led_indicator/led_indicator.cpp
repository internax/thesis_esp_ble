#include "led_indicator.hpp"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "led_indicator";

// Jas (0–255). Nízká hodnota šetří oči při testování.
static constexpr uint8_t BRIGHTNESS = 32;

namespace led {

LedIndicator::LedIndicator(gpio_num_t gpio, uint32_t led_count)
    : gpio_(gpio), led_count_(led_count), strip_(nullptr), state_queue_(nullptr)
{}

void LedIndicator::start()
{
    state_queue_ = xQueueCreate(1, sizeof(LedState));
    configASSERT(state_queue_);

    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num            = static_cast<int>(gpio_);
    strip_cfg.max_leds                  = led_count_;
    strip_cfg.led_model                 = LED_MODEL_WS2812;
    strip_cfg.color_component_format    = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_cfg.flags.invert_out          = false;

    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rmt_cfg.resolution_hz     = 10 * 1000 * 1000; // 10 MHz
    rmt_cfg.mem_block_symbols = 64;
    rmt_cfg.flags.with_dma    = false;

    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init failed: %s", esp_err_to_name(err));
        return;
    }

    led_strip_clear(strip_);

    xTaskCreate(led_task, "led_indicator", 2048, this, 3, nullptr);
}

void LedIndicator::show(LedState state)
{
    xQueueOverwrite(state_queue_, &state);
}

// -------------------------------------------------------------------------

void LedIndicator::set_color(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i = 0; i < led_count_; i++) {
        led_strip_set_pixel(strip_, i, r, g, b);
    }
    led_strip_refresh(strip_);
}

void LedIndicator::clear()
{
    led_strip_clear(strip_);
}

// -------------------------------------------------------------------------

void LedIndicator::led_task(void *arg)
{
    static_cast<LedIndicator *>(arg)->led_loop();
    vTaskDelete(nullptr);
}

void LedIndicator::led_loop()
{
    LedState   current  = LedState::IDLE;
    bool       blink_on = false;
    TickType_t timeout  = 0; // první průchod okamžitě zpracuje IDLE (rozsvítí bílou)

    while (true) {
        LedState next;
        bool got_new = (xQueueReceive(state_queue_, &next, timeout) == pdTRUE);

        if (got_new) {
            current  = next;
            blink_on = false;
        }

        switch (current) {

        case LedState::IDLE:
            set_color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS); // bílá
            timeout = portMAX_DELAY;
            break;

        case LedState::IN_PROGRESS:
            blink_on = !blink_on;
            if (blink_on)
                set_color(0, 0, BRIGHTNESS); // modrá
            else
                clear();
            timeout = pdMS_TO_TICKS(400);
            break;

        case LedState::SUCCESS:
            if (got_new) {
                set_color(0, BRIGHTNESS, 0); // zelená
                timeout = pdMS_TO_TICKS(2000);
            } else {
                current = LedState::IDLE;
                timeout = 0; // okamžitě zpracuj IDLE (rozsvítí bílou)
            }
            break;

        case LedState::FAILED:
            if (got_new) {
                set_color(BRIGHTNESS, 0, 0); // červená
                timeout = pdMS_TO_TICKS(2000);
            } else {
                current = LedState::IDLE;
                timeout = 0; // okamžitě zpracuj IDLE (rozsvítí bílou)
            }
            break;
        }
    }
}

} // namespace led

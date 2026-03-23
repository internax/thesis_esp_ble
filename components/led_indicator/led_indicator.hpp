#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "led_strip.h"

namespace led {

enum class LedState : uint8_t {
    OFF,          // zhasnuto
    IN_PROGRESS,  // modrá blikající — čeká se na NFC tag
    SUCCESS,      // zelená 2 s, pak zhasne
    FAILED,       // červená 2 s, pak zhasne
};

class LedIndicator {
public:
    /**
     * @param gpio       GPIO pin připojený na DIN WS2812B
     * @param led_count  Počet LED v pásku (typicky 1)
     */
    explicit LedIndicator(gpio_num_t gpio, uint32_t led_count = 1);

    /**
     * Inicializuje led_strip přes RMT a spustí FreeRTOS task.
     * Volat jednou po inicializaci periferií.
     */
    void start();

    /**
     * Přepne stav LED. Thread-safe — volatelné z libovolného tasku nebo callbacku.
     * Pokud fronta je plná, nový stav ji přepíše.
     */
    void show(LedState state);

private:
    gpio_num_t         gpio_;
    uint32_t           led_count_;
    led_strip_handle_t strip_;
    QueueHandle_t      state_queue_;

    static void led_task(void *arg);
    void        led_loop();

    void set_color(uint8_t r, uint8_t g, uint8_t b);
    void clear();
};

} // namespace led

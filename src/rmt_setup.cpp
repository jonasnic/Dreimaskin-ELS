#include "rmt_setup.h"

void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel) {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = channel;
    config.gpio_num = pulsePin;
    config.clk_div = 80; // 1 tick = 1us (80MHz / 80)

    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

// rmt_setup.cpp - Implementation of RMT setup and control functions
#include "rmt_setup.h"
#include "driver/rmt.h"
#include <string.h> // for memcpy

#define next_buffer_index (activeBufferIndex ^ 1)
static volatile rmt_item32_t *rmt_memory;
static volatile rmt_item32_t *rmt_buffers[BUFFERS_COUNT];
static volatile uint8_t activeBufferIndex = 0;
static volatile uint32_t steps_in_buffer[BUFFERS_COUNT] = {0,0};

const rmt_item32_t nopulse_item = RMT_ITEM(RMT_TICK_1US, 0, RMT_TICK_1US, 0);
const rmt_item32_t pulse_item_template = RMT_ITEM(RMT_TICK_1US * 2, 1, RMT_TICK_1US * 98, 0);

// User callback hook (motion.cpp sets this)
void (*rmt_user_callback)(rmt_callback_arg_t *arg) = nullptr;

// ------------------ Internal helpers ------------------
static uint32_t get_write_buffer() {
    return activeBufferIndex ^ 1; // return the inactive buffer
}

// fills next buffer with pulses + rest nopulse
void fill_next_buff_with_steps_items(uint32_t count, uint32_t low_tics) {
    uint32_t buf_index = get_write_buffer();
    volatile rmt_item32_t *buf = rmt_buffers[buf_index]; // fixed

    for (uint32_t i = 0; i < count; i++) {
        memcpy((void *)&buf[i], &pulse_item_template, sizeof(rmt_item32_t));
        buf[i].duration1 = low_tics; // can still modify
    }

    for (uint32_t i = count; i < RMT_STEPS_PER_BUFFER; i++) {
        memcpy((void *)&buf[i], &nopulse_item, sizeof(rmt_item32_t));
    }
}

// fill next buffer with individual low_ticks values
// low_ticks_array should have at least 'count' elements
void fill_next_buff_with_steps_items_array(uint32_t count, const uint32_t* low_ticks_array) {
    uint32_t buf_index = get_write_buffer();
    volatile rmt_item32_t *buf = rmt_buffers[buf_index];

    for (uint32_t i = 0; i < count; i++) {
        memcpy((void*)&buf[i], &pulse_item_template, sizeof(rmt_item32_t));
        buf[i].duration1 = low_ticks_array[i];
    }
    for (uint32_t i = count; i < RMT_STEPS_PER_BUFFER; i++) {
        memcpy((void*)&buf[i], &nopulse_item, sizeof(rmt_item32_t));
    }
}

void prefill_nopulse_items() {
    for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {
        volatile rmt_item32_t *buf = rmt_buffers[b];
        for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
            memcpy((void *)&buf[i], &nopulse_item, sizeof(rmt_item32_t));
        }
    }
}

// ------------------ ISR ------------------
void IRAM_ATTR rmt_tx_end_isr(void *arg) {
    uint32_t steps_done = steps_in_buffer[activeBufferIndex];
    // toggle active buffer
    activeBufferIndex ^= 1;

    // call motion-specific callback
    if (rmt_user_callback) {
        rmt_callback_arg_t arg = { .steps_done = steps_done };
        rmt_user_callback(&arg);
    }
}

// ------------------ Public functions ------------------
void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel) {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = channel;
    config.gpio_num = pulsePin;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = BUFFERS_COUNT; // two blocks for ping-pong
    config.tx_config.loop_en = true;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);

    // register ISR
    rmt_isr_register(rmt_tx_end_isr, nullptr, ESP_INTR_FLAG_IRAM, nullptr);
    rmt_set_tx_thr_intr_en(config.channel, true, RMT_STEPS_PER_BUFFER);

    // point buffers to hardware memory
    rmt_memory = RMTMEM.chan[RMT_CH].data32;
    rmt_buffers[0] = RMTMEM.chan[RMT_CH].data32;
    rmt_buffers[1] = RMTMEM.chan[RMT_CH].data32 + RMT_STEPS_PER_BUFFER;
}

void startRMT() {
    RMT.conf_ch[RMT_CH].conf1.mem_wr_rst = 1; // reset memory pointer
    prefill_nopulse_items();                  // fill buffers before TX
    rmt_tx_start(RMT_CH, true);
}

void loadNextBuffer(uint32_t count, uint32_t low_tics) {
    steps_in_buffer[next_buffer_index] = count;
    fill_next_buff_with_steps_items(count, low_tics);
}

void loadNextBufferWithArray(uint32_t count, const uint32_t* low_ticks_array) {
    steps_in_buffer[next_buffer_index] = count;   
    fill_next_buff_with_steps_items_array(count, low_ticks_array);
}
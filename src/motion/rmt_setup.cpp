// rmt_setup.cpp - Implementation of RMT setup and control functions
#include "rmt_setup.h"
#include <Arduino.h>
#include "driver/rmt.h"

static volatile rmt_item32_t *rmt_memory;
static volatile rmt_item32_t *rmt_buffers[BUFFERS_COUNT];
static volatile uint8_t currentTxBufferIndex = 0;
static volatile uint8_t nextWriteBufferIndex = 0;
static volatile uint32_t steps_in_buffer[BUFFERS_COUNT] = {0, 0};

// RMT uses the 80 MHz APB clock on ESP32, not CPU_CLK_FREQ.
static constexpr uint32_t kRmtTicksPerUs = 80 / RMT_CLK_DIV;
static constexpr uint32_t kHighPulseTicks = 2 * kRmtTicksPerUs;

DRAM_ATTR static const rmt_item32_t nopulse_item = RMT_ITEM(kRmtTicksPerUs, 0, kRmtTicksPerUs, 0);
DRAM_ATTR static const rmt_item32_t pulse_item_template = RMT_ITEM(kHighPulseTicks, 1, kRmtTicksPerUs * 98, 0);

// User callback hook (motion.cpp sets this)
void (*rmt_user_callback)(rmt_callback_arg_t *arg) = nullptr;

// ------------------ Internal helpers ------------------
static void IRAM_ATTR fill_buffer_with_steps_items(uint32_t buf_index, uint32_t count, uint32_t low_tics) {
    volatile rmt_item32_t *buf = rmt_buffers[buf_index];

    for (uint32_t i = 0; i < count; i++) {
        buf[i].val = pulse_item_template.val;
        buf[i].duration1 = low_tics; // can still modify
    }

    for (uint32_t i = count; i < RMT_STEPS_PER_BUFFER; i++) {
        buf[i].val = nopulse_item.val;
    }
}

// fill next buffer with individual low_ticks values
// low_ticks_array should have at least 'count' elements
static void IRAM_ATTR fill_buffer_with_steps_items_array(uint32_t buf_index, uint32_t count, const uint32_t *low_ticks_array) {
    volatile rmt_item32_t *buf = rmt_buffers[buf_index];

    for (uint32_t i = 0; i < count; i++) {
        buf[i].val = pulse_item_template.val;
        buf[i].duration1 = low_ticks_array[i];
    }
    for (uint32_t i = count; i < RMT_STEPS_PER_BUFFER; i++) {
        buf[i].val = nopulse_item.val;
    }
}

static void prefill_nopulse_items() {
    for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {
        volatile rmt_item32_t *buf = rmt_buffers[b];
        for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
            buf[i].val = nopulse_item.val;
        }
    }
}

// ------------------ ISR ------------------
void IRAM_ATTR rmt_tx_end_isr(void *arg) {
    (void)arg;
    // Check which interrupt fired and clear it; ignore if not our channel's threshold event
    const uint32_t thr_bit = 1u << (24 + RMT_CH);
    if (!(RMT.int_st.val & thr_bit)) return;
    RMT.int_clr.val = thr_bit;

    uint32_t steps_done = steps_in_buffer[currentTxBufferIndex];
    bool standing_still = (steps_done == 0);
    currentTxBufferIndex ^= 1;

    // call motion-specific callback
    if (rmt_user_callback) {
        rmt_callback_arg_t cb_arg = {
            .steps_done = steps_done,
            .standing_still = standing_still};
        rmt_user_callback(&cb_arg);
    }
}

// ------------------ Public functions ------------------
void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel) {
    pinMode((uint8_t)pulsePin, OUTPUT);
    digitalWrite((uint8_t)pulsePin, LOW);

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
    rmt_memory = RMTMEM.chan[channel].data32;
    rmt_buffers[0] = RMTMEM.chan[channel].data32;
    rmt_buffers[1] = RMTMEM.chan[channel].data32 + RMT_STEPS_PER_BUFFER;
}

void startRMT() {
    currentTxBufferIndex = 0;
    nextWriteBufferIndex = 0;
    steps_in_buffer[0] = 0;
    steps_in_buffer[1] = 0;

    prefill_nopulse_items();

    // Raw continuous mode works reliably here when we reset the read pointer
    // and start TX directly, matching the known-good standalone test project.
    RMT.conf_ch[RMT_CH].conf1.mem_rd_rst = 1;
    RMT.conf_ch[RMT_CH].conf1.mem_rd_rst = 0;
    RMT.conf_ch[RMT_CH].conf1.tx_start = 1;
}

void IRAM_ATTR loadNextBuffer(uint32_t count, uint32_t low_tics) {
    const uint32_t write_buffer = nextWriteBufferIndex;
    steps_in_buffer[write_buffer] = count;
    fill_buffer_with_steps_items(write_buffer, count, low_tics);
    nextWriteBufferIndex ^= 1;
}

void IRAM_ATTR loadNextBufferWithArray(uint32_t count, const uint32_t *low_ticks_array) {
    const uint32_t write_buffer = nextWriteBufferIndex;
    steps_in_buffer[write_buffer] = count;
    fill_buffer_with_steps_items_array(write_buffer, count, low_ticks_array);
    nextWriteBufferIndex ^= 1;
}
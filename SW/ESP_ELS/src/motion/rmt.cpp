// rmt_setup.cpp - Implementation of RMT setup and control functions
#include "rmt.h"
#include "bitpattern_table.h"
#include "driver/rmt.h"
#include <Arduino.h>

static volatile rmt_item32_t *rmt_memory;
static volatile rmt_item32_t *rmt_buffers[BUFFERS_COUNT];
static volatile uint8_t currentTxBufferIndex;
static volatile uint8_t nextWriteBufferIndex;
static volatile uint32_t steps_in_buffer[BUFFERS_COUNT] = {0, 0};

// User callback hook (motion.cpp sets this)
void (*rmt_user_callback)(rmt_callback_arg_t *arg) = nullptr;

uint64_t BitPatternPrCount(uint32_t count);

static void prefill_nopulse_items() {
    // for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {
    for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {
        volatile rmt_item32_t *buf = rmt_buffers[b];
        for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
            buf[i].val = NOPULS_ITEM_TEMPLATE.val;
        }
    }
}

bool ready_to_change_direction() {
    // we can change direction when we are standing still, which is when the RMT just finished transmitting a buffer with 0 steps, which we detect in the ISR callback when it sets standing_still to true, so we can safely change direction without overshooting the target
    return steps_in_buffer[0] == 0 && steps_in_buffer[1] == 0;
}
// MARK: ISR
//  ------------------ ISR ------------------
void IRAM_ATTR rmt_tx_end_isr(void *arg) {
    
    // Check which interrupt fired and clear it; ignore if not our channel's threshold event
    const uint32_t thr_bit = 1u << (24 + RMT_CH);
    if (!(RMT.int_st.val & thr_bit)) return;

    RMT.int_clr.val = thr_bit;
    uint32_t steps_done = steps_in_buffer[currentTxBufferIndex];
    steps_in_buffer[currentTxBufferIndex] = 0;

    // clear the just-transmitted buffer to avoid ghost steps if we ever read from it again before filling
    volatile rmt_item32_t *buf = rmt_buffers[currentTxBufferIndex];
    for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
        buf[i].val = NOPULS_ITEM_TEMPLATE.val;
    }

    bool standing_still = (steps_done == 0);
    currentTxBufferIndex ^= 1;
    nextWriteBufferIndex ^= 1;

    // call motion-specific callback
    if (rmt_user_callback) {
        rmt_callback_arg_t cb_arg = {
            .steps_done = steps_done,
            .standing_still = standing_still};
        rmt_user_callback(&cb_arg);
    }
}

// MARK: SETUP
//  ------------------ Public functions ------------------
void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel) {
    digitalWrite((uint8_t)pulsePin, LOW);
    pinMode((uint8_t)pulsePin, OUTPUT);
    // pinMode(23, OUTPUT); // Debug pin for ISR timing

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
    // rmt_memory = RMTMEM.chan[channel].data32;
    rmt_buffers[0] = RMTMEM.chan[channel].data32;
    rmt_buffers[1] = RMTMEM.chan[channel + 1].data32; // next block for ping-pong
}

void startRMT() {
    currentTxBufferIndex = 0;
    nextWriteBufferIndex = 1; // we will write to the second buffer first, while the first buffer is being transmitted, to achieve continuous output without gaps
    steps_in_buffer[0] = 0;
    steps_in_buffer[1] = 0;

    prefill_nopulse_items();

    // Raw continuous mode works reliably here when we reset the read pointer
    // and start TX directly, matching the known-good standalone test project.
    RMT.conf_ch[RMT_CH].conf1.mem_rd_rst = 1;
    RMT.conf_ch[RMT_CH].conf1.mem_rd_rst = 0;
    RMT.conf_ch[RMT_CH].conf1.tx_start = 1;
}

// MARK: Buffer loading
void loadNextBuffer(uint32_t count) {
    const uint32_t write_buffer = nextWriteBufferIndex;
    if (count != 0) {
        uint64_t puls_pattern = kBitPatterns[count - 1]; // get the bit pattern for the given count from the table for uniform distribution of pulses in the buffer
        uint8_t noPulsCounter = 0;
        for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
            if ((puls_pattern & (1ULL << (63 - i))) != 0) {
                rmt_buffers[write_buffer][i].val = PULS_ITEM_TEMPLATE.val;
            }
        }
    }
    steps_in_buffer[write_buffer] = count;
}

void printRMTBuffer(uint32_t buf_index) {
    volatile rmt_item32_t *buf = rmt_buffers[buf_index];
    Serial.printf("Buffer %u:\n", buf_index);
    for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
        Serial.printf("  Item %2u: val=0x%08X, duration0=%5u, level0=%u, duration1=%5u, level1=%u\n",
                      i, buf[i].val, buf[i].duration0, (buf[i].val >> 15) & 1, buf[i].duration1, (buf[i].val >> 31) & 1);
    }
}

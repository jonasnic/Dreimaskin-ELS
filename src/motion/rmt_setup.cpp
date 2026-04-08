// rmt_setup.cpp - Implementation of RMT setup and control functions
#include "rmt_setup.h"
#include "driver/rmt.h"
#include <Arduino.h>

static volatile rmt_item32_t *rmt_memory;
static volatile rmt_item32_t *rmt_buffers[BUFFERS_COUNT];
static volatile uint8_t currentTxBufferIndex = 0;
static volatile uint8_t nextWriteBufferIndex = 0;
static volatile uint32_t steps_in_buffer[BUFFERS_COUNT] = {0, 0};


DRAM_ATTR static const rmt_item32_t nopulse_item = RMT_ITEM(RMT_TICKS_1US, 0, RMT_TICKS_1US, 0);
DRAM_ATTR static const rmt_item32_t pulse_item_template = RMT_ITEM(RMT_HIGH_PULSE_TICKS, 1, RMT_TICKS_1US * 198, 0);

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
    // for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {
    for (uint8_t b = 0; b < BUFFERS_COUNT; b++) {

        volatile rmt_item32_t *buf = rmt_buffers[b];
        for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
            buf[i].val = nopulse_item.val;
            // buf[i].val = pulse_item_template.val; // prefill with pulse template to avoid glitches when we first read from the buffer before filling it, then overwrite duration1 with nopulse duration in the ISR after transmitting
        }
    }
}

//MARK: ISR
// ------------------ ISR ------------------
void IRAM_ATTR rmt_tx_end_isr(void *arg) {
    // const uint8_t dbg = 23; // GPIO pin for debugging the ISR timing (optional)
    // static bool  dbgstat;
    // digitalWrite(dbg, dbgstat); // Debug pin high at start of ISR
    // dbgstat = !dbgstat;

    // Check which interrupt fired and clear it; ignore if not our channel's threshold event
    const uint32_t thr_bit = 1u << (24 + RMT_CH);
    if (!(RMT.int_st.val & thr_bit)) return;
    
    RMT.int_clr.val = thr_bit;

    uint32_t steps_done = steps_in_buffer[currentTxBufferIndex];

    // clear the just-transmitted buffer to avoid ghost steps if we ever read from it again before filling
    volatile rmt_item32_t *buf = rmt_buffers[currentTxBufferIndex];
    for (uint32_t i = 0; i < steps_done; i++) {
        buf[i].val = nopulse_item.val;
    }

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

//MARK: SETUP
// ------------------ Public functions ------------------
void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel) {
    pinMode((uint8_t)pulsePin, OUTPUT);
    digitalWrite((uint8_t)pulsePin, LOW);
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
    rmt_buffers[1] = RMTMEM.chan[channel+1].data32; // next block for ping-pong
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

uint32_t lowTicksFromHz(uint32_t frequency_hz) {
    uint32_t low_ticks = (1000000 / frequency_hz - 2) * RMT_TICKS_1US; // subtract high pulse duration
    if (low_ticks > 0x7FFF) {
        low_ticks = 0x7FFF; // max duration that fits in 15 bits
    }
    return low_ticks;
}

void loadNextBufferHz(uint32_t count, uint32_t frequency_hz) {
    uint32_t low_ticks = lowTicksFromHz(frequency_hz);
    loadNextBuffer(count, low_ticks);
}

void IRAM_ATTR loadNextBufferWithArray(LowArray array) {
    const uint32_t write_buffer = nextWriteBufferIndex;
    steps_in_buffer[write_buffer] = array.count;
    fill_buffer_with_steps_items_array(write_buffer, array.count, array.low_ticks_array);
    nextWriteBufferIndex ^= 1;
}

void loadNextBufferWithArrayHz(LowArray array) {
    uint32_t low_ticks_array[RMT_STEPS_PER_BUFFER];
    for (uint32_t i = 0; i < array.count; i++) {
        low_ticks_array[i] = lowTicksFromHz(array.frequency_hz_array[i]);
    }
    LowArray array2 = {
        .low_ticks_array = low_ticks_array,
        .count = array.count
    };
    loadNextBufferWithArray(array);
}


rmt_item32_t hz2rmt_item(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return nopulse_item;
    }
    uint32_t low_ticks = lowTicksFromHz(frequency_hz);
    
    rmt_item32_t item = RMT_ITEM(RMT_HIGH_PULSE_TICKS, 1, low_ticks, 0);
    return item;
}

void printRMTBuffer(uint32_t buf_index) {
    volatile rmt_item32_t *buf = rmt_buffers[buf_index];
    Serial.printf("Buffer %u:\n", buf_index);
    for (uint32_t i = 0; i < RMT_STEPS_PER_BUFFER; i++) {
        Serial.printf("  Item %2u: val=0x%08X, duration0=%5u, level0=%u, duration1=%5u, level1=%u\n",
                      i, buf[i].val, buf[i].duration0, (buf[i].val >> 15) & 1, buf[i].duration1, (buf[i].val >> 31) & 1);
    }
}
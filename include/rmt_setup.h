#pragma once
#include "driver/rmt.h"

#define RMT_CH RMT_CHANNEL_0
#define RMT_STEPS_PER_BUFFER 64
#define BUFFERS_COUNT 2

#define RMT_CLK_DIV 4 // 1 tick = 1 microsecond at 80 MHz APB clock
constexpr uint32_t RMT_TICKS_1US = (80 / RMT_CLK_DIV);
constexpr uint32_t RMT_TICK_1MS = RMT_TICKS_1US * 1000;
constexpr uint32_t RMT_TICK_1S = RMT_TICK_1MS * 1000;
constexpr uint32_t RMT_HIGH_PULSE_TICKS = RMT_TICKS_1US * 4;
constexpr uint32_t NOPULS_LENGTH_TICKS = 2; // duration of the "no pulse" item in ticks, should be short to minimize delay when we read from a buffer before filling it, but must be > 2 and devisable by 2

// load the uint32_t with the bit pattern for the RMT item with the given parameters
#define RMT_ITEM(d0, l0, d1, l1) {.val = ((d0) | ((l0) << 15) | ((d1) << 16) | ((l1) << 31))}

typedef struct {
    uint32_t steps_done; // number of steps just transmitted
    bool standing_still; // true if the RMT just finished transmitting a buffer with 0 steps (used to detect end of motion)
} rmt_callback_arg_t;

// User callback hook (called from ISR)
extern void (*rmt_user_callback)(rmt_callback_arg_t *arg);

// ============================================================
// Public functions
// ============================================================
void setupRMT(
    gpio_num_t pulsePin,
    rmt_channel_t channel);
void startRMT();

typedef struct LowArray {
    union {
        uint32_t *low_ticks_array;    // pointer to array of low ticks for each step
        uint32_t *frequency_hz_array; // pointer to array of frequencies for each step, will be converted to low ticks internally
    };

    uint32_t count; // number of steps (length of the array)
} LowArray;

void loadNextBuffer(uint32_t pulse_count, uint32_t low_ticks);
void loadNextBufferWithArray(LowArray array);
void loadNextBufferHz(uint32_t count, uint32_t frequency_hz);
void loadNextBufferWithArrayHz(LowArray array);
rmt_item32_t hz2rmt_item(uint32_t frequency_hz);
void printRMTBuffer(uint32_t buf_index);

#pragma once
#include "driver/rmt.h"

#define RMT_CH RMT_CHANNEL_0
constexpr uint32_t RMT_STEPS_PER_BUFFER = 64;
constexpr uint32_t BUFFERS_COUNT = 2;

#define RMT_CLK_DIV 4 
constexpr uint32_t RMT_TICKS_1US = (80 / RMT_CLK_DIV);
constexpr uint32_t RMT_TICK_1MS = RMT_TICKS_1US * 1000;
constexpr uint32_t RMT_TICK_1S = RMT_TICK_1MS * 1000;
const uint32_t TIME_PR_TICK_NS = 1000000000/(80000000 / RMT_CLK_DIV ); // time in nanoseconds that each RMT tick represents, based on the APB clock and the RMT clock divider


//going for 500Hz update rate, so 2ms buffer duration, with 2µs high pulse time
// 2ms/64 givs 31250ns per step at maximum steps per buffer, which is 32kHz.
const uint32_t UPDATE_RATE_HZ_intern = 500; // should be the same as UPDATE_RATE_HZ in motion_task.h, but we define it here as well to be able to use it in the calculation of the pulse timing constants without including motion_task.h here and creating a circular dependency

const uint64_t PERIODE_TIME_NS = 1000000000/UPDATE_RATE_HZ_intern;
const uint32_t PERIODE_PER_STEP_NS = PERIODE_TIME_NS / RMT_STEPS_PER_BUFFER; // time in nanoseconds that each step should take to achieve the target update rate at maximum steps per buffer

const uint32_t PULSE_HIGH_TIME_NS = 2 * 1000; // 2µs high pulse time
constexpr uint32_t RMT_HIGH_PULSE_TICKS = RMT_TICKS_1US * 2;
constexpr uint32_t RMT_LOW_PULSE_TICKS = (PERIODE_PER_STEP_NS-PULSE_HIGH_TIME_NS) / TIME_PR_TICK_NS;


// load the uint32_t with the bit pattern for the RMT item with the given parameters
#define RMT_ITEM(d0, l0, d1, l1) {.val = ((d0) | ((l0) << 15) | ((d1) << 16) | ((l1) << 31))}

const rmt_item32_t PULS_ITEM_TEMPLATE = RMT_ITEM(RMT_HIGH_PULSE_TICKS, 1, RMT_LOW_PULSE_TICKS, 0);
const rmt_item32_t NOPULS_ITEM_TEMPLATE = RMT_ITEM(RMT_HIGH_PULSE_TICKS, 0, RMT_LOW_PULSE_TICKS, 0);

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


void loadNextBuffer(uint32_t pulse_count);
void printRMTBuffer(uint32_t buf_index);
bool ready_to_change_direction();
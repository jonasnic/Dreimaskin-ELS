#pragma once
#include "driver/rmt.h"

#define RMT_CH RMT_CHANNEL_0
#define RMT_STEPS_PER_BUFFER 64
#define BUFFERS_COUNT 2

#define RMT_CLK_DIV 4
#define RMT_TICK_1US (CPU_CLK_FREQ / RMT_CLK_DIV / 1000000)
#define RMT_TICK_1MS (RMT_TICK_1US * 1000)
#define RMT_TICK_1S (RMT_TICK_1MS * 1000)


//load the uint32_t with the bit pattern for the RMT item with the given parameters
#define RMT_ITEM(d0, l0, d1, l1) {.val = ((d0) | ((l0) << 15) | ((d1) << 16) | ((l1) << 31))}



typedef struct {
    uint32_t steps_done;   // number of steps just transmitted
} rmt_callback_arg_t;

// User callback hook (called from ISR)
extern void (*rmt_user_callback)(rmt_callback_arg_t *arg);


// ============================================================
// Public functions
// ============================================================
void setupRMT(
    gpio_num_t pulsePin, 
    rmt_channel_t channel, 
    rmt_tx_end_fn_t tx_end_callback);
void startRMT();

// fills the next buffer with pulses + nopulses (internal buffer access)
void loadNextBuffer(uint32_t pulse_count, uint32_t low_ticks);
void loadNextBufferWithArray(uint32_t pulse_count, const uint32_t* low_ticks_array);


#pragma once

#include "driver/rmt.h"

void setupRMT(gpio_num_t pulsePin, rmt_channel_t channel, rmt_tx_end_fn_t tx_end_callback);

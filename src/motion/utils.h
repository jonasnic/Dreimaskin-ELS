#pragma once

#include <stdint.h> // for uint32_t

// Converts frequency in Hz to pulse period in microseconds
uint32_t Hz2Us(uint32_t frequency);
void setDirection(bool dir,uint32_t currentSpeed);
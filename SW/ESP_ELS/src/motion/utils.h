#pragma once

#include <stdint.h> // for uint32_t

// Converts signed speed in Hz to pulse period in microseconds using magnitude.
uint32_t Hz2Us(int32_t frequency);
void setDirection(bool dir, int32_t currentSpeed);
#include "utils.h"

uint32_t Hz2Us(uint32_t frequency) {
    if (frequency == 0) {
        return 0; // Avoid division by zero
    }
    return 1000000 / frequency; // Convert frequency in Hz to period in microseconds
}

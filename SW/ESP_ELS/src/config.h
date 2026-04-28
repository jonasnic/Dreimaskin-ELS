#pragma once
#include <cstdint>


enum PossibleStepsPerRev : int32_t{
    STEPS_REV_400 = 400,
    STEPS_REV_800 = 800,
    STEPS_REV_1000 = 1000,
    STEPS_REV_1600 = 1600,
    STEPS_REV_2000 = 2000,
    STEPS_REV_3200 = 3200,
    STEPS_REV_4000 = 4000,
    STEPS_REV_5000 = 5000,
    STEPS_REV_6400 = 6400,
    STEPS_REV_8000 = 8000,
    STEPS_REV_10000 = 10000,
    STEPS_REV_12800 = 12800,
    STEPS_REV_20000 = 20000,
    STEPS_REV_25600 = 25600,
    STEPS_REV_40000 = 40000,
    STEPS_REV_51200 = 51200
};

constexpr uint32_t UPDATE_RATE_HZ = 3000;
constexpr uint32_t UPDATE_PERIOD_US = 1000000UL / UPDATE_RATE_HZ;
constexpr int32_t STEPS_REV = STEPS_REV_6400;
constexpr uint32_t NEXTION_BAUD_RATE = 9600;

constexpr uint32_t BACKLASH_STEPS = 0; // Steps to compensate for leadscrew backlash on direction change; tune for your machine
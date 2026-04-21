#include <cstdint>
#include <Arduino.h>




constexpr uint8_t CURRENT_SENSOR_PIN = 36; // ADC pin connected to the current sensor output


constexpr float OhmsShunt = 0.01f; // Shunt resistance in Ohms
constexpr uint32_t amplifierGain = 20;
constexpr uint32_t voltPowerSupply = 36; // Voltage of the power supply feeding the stepper driver

float readMotorCurrent() {
    // Read the ADC value from the current sensor pin
    float shuntVolts = (analogReadMilliVolts(CURRENT_SENSOR_PIN)/1000.0f)/amplifierGain; // Convert mV to V and account for amplifier gain
    float current = shuntVolts / OhmsShunt; // Calculate current using Ohm's law (I = V/R)
    return current;
}

float motorPower() {
    float current = readMotorCurrent();
    return current * voltPowerSupply; // Power in Watts (P = I * V)
}






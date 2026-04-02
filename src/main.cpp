#include "driver/pcnt.h"
#include "driver/rmt.h"
#include <Arduino.h>

#define PULSE_PIN 25      // Pin connected to the stepper PULSE signal
#define DIR_PIN 26        // Pin connected to the stepper DIR signal
#define ENABLE_PIN 27     // Pin connected to the stepper ENABLE signal
#define STEPS_PER_REV 200 // Number of steps per revolution for the stepper motor
#define MICROSTEPS 16     // Microstepping setting (e.g., 16 for 1/16 microstepping)

#define SPINDEL_ENCODER_CHANNEL_A_PIN 32      // Pin connected to the encoder channel A
#define SPINDEL_ENCODER_CHANNEL_B_PIN 33      // Pin connected to the encoder channel B
#define SPINDEL_ENCODER_CHANNEL_Z_PIN 34      // Pin connected to the encoder channel Z (index pulse)
#define SPINDEL_ENCODER_PPR 48                // Pulses per revolution for the encoder
#define SPINDEL_ENCODER_PCNT_UNIT PCNT_UNIT_0 // PCNT unit used for the encoder

#define RMT_CH RMT_CHANNEL_0
void setupRMT() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CH;
    config.gpio_num = (gpio_num_t)PULSE_PIN;
    config.clk_div = 80; // 1 tick = 1µs (80MHz / 80)

    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}
#define MAX_RMT_STEPS 60
void moveSteps(int steps) {
    int remaining = steps;

    while (remaining > 0) {
        int chunk = remaining;
        if (chunk > MAX_RMT_STEPS)
            chunk = MAX_RMT_STEPS;

        rmt_item32_t items[MAX_RMT_STEPS];

        for (int i = 0; i < chunk; i++) {
            items[i].level0 = 1;
            items[i].duration0 = 50; // 40µs HIGH pulse
            items[i].level1 = 0;
            items[i].duration1 = 50;
        }

        rmt_write_items(RMT_CH, items, chunk, false);

        remaining -= chunk;
    }
}
void setup() {
    digitalWrite(PULSE_PIN, LOW);  // Ensure the PULSE pin is LOW at startup
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    pinMode(PULSE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    pinMode(SPINDEL_ENCODER_CHANNEL_A_PIN, INPUT_PULLUP); // Set encoder channel A pin as input with pull-up
    pinMode(SPINDEL_ENCODER_CHANNEL_B_PIN, INPUT_PULLUP); // Set encoder channel B pin as input with pull-up
    pinMode(SPINDEL_ENCODER_CHANNEL_Z_PIN, INPUT_PULLUP); // Set encoder channel Z pin as input with pull-up

    Serial.begin(115200); // Start serial communication for debugging

    pcnt_config_t pcntConfig{
        .pulse_gpio_num = SPINDEL_ENCODER_CHANNEL_A_PIN, // GPIO number for pulse input
        .ctrl_gpio_num = SPINDEL_ENCODER_CHANNEL_B_PIN,  // GPIO number for control signal input
        .lctrl_mode = PCNT_MODE_REVERSE,                 // Keep the current mode when control signal is low
        .hctrl_mode = PCNT_MODE_KEEP,                    // Keep the current mode when control signal is high
        .pos_mode = PCNT_COUNT_INC,                      // Increment counter on positive edge
        .neg_mode = PCNT_COUNT_DEC,                      // Decrement counter on negative edge
        .counter_h_lim = 32767,                          // Maximum counter value
        .counter_l_lim = -32768,                         // Minimum counter value
        .unit = SPINDEL_ENCODER_PCNT_UNIT,               // Use PCNT unit 0
        .channel = PCNT_CHANNEL_0                        // Use channel 0 of the selected unit
    };

    pcnt_unit_config(&pcntConfig);                 // Configure the pulse counter with the specified settings
    pcnt_counter_pause(SPINDEL_ENCODER_PCNT_UNIT); // Pause the counter to prevent counting until we are ready
    pcnt_counter_clear(SPINDEL_ENCODER_PCNT_UNIT); // Clear the counter to start from zero
    // pcnt_intr_enable(SPINDEL_ENCODER_PCNT_UNIT); // Enable interrupts for the pulse counter unit
    pcnt_counter_resume(SPINDEL_ENCODER_PCNT_UNIT); // Resume counting for the pulse counter unit

    setupRMT(); // Initialize the RMT peripheral for generating step pulses
}

void loop() {
    const int StepsPrEncoderPuls = (MICROSTEPS * STEPS_PER_REV) / SPINDEL_ENCODER_PPR; // Calculate the number of steps per encoder revolution based on PPR and microstepping
    static int16_t lastEncoderCount = 0;                                               // Variable to store the last encoder count for comparison
    int16_t encoderCount;
    pcnt_get_counter_value(SPINDEL_ENCODER_PCNT_UNIT, &encoderCount); // Get the current count from the pulse counter
    if (encoderCount != lastEncoderCount) {                           // Check if the encoder count has changed
        Serial.print("Encoder Count: ");
        Serial.println(encoderCount);                                              // Print the current encoder count to the serial monitor
        digitalWrite(DIR_PIN, (encoderCount - lastEncoderCount) > 0 ? HIGH : LOW); // Set the direction pin based on whether the count has increased or decreased
        int stepsToMove = (encoderCount - lastEncoderCount) * StepsPrEncoderPuls;  // Calculate the number of steps to move based on the change in encoder count
        lastEncoderCount = encoderCount;                                           // Update the last encoder count for the next comparison

        moveSteps(abs(stepsToMove)); // Move the stepper motor by the calculated number of steps
    }
}
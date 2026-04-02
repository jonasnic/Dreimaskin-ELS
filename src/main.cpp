#include "driver/pcnt.h"
#include <Arduino.h>

const int PULSE_PIN = 25;      // Pin connected to the stepper PULSE signal
const int DIR_PIN = 26;        // Pin connected to the stepper DIR signal
const int ENABLE_PIN = 27;     // Pin connected to the stepper ENABLE signal
const int STEPS_PER_REV = 200; // Number of steps per revolution for the stepper motor
const int MICROSTEPS = 16;     // Microstepping setting (e.g., 16 for 1/16 microstepping)

const int ENCODER_CHANNEL_A_PIN = 32; // Pin connected to the encoder channel A
const int ENCODER_CHANNEL_B_PIN = 33; // Pin connected to the encoder channel
const int ENCODER_CHANNEL_Z_PIN = 34; // Pin connected to the encoder channel Z (index pulse)
const int ENCODER_PPR = 1600;         // Pulses per revolution for the encoder

struct motorData {
    int stepFrequency; // Frequency of steps in Hz
    bool direction;    // Direction of rotation (true for clockwise, false for counterclockwise)
    bool enabled;      // Whether the motor is enabled or not
};

void setup() {
    digitalWrite(PULSE_PIN, LOW);  // Ensure the PULSE pin is LOW at startup
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    pinMode(PULSE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    Serial.begin(115200); // Start serial communication for debugging

    pcnt_config_t pcntConfig{
        .pulse_gpio_num = ENCODER_CHANNEL_A_PIN, // GPIO number for pulse input
        .ctrl_gpio_num = ENCODER_CHANNEL_B_PIN,  // No control signal used
        .lctrl_mode = PCNT_MODE_KEEP,            // Keep the current mode when control signal is low
        .hctrl_mode = PCNT_MODE_KEEP,            // Keep the current mode when control signal is high
        .pos_mode = PCNT_COUNT_INC,              // Increment counter on positive edge
        .neg_mode = PCNT_COUNT_DEC,              // Decrement counter on negative edge
        .counter_h_lim = 32767,                  // Maximum counter value
        .counter_l_lim = -32768,                 // Minimum counter value
        .unit = PCNT_UNIT_0,                     // Use PCNT unit 0
        .channel = PCNT_CHANNEL_0                // Use channel 0 of the selected unit
    };
    pcnt_unit_config(&pcntConfig); // Configure the pulse counter with the specified settings
}

void loop() {
    static int sleepmicrosTime = 1000; // Time to sleep in microseconds (adjust as needed)
    static char commandBuffer[50];     // Buffer to hold incoming serial commands
    static byte commandIndex = 0;      // Index for the command buffer
    while (Serial.available()) {
        commandBuffer[commandIndex++] = Serial.read(); // Read incoming serial data into the buffer
        Serial.print(commandBuffer[commandIndex - 1]); // Echo the received character for debugging
        if (commandBuffer[commandIndex - 1] == '\n') { // Check for end of command (newline character)

            String command = String(commandBuffer).substring(0, commandIndex - 1); // Convert buffer to String and remove newline
            commandIndex = 0;                                                      // Reset command index for the next command

            if (command.startsWith("S")) {
                command = command.substring(1);      // Remove "S" from the command
                command.trim();                      // Remove any leading/trailing whitespace
                int stepFrequency = command.toInt(); // Convert command to integer for step frequency
                Serial.print("Step frequency set to: ");
                Serial.print(stepFrequency);
                Serial.println(" Hz");
                sleepmicrosTime = 1000000 / stepFrequency; // Calculate sleep time based on step frequency
            } else if (command.startsWith("D")) {
                command = command.substring(1);   // Remove "D" from the command
                command.trim();                   // Remove any leading/trailing whitespace
                bool direction = command.toInt(); // Convert command to integer for direction
                Serial.print("Direction set to: ");
                Serial.println(direction ? "Clockwise" : "Counterclockwise");
                digitalWrite(DIR_PIN, direction); // Set the direction pin accordingly
            } else if (command.startsWith("E")) {
                command = command.substring(1); // Remove "E" from the command
                command.trim();                 // Remove any leading/trailing whitespace
                bool enabled = command.toInt(); // Convert command to integer for enable state
                Serial.print("Motor ");
                Serial.println(enabled ? "Enabled" : "Disabled");
                digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH); // Enable or disable the motor driver
            } else {
                Serial.println("Invalid command. Use S<frequency>, D<direction>, or E<enable>.");
            }
        }
    }

    digitalWrite(PULSE_PIN, HIGH);          // Generate a step pulse
    delayMicroseconds(sleepmicrosTime / 2); // Wait for the appropriate time based on the step frequency
    digitalWrite(PULSE_PIN, LOW);           // End the step pulse
    delayMicroseconds(sleepmicrosTime / 2); // Wait for the appropriate time based on the step frequency
}
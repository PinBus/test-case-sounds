/**
 * Test-Case - Sounds
 *
 * Version 1.0 (21-07-2022)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/** Define Arduino pins and PCA9685 pins **/
#define PIN_MATRIX_IN_0 A0
#define PIN_MATRIX_IN_1 A1
#define PIN_MATRIX_IN_2 A2
#define PIN_MATRIX_IN_3 A3

#define PIN_MATRIX_OUT_1 5
#define PIN_MATRIX_OUT_2 6
#define PIN_MATRIX_OUT_3 7

#define PIN_PCA9685_OE 8

#define PIN_ELECTRONIC_CHIME_1 2
#define PIN_ELECTRONIC_CHIME_2 3
#define PIN_ELECTRONIC_CHIME_3 4

//  0 - Solenoid: Chime 1
//  1 - Solenoid: Chime 2
//  2 - Solenoid: Chime 3
//  3 - Solenoid: Chime 4
//  4 - Solenoid: Knocker
//  5 - Solenoid: Buzzer
//  6 - Solenoid: Bell
//  7 - Solenoid: Drum unit
#define PIN_PCA9685_CHIME_1 0
#define PIN_PCA9685_CHIME_2 1
#define PIN_PCA9685_CHIME_3 2
#define PIN_PCA9685_CHIME_4 3
#define PIN_PCA9685_KNOCKER 5
#define PIN_PCA9685_BUZZER 4
#define PIN_PCA9685_BELL 6
#define PIN_PCA9685_DRUM_UNIT 7

//  8 - LED: Chime 1 button
//  9 - LED: Chime 2 button
// 10 - LED: Chime 3 button
// 11 - LED: Chime 4 button
// 12 - LED: Knocker button
// 13 - LED: Buzzer button
// 14 - LED: Bell button
// 15 - LED: Drum unit button
#define PIN_PCA9685_LED_BUTTON_CHIME_1 8
#define PIN_PCA9685_LED_BUTTON_CHIME_2 9
#define PIN_PCA9685_LED_BUTTON_CHIME_3 10
#define PIN_PCA9685_LED_BUTTON_CHIME_4 11
#define PIN_PCA9685_LED_BUTTON_KNOCKER 12
#define PIN_PCA9685_LED_BUTTON_BUZZER 13
#define PIN_PCA9685_LED_BUTTON_BELL 14
#define PIN_PCA9685_LED_BUTTON_DRUM_UNIT 15

// LED: Mode button 1
// LED: Mode button 2
#define PIN_LED_BUTTON_MODE_1 9
#define PIN_LED_BUTTON_MODE_2 10

// Solenoid index in solenoid array
#define CHIME_1_INDEX 0
#define CHIME_2_INDEX 1
#define CHIME_3_INDEX 2
#define CHIME_4_INDEX 3
#define KNOCKER_INDEX 4
#define BELL_INDEX 5
#define DRUM_UNIT_INDEX 6
#define BUZZER_INDEX 7

// Tone index in tone array
#define TONE_CHIME_1_INDEX 0
#define TONE_CHIME_2_INDEX 1
#define TONE_CHIME_3_INDEX 2

/** Type definitions **/
typedef void (*SwitchCallback)();

struct SwitchState {
    uint32_t first_change;
    bool is_closed;
    SwitchCallback on_switch_close;
    SwitchCallback on_switch_open;
};

struct Assembly {
    uint8_t pin;
    uint32_t timer;
    uint32_t on_time;
};

struct TuneStep {
    uint8_t assembly_index;
    uint32_t start_time;
};

struct VisualizationStep {
    uint32_t active_time;
    bool state;
};

/** Global variables **/

// The PCA9685 driver object
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

/** Configuration **/
#define MATRIX_DEBOUNCE_MS 20
#define MATRIX_HEIGHT 3
#define MATRIX_WIDTH 4

#define LED_FADE_SPEED_MS 200
#define LED_MAX_VALUE 4096
#define LED_HALF_VALUE 2048
#define LED_QUARTER_VALUE 1024

#define BUTTON_DEBOUNCE_MS 2000
#define BUTTON_DEBOUNCE_TOGGLE_MS 5000

/** Switch Matrix **/
uint8_t matrix_input_pins[MATRIX_WIDTH] = {
    PIN_MATRIX_IN_0,
    PIN_MATRIX_IN_1,
    PIN_MATRIX_IN_2,
    PIN_MATRIX_IN_3
};
uint8_t matrix_output_pins[MATRIX_HEIGHT] = {
    PIN_MATRIX_OUT_1,
    PIN_MATRIX_OUT_2,
    PIN_MATRIX_OUT_3
};

uint8_t matrix_col = 0;
uint8_t matrix_row = 0;

// Scratchpad pointer for loops iterating over switch state structs
SwitchState* switch_state = NULL;

// Arduino function hoisting sucks
void on_mode_1_button_closed();
void on_mode_1_button_opened();
void on_chime_1_button_closed();
void on_chime_1_button_opened();
void on_chime_2_button_closed();
void on_chime_2_button_opened();
void on_chime_3_button_closed();
void on_chime_3_button_opened();
void on_chime_4_button_closed();
void on_knocker_button_closed();
void on_buzzer_button_closed();
void on_buzzer_button_opened();
void on_bell_button_closed();
void on_drum_unit_button_closed();
void on_mode_2_button_closed();
void on_mode_2_button_opened();

SwitchState matrix[MATRIX_WIDTH][MATRIX_HEIGHT] = {
    {
        // 0x0 - Not used
        // 0x1 - NO - Front button 1: White / Mode button 1
        {0, false, on_mode_1_button_closed, on_mode_1_button_opened},
        // 0x2 - NO - Front button 2: Blue / Chime 1 button
        {0, false, on_chime_1_button_closed, on_chime_1_button_opened},
        // 0x3 - NO - Front Button 3: Blue / Chime 2 button
        {0, false, on_chime_2_button_closed, on_chime_2_button_opened}
    },
    {
        // 1x0 - Not used
        // 1x1 - NO - Front button 4: Blue / Chime 3 button
        {0, false, on_chime_3_button_closed, on_chime_3_button_opened},
        // 1x2 - NO - Front button 5: Blue / Chime 4 button
        {0, false, on_chime_4_button_closed, NULL},
        // 1x3 - NO - Front button 6: Yellow / Knocker button
        {0, false, on_knocker_button_closed, NULL}
    },
    {
        // 2x0 - Not used
        // 2x1 - NO - Front button 7: Yellow / Buzzer button
        {0, false, on_buzzer_button_closed, on_buzzer_button_opened},
        // 2x2 - NO - Front button 8: Yellow / Bell button
        {0, false, on_bell_button_closed, NULL},
        // 2x3 - NO - Front button 9: Yellow / Drum unit button
        {0, false, on_drum_unit_button_closed, NULL}
    },
    {
        // 3x0 - Not used
        // 3x1 - NO - Front button 10: White / Mode button 2
        {0, false, on_mode_2_button_closed, on_mode_2_button_opened},
        // 3x2 - NO - Not used
        {0, false, NULL, NULL},
        // 3x3 - NO - Not used
        {0, false, NULL, NULL}
    }
};

void init_matrix() {
    // Set pin mode for output pins
    for (matrix_col = 0; matrix_col < MATRIX_HEIGHT; matrix_col++) {
        pinMode(matrix_output_pins[matrix_col], OUTPUT);
    }

    // Set pin mode for input pins
    for (matrix_row = 0; matrix_row < MATRIX_WIDTH; matrix_row++) {
        pinMode(matrix_input_pins[matrix_row], INPUT);
    }
}

void update_matrix() {
    for (matrix_col = 0; matrix_col < MATRIX_HEIGHT; matrix_col++) {
        // Set output pins
        for (matrix_row = 0; matrix_row < MATRIX_WIDTH; matrix_row++) {
            digitalWrite(matrix_output_pins[matrix_row], matrix_col != matrix_row);
        }

        // Scan input pins
        for (matrix_row = 0; matrix_row < MATRIX_WIDTH; matrix_row++) {
            switch_state = &matrix[matrix_row][matrix_col];

            // Check if switch state has been changed
            if (digitalRead(matrix_input_pins[matrix_row]) == (switch_state->is_closed ? HIGH : LOW)) {
                if (!switch_state->first_change) {
                    // Mark the first time this state change has been seen
                    switch_state->first_change = millis();
                }
                else {
                    // Check if switch is in its new state for long enough
                    if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                        if (switch_state->is_closed) {
                            // Call the on switch open callback
                            if (switch_state->on_switch_open != NULL) {
                                switch_state->on_switch_open();
                            }
                        }
                        else {
                            // Call the on switch close callback
                            if (switch_state->on_switch_close != NULL) {
                                switch_state->on_switch_close();
                            }
                        }

                        // Update internal state
                        switch_state->is_closed = !switch_state->is_closed;
                        switch_state->first_change = 0;
                    }
                }
            }
            else if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                // Reset internal state if switch went back to its previous state within the debounce time
                switch_state->first_change = 0;
            }
        }
    }
}

/** PCA9685 **/
void init_pca9685() {
    // Constructor initializes a PCA9685 device at default address 0x40
    pca9685.begin();
    // Set maximum PWM frequency in Hz
    pca9685.setPWMFreq(1600);
    // Set output to push/pull (totempole)
    pca9685.setOutputMode(true);

    // Set Output Enable (OE) pin low
    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, LOW);
}

/** Solenoid Control **/
Assembly assemblies[8] = {
    // 0 - Chime 1
    { PIN_PCA9685_CHIME_1, 0, 20 },
    // 1 - Chime 2
    { PIN_PCA9685_CHIME_2, 0, 20 },
    // 2 - Chime 3
    { PIN_PCA9685_CHIME_3, 0, 20 },
    // 3 - Chime 4
    { PIN_PCA9685_CHIME_4, 0, 20 },
    // 4 - Knocker
    { PIN_PCA9685_KNOCKER, 0, 40 },
    // 5 - Bell
    { PIN_PCA9685_BELL, 0, 40 },
    // 6 - Drum unit
    { PIN_PCA9685_DRUM_UNIT, 0, 40 },
    // 7 - Buzzer
    { PIN_PCA9685_BUZZER, 0, 500 },
};

// Scratchpad pointer for loops iterating over assemblies
Assembly* assembly = NULL;

// Tone board outputs
Assembly tones[3] = {
    // Tone chime 1
    { PIN_ELECTRONIC_CHIME_1, 0, 500 },
    // Tone chime 2
    { PIN_ELECTRONIC_CHIME_2, 0, 500 },
    // Tone chime 3
    { PIN_ELECTRONIC_CHIME_3, 0, 500 }
};

// Scratchpad pointer for loops iterating over tones
Assembly* tone_output = NULL;

// Bit of state needed in the assembly update function
bool should_debounce = true;

void fire_assembly(uint8_t index) {
    assembly = &assemblies[index];

    // Do not fire while assembly is already active
    if (assembly->timer) {
        return;
    }

    // Set timer and switch on MOSFET
    assembly->timer = millis();
    pca9685.setPin(assembly->pin, 4096, false);
}

void stop_assembly(uint8_t index) {
    assembly = &assemblies[index];

    pca9685.setPin(assembly->pin, 0, false);
    assembly->timer = 0;
}

void update_assemblies() {
    // Update assemblies (PCA9685)
    for (uint8_t i = 0; i < 7; i++) {
        assembly = &assemblies[i];

        if (assembly->timer) {
            if (millis() - assembly->timer > assembly->on_time) {
                // Reset timer and switch off MOSFET
                pca9685.setPin(assembly->pin, 0, false);
                assembly->timer = 0;
            }
        }
    }

    if (!should_debounce) {
        return;
    }

    // Update buzzer state separately to disable cutoff when debounce is off
    assembly = &assemblies[7];

    if (assembly->timer) {
        if (millis() - assembly->timer > assembly->on_time) {
            // Reset timer and switch off MOSFET
            pca9685.setPin(assembly->pin, 0, false);
            assembly->timer = 0;
        }
    }

    // Update tone drivers (GPIO)
    for (uint8_t i = 0; i < 3; i++) {
        tone_output = &tones[i];

        if (tone_output->timer) {
            if (millis() - tone_output->timer > tone_output->on_time) {
                digitalWrite(tone_output->pin, LOW);
                tone_output->timer = 0;
            }
        }
    }
}

void init_tone_board() {
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(tones[i].pin, OUTPUT);
        digitalWrite(tones[i].pin, LOW);
    }
}

void start_tone(uint8_t index) {
    tone_output = &tones[index];

    // Do not fire while tone is already active
    if (tone_output->timer) {
        return;
    }

    tone_output->timer = millis();
    digitalWrite(tone_output->pin, HIGH);
}

void end_tone(uint8_t index) {
    tone_output = &tones[index];

    digitalWrite(tone_output->pin, LOW);
    tone_output->timer = 0;
}

bool end_all_tones() {
    bool result = false;

    for (uint8_t i = 0; i < 3; i++) {
        tone_output = &tones[i];

        digitalWrite(tone_output->pin, LOW);

        result = result || tone_output->timer;

        tone_output->timer = 0;
    }

    return result;
}

/** Serial **/
void init_serial() {
    // Open the serial connection
    Serial.begin(9600);
}

void update_serial() {
    // Check for available characters on the serial port.
    if (Serial.available() > 0) {
        // If the sent character is a "c", print the credits "window"
        if (Serial.read() == 'c') {
            Serial.println("---------- Test-Case - Sounds ----------\n        Version 1.0 (01-08-2022)\n       Made in The Netherlands by\n    Cor Gravekamp & Thomas Gravekamp\n                 for the\n          Dutch Pinball Museum\n----------------------------------------");
        }
    }
}

/** LED animations **/
void init_leds() {
    // Set chime toggle button to half brightness
    analogWrite(PIN_LED_BUTTON_MODE_1, 255);

    for (uint8_t i = 8; i < 16; i++) {
        pca9685.setPin(i, LED_MAX_VALUE, true);
    }

    // Set song button to full brightness
    analogWrite(PIN_LED_BUTTON_MODE_2, 255);
}

/** State **/
bool chime_toggle = false;

bool should_debounce_toggled = false;
uint32_t should_debounce_toggle_timer = 0;

bool is_button_press_allowed = true;
bool is_non_standard_disabled_duration = false;

uint32_t button_debounce_last_press = 0;

void visualize_disabled_action_buttons() {
    for (uint8_t i = 8; i < 16; i++) {
        pca9685.setPin(i, LED_QUARTER_VALUE, true);
    }

    analogWrite(PIN_LED_BUTTON_MODE_2, 63);
}

void visualize_enabled_action_buttons() {
    uint16_t value = chime_toggle ? LED_HALF_VALUE : LED_MAX_VALUE;
    for (uint8_t i = 8; i < 16; i++) {
        pca9685.setPin(i, i < 11 ? value : LED_MAX_VALUE, true);
    }

    analogWrite(PIN_LED_BUTTON_MODE_2, 255);
}

uint32_t debounce_toggle_visualization_timer = 0;

void visualize_debounce_toggle() {
    is_button_press_allowed = false;
    debounce_toggle_visualization_timer = millis();
}

uint8_t debounce_toggle_visualization_index = 0;
VisualizationStep debounce_toggle_visualization_steps[6] = {
    { 350, true },
    { 250, false },
    { 350, true },
    { 250, false },
    { 350, true },
    { 0, true }
};

VisualizationStep* visualization_step = NULL;

void update_visualizations() {
    if (!debounce_toggle_visualization_timer) {
        return;
    }

    visualization_step = &debounce_toggle_visualization_steps[debounce_toggle_visualization_index];

    if (millis() - debounce_toggle_visualization_timer < visualization_step->active_time) {
        return;
    }

    uint16_t pwm = visualization_step->state ? LED_MAX_VALUE : 0;
    for (uint8_t i = 8; i < 16; i++) {
        pca9685.setPin(i, pwm, true);
    }

    debounce_toggle_visualization_index++;
    if (debounce_toggle_visualization_index == 6) {
        is_button_press_allowed = true;
        debounce_toggle_visualization_index = 0;
        debounce_toggle_visualization_timer = 0;
        visualize_enabled_action_buttons();
        return;
    }
    debounce_toggle_visualization_timer = millis();
}

void disable_buttons(bool is_non_standard_duration) {
    if (!should_debounce) {
        return;
    }

    if (button_debounce_last_press != 0) {
        return;
    }

    is_non_standard_disabled_duration = is_non_standard_duration;
    button_debounce_last_press = millis();
    is_button_press_allowed = false;

    visualize_disabled_action_buttons();
}

void update_disabled_buttons() {
    if (!should_debounce) {
        return;
    }

    if (is_button_press_allowed) {
        return;
    }

    if (is_non_standard_disabled_duration) {
        return;
    }

    if (millis() - button_debounce_last_press > BUTTON_DEBOUNCE_MS) {
        button_debounce_last_press = 0;
        is_button_press_allowed = true;

        visualize_enabled_action_buttons();
    }
}

uint32_t disable_debounce_mode_button_timer_1 = 0;
uint32_t disable_debounce_mode_button_timer_2 = 0;

void update_should_debounce_state() {
    if (
        disable_debounce_mode_button_timer_1 == 0 &&
        disable_debounce_mode_button_timer_2 == 0 &&
        should_debounce_toggled
    ) {
        should_debounce_toggled = false;
    }

    if (
        disable_debounce_mode_button_timer_1 == 0 ||
        disable_debounce_mode_button_timer_2 == 0 ||
        should_debounce_toggled
    ) {
        return;
    }

    if (
        millis() - disable_debounce_mode_button_timer_1 > BUTTON_DEBOUNCE_TOGGLE_MS &&
        millis() - disable_debounce_mode_button_timer_2 > BUTTON_DEBOUNCE_TOGGLE_MS
    ) {
        should_debounce = !should_debounce;
        should_debounce_toggled = true;
        visualize_debounce_toggle();
        return;
    }
}

uint8_t tune_index = 0;
uint32_t tune_start_time = 0;
uint32_t tune_time_diff = 0;

TuneStep tune_big_ben[8] = {
    { CHIME_1_INDEX, 500 },
    { CHIME_3_INDEX, 1000 },
    { CHIME_2_INDEX, 1500 },
    { CHIME_4_INDEX, 2000 },
    { CHIME_4_INDEX, 2500 },
    { CHIME_2_INDEX, 3000 },
    { CHIME_1_INDEX, 3500 },
    { CHIME_3_INDEX, 4000 }
};

TuneStep* tune_step = NULL;

void start_tune() {
    tune_index = 0;
    tune_start_time = millis();
}

/**
 * Quick and dirty implementation of something that plays a tune.
 *
 * There are a few TODO's here that need to be resolved before this function is
 * really "done":
 *
 *  - Remove the global index and replace it with multiple "tracks", one for each assembly
 *  - Allow for custom "on" times for the buzzer and electronic chimes
 *  - Allow for using the electronic chimes
 */
void update_tune() {
    if (!tune_start_time) {
        return;
    }

    tune_time_diff = millis() - tune_start_time;
    tune_step = &tune_big_ben[tune_index];

    if (tune_time_diff < tune_step->start_time) {
        return;
    }

    fire_assembly(tune_step->assembly_index);
    tune_index++;

    if (tune_index == 8) {
        tune_index = 0;
        tune_start_time = 0;

        // Manually allow buttons presses again after playing tune without debounce
        // is_button_press_allowed = true;
        is_non_standard_disabled_duration = false;

        if (!should_debounce) {
            is_button_press_allowed = true;
        }
        else {
            button_debounce_last_press = millis();
        }
    }
}

void toggle_chime_mode() {
    chime_toggle = !chime_toggle;

    analogWrite(PIN_LED_BUTTON_MODE_1, chime_toggle ? 63 : 255);

    if (!button_debounce_last_press && !debounce_toggle_visualization_timer) {
        uint16_t value = chime_toggle ? LED_HALF_VALUE : LED_MAX_VALUE;
        pca9685.setPin(PIN_PCA9685_LED_BUTTON_CHIME_1, value, true);
        pca9685.setPin(PIN_PCA9685_LED_BUTTON_CHIME_2, value, true);
        pca9685.setPin(PIN_PCA9685_LED_BUTTON_CHIME_3, value, true);
    }
}

/** Button actions **/
void on_mode_1_button_closed() {
    disable_debounce_mode_button_timer_1 = millis();

    toggle_chime_mode();
}

void on_mode_1_button_opened() {
    disable_debounce_mode_button_timer_1 = 0;

    toggle_chime_mode();

    if (end_all_tones()) {
        disable_buttons(false);
    }
}

void on_chime_1_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    if (chime_toggle) {
        start_tone(TONE_CHIME_1_INDEX);
    }
    else {
        disable_buttons(false);
        fire_assembly(CHIME_1_INDEX);
    }
}

void on_chime_1_button_opened() {
    if (chime_toggle) {
        end_tone(TONE_CHIME_1_INDEX);
        disable_buttons(false);
    }
}

void on_chime_2_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    if (chime_toggle) {
        start_tone(TONE_CHIME_2_INDEX);
    }
    else {
        disable_buttons(false);
        fire_assembly(CHIME_2_INDEX);
    }
}

void on_chime_2_button_opened() {
    if (chime_toggle) {
        end_tone(TONE_CHIME_2_INDEX);
        disable_buttons(false);
    }
}

void on_chime_3_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    if (chime_toggle) {
        start_tone(TONE_CHIME_3_INDEX);
    }
    else {
        disable_buttons(false);
        fire_assembly(CHIME_3_INDEX);
    }
}

void on_chime_3_button_opened() {
    if (chime_toggle) {
        end_tone(TONE_CHIME_3_INDEX);
        disable_buttons(false);
    }
}

void on_chime_4_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    disable_buttons(false);

    fire_assembly(CHIME_4_INDEX);
}

void on_knocker_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    disable_buttons(false);

    fire_assembly(KNOCKER_INDEX);
}

void on_buzzer_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    fire_assembly(BUZZER_INDEX);
}

void on_buzzer_button_opened() {
    disable_buttons(false);

    stop_assembly(BUZZER_INDEX);
}

void on_bell_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    disable_buttons(false);

    fire_assembly(BELL_INDEX);
}

void on_drum_unit_button_closed() {
    if (!is_button_press_allowed) {
        return;
    }

    disable_buttons(false);

    fire_assembly(DRUM_UNIT_INDEX);
}

void on_mode_2_button_closed() {
    disable_debounce_mode_button_timer_2 = millis();
}

void on_mode_2_button_opened() {
    disable_debounce_mode_button_timer_2 = 0;

    if (should_debounce_toggled) {
        return;
    }

    if (!is_button_press_allowed) {
        return;
    }

    // Disable buttons while playing tune without debounce
    if (should_debounce) {
        disable_buttons(true);
    }
    else {
        is_button_press_allowed = false;
    }

    start_tune();
}

void setup()
{
    // Initialize switch matrix
    init_matrix();

    // Initialize PCA9685
    init_pca9685();

    // Initialize pins for tone board
    init_tone_board();

    // Initialize LED state
    init_leds();

    // Initialize serial
    init_serial();
}

void loop()
{
    // Update switch matrix
    update_matrix();

    // Update solenoid state
    update_assemblies();

    // State updates
    update_disabled_buttons();

    update_should_debounce_state();

    update_tune();

    update_visualizations();

    // Update serial
    update_serial();
}

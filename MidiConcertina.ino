#include <HX711_ADC.h>

#include <pitchToNote.h>
#include <MIDIUSB.h>


enum class BellowsAction {
    None,
    Push,
    Pull
};

constexpr int hx711_dout_pin = A3;
constexpr int hx711_sck_pin = A2;
HX711_ADC bellows_sensor(hx711_dout_pin, hx711_sck_pin);


constexpr int button_row_count = 6;
constexpr int button_col_count = 5;
constexpr int button_row_pins[button_row_count] = { 10, 16, 14, 15, A0, A1 };
constexpr int button_col_pins[button_col_count] = { 5, 6, 7, 8, 9 };

constexpr int button_count = 30;
bool previous_button_states[button_count] = {0};
long debounce_timeouts[button_count] = {0};
constexpr long debounce_milliseconds = 25;

constexpr uint8_t button_channels[button_count] = {
    // Left hand
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    // Right hand
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
};

constexpr int push_pitches[button_count] = {
    // Left hand
    pitchE3, pitchA3, pitchD4b, pitchA4, pitchA4b,
    pitchC3, pitchG3, pitchC4, pitchE4, pitchG4,
    pitchB3, pitchD4, pitchG4, pitchB4, pitchD5,
    // Right hand
    pitchD5b, pitchA5, pitchA5b, pitchD6b, pitchA6,
    pitchC5, pitchE5, pitchG5, pitchC6, pitchE6,
    pitchG5, pitchB5, pitchD6, pitchG6, pitchB6
};

constexpr int pull_pitches[button_count] = {
    // Left hand
    pitchF3, pitchB3b, pitchE4b, pitchG4, pitchB4b,
    pitchG3, pitchB3, pitchD4, pitchF4, pitchA4,
    pitchA3, pitchG4b, pitchA4, pitchC5, pitchE5,
    // Right hand
    pitchE5b, pitchG5, pitchB5b, pitchE6b, pitchF6,
    pitchB4, pitchD5, pitchF5, pitchA5, pitchB5,
    pitchG5b, pitchA5, pitchC6, pitchE6, pitchG6b
};

const int *button_pitches = push_pitches;


constexpr int setting_count = 2;
int setting_pins[setting_count] = {2, 3};

//=============================================================================

void setup() {
    bellows_sensor.begin();
    bellows_sensor.start(2000);
    bellows_sensor.setSamplesInUse(1);

    for (int row = 0; row < button_row_count; ++row) {
        const int pin = button_row_pins[row];
        pinMode(pin, OUTPUT);
        buttonsSetRowActive(row, false);
    }

    for (int col = 0; col < button_col_count; ++col) {
        const int pin = button_col_pins[col];
        pinMode(pin, INPUT_PULLUP);
    }

    for (int id = 0; id < setting_count; ++id) {
        const int pin = setting_pins[id];
        pinMode(pin, INPUT_PULLUP);
    }
}

void loop() {
    static bool previous_multi_channel = multiChannelOn();
    const bool multi_channel = multiChannelOn();
    if (multi_channel != previous_multi_channel) {
        buttonsReleaseAll(previous_multi_channel);
        previous_multi_channel = multi_channel;
    }

    if (bellows_sensor.update()) {
        const auto reading = bellows_sensor.getData();
        const auto action = bellowsActionFromReading(reading);
        const auto pressure = bellowsPressureFromReading(reading);
        setBellowsAction(action, multi_channel);
        midiBellowsPressure(pressure);
    }

    bool button_states[button_count] = {0};
    buttonsReadStates(button_states);

    for (int i = 0; i < button_count; ++i) {
        long current_time = millis();
        const bool ready = (current_time >= debounce_timeouts[i]);
        const bool changed = button_states[i] != previous_button_states[i];

        if (ready && changed) {
            const uint8_t channel = multi_channel ? button_channels[i] : 0;
            const uint8_t pitch = button_pitches[i];
            if (button_states[i]) {
                midiNoteOn(channel, pitch);
            } else {
                midiNoteOff(channel, pitch);
            }

            debounce_timeouts[i] = current_time + debounce_milliseconds;
            previous_button_states[i] = button_states[i];
        }
    }
}

//=============================================================================

BellowsAction bellowsActionFromReading(float reading) {
    return (reading > 0) ? BellowsAction::Pull : BellowsAction::Push;
}

uint8_t bellowsPressureFromReading(float reading) {
    if (!dynamicsOn()) { return 127; }
    const float pressure = sqrt(abs(reading)) / 9;
    return constrain(pressure, 0, 127);
}

void setBellowsAction(BellowsAction action, bool multi_channel) {
    static BellowsAction previous_bellows_action = BellowsAction::None;
    if (action != previous_bellows_action) {
        buttonsReleaseAll(multi_channel);
        const bool pushing = (action == BellowsAction::Push);
        button_pitches = pushing ? push_pitches : pull_pitches;
        previous_bellows_action = action;
    }
}

//=============================================================================

void buttonsSetRowActive(int row, bool active) {
    const int pin = button_row_pins[row];
    digitalWrite(pin, active ? LOW : HIGH);
}

bool buttonsReadColumn(int col) {
    const int pin = button_col_pins[col];
    return digitalRead(pin) == HIGH;
}

void buttonsReadStates(bool states[]) {
    for (int row = 0; row < button_row_count; ++row) {
        buttonsSetRowActive(row, true);
        for (int col = 0; col < button_col_count; ++col) {
            const int button = (row * button_col_count) + col;
            states[button] = buttonsReadColumn(col);
        }
        buttonsSetRowActive(row, false);
    }
}

void buttonsReleaseAll(bool multi_channel) {
    for (int i = 0; i < button_count; ++i) {
        if (previous_button_states[i]) {
            const uint8_t channel = multi_channel ? button_channels[i] : 0;
            const uint8_t pitch = button_pitches[i];
            midiNoteOff(channel, pitch);
            previous_button_states[i] = false;
        }
    }
}

//=============================================================================

void midiSend(uint8_t b1, uint8_t b2, uint8_t b3) {
    const uint8_t header = b1 >> 4;
    midiEventPacket_t packet = {header, b1, b2, b3};
    MidiUSB.sendMIDI(packet);
    MidiUSB.flush();
}

void midiNoteOn(uint8_t channel, uint8_t pitch) {
    constexpr uint8_t velocity = 64;
    midiSend(0x90 | channel, pitch, velocity);
}

void midiNoteOff(uint8_t channel, uint8_t pitch) {
    constexpr uint8_t velocity = 64;
    midiSend(0x80 | channel, pitch, velocity);
}

void midiControlChange(uint8_t channel, uint8_t control_number, uint8_t value) {
    midiSend(0xB0 | channel, control_number, value);
}

void midiBellowsPressure(uint8_t pressure) {
    uint8_t control_number = 7; // channel volume
    midiControlChange(0, control_number, pressure);
    midiControlChange(1, control_number, pressure);
}

//=============================================================================

bool settingOn(int id) {
    const int pin = setting_pins[id];
    return digitalRead(pin) == LOW;
}

bool dynamicsOn() {
    return settingOn(0);
}

bool multiChannelOn() {
    return settingOn(1);
}

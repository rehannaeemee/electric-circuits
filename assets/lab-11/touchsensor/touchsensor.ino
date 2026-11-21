// Adapted from the supplied MIT 6.200 RP2040 touchsensor.ino.
// Board: Arduino Leonardo (ATmega32U4, 16 MHz).
// Uses the original charging-delay method; no CapacitiveSensor library.
//
// SINGLE LOOP WIRING:
//   D10 -- 100 kOhm -- D0 -- touch wire loop (other end unconnected).
//   D9 -- 330 Ohm -- LED anode; LED cathode -- GND.
//   Remove the fixed capacitor for touch sensing.
//   Leonardo signals are approximately 0 to 5 V, rather than 0 to 3.3 V.
//
// FIVE-PAD EXTENSION:
//   Change NUM_CHANNELS to 5. Use a separate 100 kOhm resistor from D10
//   to EACH sensing pin D0, D1, D2, D3 and D4.
//   Corresponding LED outputs are D9, D8, D7, D6 and D5.
//   Each LED needs its own 330 Ohm series resistor.
//
// CALIBRATION:
//   Upload with KEYBOARD_ON false. Open Serial Monitor at 115200 baud.
//   Channel 0 readings are approximate time constants in microseconds.
//   Record untouched and touched ranges, then replace tauThreshold below
//   with a value between them and upload again. The large initial value
//   is a placeholder, not a working touch threshold.
//   A reading of -1 means a timeout or an input that did not discharge.
//
// TIMING:
//   D10 produces an approximately 100 Hz, 50% duty square wave.
//   One sensing channel is measured on each rising edge, as in the original.
//   Timer1 runs at 0.5 microseconds per tick for finer timing than micros().
//   The original 1.44 multiplier assumes switching near half the supply.
//   The actual digital input threshold, polling and interrupts add error.
//   Treat these as approximate readings and calibrate on this Leonardo;
//   do not copy the RP2040 threshold or use this as a precision RC meter.
//   Timer1 is reserved: do not add Servo or analogWrite on D9/D10.
//   USB Serial is used; do not use Serial1 on sensing pins D0/D1.

#include <Arduino.h>
#include <Keyboard.h>
#include <avr/interrupt.h>

#if !defined(ARDUINO_AVR_LEONARDO) || F_CPU != 16000000UL
#error "Select Arduino Leonardo (16 MHz) in Tools > Board."
#endif

float tauThreshold = 1000000.0f;  // Replace after calibration; microseconds.

const bool KEYBOARD_ON = false;
const uint8_t NUM_CHANNELS = 1;   // Use 1 for the loop; 5 for all five pads.

static_assert(NUM_CHANNELS >= 1 && NUM_CHANNELS <= 5,
              "NUM_CHANNELS must be between 1 and 5.");

const uint8_t SEND_PIN = 10;
const uint8_t KEYS[5] = {0, 1, 2, 3, 4};
const uint8_t LEDS[5] = {9, 8, 7, 6, 5};

const char *const keyboardOutput[5] = {
  "~<",
  "^.^",
  ">~",
  " hooray! ",
  "circuits!"
};

const uint32_t WAVE_FREQUENCY_HZ = 100;
const uint32_t HALF_PERIOD_US = 1000000UL / (2 * WAVE_FREQUENCY_HZ);
const uint16_t TIMEOUT_TICKS = HALF_PERIOD_US * 2;  // 0.5 us per tick.
const float TIMER_TICK_US = 0.5f;
const float ORIGINAL_TAU_FACTOR = 1.44f;

volatile uint8_t *sendPort;
uint8_t sendMask;
volatile uint8_t *sensePorts[5];
uint8_t senseMasks[5];

bool lastKeyStates[5] = {false};
bool waveHigh = false;
uint8_t which = 0;
uint32_t phaseStartUs;
bool measurementValid = false;
float approxTauUs = 0.0f;

void setup() {
  Serial.begin(115200);
  // No while (!Serial): sensing also runs with Serial Monitor closed.

  pinMode(SEND_PIN, OUTPUT);
  digitalWrite(SEND_PIN, LOW);
  sendPort = portOutputRegister(digitalPinToPort(SEND_PIN));
  sendMask = digitalPinToBitMask(SEND_PIN);

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(LEDS[i], LOW);
    pinMode(KEYS[i], INPUT);
    digitalWrite(KEYS[i], LOW);  // Ensure the internal pull-up is disabled.
    sensePorts[i] = portInputRegister(digitalPinToPort(KEYS[i]));
    senseMasks[i] = digitalPinToBitMask(KEYS[i]);
  }

  // Free-running Timer1, normal mode, prescaler 8:
  // 16 MHz / 8 = 2 MHz, so one tick is 0.5 us.
  // Timer0 is untouched, so micros() and millis() keep working.
  uint8_t savedSreg = SREG;
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TIMSK1 = 0;
  TCNT1 = 0;
  TCCR1B = _BV(CS11);
  SREG = savedSreg;

  if (KEYBOARD_ON) {
    Keyboard.begin();
  }

  phaseStartUs = micros();
}

void startAndMeasureRise() {
  volatile uint8_t *inputPort = sensePorts[which];
  const uint8_t inputMask = senseMasks[which];
  const bool startedLow = ((*inputPort & inputMask) == 0);

  // Fast port access keeps software overhead small for the 100 kOhm circuit.
  const uint16_t riseStartTicks = TCNT1;
  *sendPort |= sendMask;
  measurementValid = false;

  if (!startedLow) {
    return;  // The previous low phase did not discharge this input.
  }

  uint16_t elapsedTicks;
  do {
    // Unsigned subtraction also works if the 16-bit timer wraps once.
    elapsedTicks = (uint16_t)(TCNT1 - riseStartTicks);
    if (elapsedTicks >= TIMEOUT_TICKS) {
      return;
    }
  } while ((*inputPort & inputMask) == 0);

  approxTauUs = ORIGINAL_TAU_FACTOR * TIMER_TICK_US * elapsedTicks;
  measurementValid = true;
}

void updateOutputs() {
  const bool touched = measurementValid && (approxTauUs > tauThreshold);
  digitalWrite(LEDS[which], touched ? HIGH : LOW);

  // Send one message on each new touch, not continuously while held.
  if (KEYBOARD_ON && touched && !lastKeyStates[which]) {
    Keyboard.print(keyboardOutput[which]);
  }
  lastKeyStates[which] = touched;

  // Report channel 0 only, matching the original experiment.
  // Keep USB output out of the charging-time measurement.
  if (which == 0 && Serial && Serial.availableForWrite() >= 24) {
    if (measurementValid) {
      Serial.println(approxTauUs, 2);
    } else {
      Serial.println(-1);
    }
  }

  which = (which + 1) % NUM_CHANNELS;
}

void loop() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - phaseStartUs) < HALF_PERIOD_US) {
    return;
  }

  phaseStartUs += HALF_PERIOD_US;
  // Recover without a burst of short pulses if USB activity delayed a phase.
  if ((uint32_t)(nowUs - phaseStartUs) >= HALF_PERIOD_US) {
    phaseStartUs = nowUs;
  }

  waveHigh = !waveHigh;
  if (waveHigh) {
    startAndMeasureRise();
  } else {
    *sendPort &= (uint8_t)~sendMask;
    updateOutputs();
  }
}

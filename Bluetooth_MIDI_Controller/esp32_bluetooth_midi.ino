#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>

BLEMIDI_CREATE_DEFAULT_INSTANCE()

const int buttonPins[8] = {15, 18, 33, 32, 27, 26, 25, 23};
const int noteNumbers[8] = {60, 62, 64, 65, 67, 69, 71, 72};

const int potPinPitch = 34;
const int potPinVelocity = 35;

int lastButtonStates[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  BLEMIDI.setHandleConnected(onConnected);
  BLEMIDI.setHandleDisconnected(onDisconnected);

  MIDI.begin();
}

void loop() {
  int velocity = map(analogRead(potPinVelocity), 0, 4095, 0, 127);

  //center (8192), max (16383)
  int pitchValue = analogRead(potPinPitch);
  int pitchBend = map(pitchValue, 0, 1300, 0, 8190);
  MIDI.sendPitchBend(pitchBend, 1);

  for (int i = 0; i < 8; i++) {
    int currentState = digitalRead(buttonPins[i]);

    if (lastButtonStates[i] == HIGH && currentState == LOW) {
      // Button pressed
      MIDI.sendNoteOn(noteNumbers[i], velocity, 1);
    } else if (lastButtonStates[i] == LOW && currentState == HIGH) {
      // Button released
      MIDI.sendNoteOff(noteNumbers[i], 0, 1);
    }

    lastButtonStates[i] = currentState;
  }

  delay(5); //debounce
}

void onConnected() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void onDisconnected() {
  digitalWrite(LED_BUILTIN, LOW);
}

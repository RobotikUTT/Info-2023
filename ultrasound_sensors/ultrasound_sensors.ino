#include <Wire.h>

#define SOUND_SPEED 343 // m/s
#define DETECTION_DISTANCE 300  // mm

#define DEV_ENV false

#define NB_SONAR 12

#define TRIGGER A1


const uint8_t sonarPins[NB_SONAR] {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, A0};

volatile unsigned long distances[NB_SONAR];

// unsigned long lastTrigger = 0;

void setup() {
  #if DEV_ENV
    Serial.begin(115200);
  #endif  //DEV_ENV
  pinMode(TRIGGER, OUTPUT);
  for (int i = 0; i < NB_SONAR; i++) {
    pinMode(sonarPins[i], INPUT);
  }

  Wire.begin(2);
  Wire.onRequest(sendData);
}

void loop() {
  // unsigned long loopStart = millis();
  // int numberOfChecks = 0;
  // lastTrigger = micros();
  for (int i = 0; i < NB_SONAR; i++) {
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
    uint32_t echo_round_trip = pulseIn(sonarPins[i], HIGH, 5000UL);
    distances[i] = echo_round_trip * SOUND_SPEED / 1000 / 2;
    // echo in microsec, sound_speed in m/s, distanbce is mm
    delay(10);
    #if DEV_ENV
        Serial.print(i);
        Serial.write(':');
        Serial.print(echo_round_trip);
        Serial.write(':');
        Serial.println(distances[i]);
    #endif  //DEV_ENV
  }

  bool detected = false;
  for (int i = 0; i < NB_SONAR; i++) {
    if (distances[i] && distances[i] < DETECTION_DISTANCE) {
      #if DEV_ENV
        Serial.println(i);
      #endif  //DEV_ENV
      detected = true;
      // return;
    }
  }
  digitalWrite(LED_BUILTIN, detected);
}

void sendData() {
  for (int i = 0; i < NB_SONAR; i++) {
    Wire.write(distances[i] >> 8);
    Wire.write(distances[i] & 0xff);
  }
}

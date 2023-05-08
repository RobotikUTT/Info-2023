#define DEV_ENV true

#define TRIGGER A0

unsigned long distances[12];

unsigned long lastTrigger = 0;

void setup() {
  #if DEV_ENV
    Serial.begin(9600);
  #endif
  pinMode(TRIGGER, OUTPUT);
  for (int i = 2; i <= 13; i++) {
    pinMode(i, INPUT);
  }
}

void loop() {
  unsigned long loopStart = millis();
  int numberOfChecks = 0;
  lastTrigger = micros();
  for (int i = 2; i <= 13; i++) {
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
    distances[i-2] = pulseIn(i, HIGH, 5000UL);
  }
}

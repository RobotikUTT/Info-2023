#include <Wire.h>

#define DEV_ENV true

// Horizontal motor is X
// Vetcical motor is Y

#define HORIZONTAL_DIR_PIN           2  //PD2 -> set PORTD |= 0b00000100 ou 0x04   eteindre PORTE &= 0b11111011 ou 0xFB
#define VERTICAL_DIR_PIN           3  //PD3 -> set PORTD |= 0b00001000 ou 0x08   eteindre PORTE &= 0b11110111 ou 0xF7
#define zDirPin           4  //PD4 -> set PORTD |= 0b00010000 ou 0x10   eteindre PORTG &= 0b11101111 ou 0xEF
#define HORIZONTAL_STEP_PIN          5  //PD5 -> set PORTD |= 0b00100000 ou 0x20   eteindre PORTE &= 0b11011111 ou 0xDF
#define VERTICAL_STEP_PIN          6  //PD6 -> set PORTD |= 0b01000000 ou 0x40   eteindre PORTH &= 0b10111111 ou 0xBF
#define zStepPin          7  //PD7 -> set PORTD |= 0b10000000 ou 0x80   eteindre PORTH &= 0b01111111 ou 0x7F
#define ENABLE_PIN         8

#define STEP_HIGH_HORIZONTAL       PORTE |= _BV(PE4)   // Activation Pin Pas X
#define STEP_LOW_HORIZONTAL        PORTE &= ~_BV(PE4)  // Désactivation Pin Pas X
#define STEP_HIGH_VERTICAL       PORTE |= _BV(PE5)   // Activation Pin Pas Z
#define STEP_LOW_VERTICAL        PORTE &= ~_BV(PE5)  // Désactivation Pin Pas Y
#define STEP_HIGH_Z       PORTG |= _BV(PG5)   // Activation Pin Pas Z
#define STEP_LOW_Z        PORTG &= ~_BV(PG5)  // Désactivation Pin Pas Z
#define CLOSE_DIRECTION      PORTE |= _BV(PE3)
#define OPEN_DIRECTION   PORTE &= ~_BV(PE3)
#define DOWN_DIRECTION      PORTH |= _BV(PH3)
#define UP_DIRECTION   PORTH &= ~_BV(PH3)
#define SENS_TRIGO_Z      PORTH |= _BV(PH4)
#define SENS_HORRAIRE_Z   PORTH &= ~_BV(PH4)

#define STEPS_TO_GO_UP 1000
#define STEPS_TO_OPEN 220

const PROGMEM long dtMaxSpeed = 2000;

int verticalPos = 0;
int horizontalPos = 0;

bool opening = false;
bool closing = false;
bool goingDown = false;
bool goingUp = false;

void setup() {
  Wire.begin(3);

  pinMode(HORIZONTAL_STEP_PIN, OUTPUT);  // Pin pas moteur x en mode OUTPUT
  pinMode(VERTICAL_STEP_PIN, OUTPUT);  // Pin pas moteur y en mode OUTPUT
  //pinMode(zStepPin, OUTPUT);  // Pin pas moteur z en mode OUTPUT
  pinMode(HORIZONTAL_DIR_PIN, OUTPUT);   // Pin dir moteur x en mode OUTPUT
  pinMode(VERTICAL_DIR_PIN, OUTPUT);   // Pin dir moteur y en mode OUTPUT
  //pinMode(zDirPin, OUTPUT);   // Pin dir moteur z en mode OUTPUT
  pinMode(ENABLE_PIN, OUTPUT); // Pin alimentation moteurs en mode OUTPUT

  Serial.begin(9600);

  AlimMoteurs(false); // For the security
  AlimMoteurs(true);
}

void loop() {
  if (horizontalPos == STEPS_TO_OPEN && verticalPos == STEPS_TO_GO_UP) {
    Serial.println("Closing and going down");
    Close();
    GoDown();
  } else if (horizontalPos == 0 && verticalPos == 0) {
    Serial.println("Opening and going up");
    Open();
    GoUp();
  }
  MoveMotors();
  delayMicroseconds(dtMaxSpeed);
}

void MoveMotors() {
  // Horizontal motors
  if (opening && horizontalPos < STEPS_TO_OPEN) {
    Serial.println("open");
    //OPEN_DIRECTION;
    HorizontalStep();
    horizontalPos++;
  } else if (closing && horizontalPos > 0) {
    //Serial.println("close");
    HorizontalStep();
    horizontalPos--;
  }
  // Vertical motors
  if (goingUp && verticalPos < STEPS_TO_GO_UP) {
    VerticalStep();
    verticalPos++;
  } else if (goingDown && verticalPos > 0) {
    VerticalStep();
    verticalPos--;
  }
}

void test() {
  AlimMoteurs(true);
  int i = 0;
  while (true) {
    if (i >= STEPS_TO_OPEN) {
      continue;
    }
    i++;
    Serial.println(i);
    delay(10);
    Open();
  }
}

void GoUp() {
  UP_DIRECTION;
  goingUp = true;
  goingDown = false;
}

void GoDown() {
  DOWN_DIRECTION;
  goingDown = true;
  goingUp = false;
}

void Open() {
  OPEN_DIRECTION;
  opening = true;
  closing = false;
}

void Close() {
  CLOSE_DIRECTION;
  closing = true;
  opening = false;
}

void AlimMoteurs(bool power) { // Alimentation ou coupure du courant des moteurs
  digitalWrite(ENABLE_PIN, !power);
}

void HorizontalStep() {
  STEP_HIGH_HORIZONTAL;                 //set la pin sur HIGH
  STEP_LOW_HORIZONTAL;                  //set la pin sur LOW
}

void VerticalStep() {
  STEP_HIGH_VERTICAL;                 //set la pin sur HIGH
  STEP_LOW_VERTICAL;                  //set la pin sur LOW
}

void PasMoteurZ() {
  STEP_HIGH_Z;                 //set la pin sur HIGH
  STEP_LOW_Z;                  //set la pin sur LOW
}
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

#define STEPS_TO_GO_UP 4000
#define STEPS_TO_OPEN 220

#define GO_UP_ACTION   0
#define GO_DOWN_ACTION 1
#define OPEN_ACTION    2
#define CLOSE_ACTION   3
#define STOP_ACTION    4

const PROGMEM long dtMaxSpeed = 1000;

int verticalPos = STEPS_TO_GO_UP;
int horizontalPos = STEPS_TO_OPEN;

bool opening = false;
bool closing = false;
bool goingDown = false;
bool goingUp = false;

void setup() {
  Wire.begin(3);
  Wire.onReceive(DoAction);
  Wire.onRequest(IsBusy);

  pinMode(HORIZONTAL_STEP_PIN, OUTPUT);  // Pin pas moteur x en mode OUTPUT
  pinMode(VERTICAL_STEP_PIN, OUTPUT);  // Pin pas moteur y en mode OUTPUT
  //pinMode(zStepPin, OUTPUT);  // Pin pas moteur z en mode OUTPUT
  pinMode(HORIZONTAL_DIR_PIN, OUTPUT);   // Pin dir moteur x en mode OUTPUT
  pinMode(VERTICAL_DIR_PIN, OUTPUT);   // Pin dir moteur y en mode OUTPUT
  //pinMode(zDirPin, OUTPUT);   // Pin dir moteur z en mode OUTPUT
  pinMode(ENABLE_PIN, OUTPUT); // Pin alimentation moteurs en mode OUTPUT

  Serial.begin(9600);

  AlimMoteurs(true);
  Serial.println("Initialized");
}

void loop() {
  /*if (verticalPos == STEPS_TO_GO_UP && horizontalPos == STEPS_TO_OPEN) {
    GoDown();
  }
  if (horizontalPos == STEPS_TO_OPEN && verticalPos == 0) {
    Close();
  }
  if (horizontalPos == 0 && verticalPos == 0) {
    GoUp();
  }*/
  MoveMotors();
  delayMicroseconds(dtMaxSpeed);
}

void MoveMotors() {
  // Horizontal motors
  if (opening && horizontalPos < STEPS_TO_OPEN) {
    HorizontalStep();
    horizontalPos++;
    if (horizontalPos == STEPS_TO_OPEN) {
      opening = false;
    }
  } else if (closing && horizontalPos > 0) {
    HorizontalStep();
    horizontalPos--;
    Serial.println(horizontalPos);
    if (horizontalPos == 0) {
      closing = false;
    }
  }
  // Vertical motors
  if (goingUp && verticalPos < STEPS_TO_GO_UP) {
    VerticalStep();
    verticalPos++;
    if (verticalPos == STEPS_TO_GO_UP) {
      goingUp = true;
    }
  } else if (goingDown && verticalPos > 0) {
    VerticalStep();
    verticalPos--;
    if (verticalPos == 0) {
      goingDown = false;
    }
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
  Serial.println("closinnnng !!!");
  closing = true;
  opening = false;
}

/** Code for controlling direcly motors **/

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

/** Callback from the Wire **/

void DoAction() {
  char action = Wire.read();
  switch (action) {
    case GO_UP_ACTION:
      GoUp();
    case GO_DOWN_ACTION:
      GoDown();
    case OPEN_ACTION:
      Open();
    case CLOSE_ACTION:
      Close();
    case STOP_ACTION:
      opening = false;
      closing = false;
      goingUp = false;
      goingDown = false;
  }
}

void IsBusy() {
  // Serial.print(closing);
  // Serial.print(" ");
  // Serial.print(horizontalPos);
  // Serial.print(" ");
  // Serial.print(opening);
  // Serial.print(" ");
  // Serial.print(goingUp);
  // Serial.print(" ");
  // Serial.println(goingDown);
  Wire.write(closing || opening || goingUp || goingDown);
}
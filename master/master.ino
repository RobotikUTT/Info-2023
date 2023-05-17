#include <Wire.h>

#define DEV_ENV true
#if DEV_ENV
  #define PRINT(x) Serial.print(x)
  #define PRINTLN(x) Serial.println(x)
#else
  #define PRINT(x)
  #define PRINTLN(x)
#endif

#define I2C_MOVING_ID 1
#define I2C_ULTRASOUND_SENSOR_ID 2
#define I2C_CLAWS_ID 3

#define START_PIN             0

#define CHANGE_STRATEGY_PIN   0
#define STRATEGY_ID_BIT_1_PIN 0
#define STRATEGY_ID_BIT_2_PIN 0
#define STRATEGY_ID_BIT_4_PIN 0

#define NUMBER_OF_STRATEGIES 0

#define STRATEGY_CLAWS_DO_NOTHING       0
#define STRATEGY_CLAWS_GO_DOWN          1
#define STRATEGY_CLAWS_GO_UP            2
#define STRATEGY_CLAWS_OPEN             3
#define STRATEGY_CLAWS_CLOSE            4
#define STRATEGY_CLAWS_GO_DOWN_AND_OPEN 5

#define RADIUS_ROBOT 0.15

int distances[12];

String inputString = "";


const float strategy0[] = {
  // Strat départ assiette à côté du plat adverse
  // Values at the beginning (x, y, orientation)
  // mettre rayon robot en x
  RADIUS_ROBOT, 1.875, PI/2,
  // Number of targets
  4,
  // Coords x, y, claws
  0.225, 2.1, STRATEGY_CLAWS_GO_DOWN_AND_OPEN,
  0.225, 2.225, STRATEGY_CLAWS_DO_NOTHING,
  0.225, 2.225, STRATEGY_CLAWS_CLOSE,
  0.225, 2.225, STRATEGY_CLAWS_GO_UP,
  0.225, 2.4
};
const float strategy1[] = {

};

int strategyId = 0;
// A pointer to the array (aka an array)
float* strategy = 0;
int strategyStep = -1;

bool gameStarted = false;
float strategyFinished = false;

// Angle that stores the direction we are moving to
float currentMovingDirection = 0;

float xRequest, yRequest, angleRequest;

float position[2];
float angle;

void setup() {
  #if DEV_ENV
    Serial.begin(9600);  // Démarrage port série
    Serial.setTimeout(1);  // Délai d'attente port série 1ms
    while (!Serial);
  #endif

  Wire.begin();

  // pinMode(CHANGE_STRATEGY_PIN, INPUT_PULLUP);
  // pinMode(STRATEGY_ID_BIT_1_PIN, OUTPUT);
  // pinMode(STRATEGY_ID_BIT_2_PIN, OUTPUT);
  // pinMode(STRATEGY_ID_BIT_4_PIN, OUTPUT);

  // pinMode(START_PIN, INPUT_PULLUP);

  StopMoving();
  StopClaws();
  
  PRINTLN("Initialised");
}

void loop() {
  if (!gameStarted) {
    CheckStrategyButton();
    CheckStartGameButton();
  } else if (!strategyFinished) {
    RunStrategy();
  }
}

void CheckStrategyButton() {
  if (digitalRead(CHANGE_STRATEGY_PIN) && false) {
    strategyId = (strategyId + 1) % NUMBER_OF_STRATEGIES;
    digitalWrite(STRATEGY_ID_BIT_1_PIN, strategyId & 1);
    digitalWrite(STRATEGY_ID_BIT_2_PIN, strategyId & 2);
    digitalWrite(STRATEGY_ID_BIT_4_PIN, strategyId & 4);
    // Wait until the button is released
    while (digitalRead(CHANGE_STRATEGY_PIN));
  }
}

void CheckStartGameButton() {
  if (digitalRead(START_PIN) || true) {
    gameStarted = true;
    // strategyId = digitalRead(STRATEGY_ID_BIT_1_PIN) |
    //              digitalRead(STRATEGY_ID_BIT_2_PIN) << 1 |
    //              digitalRead(STRATEGY_ID_BIT_4_PIN) << 2;
    strategyId = 0;
    switch(strategyId) {
      case 0:
        strategy = strategy0;
        break;
      default:
        // The strategy does not exist ! To avoid any strange thing with memory, it is better to just do nothing. And hope this case never happens >.<
        strategyFinished = true;
        return;
    }
    position[0] = strategy[0];
    position[1] = strategy[1];
    angle = strategy[2];
  }
}

void CheckRobotsAround() {
  FetchDistances();
  float* relativeTargetPosition = GetTarget();
  // If we are rotating, there should not be any problem with collisions
  if (relativeTargetPosition[1] < 0) {
    return;
  }
  relativeTargetPosition[0] -= position[0];
  relativeTargetPosition[1] -= position[1];
  for (int i = 0; i < 12; i++) {
    if (distances[i] < 1500) {
      // float array of length 2
      
      float movingAngle;
      if (relativeTargetPosition[0] == 0) {
        // y should never equal 0 at the same time as x. If it does, well, it should not (and it should not break things either anyway)
        movingAngle = PI / 2;
      } else {
        movingAngle = atan(relativeTargetPosition[1] / relativeTargetPosition[0]);
      }
      float obstacleAngle = fmod(i * PI/6 - movingAngle, 2*PI); // Angle relative to the angle of the movement
      // The obstacle is at the back
      if (obstacleAngle > PI/2 && obstacleAngle < 3*PI/2) {
        continue;
      }
      float distanceInMeters = 344 * (distances[i]/1000000.0) / 2;
      float relativeObstacleX = cos(movingAngle) * distanceInMeters;
      float relativeObstacleY = sin(movingAngle) * distanceInMeters;
      // Changing base to the absolute one
      float absoluteObstacleX = cos(angle) * relativeObstacleX + sin(angle) * relativeObstacleY;
      float absoluteObstacleY = -sin(angle) * relativeObstacleX + cos(angle) * relativeObstacleY;
      float turnAngle = PI - 2 * obstacleAngle;
      Rotation(absoluteObstacleX, absoluteObstacleY, turnAngle);
    }
  }
}


void RunStrategy() {
  // float array of length 2
  float* target = GetTarget();
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  if (strategyStep == -1 || (abs(position[0] + xMoved - target[0]) < 0.005 && abs(position[1] + yMoved - target[1]) < 0.005)) {
    // If claws haven't finished yet, we let them finish their job
    if (AreClawsBusy()) {
      return;
    }
    strategyStep++;
    // If we reached the end of the strategy, we leave
    if (strategyStep >= strategy[3]) {
      strategyFinished = true;
      PRINTLN("STRATEGY finished !!");
      return;
    }
    target = GetTarget();
    // If this is a rotation
    if (target[1] < 0) {
      Rotation(0, 0, target[0]);
    } else if (target[0] != position[0] || target[1] != position[1]) {
      Move(target[0], target[1]);
    }
    switch ((int) target[2]) {
      case STRATEGY_CLAWS_DO_NOTHING:
        break;
      case STRATEGY_CLAWS_GO_DOWN:
        GoDownClaws();
        break;
      case STRATEGY_CLAWS_GO_UP:
        GoUpClaws();
        break;
      case STRATEGY_CLAWS_OPEN:
        OpenClaws();
        break;
      case STRATEGY_CLAWS_CLOSE:
        CloseClaws();
        break;
      case STRATEGY_CLAWS_GO_DOWN_AND_OPEN:
        GoDownClaws();
        OpenClaws();
        break;
    }
  } else if (abs(xMoved - xRequest) < 0.001 && abs(yMoved - yRequest) < 0.001 && abs(angleMoved - angleRequest) < 0.001) {
    // If we enter in this condition, movement should always be a straight path, and never a rotation
    Move(target[0], target[1]);
  }
}

// Returns the position of the current target in the strategy
float* GetTarget() {
  // We return a pointer to the right value of the array
  return strategy + 4 + strategyStep * 3;
}

void TestEachDirection() {
  float angle = 0;
  float x, y;
  float xMoved = 9999, yMoved = 9999;
  while (angle < 2 * PI) {
    PRINT("Testing angle ");
    PRINT(angle * 180 / PI);
    PRINTLN("°");
    x = cos(angle) * 0.5;
    y = sin(angle) * 0.5;
    Move(x, y);
    while (xMoved != x || yMoved != y) {
      FetchDistanceMoved(xMoved, yMoved);
    }
    Move(-x, -y);
    while (xMoved != -x || yMoved != -y) {
      FetchDistanceMoved(xMoved, yMoved);
    }
    angle += 0.5;
  }
}

/** WHEELS COMMUNICATION **/

void Move(float x, float y) {
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  // PRINT("distance Moved x : ");
  // PRINTLN(xMoved);
  position[0] += xMoved;
  position[1] += yMoved;
  angle += angleMoved;
  float relativeXToMove = x - position[0];
  float relativeYToMove = y - position[1];
  // PRINT("Relative position of target : ");
  // PRINT(relativeXToMove);
  // PRINT(", ");
  // PRINTLN(relativeYToMove);
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(1);
  writeFloatToWire(relativeXToMove);
  writeFloatToWire(relativeYToMove);
  Wire.write(100);
  Wire.endTransmission();
  xRequest = x - position[0];
  yRequest = y - position[1];
  angleRequest = 0;
  PRINTLN((String) "Consigne 1 envoyée : Move " + (x-position[0]) + " " + (y-position[1]));
}

void Rotation(float xCentre, float yCentre, float angle) {
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  position[0] += xMoved;
  position[1] += yMoved;
  angle += angleMoved;
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(2);
  writeFloatToWire(xCentre - position[0]);
  writeFloatToWire(yCentre - position[1]);
  writeFloatToWire(angle);
  Wire.write(100);
  Wire.endTransmission();
  xRequest = xCentre - position[0];
  yRequest = yCentre - position[1];
  angleRequest = angle;
  PRINTLN("Consigne 2 envoyée : Rotation");
}

void StopMoving() {
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(0);
  Wire.endTransmission();
  PRINTLN("Consigne 3 envoyée : Arret");
}

void FetchDistanceMoved(float &x, float &y) {
  Wire.requestFrom(I2C_MOVING_ID, 12);
  x = readFloatFromWire();
  y = readFloatFromWire();
  // we don't care about the angle in this implementation
  readFloatFromWire();
}

void FetchDistanceMoved(float &x, float &y, float &angle) {
  Wire.requestFrom(I2C_MOVING_ID, 12);
  x = readFloatFromWire();
  y = readFloatFromWire();
  angle = readFloatFromWire();
}

/** ULTRASOUND SENSORS COMMUNICATION **/

void FetchDistances() {
  Wire.requestFrom(I2C_ULTRASOUND_SENSOR_ID, 24);
  for (int i = 0; i < 12; i++) {
    distances[i] = (Wire.read() << 8) | Wire.read();
  }
}

/** CLAWS COMMUNICATION **/

void GoUpClaws() {
  _SendActionToClaws(0);
  PRINTLN("Asking claws to go up");
}

void GoDownClaws() {
  _SendActionToClaws(1);
  PRINTLN("Asking claws to go down");
}

void OpenClaws() {
  _SendActionToClaws(2);
  PRINTLN("Asking claws to open");
}

void CloseClaws() {
  _SendActionToClaws(3);
  PRINTLN("Asking claws to close");
}

void StopClaws() {
  _SendActionToClaws(4);
  PRINTLN("Asking claws to stop");
}

void _SendActionToClaws(int action) {
  Wire.beginTransmission(I2C_CLAWS_ID);
  Wire.write(action);
  Wire.endTransmission();
}

bool AreClawsBusy() {
  Wire.requestFrom(I2C_CLAWS_ID, 1);
  return Wire.read();
}

/** UTILITY FUNCTION TO READ/WRITE FROM/TO Wire **/

void writeFloatToWire(float value) {
  uint32_t bits = *((uint32_t*) &value);
  Wire.write((bits >> 24) & 0xff);
  Wire.write((bits >> 16) & 0xff);
  Wire.write((bits >> 8) & 0xff);
  Wire.write(bits & 0xff);
}

float readFloatFromWire() {
  uint32_t bits = ((uint32_t) Wire.read() << 24) |
                  ((uint32_t) Wire.read() << 16) |
                  ((uint32_t) Wire.read() << 8) |
                  (uint32_t) Wire.read();
  return *((float*) &bits);
}


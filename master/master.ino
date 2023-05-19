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

#define START_PIN             2

#define CHANGE_STRATEGY_PIN   3
#define STRATEGY_ID_BIT_1_PIN 0
#define STRATEGY_ID_BIT_2_PIN 0
#define STRATEGY_ID_BIT_4_PIN 0

#define NUMBER_OF_STRATEGIES 2

#define RADIUS_ROBOT 0.15

#define BUILTIN_LED_PIN 13

int distances[12];
//const uint8_t sonarIds[12] {8, 11, 6, 2, 5, 4, 3, 0, 11, 11, 11, 9};
const uint8_t sonarIds[12] {8, 10, 2, 6, 5, 4, 1, 10, 0, 11, 10, 10};

String inputString = "";
long startTime_;

/*const float strategy0[] = {
  // Strat départ assiette à côté du plat adverse
  // Values at the beginning (x, y, orientation)
  // mettre rayon robot en x
  0, 0, 0, //RADIUS_ROBOT, 1.875, PI/2,
  // Number of targets
  4,
  // Coords x, y, claws
  10, 0,//0.225, 2.1, STRATEGY_CLAWS_DO_NOTHING,//STRATEGY_CLAWS_GO_DOWN_AND_OPEN,
  0.225, 2.225,//STRATEGY_CLAWS_DO_NOTHING,
  0.225, 2.225,//STRATEGY_CLAWS_CLOSE,
  0.225, 2.225,//STRATEGY_CLAWS_GO_UP,
  0.225, 2.4,
};*/

// Homologation
const float strategy0[] = {
  2.3, 2.56, -PI/2,
  9,
  1.3, 0,//2.3, 1.26,
  -0.44, 0,//2.3, 1.70,
  PI/2, -1,
  //2.3, -2,
  //1.7, -3,
  0.495, 0,
  -PI/2, -1,
  //0.725, -2,
  //1.70, -3,
  0.46, 0,//0.725, 1.24,
  1.17, 0, //0.725, 0.07,
  -3, 0,
  0, -0.5
  //-3, 0,//0.725, 0.27,
  /*PI, -1,
  0.725, -2,
  0.27, -3,
  0.135, 0.33,
  0.135, -2,
  0.33, -1,
  0.23, 0.4,*/
  //0.23, 0.45,
};

/*const float strategy0[] = {
  0, 0, 0,
  1,
  0, 1,
};*/

const float strategy1[] = {
  2.3, 2.56, -PI/2,
  9,
  1.3, 0,
  -0.44, 0,
  -PI/2, -1,
  0.495, 0,
  PI/2, -1,
  0.46, 0,
  1.17, 0,
  -2, 0,
  0, 0.5
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

//float position[2];
//float angle;

void setup() {
  #if DEV_ENV
    Serial.begin(9600);  // Démarrage port série
    Serial.setTimeout(1);  // Délai d'attente port série 1ms
    while (!Serial);
  #endif

  PRINTLN("Starting");

  Wire.begin();

  pinMode(CHANGE_STRATEGY_PIN, INPUT_PULLUP);
  // pinMode(STRATEGY_ID_BIT_1_PIN, OUTPUT);
  // pinMode(STRATEGY_ID_BIT_2_PIN, OUTPUT);
  // pinMode(STRATEGY_ID_BIT_4_PIN, OUTPUT);

  pinMode(START_PIN, INPUT_PULLUP);

  pinMode(BUILTIN_LED_PIN, OUTPUT);

  StopMoving();
  
  PRINTLN("Initialised");
}

void loop() {
  if (!gameStarted) {
    CheckStrategyButton();
    CheckStartGameButton();
    delay(100);
  } else if (!strategyFinished) {
    if (millis() - startTime_ > 95000) {
      StopMoving();
      strategyFinished = true;
    }
    RunStrategy();
    if (millis() - startTime_ > 5000 && millis() - startTime_ < 94000) {
      CheckRobotsAround();
    }
  }
}

void CheckStrategyButton() {
  if (!digitalRead(CHANGE_STRATEGY_PIN)) {
    strategyId = (strategyId + 1) % NUMBER_OF_STRATEGIES;
    //digitalWrite(STRATEGY_ID_BIT_1_PIN, strategyId & 1);
    //digitalWrite(STRATEGY_ID_BIT_2_PIN, strategyId & 2);
    //digitalWrite(STRATEGY_ID_BIT_4_PIN, strategyId & 4);
    // Wait until the button is released
    while (!digitalRead(CHANGE_STRATEGY_PIN));
    for (int i = 0; i < strategyId + 1; i++) {
      digitalWrite(BUILTIN_LED_PIN, HIGH);
      delay(500);
      digitalWrite(BUILTIN_LED_PIN, LOW);
      delay(500);
    }
  }
}

void CheckStartGameButton() {
  Serial.println(digitalRead(START_PIN));
  if (digitalRead(START_PIN)) {
    gameStarted = true;
    startTime_ = millis();
    //strategyId = 1;
    switch(strategyId) {
      case 0:
        strategy = strategy0;
        break;
      case 1:
        strategy = strategy1;
        break;
      default:
        // The strategy does not exist ! To avoid any strange thing with memory, it is better to just do nothing. And hope this case never happens >.<
        strategyFinished = true;
        // Serial.println("unknown");
        return;
    }
    //position[0] = strategy[0];
    //position[1] = strategy[1];
    //angle = strategy[2];
    delay(1000);
  }
}

void CheckRobotsAround() {
  bool canContinueMoving = false;
  bool hasStopped = false;
  while (!canContinueMoving) {
    canContinueMoving = true;
    FetchDistances();
    // float array of length 2
    float* targetPosition = GetTarget();
    // If we are rotating, there should not be any problem with collisions
    if (targetPosition[1] < 0) {
      return;
    }
    //float relativeTargetPosition[2] = {targetPosition[0] - position[0], targetPosition[1] - position[1]};
    //relativeTargetPosition[0] -= position[0];
    //relativeTargetPosition[1] -= position[1];
    //bool forward = (targetPosition[0] - position[0]) * cos(angle) > 0.01 || (targetPosition[1] - position[1]) * sin(angle) > 0.01;
    //Serial.println("in the loop");
    for (int j = 0; j < 12; j++) {
      int i = sonarIds[j];
      if (distances[j] > 300 || distances[j] < 10) {
        continue;
      }
      //Serial.print(distances[j]);
      //Serial.print(" ");
      if (i != 11 && i != 0 && i != 1 && (i < 4 || i > 8)) {
        continue;
      }
      StopMoving();
      canContinueMoving = false;
      hasStopped = true;
      //float movingAngle = angle;
      /*if (relativeTargetPosition[0] == 0) {
        // y should never equal 0 at the same time as x. If it does, well, it should not (and it should not break things either anyway)
        if (relativeTargetPosition[1] > 0) {
          movingAngle = PI / 2;
        } else {
          movingAngle = -PI / 2;
        }
      } else {
        movingAngle = atan(relativeTargetPosition[1] / relativeTargetPosition[0]);
        if (relativeTargetPosition[0] > 0) {
          movingAngle += PI;
        }
      }*/
      //movingAngle = fmod(movingAngle, 2*PI);
      //float obstacleAngleRelativeToDirection = fmod(i * PI/6 - movingAngle, 2*PI); // Angle relative to the angle of the movement
      //Serial.println(obstacleAngleRelativeToDirection);
      // The obstacle is at the back
      /*if (obstacleAngleRelativeToDirection > PI/2 && obstacleAngleRelativeToDirection < 3*PI/2) {
        continue;
      }*/
      //float obstacleAngleRelativeToRobot = fmod(i * PI/6 - movingAngle, 2*PI); // The angle relative to the robot
      //float relativeObstacleX = cos(obstacleAngleRelativeToRobot) * distances[i]/1000.0;
      //float relativeObstacleY = sin(obstacleAngleRelativeToRobot) * distances[i]/1000.0;
      // Changing base to the absolute one
      //float absoluteObstacleX = cos(angle-PI/2) * relativeObstacleX + sin(angle-PI/2) * relativeObstacleY + position[0];
      //float absoluteObstacleY = -sin(angle-PI/2) * relativeObstacleX + cos(angle-PI/2) * relativeObstacleY + position[1];
      // The obstacle is outside of the terrain
      // if (absoluteObstacleX <= 0.05 || absoluteObstacleX >= 1.95 || absoluteObstacleY <= 0.05 || absoluteObstacleY >= 2.95) {
      //   continue;
      // }
      // StopMoving();
      // Serial.print("stop ! ");
      // Serial.print(absoluteObstacleX);
      // Serial.print(" ");
      // Serial.println(absoluteObstacleY);
      // canContinueMoving = false;
      // hasStopped = true;
      /*if (distances[i] < 1000) {
        Mouvement(-relativeObstacleX, -relativeObstacleY);
      } else if (distances[i] < 1400) {
        float turnAngle = fmod(fmod(PI - 2 * obstacleAngleRelativeToDirection, 2*PI) + PI, 2*PI) - PI;
        Rotation(absoluteObstacleX, absoluteObstacleY, turnAngle);
      }*/
    }
    //Serial.println();
    //Serial.println(canContinueMoving);
  }
  //Serial.println("donc c'est bon, on est sortis");
  // If we stopped, we need to start moving again
  if (hasStopped) {
    float* target = GetTarget();
    // If this is a rotation
    float xMoved, yMoved, angleMoved;
    FetchDistanceMoved(xMoved, yMoved, angleMoved);
    if (target[1] < 0) {
      Rotation(0, 0, target[0] - angleMoved);
    } else if (xRequest - xMoved != 0 || yRequest - yMoved != 0) {
      Move(xRequest - xMoved, yRequest - yMoved);
    }
  }
}


void RunStrategy() {
  // float array of length 2
  float* target = GetTarget();
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  //Serial.print("on en est ici : ");
  //Serial.println(strategyStep);
  // Serial.print(xMoved);
  // Serial.print(" ");
  // Serial.println(target[0]);
  if (strategyStep == -1
     || target[1] <= -2
     || (target[1] != -1 && abs(xMoved - xRequest) < 0.005 && abs(yMoved - yRequest) < 0.005)
     || target[1] == -1 /*&& (fmod(fmod(angleMoved, 2*PI) - fmod(angleRequest, 2*PI), 2*PI) < 0.05 || fmod(fmod(angleMoved, 2*PI) - fmod(angleRequest, 2*PI), 2*PI) > 2*PI-0.05)*/) {
    strategyStep++;
    StopMoving();
    // Serial.print("starting step ");
    // Serial.print(strategyStep);
    // Serial.print("/");
    // Serial.println(strategy[3]);
    //Serial.println("on rentre ici !");
    // Serial.println(strategyStep);
    // If we reached the end of the strategy, we leave
    if (strategyStep >= strategy[3]) {
      strategyFinished = true;
      PRINTLN("STRATEGY finished !!");
      return;
    }
    // Serial.println("ici !");
    target = GetTarget();
    // Serial.println(target[0]);
    // Serial.println(target[1]);
    // Serial.println(position[0]);
    // Serial.println(position[1]);
    // If this is a rotation
    //Serial.println("en runnant la strat");
    //Serial.println(target[1]);
    // Serial.println(target[1]);
    if (target[1] == -1) {
      // float xMoved, yMoved, angleMoved;
      // FetchDistanceMoved(xMoved, yMoved, angleMoved);
      Rotation(0, 0, target[0]);
      delay(3000);
    } else if (target[1] == -2) {
      //position[0] = target[0];
      StopMoving();
    } else if (target[1] == -3) {
      //position[1] = target[0];
      StopMoving();
    } else {
      // Serial.print("moving to ");
      // Serial.print(target[0]);
      // Serial.print(" ");
      // Serial.println(target[1]);
      Move(target[0], target[1]);
    }
  }/* else if (abs(xMoved - xRequest) < 0.001 && abs(yMoved - yRequest) < 0.001 && abs(angleMoved - angleRequest) < 0.001) {
    // If we enter in this condition, movement should always be a straight path, and never a rotation
    Move(target[0], target[1]);
  }*/
}

// Returns the position of the current target in the strategy
float* GetTarget() {
  // We return a pointer to the right value of the array
  return strategy + (4 + strategyStep * 2);
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
  //PRINT("distance Moved x : ");
  //PRINTLN(xMoved);
  //position[0] += xMoved;
  //position[1] += yMoved;
  //angle += angleMoved;
  //xRequest = x - position[0];
  //yRequest = y - position[1];
  /*if (xRequest == 0 && yRequest == 0) {
    return;
  }*/
  // Changing the base
  xRequest = x;
  // We need to inverse the direction of the y axis so that the coordinate system is turning in the trigonometric sens
  yRequest = y;
  float xReq = -cos(PI/2) * x - sin(PI/2) * y;
  float yReq = -sin(PI/2) * x + cos(PI/2) * y;
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(1);
  writeFloatToWire(xReq);
  writeFloatToWire(yReq);
  Wire.write(100);
  Wire.endTransmission();
  angleRequest = 0;
  PRINT("Consigne 1 envoyée : Move ");
  PRINT(x);
  PRINT(" ");
  PRINTLN(y);
}

void Rotation(float xCentre, float yCentre, float angle_) {
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  //position[0] += xMoved;
  //position[1] += yMoved;
  //angle += angleMoved;
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(2);
  writeFloatToWire(xCentre);
  writeFloatToWire(yCentre);
  writeFloatToWire(angle_);
  Wire.write(100);
  Wire.endTransmission();
  // xRequest = /*xCentre - */position[0];
  // yRequest = /*yCentre - */position[1];
  angleRequest = angle_;
  PRINTLN("Consigne 2 envoyée : Rotation");
}

void StopMoving() {
  float xMoved, yMoved, angleMoved;
  FetchDistanceMoved(xMoved, yMoved, angleMoved);
  PRINT("distance Moved x : ");
  PRINTLN(xMoved);
  // position[0] += xMoved;
  // position[1] += yMoved;
  //angle += angleMoved;
  // Serial.println("in the func");
  Wire.beginTransmission(I2C_MOVING_ID);
  // Serial.println("après");
  Wire.write(0);
  // Serial.println("encore après");
  Wire.endTransmission();
  delay(1000);
  // Serial.println("toujours après");
  PRINTLN("Consigne 3 envoyée : Arret");
}

void FetchDistanceMoved(float &x, float &y) {
  Wire.requestFrom(I2C_MOVING_ID, 12);
  float x_ = readFloatFromWire();
  float y_ = readFloatFromWire();
  // Serial.print("La distance déplacée : ");
  // Serial.print(x_);
  // Serial.print(" ");
  // Serial.println(y_);
  x = -cos(PI/2) * x_ + sin(PI/2) * y_;
  y = sin(PI/2) * x_ + cos(PI/2) * y_;
  // we don't care about the angle in this implementation
  readFloatFromWire();
}

void FetchDistanceMoved(float &x, float &y, float &angle) {
  Wire.requestFrom(I2C_MOVING_ID, 12);
  float x_ = readFloatFromWire();
  float y_ = readFloatFromWire();
  // Serial.print("La distance déplacée : ");
  // Serial.print(x_);
  // Serial.print(" ");
  // Serial.println(y_);
  x = -cos(PI/2) * x_ - sin(PI/2) * y_;
  y = sin(PI/2) * x_ + cos(PI/2) * y_;
  angle = readFloatFromWire();
}

/** ULTRASOUND SENSORS COMMUNICATION **/

void FetchDistances() {
  Wire.requestFrom(I2C_ULTRASOUND_SENSOR_ID, 24);
  for (int i = 0; i < 12; i++) {
    distances[i] = (Wire.read() << 8) | Wire.read();
    //Serial.print(distances[i]);
  }
  //Serial.println();
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


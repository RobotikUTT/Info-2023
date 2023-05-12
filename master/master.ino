#include <Wire.h>
#include <SoftwareSerial.h>

#define DEV_ENV true

#define SDA_MPU6050_Serial A4
#define SCL_MPU6050_Serial A5

#define I2C_MOVING_ID 8
#define I2C_ULTRASOUND_SENSOR_ID 2

unsigned long prevT = 0;
int dt = 10;
float xCentre = 0;
float yCentre = 0;
float phiReel = 0;
bool enMvmt = false;
bool attente = false;
bool balladeFinie = false;
int onSeBallade = 0;

int distances[12];

String inputString = "";

void setup() {
  Serial.begin(9600);  // Démarrage port série
  Serial.setTimeout(1);  // Délai d'attente port série 1ms
  
  Wire.begin();

  Serial.println(F("Initialisé"));
}

void loop() {
  FetchDistances();
  Deplacement(1/2, -sqrt(3)/2);
  //Rotation(0, 0, 2 * PI);
  //delay(4000);
  //Rotation(0, 0, -2 * PI);
  delay(100000);
}

/** WHEELS COMMUNICATION **/

void Deplacement(float x, float y) {
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(1);
  writeFloatToWire(x);
  writeFloatToWire(y);
  Wire.write(100);
  Wire.endTransmission();
  #if DEV_ENV
    Serial.println((String) "Consigne 1 envoyée : Deplacement " + x + " " + y);
  #endif
}

void Rotation(float xCentre, float yCentre, float angle) {
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(2);
  writeFloatToWire(xCentre);
  writeFloatToWire(yCentre);
  writeFloatToWire(angle);
  Wire.write(100);
  Wire.endTransmission();
  #if DEV_ENV
    Serial.println("Consigne 2 envoyée : Rotation");
  #endif
}

void StopMoving() {
  Wire.beginTransmission(I2C_MOVING_ID);
  Wire.write(0);
  Wire.endTransmission();
  #if DEV_ENV
    Serial.println("Consigne 3 envoyée : Arret");
  #endif
}

void FetchDistanceMoved(int &x, int &y) {
  Wire.request(I2C_MOVING_ID, 8);
  x = readFloatFromWire();
  y = readFloatFromWire();
}

/** ULTRASOUND SENSORS COMMUNICATION **/

void FetchDistances() {
  Wire.requestFrom(I2C_ULTRASOUND_SENSOR_ID, 24);
  for (int i = 0; i < 12; i++) {
    distances[i] = (Wire.read() << 8) | Wire.read();
  }
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

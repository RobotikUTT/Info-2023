#include <Wire.h>
#include <SoftwareSerial.h>

#define RX_Wheels_Serial 8
#define TX_Wheels_Serial 9
#define SDA_MPU6050_Serial A4
#define SCL_MPU6050_Serial A5

unsigned long prevT = 0;
int dt = 10;
float xCentre = 0;
float yCentre = 0;
float phiReel = 0;
bool enMvmt = false;
bool attente = false;
bool balladeFinie = false;
int onSeBallade = 0;

SoftwareSerial wheelsSerial(RX_Wheels_Serial, TX_Wheels_Serial);
String inputString = "";

void setup() {
  Serial.begin(9600);  // Démarrage port série
  Serial.setTimeout(1);  // Délai d'attente port série 1ms
  wheelsSerial.begin(57600);
  wheelsSerial.setTimeout(1);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  //synchro();

  Serial.println(F("Initialisé"));
}

// void synchro() {
//   Serial.println("synchronisation");
//   while (wheelsSerial.available()) { // vidage serial
//     wheelsSerial.parseFloat();
//   }
//   while (!wheelsSerial.available()) {  // Ecriture en boucle de 9 tant que pas de réponse
//     wheelsSerial.println("9");
//     delay(100);
//   }
//   while (wheelsSerial.available()) { // vidage serial
//     wheelsSerial.parseFloat();
//   }
//   delay(200);
// }

void loop() {
  CheckSerial();
  CheckWheelsSerial();
  if (millis() - prevT >= dt) {
    prevT = millis();
  }
  if (!enMvmt && !attente && !balladeFinie) {
    ballade();
  }
}

void ballade() {
  Serial.println((String) "ballade : " + onSeBallade);
  switch (onSeBallade) {
    case 0:
      onSeBallade++;
      Deplacement(0, 1.0);
      break;
    case 1:
      onSeBallade++;
      delay(100);
      Deplacement(1, 0);
      break;
    case 2:
      onSeBallade++;
      delay(100);
      Deplacement(-1, -1);
      break;
    case 3:
      onSeBallade++;
      delay(100);
      Rotation(0, 0.3, 6.28);
      break;
    case 4:
      onSeBallade++;
      delay(100);
      Rotation(0, -0.3, -6.28);
      break;
    /*
      case 5:
      onSeBallade++;
      delay(100);
      Rotation(0, 0, -6.28);
      break;
      case 6:
      onSeBallade++;
      delay(100);
      Deplacement(0, 1);
      break;
      case 7:
      onSeBallade++;
      delay(100);
      Deplacement(0, 1);
      break;
      case 8:
      onSeBallade++;
      delay(100);
      Deplacement(0, 1);
      break;
    */
    default:
      delay(500);
      Ancrage(false);
      balladeFinie = true;
      break;
  }
}

float borner(float angle) {  //
  if (angle > PI) {          //
    return angle - 2. * PI;  //
  } else if (angle <= PI) {  // On remet l'angle entre -180 et 180
    return angle + 2. * PI;  //
  }                          //
}

void CheckWheelsSerial() {             // Récupération des consignes via le port SWSerial (RX TX)
  if (wheelsSerial.available() > 0) {  //pour recuperer des valeurs du moniteur
    //Serial.print("Reception consigne : ");
    int command = wheelsSerial.parseInt();
    if (command == 0) {  //Fin mouvement
      Serial.println("Fin deplacement");
      enMvmt = false;
    } else if (command == 1) {  // Deplacement
      Serial.println("Deplacement commencé");
      enMvmt = true;
      attente = false;
    } /*else if (command == 2) {  // Demande rotation
      wheelsSerial.println((float)phiReel);
      Serial.println((String) "Renvoi valeur angle " + phiReel);
    }*/
    while (wheelsSerial.available() > 0) {
      //synchro();
      wheelsSerial.parseFloat();
    }
  }
}

void CheckSerial() {             // Récupération des consignes
  if (Serial.available() > 0) {  //pour recup des valeurs du moniteur
    enMvmt = false;
    attente = false;
    onSeBallade = 0;
    balladeFinie = false;
    while (Serial.available() > 0) {
      Serial.parseFloat();
    }
  }
}

void Arret() {
  wheelsSerial.println((int)0);
}

void Deplacement(float x, float y) {
  //synchro();
  wheelsSerial.print((String) "1" + " " + x + " " + y);
  attente = true;
  phiReel = 0;
  Serial.println((String) "Consigne 1 envoyée : Deplacement " + x + " " + y);
}

void Rotation(float xCentre, float yCentre, float angle) {
  //synchro();
  wheelsSerial.print(2);
  wheelsSerial.print(" ");
  wheelsSerial.print(xCentre);
  wheelsSerial.print(" ");
  wheelsSerial.print(yCentre);
  wheelsSerial.print(" ");
  wheelsSerial.print(angle);
  attente = true;
  phiReel = 0;
  Serial.println("Consigne 2 envoyée : Rotation");
}

void Ancrage(bool a) {
  if (a)
    wheelsSerial.println((String)3 + " " + 1);
  else
    wheelsSerial.println((String)3 + " " + 0);
}

float degToRad(float deg) {
  return deg * PI / 180.;
}

//-------------------------------------------------------------------
//////////////////////////      MPU       ///////////////////////////
//-------------------------------------------------------------------

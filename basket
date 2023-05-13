#include "Wire.h"
#include "LiquidCrystal_I2C.h"

// Position LCD sur A4 et A5 et 5V 

LiquidCrystal_I2C LCD(0x27,16,2); 
#define cny A0 

uint8_t compteur = 0;
uint8_t valeur = analogRead(cny);


void setup() {
   LCD.init();
   LCD.backlight();
   LCD.setCursor(1, 0);
   LCD.print("NOMBRE CERISE");
   LCD.setCursor(7, 1);
   LCD.print(valeur);
   Serial.begin(9600);
}

void loop() {
  Serial.println(valeur);
  if (cny > 200){
    valeur ++;
    LCD.print(compteur);
    delay(50);
  }
}

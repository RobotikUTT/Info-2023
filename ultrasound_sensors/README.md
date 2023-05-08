# Info-2023/ultrasound_sensors

Ce script gère les 12 capteurs ultrason (HCSR04) autour du robot. Il peut être utilisé sur une Arduino Nano.

## Branchements

* Les broches GND de tous les capteurs sont branchées sur le même pin GRN de l'Arduino.
* Les broches 3V3 de tous les capteurs sont branchées sur le même pin 3V3 de l'Arduino.
* Les broches TRIG de tous les capteurs sont branchées sur le pin A0 (constante `TRIGGER` dans le script) de l'Arduino
* La broche ECHO de chaque capteur est branchée sur les pins `D2`, `D3`, ..., `D13`. L'ID de chaque capteur est le numéro du pin sur lequel il est branché auquel on soustrait 2 (par exemple, l'ID du capteur branché sur le pin `D5` est 3)

/*
  06_RadarUS
  ----------
  Lecture d'un radar ultrasonique I2C avec CoVACIEL_radarUS.

  La mesure tourne en tâche FreeRTOS après start() :
  getDistance() retourne la dernière valeur mesurée sans bloquer.

  Il est possible de brancher plusieurs capteurs sur le même bus I2C
  en leur attribuant des adresses différentes (0x70, 0x71, 0x72...).

  Brochage :
    SDA -> GPIO 8
    SCL -> GPIO 9

  Dépendances : CoVACIEL
*/

#include <CoVACIEL.h>

#define SDA_PIN 8
#define SCL_PIN 9

CoVACIEL_radarUS radar;

void setup() {
  Serial.begin(115200);

  // Initialisation : adresse 0x70, mesure en centimètres
  radar.init(SDA_PIN, SCL_PIN, UNITE_CM, 0x70);

  // Vérification que le capteur répond
  if (radar.getVersion() < 0) {
    Serial.println("Capteur non détecté !");
    while (true);
  }
  Serial.printf("Capteur OK (firmware v%d)\n", radar.getVersion());

  // Démarrer la tâche de mesure en arrière-plan
  radar.start();
  Serial.println("Mesure en cours...");
}

void loop() {
  int d = radar.getDistance();  // cm
  Serial.printf("Distance : %d cm  (%d mm)\n", d, d * 10);
  delay(200);
}

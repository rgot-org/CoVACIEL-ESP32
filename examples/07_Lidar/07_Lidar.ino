/*
  07_Lidar
  --------
  Lecture d'un LiDAR rotatif RPLidar avec CoVACIEL_lidar.

  Le LiDAR tourne en tâche FreeRTOS après start().
  Les distances sont disponibles dans le tableau distance[23],
  indexé par secteur angulaire (0 = 0°, 22 = 352°, pas = 16°).

  ORIENTATION :
    Le câble de données du LiDAR doit être positionné vers l'ARRIÈRE
    du véhicule. L'avant du véhicule correspond donc au secteur 176°
    du LiDAR, soit l'index 11 du tableau distance[].

        AVANT véhicule
              ↑
             176° (index 11)
   80° ←  [LiDAR]  → 256°
             0°  (index 0)
              ↓
        ARRIÈRE véhicule (câble ici)

  Brochage :
    LiDAR RX    -> GPIO 17
    LiDAR TX    -> GPIO 16
    Moteur PWM  -> GPIO 5  (laisser -1 si non utilisé)

  Dépendances : CoVACIEL
*/

#include <CoVACIEL.h>

#define LIDAR_RX    16
#define LIDAR_TX    17
#define LIDAR_MOTOR  5   // -1 si la vitesse moteur n'est pas gérée par l'ESP32

CoVACIEL_lidar lidar(Serial1);

void setup() {
  Serial.begin(115200);
  Serial.println("Démarrage LiDAR...");

  lidar.init(115200, LIDAR_RX, LIDAR_TX, LIDAR_MOTOR);
  lidar.start();

  Serial.println("LiDAR en rotation — affichage toutes les secondes");
}

void loop() {
  static uint32_t lastMs = 0;

  if (millis() - lastMs >= 1000) {
    lastMs = millis();

    Serial.println("--- Distances LiDAR (mm) ---");
    for (int i = 0; i < 23; i++) {
      int angle = i * 16;  // angle en degrés (pas = 16°)
      Serial.printf("  Secteur %2d (%3d°) : %d mm\n", i, angle, lidar.distance[i]);
    }
  }
}

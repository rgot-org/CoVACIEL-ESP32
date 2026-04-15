/*
  01_CAN_Calculateur
  ------------------
  Carte "calculateur" : reçoit les distances avant via le bus CAN,
  calcule une consigne de direction et de vitesse, et les émet en retour.

  Brochage (ESP32-S3-Zero) :
    CAN RX -> GPIO 13
    CAN TX -> GPIO 12

  Dépendances : CoVACIEL, ArduinoJson
*/

#include <CoVACIEL.h>

#define CAN_RX 13
#define CAN_TX 12

CoVACIEL_CAN canbus;

void setup() {
  Serial.begin(115200);

  if (!canbus.init(CAN_RX, CAN_TX)) {
    Serial.println("Erreur init CAN");
    while (true);
  }
  Serial.println("Calculateur prêt");
}

void loop() {
  // 1. Recevoir les trames CAN et mettre à jour les distances
  canbus.updateRx();

  // 2. Lire les distances avant (en mm)
  int dGauche = canbus.getDistAvGauche45();
  int dCentre = canbus.getDistanceAvant();
  int dDroite = canbus.getDistAvDroite45();

  // Limiter à 1500 mm pour le calcul
  dGauche = constrain(dGauche, 0, 1500);
  dCentre = constrain(dCentre, 0, 1500);
  dDroite = constrain(dDroite, 0, 1500);

  // 3. Calcul simple : direction proportionnelle à l'asymétrie gauche/droite
  int angle = (dDroite - dGauche) / 60;   // degrés, environ ±18°
  angle = constrain(angle, -18, 18);

  // 4. Calcul vitesse : ralentir si obstacle devant
  int vitesse = map(dCentre, 0, 1500, 0, 60);
  if (dCentre < 150) vitesse = -20;        // marche arrière si trop proche

  // 5. Envoyer les commandes sur le bus CAN
  canbus.setAngleDirection(angle);
  canbus.setVitesse(vitesse);

  Serial.printf("G:%4d C:%4d D:%4d -> angle:%3d vit:%3d\n",
                dGauche, dCentre, dDroite, angle, vitesse);

  delay(100);
}

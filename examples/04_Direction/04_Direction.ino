/*
  04_Direction
  ------------
  Pilotage du servo de direction avec CoVACIEL_direction.

  Le servo est commandé en degrés :
    -18 = butée gauche
      0 = tout droit
    +18 = butée droite

  Les butées sont calibrées avec reglageButees() et sauvegardées
  en flash (Preferences). Un seul calibrage suffit.

  Brochage :
    Servo signal -> GPIO 10

  Dépendances : CoVACIEL
*/

#include <CoVACIEL.h>

#define PIN_SERVO 10

CoVACIEL_direction direction;

void setup() {
  Serial.begin(115200);
  direction.init(PIN_SERVO);
  Serial.println("Direction prête");
  Serial.println("Envoyez 'c' pour lancer le calibrage des butées");
}

void loop() {
  // Exemple : balayage gauche / centre / droite toutes les secondes
  static uint32_t lastMs = 0;
  static int etape = 0;

  if (millis() - lastMs >= 1000) {
    lastMs = millis();
    switch (etape % 3) {
      case 0: direction.setDirectionDegre(-18); Serial.println("Butée gauche"); break;
      case 1: direction.setDirectionDegre(0);   Serial.println("Tout droit");   break;
      case 2: direction.setDirectionDegre(18);  Serial.println("Butée droite"); break;
    }
    etape++;
  }

  // Lancer le calibrage interactif si demandé
  if (Serial.available() && Serial.read() == 'c') {
    direction.reglageButees();  // bloquant — quitter avec 'Q'
  }
}

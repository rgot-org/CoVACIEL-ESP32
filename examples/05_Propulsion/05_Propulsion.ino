/*
  05_Propulsion
  -------------
  Pilotage de l'ESC de propulsion avec CoVACIEL_propulsion.

  La propulsion est commandée en pourcentage :
    -100 = marche arrière max
       0 = arrêt
    +100 = marche avant max

  Les seuils PWM sont calibrés avec reglageButees() et sauvegardés
  en flash (Preferences). Un seul calibrage suffit.

  Note : init() envoie un signal d'arrêt pendant 3 secondes pour
  initialiser l'ESC — c'est normal.

  Brochage :
    ESC signal -> GPIO 9

  Dépendances : CoVACIEL
*/

#include <CoVACIEL.h>

#define PIN_ESC 9

CoVACIEL_propulsion propulsion;

void setup() {
  Serial.begin(115200);
  Serial.println("Initialisation ESC (3 secondes)...");
  propulsion.init(PIN_ESC);   // envoie le signal d'arrêt 3s pour initialiser l'ESC
  Serial.println("Propulsion prête");
  Serial.println("Envoyez 'c' pour lancer le calibrage, 'x' pour effacer la config");
}

void loop() {
  // Exemple : avant 2s → arrêt 1s → arrière 2s → arrêt 1s
  static uint32_t lastMs = 0;
  static int etape = 0;
  static const int sequence[]   = { 40,  0, -30,   0 };
  static const int durees[]     = { 2000, 1000, 2000, 1000 };

  if (millis() - lastMs >= (uint32_t)durees[etape % 4]) {
    lastMs = millis();
    int val = sequence[etape % 4];
    propulsion.setPropulsion(val);
    Serial.printf("Propulsion : %d%%\n", val);
    etape++;
  }

  // Calibrage ou effacement via Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') {
      propulsion.reglageButees();   // bloquant — quitter avec 'Q'
    } else if (c == 'x') {
      propulsion.clearParams();     // efface la config et redémarre
    }
  }
}

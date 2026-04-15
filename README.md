# CoVACIEL

Bibliothèque Arduino pour le véhicule autonome CoVACIEL (**ESP32**).

Bus CAN TWAI natif · Servo direction · ESC propulsion · Radar US I2C · LiDAR RPLidar

## Dépendances

- [ArduinoJson](https://arduinojson.org/) ≥ 7.x
- ESP32 Arduino Core ≥ 3.x

## Installation

Rechercher **CoVACIEL** dans le gestionnaire de bibliothèques Arduino, ou copier ce dossier dans `Arduino/libraries/`.

## Exemple minimal

```cpp
#include <CoVACIEL.h>

CoVACIEL_CAN canbus;

void setup() {
    canbus.init(13, 12);   // RX=13, TX=12
}

void loop() {
    canbus.updateRx();
    canbus.setAngleDirection(0);   // tout droit
    canbus.setVitesse(30);         // 30 % avant
}
```

## Documentation

Voir le dossier [docs/](docs/index.md) pour l'architecture, les identifiants CAN et la référence API complète.

## Licence

MIT — François Riotte

# CoVACIEL_Actionneur — Référence API

Ce fichier documente les deux classes d'actionneurs : `CoVACIEL_direction` (servo PWM) et `CoVACIEL_propulsion` (ESC brushless).

Les deux classes utilisent un signal PWM à **50 Hz** (période 20 ms) avec une résolution de **14 bits** (quantum ≈ 1,2 µs).

---

## Constantes globales

```cpp
#define RESOLUTION      14         // résolution PWM (14 bits)
#define SERVO_BASE_FREQ 50         // fréquence PWM en Hz
#define DIRECTION_CHANNEL  0       // canal LEDC pour la direction
#define PROPULSION_CHANNEL 1       // canal LEDC pour la propulsion

const int pleineEchelle = 16383;   // 2^14 - 1
const int angleDegreMax = 18;      // débattement max en degrés (±18°)
```

### Broche LED selon la carte

| Carte | `LED_PIN` |
|---|---|
| M5Stack ATOM | 27 |
| ESP32-S3-Zero | 21 |
| M5Stack Stamp C3 | 2 |
| M5Stack Stamp S3 | 21 |

---

## CoVACIEL_direction

Pilote un servo de direction via PWM. Le centre correspond à la position "tout droit", les butées gauche et droite sont configurables et persistées en mémoire flash (via `Preferences`).

### Initialisation

#### `void init(int pin)`

Initialise le signal PWM sur la broche indiquée.

```cpp
CoVACIEL_direction direction;

void setup() {
    direction.init(10); // broche GPIO 10
}
```

---

### Pilotage

#### `void setDirectionDegre(int value)`

Positionne le servo à l'angle souhaité.

| Paramètre | Description |
|---|---|
| `value` | Angle en degrés. `-18` = butée gauche, `0` = tout droit, `+18` = butée droite |

```cpp
direction.setDirectionDegre(0);   // tout droit
direction.setDirectionDegre(-10); // virage gauche
direction.setDirectionDegre(18);  // butée droite
```

---

### Configuration des butées

Les butées définissent les valeurs PWM correspondant aux positions extrêmes du servo. Elles sont stockées en flash et survivent à un redémarrage.

#### `void setButeeDroite(int value)` / `void setButeeGauche(int value)`

Enregistre la valeur PWM de la butée droite ou gauche.

#### `int getButeeDroite()` / `int getButeeGauche()`

Retourne la valeur PWM actuellement enregistrée pour chaque butée.

#### `void reglageButees()`

Mode interactif de réglage via `Serial`. Les commandes disponibles sont :

| Touche | Action |
|---|---|
| `bg` | Aller à la butée gauche |
| `bd` | Aller à la butée droite |
| `g` | Diminuer la butée gauche |
| `G` | Augmenter la butée gauche |
| `d` | Diminuer la butée droite |
| `D` | Augmenter la butée droite |

#### `void clearParams()`

Efface tous les paramètres de butée enregistrés en flash.

---

## CoVACIEL_propulsion

Pilote un ESC (Electronic Speed Controller) brushless via PWM. Supporte la marche avant, la marche arrière et l'arrêt. Les seuils PWM (pleine échelle avant, pleine échelle arrière, point d'arrêt) sont configurables et persistés en flash.

### Initialisation

#### `void init(int pin)`

Initialise le signal PWM sur la broche indiquée.

```cpp
CoVACIEL_propulsion propulsion;

void setup() {
    propulsion.init(9); // broche GPIO 9
}
```

---

### Pilotage

#### `void setPropulsion(int value)`

Règle la puissance de propulsion.

| Paramètre | Description |
|---|---|
| `value` | Valeur en % : `-100` = marche arrière max, `0` = arrêt, `+100` = marche avant max |

```cpp
propulsion.setPropulsion(50);   // 50% avant
propulsion.setPropulsion(0);    // arrêt
propulsion.setPropulsion(-30);  // 30% arrière
```

#### `int getPropulsion()`

Retourne la valeur de propulsion courante (entre -100 et +100).

---

### Configuration des seuils PWM

Les seuils délimitent les valeurs PWM correspondant aux états extrêmes de l'ESC.

| Méthode | Description |
|---|---|
| `void setPWMAvMax(int value)` | PWM correspondant à 100% avant |
| `void setPWMAvMin(int value)` | PWM correspondant à 0% (arrêt) |
| `void setPWMArMax(int value)` | PWM correspondant à 100% arrière |
| `int getPWMAvMax()` | Lit le seuil avant max |
| `int getPWMAvMin()` | Lit le seuil d'arrêt |
| `int getPWMArMax()` | Lit le seuil arrière max |

#### `void reglageButees()`

Mode interactif de réglage via `Serial`. Les commandes disponibles sont :

| Touche | Action |
|---|---|
| `M` | Aller au PWM MAX avant |
| `m` | Aller au PWM MIN (arrêt) |
| `D` | Augmenter le PWM MAX avant |
| `d` | Diminuer le PWM MAX avant |
| `R` | Augmenter le PWM MAX arrière |
| `r` | Diminuer le PWM MAX arrière |
| `N` | Augmenter le PWM d'arrêt |
| `n` | Diminuer le PWM d'arrêt |

#### `void clearParams()`

Efface tous les paramètres PWM enregistrés en flash.

---

## Exemple complet

```cpp
#include <CoVACIEL.h>

CoVACIEL_direction  direction;
CoVACIEL_propulsion propulsion;
CoVACIEL_CAN        canbus;

void setup() {
    direction.init(10);
    propulsion.init(9);
    canbus.init(13, 12);
}

void loop() {
    canbus.updateRx();

    // Appliquer les consignes reçues par CAN aux actionneurs
    direction.setDirectionDegre(canbus.getAngleDirection());
    propulsion.setPropulsion(canbus.getVitesse());
}
```

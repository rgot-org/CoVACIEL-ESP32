# CoVACIEL_CAN — Référence API

La classe `CoVACIEL_CAN` gère la communication sur le bus CAN entre les différentes cartes du véhicule. Elle s'appuie directement sur le pilote TWAI natif de l'ESP-IDF (`driver/twai.h`).

---

## Initialisation

### `bool init(uint8_t rx, uint8_t tx)`

Démarre le bus CAN à **250 kbit/s** en mode normal, avec des queues d'émission et de réception de 5 trames.

| Paramètre | Description |
|---|---|
| `rx` | Numéro de broche GPIO pour la réception (RX) |
| `tx` | Numéro de broche GPIO pour l'émission (TX) |

**Retourne** `true` si le pilote est correctement démarré.

```cpp
CoVACIEL_CAN canbus;

void setup() {
    if (!canbus.init(13, 12)) {
        // erreur init
    }
}
```

---

## Réception

### `void updateRx()`

À appeler dans chaque `loop()`. Vide la queue de réception et met à jour toutes les variables internes (distances, propulsion, direction, vitesse, angle).

```cpp
void loop() {
    canbus.updateRx();
}
```

> La carte arrière doit aussi appeler `updateRx()` pour détecter les requêtes `CAN_ID_ARRIERE_REQUEST` et y répondre automatiquement.

---

## Émission

### `bool updateTx(int canId = 0)`

Émet une ou toutes les trames sur le bus.

| Valeur de `canId` | Comportement |
|---|---|
| `0` (défaut) | Émet toutes les trames (arrière, avant, avant-ext, direction, propulsion) |
| `CAN_ID_AVANT` | Émet uniquement la trame distances avant |
| `CAN_ID_ARRIERE` | Émet uniquement la trame distances arrière |
| *(autre ID)* | Émet la trame correspondante |

**Retourne** `true` si toutes les émissions ont réussi.

---

## Distances

### Getters

Toutes les valeurs sont exprimées en **millimètres**.

| Méthode | Description |
|---|---|
| `int getDistanceAvant()` | Distance avant centre |
| `int getDistAvGauche45()` | Distance avant gauche à 45° |
| `int getDistAvDroite45()` | Distance avant droite à 45° |
| `int getDistAvGauche90()` | Distance avant gauche à 90° |
| `int getDistAvDroite90()` | Distance avant droite à 90° |
| `int getDistanceArriere()` | Distance arrière centre — envoie une requête `CAN_ID_ARRIERE_REQUEST` et retourne la dernière valeur connue |
| `int getDistanceArGauche()` | Distance arrière gauche |
| `int getDistanceArDroite()` | Distance arrière droite |
| `int getDistance(byte secteur)` | Distance d'un secteur quelconque (0–22), retourne `-1` si hors bornes |

#### Tableau `distance[]`

Toutes les distances sont également accessibles directement via le tableau public :

```cpp
int distance[NB_SECTEURS]; // 23 entrées, indices définis par les constantes DISTANCE_xxx
```

```cpp
int d = canbus.distance[DISTANCE_AVANT];       // équivalent à getDistanceAvant()
int d = canbus.distance[DISTANCE_AR_GAUCHE];   // équivalent à getDistanceArGauche()
```

#### Pattern requête/réponse pour la distance arrière

`getDistanceArriere()` émet une trame `CAN_ID_ARRIERE_REQUEST` sur le bus avant de retourner la valeur. La réponse de la carte arrière sera reçue et intégrée au **prochain** appel de `updateRx()`.

```cpp
void loop() {
    canbus.updateRx();                          // intègre la réponse du cycle précédent

    if (millis() - last >= 100) {
        last = millis();
        int ar = canbus.getDistanceArriere();   // envoie la requête, retourne valeur actuelle
        Serial.printf("AR: %d mm\n", ar);
    }
}
```

### Setters

| Méthode | Description |
|---|---|
| `bool setDistAv(uint16_t mm, bool send2canbus=false)` | Distance avant centre |
| `bool setDistAvGauche45(uint16_t mm, bool send2canbus=false)` | Distance avant gauche 45° |
| `bool setDistAvDroite45(uint16_t mm, bool send2canbus=false)` | Distance avant droite 45° |
| `bool setDistAvGauche90(uint16_t mm, bool send2canbus=false)` | Distance avant gauche 90° |
| `bool setDistAvDroite90(uint16_t mm, bool send2canbus=false)` | Distance avant droite 90° |
| `bool setDistAr(uint16_t mm, bool send2canbus=false)` | Distance arrière centre |
| `bool setDistArGauche(uint16_t mm, bool send2canbus=false)` | Distance arrière gauche |
| `bool setDistArDroite(uint16_t mm, bool send2canbus=false)` | Distance arrière droite |

Le paramètre `send2canbus` permet d'émettre la trame immédiatement après la mise à jour. Par défaut (`false`), la valeur est stockée localement et sera émise lors du prochain appel à `updateTx()` ou sur réception d'une requête.

```cpp
// Carte arrière : stocker sans émettre (émission sur demande uniquement)
canbus.setDistAr(radar.getDistance() * 10);

// Carte avant : émettre immédiatement
canbus.setDistAv(capteur.mesure(), true);
```

#### Setters groupés

```cpp
bool setDistance(int* distances, byte length);
```
Charge un tableau de `NB_SECTEURS` (23) distances et émet les trames avant et arrière. `length` doit être `>= 23`.

```cpp
bool setDistanceJson(String jsonDistance);
```
Charge les distances depuis un objet JSON de la forme :
```json
{"distances": [sect0, sect1, ..., sect22]}
```

---

## Direction

### Getters

| Méthode | Retourne |
|---|---|
| `int getDirection()` | `0` = gauche, `1` = droite, `2` = tout droit |
| `String getDirectionStr()` | `"Gauche"`, `"Droite"`, `"Tout droit"` |
| `int getAngleDirection()` | Angle en degrés (int16_t) |

### Setters

| Méthode | Description |
|---|---|
| `bool setDirection(byte commande)` | `gauche=0`, `droit=1`, `devant=2` — émet immédiatement |
| `bool setAngleDirection(int angle)` | Angle en degrés — émet immédiatement |

---

## Propulsion

### Getters

| Méthode | Retourne |
|---|---|
| `int getPropulsion()` | `0`=accélérer, `1`=ralentir, `2`=stop, `3`=reculer |
| `String getPropulsionStr()` | `"Accelerer"`, `"Ralentir"`, `"Stop"`, `"Arriere"` |
| `int getVitesse()` | Vitesse signée entre -100 et +100 |

### Setters

| Méthode | Description |
|---|---|
| `bool setPropulsion(byte commande)` | Commande de propulsion (enum `CommandePropulsion`) — émet immédiatement |
| `bool setVitesse(int vitesse)` | Vitesse signée (-100 à +100) — émet immédiatement |

---

## Sérialisation JSON

| Méthode | Description |
|---|---|
| `void canBus2SerialJson()` | Émet sur `Serial` un JSON de toutes les trames reçues depuis le dernier appel |
| `String parseDistancesJson()` | Retourne `{"distances":[...]}` avec les 23 secteurs si un nouveau message a été reçu, sinon `""` |
| `String parseCanFrame2json()` | Retourne un JSON brut des trames CAN reçues (`{"payload":[{"canId":..., "data":[...]}]}`) |

---

## Enums et constantes

```cpp
enum CommandeDirection { gauche, droit, devant };
enum CommandePropulsion { accelerer, ralentir, stop, reculer };
enum Can_id { PROPULSION, DIRECTION, AVANT, ARRIERE, AVANT_EXT };
```

```cpp
#define NB_SECTEURS 23

// Indices dans le tableau distance[]
#define DISTANCE_AR           0
#define DISTANCE_AR_DROITE    3
#define DISTANCE_AV_GAUCHE_90 7
#define DISTANCE_AV_GAUCHE_45 9
#define DISTANCE_AVANT        11
#define DISTANCE_AV_DROITE_45 13
#define DISTANCE_AV_DROITE_90 15
#define DISTANCE_AR_GAUCHE    20
```

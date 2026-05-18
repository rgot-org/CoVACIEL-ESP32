# CoVACIEL_CAN — Référence API

La classe `CoVACIEL_CAN` gère la communication sur le bus CAN entre les différentes cartes du véhicule. Elle s'appuie directement sur le pilote TWAI natif de l'ESP-IDF (`driver/twai.h`).

---

## Identifiants CAN

### `CAN_ID_PROPULSION` — `0x660` — DLC 6

Commande de propulsion et vitesse. Deux modes exclusifs :

| Octet | Valeur | Description |
|---|---|---|
| `data[0]` | `0xFF` | Commande : **accélérer** |
| `data[1]` | `0xFF` | Commande : **ralentir** |
| `data[2]` | `0xFF` | Commande : **stop** |
| `data[3]` | `0xFF` | Commande : **reculer** |
| `data[4–5]` | int16_t big-endian | Vitesse signée (−100 à +100) — utilisé à la place des octets 0–3 quand `_propulsion = 255` |

> Un seul des octets 0–3 vaut `0xFF` à la fois ; les autres sont à `0x00`.

---

### `CAN_ID_DIRECTION` — `0x664` — DLC 6

Commande de direction et angle :

| Octet | Valeur | Description |
|---|---|---|
| `data[0]` | `0xFF` | Commande : **gauche** |
| `data[1]` | `0xFF` | Commande : **droite** |
| `data[2]` | `0xFF` | Commande : **tout droit** |
| `data[3]` | `0x00` | (réservé) |
| `data[4–5]` | int16_t big-endian | Angle en degrés |

> Un seul des octets 0–2 vaut `0xFF` à la fois.

---

### `CAN_ID_AVANT` — `0x666` — DLC 6

Distances avant en millimètres :

| Octet | Description |
|---|---|
| `data[0–1]` | Distance avant gauche 45° (uint16_t big-endian, mm) |
| `data[2–3]` | Distance avant centre (uint16_t big-endian, mm) |
| `data[4–5]` | Distance avant droite 45° (uint16_t big-endian, mm) |

---

### `CAN_ID_AVANT_GAUCHE_CM` — `0x668` — DLC 8

Distances secteurs 3 à 10 en centimètres (1 octet par secteur, saturé à 255 cm) :

| Octet | Secteur |
|---|---|
| `data[0]` | Secteur 3 |
| `data[1]` | Secteur 4 |
| … | … |
| `data[7]` | Secteur 10 |

---

### `CAN_ID_AVANT_DROITE_CM` — `0x665` — DLC 8

Distances secteurs 11 à 18 en centimètres (1 octet par secteur, saturé à 255 cm) :

| Octet | Secteur |
|---|---|
| `data[0]` | Secteur 11 |
| `data[1]` | Secteur 12 |
| … | … |
| `data[7]` | Secteur 18 |

---

### `CAN_ID_ARRIERE` — `0x667` — DLC 6

Distances arrière en millimètres :

| Octet | Description |
|---|---|
| `data[0–1]` | Distance arrière gauche (uint16_t big-endian, mm) |
| `data[2–3]` | Distance arrière centre (uint16_t big-endian, mm) |
| `data[4–5]` | Distance arrière droite (uint16_t big-endian, mm) |

---

### `CAN_ID_ARRIERE_REQUEST` — `0x669` — DLC 0

Trame vide (aucun octet de données). Envoyée par le calculateur pour demander à la carte arrière d'émettre immédiatement sa trame `CAN_ID_ARRIERE`.

---

### `CAN_ID_LIDAR_CTRL` — `0x670` — DLC 1

Commande de démarrage ou d'arrêt du lidar :

| Octet | Valeur | Description |
|---|---|---|
| `data[0]` | `0x01` | Démarrer le lidar |
| `data[0]` | `0x00` | Arrêter le lidar |

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

## Contrôle du lidar

### `bool setLidarStart(bool start)`

Envoie une commande de démarrage ou d'arrêt du lidar sur le bus CAN (`CAN_ID_LIDAR_CTRL 0x670`). La trame contient 1 octet : `0x01` pour démarrer, `0x00` pour arrêter.

**Retourne** `true` si la trame a été émise avec succès.

### `bool getLidarStart()`

Retourne l'état de démarrage du lidar tel que reçu depuis le bus CAN. La valeur est mise à jour par `updateRx()` à chaque réception d'une trame `CAN_ID_LIDAR_CTRL`.

**Retourne** `true` si le lidar doit être démarré, `false` sinon.

#### Exemple — carte calculateur (émetteur)

```cpp
// Démarrer le lidar
canbus.setLidarStart(true);

// Arrêter le lidar
canbus.setLidarStart(false);
```

#### Exemple — carte lidar (récepteur)

```cpp
void loop() {
    canbus.updateRx();                   // met à jour _lidarStart si trame reçue

    if (canbus.getLidarStart()) {
        lidar.start();
    } else {
        lidar.stop();
    }
}
```

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

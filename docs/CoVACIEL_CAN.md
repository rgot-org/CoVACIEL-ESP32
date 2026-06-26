# CoVACIEL_CAN — Référence API

La classe `CoVACIEL_CAN` gère la communication CAN entre les cartes du véhicule via le pilote TWAI natif de l'ESP-IDF (`driver/twai.h`).

Elle offre deux modes d'utilisation :

- **Mode simple** — commandes discrètes tout-ou-rien (gauche/droite/devant, accélérer/stop/reculer) et distances nommées (avant-centre, avant-45°…). Adapté à une logique de conduite par règles.
- **Mode avancé** — angle de direction en degrés, vitesse signée (−100 à +100), accès direct au tableau de 23 secteurs LiDAR et sérialisation JSON. Adapté à un pilotage continu ou à une supervision externe.

---

## Initialisation

### `bool init(uint8_t rx, uint8_t tx)`

Démarre le bus CAN à **250 kbit/s** en mode normal.

| Paramètre | Description |
|---|---|
| `rx` | Broche GPIO RX |
| `tx` | Broche GPIO TX |

**Retourne** `true` si le pilote démarre correctement.

```cpp
CoVACIEL_CAN canbus;

void setup() {
    canbus.init(13, 12);
}
```

---

## Réception / Émission

### `void updateRx()`

À appeler dans chaque `loop()`. Vide la queue CAN et met à jour toutes les variables internes.

```cpp
void loop() {
    canbus.updateRx();
}
```

> La carte arrière doit également appeler `updateRx()` pour détecter les requêtes de distance arrière et y répondre automatiquement.

### `bool updateTx(int canId = 0)`

Émet une ou toutes les trames sur le bus.

| Valeur de `canId` | Comportement |
|---|---|
| `0` (défaut) | Émet toutes les trames |
| `CAN_ID_AVANT` | Trame distances avant uniquement |
| `CAN_ID_ARRIERE` | Trame distances arrière uniquement |
| *(autre ID)* | Trame correspondante |

**Retourne** `true` si toutes les émissions ont réussi.

---

## Mode simple

Commandes discrètes et distances nommées. Adaptées à une logique par règles (`if (distAvant < 300) → stop`).

### Direction

| Méthode | Description |
|---|---|
| `bool setDirection(byte commande)` | `gauche=0`, `droit=1`, `devant=2` — émet immédiatement |
| `int getDirection()` | `0`=gauche, `1`=droite, `2`=tout droit |
| `String getDirectionStr()` | `"Gauche"`, `"Droite"`, `"Tout droit"` |

```cpp
canbus.setDirection(devant);           // tout droit
canbus.setDirection(gauche);           // tourner à gauche

int dir = canbus.getDirection();       // 0 / 1 / 2
Serial.println(canbus.getDirectionStr());
```

### Propulsion

| Méthode | Description |
|---|---|
| `bool setPropulsion(byte commande)` | `accelerer=0`, `ralentir=1`, `stop=2`, `reculer=3` — émet immédiatement |
| `int getPropulsion()` | `0`=accélérer, `1`=ralentir, `2`=stop, `3`=reculer |
| `String getPropulsionStr()` | `"Accelerer"`, `"Ralentir"`, `"Stop"`, `"Arriere"` |

```cpp
canbus.setPropulsion(accelerer);
canbus.setPropulsion(stop);

Serial.println(canbus.getPropulsionStr());
```

### Distances — getters

Toutes les valeurs sont en **millimètres**. Utilisés par le calculateur pour lire les distances reçues depuis les cartes capteurs.

| Méthode | Description |
|---|---|
| `int getDistanceAvant()` | Avant centre |
| `int getDistAvGauche45()` | Avant gauche 45° |
| `int getDistAvDroite45()` | Avant droite 45° |
| `int getDistAvGauche90()` | Avant gauche 90° |
| `int getDistAvDroite90()` | Avant droite 90° |
| `int getDistanceArriere()` | Arrière centre — envoie une requête et retourne la dernière valeur connue |
| `int getDistanceArGauche()` | Arrière gauche |
| `int getDistanceArDroite()` | Arrière droite |

```cpp
int dAv  = canbus.getDistanceAvant();
int dG45 = canbus.getDistAvGauche45();
int dAr  = canbus.getDistanceArriere(); // requête + dernière valeur
```

> **Pattern requête/réponse (distance arrière)** : `getDistanceArriere()` émet une trame `CAN_ID_ARRIERE_REQUEST`. La réponse de la carte arrière sera intégrée au prochain appel de `updateRx()`.
>
> ```cpp
> void loop() {
>     canbus.updateRx();                       // intègre la réponse du cycle précédent
>     int ar = canbus.getDistanceArriere();    // envoie la requête, retourne valeur actuelle
> }
> ```

### Distances — setters

Utilisés par les **cartes capteurs** pour publier leurs mesures sur le bus. Le paramètre `send2canbus` permet d'émettre immédiatement (`true`) ou d'attendre le prochain `updateTx()` (`false`, défaut).

| Méthode | Description |
|---|---|
| `bool setDistAv(uint16_t mm, bool send2canbus=false)` | Avant centre |
| `bool setDistAvGauche45(uint16_t mm, bool send2canbus=false)` | Avant gauche 45° |
| `bool setDistAvDroite45(uint16_t mm, bool send2canbus=false)` | Avant droite 45° |
| `bool setDistAvGauche90(uint16_t mm, bool send2canbus=false)` | Avant gauche 90° |
| `bool setDistAvDroite90(uint16_t mm, bool send2canbus=false)` | Avant droite 90° |
| `bool setDistAr(uint16_t mm, bool send2canbus=false)` | Arrière centre |
| `bool setDistArGauche(uint16_t mm, bool send2canbus=false)` | Arrière gauche |
| `bool setDistArDroite(uint16_t mm, bool send2canbus=false)` | Arrière droite |

```cpp
// Carte avant : émettre immédiatement après chaque mesure
canbus.setDistAv(lidar.distance[11], true);
canbus.setDistAvGauche45(lidar.distance[8], true);
canbus.setDistAvDroite45(lidar.distance[14], true);

// Carte arrière : stocker sans émettre (émission sur requête uniquement)
canbus.setDistAr(radar.getDistance() * 10);
```

---

## Mode avancé

Angle continu, vitesse signée et accès au tableau complet des secteurs. Adaptés à un pilotage à base de régulateurs ou à une supervision JSON.

### Direction — angle

| Méthode | Description |
|---|---|
| `bool setAngleDirection(int angle)` | Angle en degrés (signé) — émet immédiatement |
| `int getAngleDirection()` | Dernière consigne d'angle reçue |

```cpp
canbus.setAngleDirection(0);    // tout droit
canbus.setAngleDirection(-30);  // virage gauche 30°
canbus.setAngleDirection(15);   // virage droite 15°

int a = canbus.getAngleDirection();
```

### Propulsion — vitesse signée

| Méthode | Description |
|---|---|
| `bool setVitesse(int vitesse)` | Vitesse signée de −100 à +100 — émet immédiatement |
| `int getVitesse()` | Dernière consigne de vitesse reçue |

```cpp
canbus.setVitesse(50);    // 50 % en avant
canbus.setVitesse(-20);   // 20 % en arrière
canbus.setVitesse(0);     // stop
```

### Distances — tableau de secteurs

Accès direct au tableau public de 23 secteurs (indices 0–22, pas = 16°) :

```cpp
int distance[NB_SECTEURS]; // 23 entrées
```

```cpp
int d = canbus.distance[DISTANCE_AVANT];     // équivalent à getDistanceAvant()
int d = canbus.distance[DISTANCE_AR_GAUCHE]; // équivalent à getDistanceArGauche()
```

| Méthode | Description |
|---|---|
| `int getDistance(byte secteur)` | Distance du secteur 0–22 en mm, retourne `-1` si hors bornes |

```cpp
for (int i = 0; i < NB_SECTEURS; i++) {
    Serial.printf("Secteur %d : %d mm\n", i, canbus.getDistance(i));
}
```

#### Chargement groupé

```cpp
bool setDistance(int* distances, byte length);
```
Charge un tableau de 23 distances et émet les trames avant et arrière. `length` doit être `>= 23`.

```cpp
bool setDistanceJson(String jsonDistance);
```
Charge les distances depuis un JSON `{"distances": [sect0, sect1, ..., sect22]}`.

### Contrôle du lidar

| Méthode | Description |
|---|---|
| `bool setLidarStart(bool start)` | Envoie `CAN_ID_LIDAR_CTRL` : `true`=démarrer, `false`=arrêter |
| `bool getLidarStart()` | État reçu via `updateRx()` |

```cpp
// Calculateur
canbus.setLidarStart(true);

// Carte lidar
void loop() {
    canbus.updateRx();
    if (canbus.getLidarStart()) lidar.start();
    else                        lidar.stop();
}
```

### Sérialisation JSON

| Méthode | Description |
|---|---|
| `void canBus2SerialJson()` | Émet sur `Serial` un JSON de toutes les trames reçues |
| `String parseDistancesJson()` | Retourne `{"distances":[...]}` si nouveau message, sinon `""` |
| `String parseCanFrame2json()` | JSON brut des trames CAN reçues |

---

## Référence

### Identifiants CAN

| Constante | ID | DLC | Description |
|---|---|---|---|
| `CAN_ID_PROPULSION` | `0x660` | 6 | Commande propulsion / vitesse signée |
| `CAN_ID_DIRECTION` | `0x664` | 6 | Commande direction / angle |
| `CAN_ID_AVANT_DROITE_CM` | `0x665` | 8 | Secteurs 11–18 en cm |
| `CAN_ID_AVANT` | `0x666` | 6 | Distances avant en mm |
| `CAN_ID_ARRIERE` | `0x667` | 6 | Distances arrière en mm |
| `CAN_ID_AVANT_GAUCHE_CM` | `0x668` | 8 | Secteurs 3–10 en cm |
| `CAN_ID_ARRIERE_REQUEST` | `0x669` | 0 | Requête distance arrière (trame vide) |
| `CAN_ID_LIDAR_CTRL` | `0x670` | 1 | Démarrage/arrêt lidar |

### Enums et constantes

```cpp
enum CommandeDirection  { gauche, droit, devant };
enum CommandePropulsion { accelerer, ralentir, stop, reculer };
enum Can_id             { PROPULSION, DIRECTION, AVANT, ARRIERE, AVANT_EXT };
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

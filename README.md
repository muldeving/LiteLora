# LiteLora

Bibliothèque Arduino légère pour piloter un module **SX1278 LoRa** via SPI, sans dépendances externes (uniquement `SPI.h`). Conçue pour ESP32-C3 à 40 MHz mais compatible avec tout ESP32.

## Fonctionnalités

- Communication SPI directe avec le SX1278 (pas de bibliothèque tierce)
- Configuration complète : fréquence, spreading factor, bande passante, coding rate, puissance TX, sync word, préambule
- Émission et réception par polling (pas d'interruption matérielle requise)
- Gestion du partage du bus SPI avec une carte SD (`releaseBus` / `acquireBus`)
- Mode deep sleep ESP32 avec réveil par GPIO

## Structure du projet

```
LiteLora/
├── src/
│   ├── LiteLora.h        # En-tête : classe, registres, constantes
│   └── LiteLora.cpp      # Implémentation complète
├── examples/
│   └── basic/
│       └── basic.ino      # Exemple : terminal série + carte SD
├── library.properties     # Métadonnées Arduino
├── LICENSE
└── README.md
```

## Installation

### Arduino IDE

Copier le dossier `LiteLora` dans votre répertoire de bibliothèques Arduino :

```
~/Arduino/libraries/LiteLora/
```

### PlatformIO

Ajouter le dossier comme dépendance locale dans `platformio.ini` :

```ini
lib_deps = /chemin/vers/LiteLora
```

## Utilisation rapide

```cpp
#include <LiteLora.h>

LiteLora lora;

void setup() {
  Serial.begin(115200);

  LiteLoraConfig cfg = LiteLora::defaultConfig();
  // cfg.frequency = 868000000;  // Personnaliser si besoin

  if (!lora.begin(cfg)) {
    Serial.println("Module LoRa non détecté!");
    while (1) delay(1000);
  }

  lora.receive();
}

void loop() {
  if (lora.available()) {
    uint8_t buffer[LORA_MAX_PKT_LENGTH];
    uint8_t len = lora.readPacket(buffer, sizeof(buffer));

    if (len > 0) {
      Serial.printf("Reçu %d octets, RSSI: %d dBm\n", len, lora.packetRssi());
    }

    lora.receive();
  }
}
```

## Référence API

### Configuration

#### `LiteLoraConfig`

Structure contenant tous les paramètres :

| Champ             | Type       | Défaut       | Description                        |
|-------------------|------------|--------------|------------------------------------|
| `sckPin`          | `uint8_t`  | `4`          | Broche SPI Clock                   |
| `misoPin`         | `uint8_t`  | `5`          | Broche SPI MISO                    |
| `mosiPin`         | `uint8_t`  | `6`          | Broche SPI MOSI                    |
| `csPin`           | `uint8_t`  | `21`         | Broche Chip Select LoRa            |
| `dio0Pin`         | `uint8_t`  | `8`          | Broche DIO0 (statut RX/TX)         |
| `frequency`       | `uint32_t` | `433000000`  | Fréquence radio (Hz)               |
| `bandwidth`       | `uint32_t` | `250000`     | Bande passante (Hz)                |
| `spreadingFactor` | `uint8_t`  | `7`          | Spreading Factor (6-12)            |
| `codingRate`      | `uint8_t`  | `5`          | Coding Rate : 5 = 4/5, 8 = 4/8    |
| `preambleLength`  | `uint16_t` | `8`          | Longueur du préambule              |
| `txPower`         | `int8_t`   | `17`         | Puissance TX (2-17 dBm)            |
| `syncWord`        | `uint8_t`  | `0x12`       | Mot de synchronisation              |
| `spiFrequency`    | `uint32_t` | `8000000`    | Fréquence SPI (Hz)                 |

#### `LiteLora::defaultConfig()`

Retourne un `LiteLoraConfig` pré-rempli avec les valeurs par défaut (broches ESP32-C3 + paramètres haut débit).

### Initialisation

#### `bool begin(const LiteLoraConfig& config)`

Initialise le bus SPI, vérifie la présence du SX1278 (version `0x12`), applique toute la configuration radio. Retourne `false` si le module ne répond pas.

### Contrôle de mode

| Méthode                             | Description                                        |
|--------------------------------------|----------------------------------------------------|
| `sleep()`                            | Mode sleep basse consommation                      |
| `standby()`                          | Mode standby                                       |
| `enterDeepSleep(uint8_t wakeupPin)`  | Deep sleep ESP32, réveil par GPIO `wakeupPin`      |

### Configuration radio (modifiable à chaud)

Chaque méthode met à jour le registre correspondant immédiatement :

| Méthode                               | Plage                | Description                 |
|----------------------------------------|----------------------|-----------------------------|
| `setFrequency(uint32_t hz)`           | 137-525 MHz          | Fréquence radio             |
| `setTxPower(int8_t dBm)`             | 2-17 dBm             | Puissance d'émission        |
| `setSpreadingFactor(uint8_t sf)`      | 6-12                 | Spreading Factor            |
| `setBandwidth(uint32_t hz)`           | 7800-500000 Hz       | Bande passante              |
| `setCodingRate(uint8_t cr)`           | 5-8 (= 4/5 à 4/8)   | Taux de correction          |
| `setPreambleLength(uint16_t len)`     | 6-65535              | Longueur du préambule       |
| `setSyncWord(uint8_t sw)`            | 0x00-0xFF            | Mot de synchronisation      |

### Émission

#### `void transmit(const uint8_t* data, uint8_t length)`

Charge les données dans la FIFO du SX1278 et démarre la transmission. Taille maximale : 255 octets.

#### `bool waitTransmitDone(uint32_t timeoutUs = 5000000)`

Attend la fin de la transmission par polling. Retourne `true` si l'émission s'est terminée, `false` en cas de timeout.

### Réception

#### `void receive()`

Active le mode réception continue. Doit être appelé après chaque lecture de paquet ou après une émission.

#### `bool available()`

Vérifie si un paquet a été reçu (polling du registre IRQ). Retourne `true` si un paquet est prêt à être lu.

#### `uint8_t readPacket(uint8_t* buffer, uint8_t maxLength)`

Lit le paquet reçu dans `buffer`. Retourne le nombre d'octets lus, ou `0` en cas d'erreur CRC.

### Informations paquet

| Méthode          | Retour    | Description                          |
|------------------|-----------|--------------------------------------|
| `packetRssi()`   | `int16_t` | RSSI du dernier paquet reçu (dBm)   |
| `packetSnr()`    | `float`   | SNR du dernier paquet reçu (dB)     |
| `currentRssi()`  | `int16_t` | RSSI instantané du canal (dBm)      |

### État

#### `void printStatus()`

Affiche sur `Serial` la configuration complète du module (fréquence, SF, BW, CR, mode, RSSI).

#### `uint32_t getFrequency() const`

Retourne la fréquence actuellement configurée en Hz.

### Gestion du bus SPI partagé

Lorsque le bus SPI est partagé avec un autre périphérique (ex. carte SD), ces méthodes gèrent la commutation :

#### `void releaseBus()`

Libère le bus SPI en garantissant que le CS du LoRa est HIGH. Appeler **avant** d'accéder à la carte SD.

#### `void acquireBus()`

Reprend le contrôle du bus en restaurant la fréquence SPI configurée pour le LoRa. Appeler **après** les opérations SD.

**Exemple :**

```cpp
lora.releaseBus();
// ... opérations SD ...
lora.acquireBus();
```

## Brochage par défaut (ESP32-C3)

```
ESP32-C3          SX1278          Carte SD
─────────         ──────          ────────
GPIO 4  (SCK)  ── SCK          ── SCK
GPIO 5  (MISO) ── MISO         ── MISO
GPIO 6  (MOSI) ── MOSI         ── MOSI
GPIO 21 (CS)   ── NSS
GPIO 7  (CS)                   ── CS
GPIO 8  (DIO0) ── DIO0
```

## Licence

MIT - voir [LICENSE](LICENSE).

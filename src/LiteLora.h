/*
 * LiteLora - Bibliothèque légère pour module SX1278 LoRa
 *
 * Pilote SX1278 sans dépendances externes (uniquement SPI.h).
 * Conçu pour ESP32-C3 à 40 MHz mais compatible avec tout ESP32.
 *
 * Fonctionnalités :
 *  - Communication SPI directe avec le SX1278
 *  - Configuration complète (fréquence, SF, BW, CR, puissance, etc.)
 *  - Transmission et réception par polling (pas d'interruption matérielle)
 *  - Gestion du partage du bus SPI avec une carte SD
 *
 * Licence : MIT
 */

#ifndef LITELORA_H
#define LITELORA_H

#include <Arduino.h>
#include <SPI.h>

// ========== Registres SX1278 ==========
#define LORA_REG_FIFO                 0x00
#define LORA_REG_OP_MODE              0x01
#define LORA_REG_FRF_MSB              0x06
#define LORA_REG_FRF_MID              0x07
#define LORA_REG_FRF_LSB              0x08
#define LORA_REG_PA_CONFIG            0x09
#define LORA_REG_LNA                  0x0C
#define LORA_REG_FIFO_ADDR_PTR        0x0D
#define LORA_REG_FIFO_TX_BASE_ADDR    0x0E
#define LORA_REG_FIFO_RX_BASE_ADDR    0x0F
#define LORA_REG_FIFO_RX_CURRENT_ADDR 0x10
#define LORA_REG_IRQ_FLAGS_MASK       0x11
#define LORA_REG_IRQ_FLAGS            0x12
#define LORA_REG_RX_NB_BYTES          0x13
#define LORA_REG_PKT_SNR_VALUE        0x19
#define LORA_REG_PKT_RSSI_VALUE       0x1A
#define LORA_REG_RSSI_VALUE           0x1B
#define LORA_REG_MODEM_CONFIG_1       0x1D
#define LORA_REG_MODEM_CONFIG_2       0x1E
#define LORA_REG_PREAMBLE_MSB         0x20
#define LORA_REG_PREAMBLE_LSB         0x21
#define LORA_REG_PAYLOAD_LENGTH       0x22
#define LORA_REG_MODEM_CONFIG_3       0x26
#define LORA_REG_FREQ_ERROR_MSB       0x28
#define LORA_REG_FREQ_ERROR_MID       0x29
#define LORA_REG_FREQ_ERROR_LSB       0x2A
#define LORA_REG_RSSI_WIDEBAND        0x2C
#define LORA_REG_DETECTION_OPTIMIZE   0x31
#define LORA_REG_INVERTIQ             0x33
#define LORA_REG_DETECTION_THRESHOLD  0x37
#define LORA_REG_SYNC_WORD            0x39
#define LORA_REG_INVERTIQ2            0x3B
#define LORA_REG_DIO_MAPPING_1        0x40
#define LORA_REG_VERSION              0x42
#define LORA_REG_PA_DAC               0x4D

// ========== Modes d'opération ==========
#define LORA_MODE_LONG_RANGE         0x80
#define LORA_MODE_SLEEP              0x00
#define LORA_MODE_STDBY              0x01
#define LORA_MODE_TX                 0x03
#define LORA_MODE_RX_CONTINUOUS      0x05
#define LORA_MODE_RX_SINGLE          0x06

// ========== IRQ Flags ==========
#define LORA_IRQ_TX_DONE             0x08
#define LORA_IRQ_PAYLOAD_CRC_ERROR   0x20
#define LORA_IRQ_RX_DONE             0x40

// ========== Taille maximale d'un paquet ==========
#define LORA_MAX_PKT_LENGTH          255

// ========== Configuration par défaut ==========
#define LORA_DEFAULT_FREQUENCY       433000000
#define LORA_DEFAULT_BANDWIDTH       250000
#define LORA_DEFAULT_SPREADING       7
#define LORA_DEFAULT_CODING_RATE     5
#define LORA_DEFAULT_PREAMBLE        8
#define LORA_DEFAULT_TX_POWER        17
#define LORA_DEFAULT_SYNC_WORD       0x12
#define LORA_DEFAULT_SPI_FREQUENCY   8000000

/**
 * Structure de configuration pour LiteLora.
 *
 * Permet de personnaliser tous les paramètres radio et brochage.
 * Utiliser LiteLora::defaultConfig() pour obtenir une configuration
 * pré-remplie avec les valeurs par défaut.
 */
struct LiteLoraConfig {
  uint8_t  sckPin;
  uint8_t  misoPin;
  uint8_t  mosiPin;
  uint8_t  csPin;
  uint8_t  dio0Pin;
  uint32_t frequency;
  uint32_t bandwidth;
  uint8_t  spreadingFactor;
  uint8_t  codingRate;
  uint16_t preambleLength;
  int8_t   txPower;
  uint8_t  syncWord;
  uint32_t spiFrequency;
};

/**
 * Classe principale de la bibliothèque LiteLora.
 *
 * Gère un module SX1278 via SPI, incluant l'initialisation,
 * la configuration radio, l'émission et la réception.
 * Gère également le partage du bus SPI avec un
 * périphérique SD (chip-select séparé).
 */
class LiteLora {
public:
  LiteLora();

  /**
   * Retourne une configuration par défaut.
   * Les broches doivent être renseignées par l'utilisateur.
   */
  static LiteLoraConfig defaultConfig();

  /**
   * Initialise le module SX1278 avec la configuration donnée.
   * Configure SPI, vérifie la présence du module et applique
   * tous les paramètres radio.
   *
   * @param config  Configuration complète (broches + radio).
   * @return true si le module répond correctement (version 0x12).
   */
  bool begin(const LiteLoraConfig& config);

  // ---- Contrôle de mode ----

  /** Passe le module en mode sleep (basse consommation). */
  void sleep();

  /** Passe le module en mode standby. */
  void standby();

  // ---- Configuration radio (modifiable à chaud) ----

  void setFrequency(uint32_t frequency);
  void setTxPower(int8_t power);
  void setSpreadingFactor(uint8_t sf);
  void setBandwidth(uint32_t bw);
  void setCodingRate(uint8_t cr);
  void setPreambleLength(uint16_t length);
  void setSyncWord(uint8_t sw);

  // ---- Émission ----

  /**
   * Charge les données dans la FIFO et démarre la transmission.
   *
   * @param data    Pointeur vers les données à envoyer.
   * @param length  Nombre d'octets (max LORA_MAX_PKT_LENGTH).
   */
  void transmit(const uint8_t* data, uint8_t length);

  /**
   * Attend la fin de la transmission en cours (polling).
   *
   * @param timeoutUs  Timeout en microsecondes (défaut 5 s).
   * @return true si la transmission s'est terminée avant le timeout.
   */
  bool waitTransmitDone(uint32_t timeoutUs = 5000000);

  // ---- Réception ----

  /** Active le mode réception continue. */
  void receive();

  /**
   * Indique si un paquet a été reçu (polling IRQ).
   * @return true si un paquet est en attente de lecture.
   */
  bool available();

  /**
   * Lit le paquet reçu dans le buffer fourni.
   *
   * @param buffer     Buffer de destination.
   * @param maxLength  Taille du buffer (max LORA_MAX_PKT_LENGTH).
   * @return Nombre d'octets lus, 0 en cas d'erreur CRC.
   */
  uint8_t readPacket(uint8_t* buffer, uint8_t maxLength);

  // ---- Informations paquet ----

  /** RSSI du dernier paquet reçu (dBm). */
  int16_t packetRssi();

  /** SNR du dernier paquet reçu (dB). */
  float packetSnr();

  /** RSSI instantané du canal (dBm). */
  int16_t currentRssi();

  // ---- État ----

  /** Affiche l'état complet du module sur Serial. */
  void printStatus();

  /** Retourne la fréquence actuellement configurée (Hz). */
  uint32_t getFrequency() const;

  // ---- Gestion bus SPI partagé (SD / LoRa) ----

  /**
   * Libère le bus SPI pour un autre périphérique (ex. SD).
   * Garantit que CS LoRa est HIGH.
   */
  void releaseBus();

  /**
   * Reprend le contrôle du bus SPI après utilisation par
   * un autre périphérique. Restaure la fréquence SPI.
   */
  void acquireBus();

private:
  uint8_t  _csPin;
  uint8_t  _dio0Pin;
  uint32_t _frequency;
  uint32_t _spiFrequency;
  uint32_t _bandwidth;
  uint8_t  _spreadingFactor;
  uint8_t  _codingRate;
  uint16_t _preambleLength;
  int8_t   _txPower;
  uint8_t  _syncWord;

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  void softReset();
};

#endif // LITELORA_H

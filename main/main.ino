/*
 * ESP32-C3 SX1278 LoRa - Optimisé pour débit élevé
 * Fréquence CPU: 40MHz
 * Sans bibliothèques (sauf SPI.h)
 * 
 * Commandes série:
 * - SEND:message      -> Envoyer un message
 * - FREQ:433000000    -> Changer fréquence (Hz)
 * - SLEEP             -> Mode deep sleep
 * - RSSI              -> Afficher RSSI
 * - STATUS            -> Afficher état module
 */

#include <SPI.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// Configuration des broches SPI pour ESP32-C3
#define LORA_SCK    4
#define LORA_MISO   5
#define LORA_MOSI   6
#define LORA_CS     21
#define LORA_DIO0   8   // Interruption RX/TX Done

// Registres SX1278
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2A
#define REG_RSSI_WIDEBAND        0x2C
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3B
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// Modes d'opération
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// IRQ Flags
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

// Configuration rapide pour débit élevé
#define LORA_BANDWIDTH      250000   // 250kHz
#define LORA_SPREADING      7        // SF7 (plus rapide)
#define LORA_CODING_RATE    5        // 4/5
#define LORA_PREAMBLE       8        // Préambule court
#define LORA_TX_POWER       17       // 17dBm
#define LORA_SYNC_WORD      0x12     // Réseau privé

// Variables globales
uint32_t currentFrequency = 433000000; // Fréquence par défaut 433MHz

// Buffers
#define MAX_PKT_LENGTH 255
uint8_t rxBuffer[MAX_PKT_LENGTH];
uint8_t rxLength = 0;

// Prototypes
uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address, uint8_t value);
void setFrequency(uint32_t frequency);
void setTxPower(int8_t power);
void setSpreadingFactor(uint8_t sf);
void setBandwidth(uint32_t bw);
void setCodingRate(uint8_t cr);
void setPreambleLength(uint16_t length);
void setSyncWord(uint8_t sw);
void reset();
bool begin();
void sleep();
void standby();
void transmit(uint8_t* data, uint8_t length);
void receive();
void readPacket();
int16_t packetRssi();
float packetSnr();
void printStatus();
void enterDeepSleep();

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== ESP32-C3 SX1278 LoRa - Démarrage ===");
  Serial.println("Fréquence CPU: 40MHz");
  
  // Configuration SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  SPI.setFrequency(8000000); // 8MHz SPI (max stable pour 40MHz CPU)
  
  // Configuration des broches
  pinMode(LORA_CS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  
  digitalWrite(LORA_CS, HIGH);
  
  // Réinitialisation du module
  if (!begin()) {
    Serial.println("ERREUR: Module LoRa non détecté!");
    while (1) delay(1000);
  }
  
  Serial.println("Module LoRa initialisé avec succès!");
  Serial.printf("Fréquence: %lu Hz\n", currentFrequency);
  Serial.println("\nCommandes disponibles:");
  Serial.println("  SEND:votre_message  - Envoyer un message");
  Serial.println("  FREQ:433000000      - Changer fréquence (Hz)");
  Serial.println("  SLEEP               - Mode deep sleep");
  Serial.println("  RSSI                - Afficher RSSI");
  Serial.println("  STATUS              - Afficher état");
  Serial.println("\nEn attente de messages...\n");

  // Passer en mode réception
  receive();
}

void loop() {
  // Vérifier commandes série
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("SEND:")) {
      String msg = cmd.substring(5);
      Serial.printf("Envoi: %s\n", msg.c_str());
      
      uint32_t startTime = micros();
      transmit((uint8_t*)msg.c_str(), msg.length());

      // Attendre fin de transmission (polling registre IRQ)
      while (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)) {
        if (micros() - startTime > 5000000) break;
        delay(1);
      }

      if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        uint32_t duration = micros() - startTime;
        writeRegister(REG_IRQ_FLAGS, 0xFF);
        Serial.printf("✓ Envoyé en %lu µs\n", duration);
      } else {
        Serial.println("✗ Timeout transmission");
      }

      // Retour en réception
      receive();
    }
    else if (cmd.startsWith("FREQ:")) {
      uint32_t freq = cmd.substring(5).toInt();
      if (freq >= 137000000 && freq <= 525000000) {
        currentFrequency = freq;
        setFrequency(freq);
        Serial.printf("Fréquence changée: %lu Hz\n", freq);
        receive();
      } else {
        Serial.println("Fréquence invalide (137-525 MHz)");
      }
    }
    else if (cmd == "SLEEP") {
      Serial.println("Entrée en deep sleep...");
      Serial.flush();
      enterDeepSleep();
    }
    else if (cmd == "RSSI") {
      int16_t rssi = readRegister(REG_RSSI_VALUE) - 164;
      Serial.printf("RSSI actuel: %d dBm\n", rssi);
    }
    else if (cmd == "STATUS") {
      printStatus();
    }
    else {
      Serial.println("Commande inconnue");
    }
  }
  
  // Vérifier réception de paquets (polling registre IRQ)
  if (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) {
    uint32_t startTime = micros();
    readPacket();
    uint32_t readTime = micros() - startTime;

    if (rxLength > 0) {
      Serial.println("\n--- Paquet reçu ---");
      Serial.printf("Longueur: %d octets\n", rxLength);
      Serial.printf("RSSI: %d dBm\n", packetRssi());
      Serial.printf("SNR: %.2f dB\n", packetSnr());
      Serial.printf("Temps lecture: %lu µs\n", readTime);
      Serial.print("Données: ");

      for (int i = 0; i < rxLength; i++) {
        if (rxBuffer[i] >= 32 && rxBuffer[i] <= 126) {
          Serial.print((char)rxBuffer[i]);
        } else {
          Serial.printf("[0x%02X]", rxBuffer[i]);
        }
      }
      Serial.println("\n-------------------\n");
    }

    // Retour en réception immédiatement
    receive();
  }
}

// ========== Fonctions SPI ==========
uint8_t readRegister(uint8_t address) {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(address & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(LORA_CS, HIGH);
  return value;
}

void writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(LORA_CS, HIGH);
}

// ========== Configuration module ==========
bool begin() {
  reset();
  
  // Vérifier version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    Serial.printf("Version SX1278 incorrecte: 0x%02X (attendu 0x12)\n", version);
    return false;
  }
  
  // Mode Sleep pour configuration
  sleep();
  
  // Mode LoRa
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  
  // Configuration pour débit élevé
  setFrequency(currentFrequency);
  setTxPower(LORA_TX_POWER);
  setSpreadingFactor(LORA_SPREADING);
  setBandwidth(LORA_BANDWIDTH);
  setCodingRate(LORA_CODING_RATE);
  setPreambleLength(LORA_PREAMBLE);
  setSyncWord(LORA_SYNC_WORD);
  
  // CRC activé
  writeRegister(REG_MODEM_CONFIG_2, 
    (readRegister(REG_MODEM_CONFIG_2) & 0xFB) | 0x04);
  
  // AGC auto activé
  writeRegister(REG_MODEM_CONFIG_3, 0x04);
  
  // LNA gain max
  writeRegister(REG_LNA, 0x23);
  
  // FIFO base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
  
  // DIO0 -> RxDone/TxDone
  writeRegister(REG_DIO_MAPPING_1, 0x00);
  
  standby();
  
  return true;
}

void reset() {
  // Pas de reset matériel (RST non connecté)
  // Reset logiciel : forcer le passage par le mode sleep
  writeRegister(REG_OP_MODE, 0x00);  // FSK sleep
  delay(10);
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);  // LoRa sleep
  delay(10);
}

void sleep() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void standby() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void setFrequency(uint32_t frequency) {
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void setTxPower(int8_t power) {
  if (power > 17) power = 17;
  if (power < 2) power = 2;
  
  // PA_BOOST
  writeRegister(REG_PA_CONFIG, 0x80 | (power - 2));
  
  // High power settings
  if (power > 17) {
    writeRegister(REG_PA_DAC, 0x87);
  } else {
    writeRegister(REG_PA_DAC, 0x84);
  }
}

void setSpreadingFactor(uint8_t sf) {
  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;
  
  // Optimisation détection pour SF6
  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xC5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xC3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
  }
  
  uint8_t config2 = readRegister(REG_MODEM_CONFIG_2);
  config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
  writeRegister(REG_MODEM_CONFIG_2, config2);
}

void setBandwidth(uint32_t bw) {
  uint8_t bwVal;
  
  if (bw <= 7800) bwVal = 0;
  else if (bw <= 10400) bwVal = 1;
  else if (bw <= 15600) bwVal = 2;
  else if (bw <= 20800) bwVal = 3;
  else if (bw <= 31250) bwVal = 4;
  else if (bw <= 41700) bwVal = 5;
  else if (bw <= 62500) bwVal = 6;
  else if (bw <= 125000) bwVal = 7;
  else if (bw <= 250000) bwVal = 8;
  else bwVal = 9; // 500kHz
  
  uint8_t config1 = readRegister(REG_MODEM_CONFIG_1);
  config1 = (config1 & 0x0F) | (bwVal << 4);
  writeRegister(REG_MODEM_CONFIG_1, config1);
}

void setCodingRate(uint8_t cr) {
  if (cr < 5) cr = 5;
  if (cr > 8) cr = 8;
  
  uint8_t crVal = cr - 4;
  uint8_t config1 = readRegister(REG_MODEM_CONFIG_1);
  config1 = (config1 & 0xF1) | (crVal << 1);
  writeRegister(REG_MODEM_CONFIG_1, config1);
}

void setPreambleLength(uint16_t length) {
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length & 0xFF));
}

void setSyncWord(uint8_t sw) {
  writeRegister(REG_SYNC_WORD, sw);
}

// ========== Transmission/Réception ==========
void transmit(uint8_t* data, uint8_t length) {
  if (length > MAX_PKT_LENGTH) length = MAX_PKT_LENGTH;
  
  standby();
  
  // Effacer IRQ flags
  writeRegister(REG_IRQ_FLAGS, 0xFF);
  
  // FIFO pointer au début
  writeRegister(REG_FIFO_ADDR_PTR, 0x00);
  
  // Écrire données dans FIFO
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(REG_FIFO | 0x80);
  for (uint8_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(LORA_CS, HIGH);
  
  // Définir longueur payload
  writeRegister(REG_PAYLOAD_LENGTH, length);
  
  // Mode TX
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void receive() {
  standby();
  
  // Effacer IRQ flags
  writeRegister(REG_IRQ_FLAGS, 0xFF);
  
  // FIFO pointer au début
  writeRegister(REG_FIFO_ADDR_PTR, 0x00);
  
  // Mode RX continu
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void readPacket() {
  // Vérifier CRC
  uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
  if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
    Serial.println("Erreur CRC!");
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    rxLength = 0;
    return;
  }
  
  // Lire longueur
  rxLength = readRegister(REG_RX_NB_BYTES);
  
  if (rxLength > MAX_PKT_LENGTH) {
    rxLength = MAX_PKT_LENGTH;
  }
  
  // Positionner FIFO pointer
  uint8_t fifoAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
  writeRegister(REG_FIFO_ADDR_PTR, fifoAddr);
  
  // Lire données
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(REG_FIFO & 0x7F);
  for (uint8_t i = 0; i < rxLength; i++) {
    rxBuffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(LORA_CS, HIGH);
  
  // Effacer IRQ
  writeRegister(REG_IRQ_FLAGS, 0xFF);
}

int16_t packetRssi() {
  return (readRegister(REG_PKT_RSSI_VALUE) - 164);
}

float packetSnr() {
  int8_t snr = readRegister(REG_PKT_SNR_VALUE);
  return snr * 0.25;
}

void printStatus() {
  Serial.println("\n=== État du module LoRa ===");
  Serial.printf("Fréquence: %lu Hz\n", currentFrequency);
  Serial.printf("SF: %d\n", LORA_SPREADING);
  Serial.printf("BW: %lu Hz\n", LORA_BANDWIDTH);
  Serial.printf("CR: 4/%d\n", LORA_CODING_RATE);
  Serial.printf("Préambule: %d\n", LORA_PREAMBLE);
  Serial.printf("Sync Word: 0x%02X\n", LORA_SYNC_WORD);
  
  uint8_t mode = readRegister(REG_OP_MODE);
  Serial.print("Mode: ");
  switch (mode & 0x07) {
    case MODE_SLEEP: Serial.println("Sleep"); break;
    case MODE_STDBY: Serial.println("Standby"); break;
    case MODE_TX: Serial.println("TX"); break;
    case MODE_RX_CONTINUOUS: Serial.println("RX Continu"); break;
    default: Serial.println("Autre"); break;
  }
  
  int16_t rssi = readRegister(REG_RSSI_VALUE) - 164;
  Serial.printf("RSSI: %d dBm\n", rssi);
  Serial.println("===========================\n");
}

void enterDeepSleep() {
  // Configurer DIO0 comme source de réveil
  esp_deep_sleep_enable_gpio_wakeup(1 << 1, ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_set_direction((gpio_num_t)1, GPIO_MODE_INPUT);  // <<<=== Add this line
  
  // Mettre LoRa en mode RX pour détecter paquets
  receive();
  delay(100); // Laisser le module se stabiliser
  
  // Deep sleep
  esp_deep_sleep_start();
}

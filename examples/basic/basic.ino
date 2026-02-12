/*
 * LiteLora - Exemple de base
 *
 * Terminal série interactif avec réception continue et
 * enregistrement optionnel sur carte SD.
 *
 * Commandes série (115200 bauds) :
 *   SEND:message      Envoyer un message LoRa
 *   FREQ:433000000    Changer la fréquence (Hz)
 *   SLEEP             Entrer en deep sleep
 *   RSSI              Afficher le RSSI instantané
 *   STATUS            Afficher l'état du module
 */

#include <LiteLora.h>
#include <SD.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// Broche chip-select de la carte SD (même bus SPI que le LoRa)
#define SD_CS 7

LiteLora lora;
bool sdOk = false;

// ---- Carte SD ----

void logToSD(const uint8_t* data, uint8_t len, int16_t rssi, float snr) {
  if (!sdOk) return;

  lora.releaseBus();

  File f = SD.open("/recu.txt", FILE_APPEND);
  if (!f) {
    Serial.println("SD: erreur ouverture recu.txt");
    lora.acquireBus();
    return;
  }

  f.printf("RSSI:%d SNR:%.2f | ", rssi, snr);
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] >= 32 && data[i] <= 126) {
      f.print((char)data[i]);
    } else {
      f.printf("[0x%02X]", data[i]);
    }
  }
  f.println();
  f.close();

  lora.acquireBus();

  Serial.println("SD: message enregistré");
}

// ---- Setup ----

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== LiteLora - Exemple de base ===");

  // Configuration du module LoRa
  LiteLoraConfig cfg = LiteLora::defaultConfig();
  // Personnaliser si besoin :
  // cfg.frequency = 868000000;
  // cfg.spreadingFactor = 9;

  if (!lora.begin(cfg)) {
    Serial.println("ERREUR: Module LoRa non détecté!");
    while (1) delay(1000);
  }

  Serial.println("Module LoRa initialisé!");
  Serial.printf("Fréquence: %lu Hz\n", lora.getFrequency());

  // Initialisation carte SD (partage le bus SPI)
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  lora.releaseBus();
  if (SD.begin(SD_CS, SPI, 4000000)) {
    sdOk = true;
    Serial.println("Carte SD initialisée.");
  } else {
    Serial.println("Carte SD absente ou erreur.");
  }
  lora.acquireBus();

  Serial.println("\nCommandes disponibles:");
  Serial.println("  SEND:message   - Envoyer un message");
  Serial.println("  FREQ:Hz        - Changer fréquence");
  Serial.println("  SLEEP          - Deep sleep");
  Serial.println("  RSSI           - RSSI instantané");
  Serial.println("  STATUS         - État du module");
  Serial.println("\nEn attente de messages...\n");

  lora.receive();
}

// ---- Loop ----

void loop() {
  // Commandes série
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("SEND:")) {
      String msg = cmd.substring(5);
      Serial.printf("Envoi: %s\n", msg.c_str());

      uint32_t startTime = micros();
      lora.transmit((const uint8_t*)msg.c_str(), msg.length());

      if (lora.waitTransmitDone()) {
        Serial.printf("Envoyé en %lu µs\n", micros() - startTime);
      } else {
        Serial.println("Timeout transmission");
      }

      lora.receive();
    }
    else if (cmd.startsWith("FREQ:")) {
      uint32_t freq = cmd.substring(5).toInt();
      if (freq >= 137000000 && freq <= 525000000) {
        lora.setFrequency(freq);
        Serial.printf("Fréquence changée: %lu Hz\n", freq);
        lora.receive();
      } else {
        Serial.println("Fréquence invalide (137-525 MHz)");
      }
    }
    else if (cmd == "SLEEP") {
      Serial.println("Entrée en deep sleep...");
      Serial.flush();
      lora.receive();
      delay(100);
      esp_deep_sleep_enable_gpio_wakeup(1 << 1, ESP_GPIO_WAKEUP_GPIO_HIGH);
      gpio_set_direction((gpio_num_t)1, GPIO_MODE_INPUT);
      esp_deep_sleep_start();
    }
    else if (cmd == "RSSI") {
      Serial.printf("RSSI actuel: %d dBm\n", lora.currentRssi());
    }
    else if (cmd == "STATUS") {
      lora.printStatus();
    }
    else {
      Serial.println("Commande inconnue");
    }
  }

  // Réception de paquets
  if (lora.available()) {
    uint8_t buffer[LORA_MAX_PKT_LENGTH];
    uint32_t startTime = micros();
    uint8_t len = lora.readPacket(buffer, sizeof(buffer));
    uint32_t readTime = micros() - startTime;

    if (len > 0) {
      int16_t rssi = lora.packetRssi();
      float snr = lora.packetSnr();

      Serial.println("\n--- Paquet reçu ---");
      Serial.printf("Longueur: %d octets\n", len);
      Serial.printf("RSSI: %d dBm\n", rssi);
      Serial.printf("SNR: %.2f dB\n", snr);
      Serial.printf("Temps lecture: %lu µs\n", readTime);
      Serial.print("Données: ");

      for (uint8_t i = 0; i < len; i++) {
        if (buffer[i] >= 32 && buffer[i] <= 126) {
          Serial.print((char)buffer[i]);
        } else {
          Serial.printf("[0x%02X]", buffer[i]);
        }
      }
      Serial.println("\n-------------------\n");

      logToSD(buffer, len, rssi, snr);
    }

    lora.receive();
  }
}

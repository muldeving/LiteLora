/*
 * LiteLora - Implémentation
 */

#include "LiteLora.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

// ========== Constructeur ==========

LiteLora::LiteLora()
  : _csPin(0)
  , _dio0Pin(0)
  , _frequency(LORA_DEFAULT_FREQUENCY)
  , _spiFrequency(LORA_DEFAULT_SPI_FREQUENCY)
  , _bandwidth(LORA_DEFAULT_BANDWIDTH)
  , _spreadingFactor(LORA_DEFAULT_SPREADING)
  , _codingRate(LORA_DEFAULT_CODING_RATE)
  , _preambleLength(LORA_DEFAULT_PREAMBLE)
  , _txPower(LORA_DEFAULT_TX_POWER)
  , _syncWord(LORA_DEFAULT_SYNC_WORD)
{}

// ========== Configuration par défaut ==========

LiteLoraConfig LiteLora::defaultConfig() {
  LiteLoraConfig cfg;
  cfg.sckPin          = 4;
  cfg.misoPin         = 5;
  cfg.mosiPin         = 6;
  cfg.csPin           = 21;
  cfg.dio0Pin         = 8;
  cfg.frequency       = LORA_DEFAULT_FREQUENCY;
  cfg.bandwidth       = LORA_DEFAULT_BANDWIDTH;
  cfg.spreadingFactor = LORA_DEFAULT_SPREADING;
  cfg.codingRate      = LORA_DEFAULT_CODING_RATE;
  cfg.preambleLength  = LORA_DEFAULT_PREAMBLE;
  cfg.txPower         = LORA_DEFAULT_TX_POWER;
  cfg.syncWord        = LORA_DEFAULT_SYNC_WORD;
  cfg.spiFrequency    = LORA_DEFAULT_SPI_FREQUENCY;
  return cfg;
}

// ========== Initialisation ==========

bool LiteLora::begin(const LiteLoraConfig& config) {
  _csPin           = config.csPin;
  _dio0Pin         = config.dio0Pin;
  _frequency       = config.frequency;
  _spiFrequency    = config.spiFrequency;
  _bandwidth       = config.bandwidth;
  _spreadingFactor = config.spreadingFactor;
  _codingRate      = config.codingRate;
  _preambleLength  = config.preambleLength;
  _txPower         = config.txPower;
  _syncWord        = config.syncWord;

  // Configuration SPI
  SPI.begin(config.sckPin, config.misoPin, config.mosiPin, _csPin);
  SPI.setFrequency(_spiFrequency);

  // Broches
  pinMode(_csPin, OUTPUT);
  pinMode(_dio0Pin, INPUT);
  digitalWrite(_csPin, HIGH);

  // Reset logiciel et vérification
  softReset();

  uint8_t version = readRegister(LORA_REG_VERSION);
  if (version != 0x12) {
    return false;
  }

  // Mode Sleep pour configuration
  sleep();

  // Activer mode LoRa
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_SLEEP);
  delay(10);

  // Appliquer configuration radio
  setFrequency(_frequency);
  setTxPower(_txPower);
  setSpreadingFactor(_spreadingFactor);
  setBandwidth(_bandwidth);
  setCodingRate(_codingRate);
  setPreambleLength(_preambleLength);
  setSyncWord(_syncWord);

  // CRC activé
  writeRegister(LORA_REG_MODEM_CONFIG_2,
    (readRegister(LORA_REG_MODEM_CONFIG_2) & 0xFB) | 0x04);

  // AGC auto activé
  writeRegister(LORA_REG_MODEM_CONFIG_3, 0x04);

  // LNA gain max
  writeRegister(LORA_REG_LNA, 0x23);

  // FIFO base addresses
  writeRegister(LORA_REG_FIFO_TX_BASE_ADDR, 0x00);
  writeRegister(LORA_REG_FIFO_RX_BASE_ADDR, 0x00);

  // DIO0 -> RxDone/TxDone
  writeRegister(LORA_REG_DIO_MAPPING_1, 0x00);

  standby();
  return true;
}

// ========== SPI bas niveau ==========

uint8_t LiteLora::readRegister(uint8_t address) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(address & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  return value;
}

void LiteLora::writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(_csPin, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
}

// ========== Reset ==========

void LiteLora::softReset() {
  writeRegister(LORA_REG_OP_MODE, 0x00);  // FSK sleep
  delay(10);
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_SLEEP);
  delay(10);
}

// ========== Contrôle de mode ==========

void LiteLora::sleep() {
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_SLEEP);
}

void LiteLora::standby() {
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_STDBY);
}

void LiteLora::enterDeepSleep(uint8_t wakeupPin) {
  receive();
  delay(100);

  esp_deep_sleep_enable_gpio_wakeup(1 << wakeupPin, ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_set_direction((gpio_num_t)wakeupPin, GPIO_MODE_INPUT);
  esp_deep_sleep_start();
}

// ========== Configuration radio ==========

void LiteLora::setFrequency(uint32_t frequency) {
  _frequency = frequency;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  writeRegister(LORA_REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(LORA_REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(LORA_REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LiteLora::setTxPower(int8_t power) {
  if (power > 17) power = 17;
  if (power < 2) power = 2;
  _txPower = power;

  writeRegister(LORA_REG_PA_CONFIG, 0x80 | (power - 2));

  if (power > 17) {
    writeRegister(LORA_REG_PA_DAC, 0x87);
  } else {
    writeRegister(LORA_REG_PA_DAC, 0x84);
  }
}

void LiteLora::setSpreadingFactor(uint8_t sf) {
  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;
  _spreadingFactor = sf;

  if (sf == 6) {
    writeRegister(LORA_REG_DETECTION_OPTIMIZE, 0xC5);
    writeRegister(LORA_REG_DETECTION_THRESHOLD, 0x0C);
  } else {
    writeRegister(LORA_REG_DETECTION_OPTIMIZE, 0xC3);
    writeRegister(LORA_REG_DETECTION_THRESHOLD, 0x0A);
  }

  uint8_t config2 = readRegister(LORA_REG_MODEM_CONFIG_2);
  config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
  writeRegister(LORA_REG_MODEM_CONFIG_2, config2);
}

void LiteLora::setBandwidth(uint32_t bw) {
  _bandwidth = bw;
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

  uint8_t config1 = readRegister(LORA_REG_MODEM_CONFIG_1);
  config1 = (config1 & 0x0F) | (bwVal << 4);
  writeRegister(LORA_REG_MODEM_CONFIG_1, config1);
}

void LiteLora::setCodingRate(uint8_t cr) {
  if (cr < 5) cr = 5;
  if (cr > 8) cr = 8;
  _codingRate = cr;

  uint8_t crVal = cr - 4;
  uint8_t config1 = readRegister(LORA_REG_MODEM_CONFIG_1);
  config1 = (config1 & 0xF1) | (crVal << 1);
  writeRegister(LORA_REG_MODEM_CONFIG_1, config1);
}

void LiteLora::setPreambleLength(uint16_t length) {
  _preambleLength = length;
  writeRegister(LORA_REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(LORA_REG_PREAMBLE_LSB, (uint8_t)(length & 0xFF));
}

void LiteLora::setSyncWord(uint8_t sw) {
  _syncWord = sw;
  writeRegister(LORA_REG_SYNC_WORD, sw);
}

// ========== Émission ==========

void LiteLora::transmit(const uint8_t* data, uint8_t length) {
  if (length > LORA_MAX_PKT_LENGTH) length = LORA_MAX_PKT_LENGTH;

  standby();

  // Effacer IRQ flags
  writeRegister(LORA_REG_IRQ_FLAGS, 0xFF);

  // FIFO pointer au début
  writeRegister(LORA_REG_FIFO_ADDR_PTR, 0x00);

  // Écrire données dans FIFO
  digitalWrite(_csPin, LOW);
  SPI.transfer(LORA_REG_FIFO | 0x80);
  for (uint8_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(_csPin, HIGH);

  // Longueur payload
  writeRegister(LORA_REG_PAYLOAD_LENGTH, length);

  // Démarrer TX
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_TX);
}

bool LiteLora::waitTransmitDone(uint32_t timeoutUs) {
  uint32_t start = micros();

  while (!(readRegister(LORA_REG_IRQ_FLAGS) & LORA_IRQ_TX_DONE)) {
    if (micros() - start > timeoutUs) return false;
    delay(1);
  }

  // Effacer IRQ flags
  writeRegister(LORA_REG_IRQ_FLAGS, 0xFF);
  return true;
}

// ========== Réception ==========

void LiteLora::receive() {
  standby();

  writeRegister(LORA_REG_IRQ_FLAGS, 0xFF);
  writeRegister(LORA_REG_FIFO_ADDR_PTR, 0x00);
  writeRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE | LORA_MODE_RX_CONTINUOUS);
}

bool LiteLora::available() {
  return (readRegister(LORA_REG_IRQ_FLAGS) & LORA_IRQ_RX_DONE) != 0;
}

uint8_t LiteLora::readPacket(uint8_t* buffer, uint8_t maxLength) {
  uint8_t irqFlags = readRegister(LORA_REG_IRQ_FLAGS);

  if (irqFlags & LORA_IRQ_PAYLOAD_CRC_ERROR) {
    writeRegister(LORA_REG_IRQ_FLAGS, 0xFF);
    return 0;
  }

  uint8_t length = readRegister(LORA_REG_RX_NB_BYTES);
  if (length > maxLength) length = maxLength;

  uint8_t fifoAddr = readRegister(LORA_REG_FIFO_RX_CURRENT_ADDR);
  writeRegister(LORA_REG_FIFO_ADDR_PTR, fifoAddr);

  digitalWrite(_csPin, LOW);
  SPI.transfer(LORA_REG_FIFO & 0x7F);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);

  writeRegister(LORA_REG_IRQ_FLAGS, 0xFF);
  return length;
}

// ========== Informations paquet ==========

int16_t LiteLora::packetRssi() {
  return readRegister(LORA_REG_PKT_RSSI_VALUE) - 164;
}

float LiteLora::packetSnr() {
  int8_t snr = readRegister(LORA_REG_PKT_SNR_VALUE);
  return snr * 0.25;
}

int16_t LiteLora::currentRssi() {
  return readRegister(LORA_REG_RSSI_VALUE) - 164;
}

// ========== État ==========

void LiteLora::printStatus() {
  Serial.println("\n=== LiteLora - État du module ===");
  Serial.printf("Fréquence: %lu Hz\n", _frequency);
  Serial.printf("SF: %d\n", _spreadingFactor);
  Serial.printf("BW: %lu Hz\n", _bandwidth);
  Serial.printf("CR: 4/%d\n", _codingRate);
  Serial.printf("Préambule: %d\n", _preambleLength);
  Serial.printf("Puissance TX: %d dBm\n", _txPower);
  Serial.printf("Sync Word: 0x%02X\n", _syncWord);

  uint8_t mode = readRegister(LORA_REG_OP_MODE);
  Serial.print("Mode: ");
  switch (mode & 0x07) {
    case LORA_MODE_SLEEP:         Serial.println("Sleep"); break;
    case LORA_MODE_STDBY:         Serial.println("Standby"); break;
    case LORA_MODE_TX:            Serial.println("TX"); break;
    case LORA_MODE_RX_CONTINUOUS: Serial.println("RX Continu"); break;
    default:                      Serial.println("Autre"); break;
  }

  Serial.printf("RSSI: %d dBm\n", currentRssi());
  Serial.println("=================================\n");
}

uint32_t LiteLora::getFrequency() const {
  return _frequency;
}

// ========== Gestion bus SPI partagé ==========

void LiteLora::releaseBus() {
  digitalWrite(_csPin, HIGH);
}

void LiteLora::acquireBus() {
  SPI.setFrequency(_spiFrequency);
}

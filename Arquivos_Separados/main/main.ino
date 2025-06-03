//Main
#include "MPU.h"
#include "SIM.h"
#include <EEPROM.h>
#include <HardwareSerial.h>

// Definição de pinos (ajuste conforme seu mapa de pinos ESP32-C3 Super Mini):
#define sda           5  // Comunicação I2C com o MPU6050 (SDA)
#define scl           6  // Comunicação I2C com o MPU6050 (SCL)
#define rx            1  // RX do SIM800L (UART2)
#define tx            2  // TX do SIM800L (UART2)
#define MPUIntPin     3  // Pino de interrupção DATA_READY do MPU6050
#define buttonPin     10  // Botão de Pânico

// Definição de constantes
#define device_name   "DMIF2.0"         // Nome do dispositivo BLE
#define MPU_ADDR      0x68
#define timeReadMPU   20                // Período de leitura do MPU (ms)
#define timeAlarm     10                // Tempo entre tentativas de ligação (ms)
#define delaySerial   1000               // Timeout para respostas AT (ms)

static float       m;                      // Armazena o valor da massa
static byte        _quantContate;
static uint64_t    numberInt[4];           // Armazenam os números de telefone como uint64_t
byte               alarmType;              // 0: indefinido, 1: pânico, 2: queda, 3: erro
volatile bool      flagButton;             // Interrupção do botão

MPU mpu6050;
SIM sim800L;

void butonInterrupt();

// Interrupção do botão de pânico
void IRAM_ATTR butonInterrupt() {
  flagButton = true;
}

void setup() {

  Serial.begin(9600);
  Serial.println("Setup Iniciado");

  // Inicializa EEPROM com 28 bytes
  EEPROM.begin(28);

  // Carrega valor da massa armazenada (2 bytes em [24],[25])
  m = float((EEPROM.read(24) << 8) | EEPROM.read(25)) / 100.0;
  Serial.print("Valor da massa (EEPROM): ");
  Serial.println(m);

  // Carrega números salvos na EEPROM
  numberInt[0] = 0;
  numberInt[1] = 0;
  numberInt[2] = 0;
  numberInt[3] = 0;

  bool eof = false;
  for (int i = 0; i < 24 && !eof; i++) {
    uint8_t idx   = i / 6;   // 0..3
    uint8_t byteN = i % 6;   // 0..5
    numberInt[idx] |= (uint64_t(EEPROM.read(i)) << ((5 - byteN) * 8));
    if (byteN == 5) {
      if (numberInt[idx] == 0) {
        _quantContate = idx;
        eof = true;
      }
    }
  }

  sim800L.begin(rx, tx, delaySerial, 4, numberInt);

  flagButton = false;
  alarmType  = 0;

  
  mpu6050.begin(sda, scl, MPUIntPin, MPU_ADDR, timeReadMPU);
  attachInterrupt(digitalPinToInterrupt(buttonPin), butonInterrupt, RISING);// Ativa interrupção do botão de pânico
}

void loop() {}

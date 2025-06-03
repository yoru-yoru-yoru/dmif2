//SIM body
#include "SIM.h"

static HardwareSerial simSerial(1);        // UART2 para o SIM800L (pins rx, tx)

SIM::SIM(){}

void SIM::begin(byte rxPin, byte txPin, int default_Serial_Delay, byte quant_Contate, uint64_t number_Int[4]) {
  // Nota: pinos rx/tx do SIM800L são configurados no _serial->begin()
  _serial = &simSerial; // Inicializa o objeto HardwareSerial
  this->quantContate = quant_Contate; // Define a quantidade de contatos
  this->defaultSerialDelay = default_Serial_Delay; // Define o tempo de espera padrão para comandos

  // Constrói as strings para cada número salvo
  for (int cont = 0; cont < this->quantContate; cont++) {
    uint64_t temp = number_Int[cont];
    for (int j = 14; j >= 1; j--) {
      numberStr[cont][j] = '0' + (temp % 10);
      temp /= 10;
    }
    numberStr[cont][0]  = '+';
    numberStr[cont][15] = '\0';
    Serial.print("Number salve in ");
    Serial.print(char('A' + cont));
    Serial.print(": ");
    Serial.println(numberStr[cont]);
  }
    
  simSerial.setPins(rxPin, txPin);         // Define pinos RX/TX
  simSerial.begin(9600);                   // Inicializa UART
  Serial.print("SIM status: ");
  Serial.println(verifySim());
}

String SIM::sendAndReceive(const char* cmd, uint32_t timeout) {
  unsigned long start = millis();
  while (simSerial.available()) simSerial.read();  // Limpa buffer
  simSerial.println(cmd);
  String resp;
  while ((millis() - start) < timeout) {
    if (simSerial.available()) {
      resp += (char)simSerial.read();
    }
  }
  Serial.println(resp);
  return resp;
}

byte SIM::verifySim() {
  String resp;

  resp = sendAndReceive("AT", defaultSerialDelay);
  if (resp.indexOf("OK") < 0){
    return 1;
  }

  resp = sendAndReceive("AT+COPS?", defaultSerialDelay);
  Serial.println(resp);
  if (resp.indexOf('"') < 0) return 2;

  resp = sendAndReceive("AT+CSQ", defaultSerialDelay);
  if (resp.indexOf("CSQ:") < 0) return 3;

  int colon = resp.indexOf(":");
  int comma = resp.indexOf(",");
  int rssi = resp.substring(colon + 1, comma).toInt();
  int ber  = resp.substring(comma + 1).toInt();
  if (rssi < 10 || rssi > 31 || ber < 0 || ber > 1) return 3;

  return 0;
}

void SIM::foneSMS(char* msg) {
  for (int i = 0; i < this->quantContate; i++) {
    _serial->println("AT+CMGF=1"); // Modo texto
    delay(500);
    _serial->print("AT+CMGS=\"");
    _serial->print(numberStr[i]);
    _serial->println("\"");
    delay(500);
    _serial->print(msg);
    _serial->write(26); // Ctrl+Z
    delay(3000);
  }
}


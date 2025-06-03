//Cabeçario SIM
#ifndef SIM_H
#define SIM_H
#include <Arduino.h>
#include <HardwareSerial.h>

class SIM{
    private:
    HardwareSerial* _serial;
    char numberStr[4][16];       // Armazenam os números de telefone como string (“+55…”)
    byte quantContate;           // Quantos contatos serão usados (1 a 4)
    int defaultSerialDelay;      // Tempo padrão de espera para respostas do SIM800L
    
    public:
    SIM();
    void begin(byte rxPin, byte txPin, int default_Serial_Delay, byte quant_Contate, uint64_t number_Int[4]);
    String sendAndReceive(const char* cmd, uint32_t timeout);
    byte verifySim();
    void foneSMS(char* msg);
};

#endif
//Cabeçario MPU
#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

class MPU{
    private:
        MPU6050 mpu6050;              // Para comunicação I2C com MPU6050
        TaskHandle_t hTaskMPU;
        volatile bool flagMPU;        // Interrupção do MPU (dados prontos)
        uint counter;          // Contador interno
        int timeReadMPU;
        byte intPin;                 // Pino de interrupção do MPU
        static MPU *instance; // Instância única do MPU

        static void IRAM_ATTR newDataMPU();
        static void taskWrapper(void*);
        bool pico(float input, float margem);
        void read(void *pvParameters);
    public:
        void begin(byte sdaPin, byte sclPin, byte intPin, byte ADDR, int timeTask);
        MPU();
};
#endif

#include "portmacro.h"
//MPU body
#include "MPU.h"

MPU* MPU::instance = nullptr; // Inicializa a instância estática

MPU::MPU() : mpu6050(Wire){

}

void MPU::begin(byte sdaPin, byte sclPin, byte intPin, byte ADDR, int timeTask) {
  // Configurações iniciais de pinos
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  this->intPin = intPin; // Define o pino de interrupção do MPU
  pinMode(intPin, INPUT_PULLUP);
  
  timeReadMPU = timeTask; // Define o tempo de leitura do MPU
  hTaskMPU = NULL; // Inicializa o handle da task do MPU
  flagMPU = false; // Inicializa a flag de interrupção do MPU
  counter = 0; // Inicializa o contador interno
  instance = this; // Define a instância única do MPU

  Wire.begin(sdaPin, sclPin);// Inicia I2C para o MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Habilita interrupção “Data Ready” no MPU6050
  Wire.beginTransmission(ADDR);
  Wire.write(0x38);   // Registrador INT_ENABLE
  Wire.write(0x01);   // Habilita interrupção “Data Ready”
  Wire.endTransmission();

  attachInterrupt(digitalPinToInterrupt(intPin), newDataMPU, RISING);
  // Cria task FreeRTOS para leitura do MPU
  Serial.println("Criou a task");
  xTaskCreate(taskWrapper, "readMPU", 2048, this, 4, &hTaskMPU);
}
  
// Interrupção de dados prontos do MPU6050
void IRAM_ATTR MPU::newDataMPU() {
  if(instance){
    instance->flagMPU = true;
    //detachInterrupt(digitalPinToInterrupt(instance->intPin));
  }
}

void MPU::read(void *pvParameters) {
  (void)pvParameters;
  static float accel[3];
  static double v[3] = {0, 0, 0};
  static double deltaT;
  static float accelA[3] = {0, 0, 0};
  static float A = 0, V = 0, Va = 0, E = 0;
  static unsigned long timeAtu, timeAnt = 0;
  float m;

  while (true) {
    for (int i = 0; i < 1000; i++) {
      if (flagMPU) {
        mpu6050.update();
        timeAtu = micros();
        deltaT = double(timeAtu - timeAnt) / 1e6;
        timeAnt = timeAtu;

        accel[0] = mpu6050.getAccX() * 10.0;
        accel[1] = mpu6050.getAccY() * 10.0;
        accel[2] = mpu6050.getAccZ() * 10.0;

        for (int ax = 0; ax < 3; ax++) {
          v[ax] += deltaT * double((accel[ax] + accelA[ax]) / 2.0);
          accelA[ax] = accel[ax];
        }

        A = sqrt(pow(accel[0], 2) + pow(accel[1], 2) + pow(accel[2], 2));
        if (pico(A, 0.00)) {
          V = sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
          for (int ax = 0; ax < 3; ax++) {
            v[ax] = 0.0;
          }
          E = (m / 2.0) * (V * V - Va * Va);
          Va = V;
          // Aqui você pode definir lógica de detecção de queda:
          // if (E > limiarEnergia) { alarmType = 2; }
        }
        vPortEnterCritical();
        flagMPU = false;
        vPortExitCritical();

        //attachInterrupt(digitalPinToInterrupt(intPin), newDataMPU, RISING);
        counter = 0;
      } else {
        counter++;
         if (counter > 1000) { Serial.println("MPU parou!"); }
      }
    }
    Serial.print("Aceleracao: ");
    Serial.println(accel[0]);
    vTaskDelay(pdMS_TO_TICKS(timeReadMPU));
  }
}

// Função para detectar picos de aceleração
bool MPU::pico(float input, float margem) {
  static float inputAnt = 0.0, picoAnt = 0;
  static bool state = false, stateAnt = false;

  if (inputAnt < input) {
    state = true;
  } else if (inputAnt > input) {
    state = false;
  }

  if (state != stateAnt) {
    stateAnt = state;
    if (abs(inputAnt - picoAnt) > margem) {
      picoAnt = inputAnt;
      inputAnt = input;
      return true;
    }
    picoAnt = inputAnt;
  }
  inputAnt = input;
  return false;
}

void MPU::taskWrapper(void* pvParameters) {
  MPU* mpuInstance = static_cast<MPU*>(pvParameters);
  mpuInstance->read(pvParameters);
}


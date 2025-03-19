#include <MPU6050_tockn.h>
#include <Wire.h>

#define sda A4
#define scl A5

MPU6050 mpu6050(Wire);

bool pico(float input){
  static float inputAnt = 0.0;
  static bool state, stateAnt;

  if(inputAnt < input){
    state = true;
  }else if(inputAnt > input){
    state = false;
  }

  if(state != stateAnt){
    return true;
  }else{
    return false;
  }
}

//Mede a asceleração e o tempo entre as medidas
void updateAscelTime(float (&Ascel)[3], double &DeltaT){
  static unsigned long timeAtu, timeAnt = 0;//Variáveis para cálculo de DeltaT
  mpu6050.update();//Obtém as leituras do senssor
  //Calcula o tempo entre a medição atual e a anterior
  timeAtu = millis();
  DeltaT = (double)(timeAtu - timeAnt) / 1000;
  timeAnt = timeAtu;
  //Faz uma correção preliminar removendo a constante gravitacional com base nos ângulos. 
  Ascel[0] = mpu6050.getAccX() * 10;
  Ascel[1] = mpu6050.getAccY() * 10;
  Ascel[2] = mpu6050.getAccZ() * 10;
}

void setup() {
  pinMode(sda,INPUT);
  pinMode(scl,INPUT);
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
}

void loop() {
  //Array para a leitura da asceleração
  static float ascel[3];

  //Variáveis integração
  static float v[3] = {0,0,0};//Armazenam a velocidade nos tres eixos
  static double deltaT;//Armazena o tempo entre a medida atual e a anterior
  static float ascelA[3] = {0,0,0};

  //Variáveis do calculo da potência de impacto:
  static float P, A, V, m = 0.015;

  updateAscelTime(ascel, deltaT);

  for(int i=0; i<3; i++){
    v[i] += deltaT * (ascel[i] + ascelA[i]) / 2;
    ascelA[i] = ascel[i];
  }
  Serial.print("DeltaT:");
  Serial.println(deltaT);
/*
  Serial.print("VelX:");
  Serial.println(v[0]);
  Serial.print("VelY:");
  Serial.println(v[1]);
  Serial.print("VelZ:");
  Serial.println(v[2]);
*/
  A = sqrt(pow(ascel[0],2)+pow(ascel[1],2)+pow(ascel[2],2));//Calcula a asceleração total com pitágoras

  if(pico(A)){
    V = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));//Calcula a velocidade total com pitágoras  
    Serial.print("VelT:");
    Serial.println(V);
    for(int i=0; i<3; i++){v[i] = 0;}
    P = m * A * V;//Calcula a potência cinética
  }

  //Aqui tem que  implementar toda a mecânica do alarme.

  //Debug:
  Serial.print("Pot:");
  Serial.println(P);
/* 
  Serial.print("AccX:");
  Serial.println(ascel[0]);
  Serial.print("AccY:");
  Serial.println(ascel[1]);
  Serial.print("AccZ:");
  Serial.println(ascel[2]);
  */
  Serial.print("AccT:");
  Serial.println(A);
  
}
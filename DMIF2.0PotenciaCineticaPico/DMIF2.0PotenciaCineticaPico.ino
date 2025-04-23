#include <MPU6050_tockn.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <SIM800L.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

//Definição de pinos:
#define sda 21//Comunicação com o MPU6050.
#define scl 22//Comunicação com o MPU6050.
#define rx 16//Comunicação com o Sim800L.
#define tx 17//Comunicação com o Sim800L.
#define MPUIntPin 15

//Devinição de constantes
#define device_name "DMIF2.0"//Nome do dispositivo blooetooth.
#define MPU_ADDR 0x68

//Coisa do bluethooth, só confia:
  // Check if Bluetooth is available
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif
  // Check Serial Port Profile
  #if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
  #endif
//

//Instanciação de objetos
BluetoothSerial SerialBT;//Para a comunicação bluetooth.
MPU6050 mpu6050(Wire);//Para a comunicação com o MPU6050 por meio da biblioteca Wire.
SIM800L Sim800L;//Para a comunicação com o Sim800L.
HardwareSerial mySerial(2);//Instancia a comunicação RX-TX pela UART2.

//Variáveis globais
static float m;//Armazena o valor da massa da pessoa.
char strA[12], strB[12], strC[12], strD[12];//Armazenam os numeros de telefone.
static uint64_t numberA, numberB, numberC, numberD;
bool readMPU;

void foneSMS(char* msg){
  Sim800L.sendSMS(strA, msg);
  Sim800L.sendSMS(strB, msg);
  Sim800L.sendSMS(strC, msg);
  Sim800L.sendSMS(strD, msg);
}

void foneCall(){
  
}

/*
void impactDetection(aceleration, velocity, energy){}//Implementar aqui o algoritmo para diferenciar um impacto de um movimento normal
void alarme(float statistic, int type){}//Implementar aqui o algoritmo de alarme
*/

void salveNumber() {
  int address = 0; // Endereço inicial na EEPROM
  uint64_t number = 0;// Variável para armazenar o número lido
  char memory, c;
  //Serial.println("salveNumber");

  //Se tiver caracter, lê ele.  
    if(SerialBT.available()){
      memory = SerialBT.read(); // Lê o caractere que indica a memória
    }else{
      Serial.println("Sem caracter");
    }
  //

  //Seleciona o endereço da memória de acordo com a letra do código.
    if(memory == 'A' || memory == 'a') {
      address = 0; // Endereço 0 para A
    }else if(memory == 'B' || memory == 'b') {
      address = 5; // Endereço 5 para B
    }else if(memory == 'C' || memory == 'c') {
      address = 10; // Endereço 10 para C
    }else if(memory == 'D' || memory == 'd'){
      address = 15; // Endereço 15 para D
    }else{
      Serial.println("Endereco desconhecido");
    }
  //

  while(SerialBT.available()){//Enquanto tiver numeros para ler ele concatena os numeros em um uint64_t.
    c = SerialBT.read(); // Lê o caractere
    if(c >= '0' && c <= '9'){
      number = (number * 10) + static_cast<uint64_t>(c - '0');
      //Serial.println(number);
    }else{
      Serial.println("Caracter nao numerico");
    }
  }

  switch(address){//Atualiza a variável de acordo com o endereço escolhido.
    case 0:
      numberA = number;
    break;
    case 5:
      numberB = number;
    break;
    case 10:
      numberC = number;
    break;
    case 15:
      numberD = number;
  }

  for(int i = 0; i < 5; i++) {//Armazena os 5 bytes do número.
    //Serial.println((number >> ((4-i) * 8)) & 0xFF);//Debug
    EEPROM.write(address + i, (number >> ((4-i) * 8)) & 0xFF);
  }

  EEPROM.commit();// Grava as alterações na EEPROM.

  // Envia uma mensagem de confirmação via Bluetooth:
    SerialBT.println("Numero salvo com sucesso");
    SerialBT.print("Endereco: ");
    SerialBT.println(address);
    SerialBT.print("Numero: ");
    SerialBT.println(number);
  //

  /*//Debug.
    for(int i = 0; i<20; i++){
      Serial.print("Endereco:");
      Serial.print(i);
      Serial.print(":");
      Serial.println(EEPROM.read(i));  
    }
  */
}

bool pico(float input){//Verifica se o gráfico de input teve um pico
  static float inputAnt = 0.0;
  static bool state, stateAnt;
  if(inputAnt < input){
    state = true;
  }else if(inputAnt > input){
    state = false;
  }
  inputAnt = input;
  if(state != stateAnt){
    stateAnt = state;
    return true;
  }else{
    stateAnt = state;
    return false;
  }
}

void updateAscelTime(float (&Ascel)[3], double &DeltaT){//Mede a asceleração e o tempo entre as medidas
  static unsigned long timeAtu, timeAnt = 0;//Variáveis para cálculo de DeltaT
  mpu6050.update();//Obtém as leituras do senssor
  //Calcula o tempo entre a medição atual e a anterior
  timeAtu = millis();
  DeltaT = (double)(timeAtu - timeAnt) / 1000;
  timeAnt = timeAtu;
  //Salva as ascelerações por 10 para converter de g para m/s e salva no vetor Ascel. 
  Ascel[0] = mpu6050.getAccX() * 10;
  Ascel[1] = mpu6050.getAccY() * 10;
  Ascel[2] = mpu6050.getAccZ() * 10;
}

void bluetoothReadMass(){
  char caracter;
  int potencia = 1, point;
  float dataFloat = 0;
  for(int i = 0; SerialBT.available(); i++){
    caracter = SerialBT.read();
    if(caracter == '.' || caracter == ','){
      point = i;
    }else if(caracter >= '0' && caracter <= '9'){
      dataFloat += static_cast<float>(caracter - '0') / (float)potencia;
      potencia *= 10;
    }else{
      Serial.println("Caracter nao numerico");//Tratamento de erro.
    }
  }
  for(int i =0; i< point-1; i++){dataFloat *= 10.0;}
  
  m = dataFloat;
  
  int mass = static_cast<int>(dataFloat * 100);
  Serial.print("mass:");
  Serial.println(mass);
  
  for(int i = 0; i < 2; i++) {//Armazena os 2 bytes do número.
    EEPROM.write(20 + i, (mass >> ((1-i) * 8)) & 0xFF);
    Serial.print("Endereco:");
    Serial.print(i);
    Serial.print(":");
    Serial.println((mass >> ((1-i) * 8)) & 0xFF);
  }
  
  EEPROM.commit();// Grava as alterações na EEPROM

  SerialBT.print("A massa foi atualizada para:");
  SerialBT.println(m);
}

void setup() {


  //Definição dos modos dos pinos:
    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
    pinMode(MPUIntPin, INPUT_PULLUP);
  //Obs.: Os pinos rx e tx já tem os modos definidos automaticamente pelo método mySerial.begin(rx, tx).
  
  //Begins:
    Serial.begin(9600);//Apenas para debug.
    SerialBT.begin(device_name);//Inicia a comunicação bluetooth.
    EEPROM.begin(25);//Inicia a memória EEPROM com 25 endereços (bytes).
    Wire.begin();//Para a comunicação I2C com o MPU6050.
    mySerial.begin(9600, SERIAL_8N1, rx, tx);//Inicia a comunicação RX-TX que será usada com o Sim800L.
    Sim800L.begin(mySerial);//Inicia o Sim800L.
    mpu6050.begin();//Inicia o MPU6050.
    mpu6050.calcGyroOffsets(true);//Calcula os offsets do giroscópio.
  //
  readMPU = false;
  numberA = 0;
  numberB = 0;
  numberC = 0;
  numberD = 0;

  //Pega os dados da EEPROM e joga nas respectivas variáveis:
    m = static_cast<float>((EEPROM.read(20) << 8) | EEPROM.read(21)) / 100.0;//Extrai o valor da EEPROM para m.
    Serial.print("Valor da massa:");//Debug.
    Serial.println(m);//Debug.

    for(int i = 0; i<5; i++){numberA |= (static_cast<uint64_t>( EEPROM.read(i) ) << ( (4-i) *8) );}//Extrai o valor da EEPROM para numberA.
    Serial.print("Numero salvo em A:");//Debug.
    Serial.println(numberA);//Debug.

    for(int i = 5; i<10; i++){numberB |= (static_cast<uint64_t>( EEPROM.read(i) ) << ( (9-i) *8) );}//Extrai o valor da EEPROM para numberB.
    Serial.print("Numero salvo em B:");//Debug.
    Serial.println(numberB);//Debug.

    for(int i = 10; i<15; i++){numberC |= (static_cast<uint64_t>( EEPROM.read(i) ) << ((14-i) *8) );}//Extrai o valor da EEPROM para numberC.
    Serial.print("Numero salvo em C:");//Debug.
    Serial.println(numberC);//Debug.

    for(int i = 15; i<20; i++){numberD |= (static_cast<uint64_t>( EEPROM.read(i) ) << ( (19-i) *8) );}//Extrai o valor da EEPROM para numberD.
    Serial.print("Numero salvo em D:");//Debug.
    Serial.println(numberD);//Debug.
  //

  for(int i = 0; i<11; i++){
    numberA /= 10;
    numberB /= 10;
    numberC /= 10;
    numberD /= 10;
    strA[i] = numberA % 10;
    strB[i] = numberB % 10;
    strC[i] = numberC % 10;
    strD[i] = numberD % 10;
  }

  strA[11] = '\0';
  strB[11] = '\0';
  strC[11] = '\0';
  strD[11] = '\0';

  // Habilita a interrupção de "Data Ready" no 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38); // Registrador INT_ENABLE
  Wire.write(0x01); // Habilita a interrupção de "Data Ready"
  Wire.endTransmission();
  attachInterrupt(digitalPinToInterrupt(MPUIntPin), newDataMPU, RISING);
}

void newDataMPU(){readMPU = true;}

void loop() {
  //Variáveis:
    //Variáveis para armazenar os dados do MPU6050:
    static float ascel[3];//Armazena a leitura da asceleração.
    static double v[3] = {0,0,0};//Armazena a velocidade nos tres eixos.
    static double deltaT;//Armazena o tempo entre a medida atual e a anterior.
    static float ascelA[3] = {0,0,0};//Assceleração anterior.
    static float A, V, Va, E;//Armazenam os dados de saida do algoritmo.
    //Variáveis gerais:
    static char caracter1, caracter2;//Variável para receber código bluetooth
    static unsigned long interruptAscelTime = 0;
  //
  //adicionar interrupção de tempo.
  if(readMPU){
    readMPU = false;
    updateAscelTime(ascel, deltaT);//Obtém os valores de asceleração e o tempo entre as medidas.
    for(int i=0; i<3; i++){//Integra a asceleração em cada eixo e armazena em v[]:
      v[i] += deltaT * (double)((ascel[i] + ascelA[i]) / 2);
      ascelA[i] = ascel[i];
    }
  }

  A = sqrt(pow(ascel[0],2)+pow(ascel[1],2)+pow(ascel[2],2));//Calcula a asceleração total com pitágoras.
  Serial.print("Asceleracao:");
  Serial.println(A);
  
  if(pico(A)){//Se houver um pico ou vale em A, reseta a integração e trata os dados.
    V = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));//Calcula a velocidade total com pitágoras.
    for(int i=0; i<3; i++){v[i] = 0.0;}//Reseta a integração setando o vetor de velocidade para 0.
    E = (m/2) * (V * V - Va * Va);//Calcula a variação da energia cinética entre o pico atual e o anterior.
    Va = V;
    //impactDetection(A, V, E);
  }

  if(SerialBT.available()){//Caso receba uma menssagem via bluetooth, trata ela.
    caracter1 = SerialBT.read();//Analisa o primeiro caracter para verificar a que a informação se refere.
    caracter2 = SerialBT.read();//Analisa o segundo caracter para verificar a que a informação se refere.
    if((caracter1 == 'm' || caracter1 == 'M') && (caracter2 == 'm' || caracter2 == 'M')){
      bluetoothReadMass();//Atualisa o valor da massa.
    }else if((caracter1 == 'n' || caracter1 == 'N') && (caracter2 == 'n' || caracter2 == 'N')){
      salveNumber();//Atualiza um numero de telefone.
    }else{
      Serial.println("Caracter inicial desconhecido.");
    }//Adicionar else ifs para outras configurações.
  }

}
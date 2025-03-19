/*
CONSIDERAÇÕES PRELIMINARES:
Explicações geais:
  As variáveis são todas static para evitar realocação de memória e melhorar a eficiência.
  Isso também serve para manter os valores de timeAtu e timeAnt entre as chamadas de updateAscelTime()
  e iniciar as variáveis no loop() com um valor inicial sem ter que setar elas no setup().

  O filtro de média variável atenua variações pontuais no sinal, fazendo-o tender à constância. Como a gravidade é constante, a ideia é
  que, descontando o valor filtrado do valor bruto, na prática, acabe descontando a gravidade que tenha escapado da correção prelimi nar.

  O filtro de média variável dependeria de um laço for, mas isso obrigaria a ler os n valores para depois fazer algo, o que causaria atraso.
  Então preferi usar o próprio laço do void loop() para poder ler e executar o resto ao mesmo tempo. Por isso é necessário counter; já que não tem o contador do for.

  A convenção nos arrays é 0 para x, 1 para y e 2 para z.

Precida adicionar:
  A interface para colocar a massa
  Implementar a integração para obter a gravidade

*/

#include <MPU6050_tockn.h>
#include <Wire.h>

#define n 100//Tamanho do buffer do filtro.
#define sda A4
#define scl A5


MPU6050 mpu6050(Wire);

static float g;//Armazena a gravidade

//Retorna o sinal
//Não era para precisar disso, mas o calculo da componente gravitacional no eixo z não retorna sinal, então precisa dissol.
int sig(float value){
  if(value > 0){
    return 1;
  }else if(value < 0){
    return -1;
  }else{
    return 0;
  }
}

//Mede a asceleração e o tempo entre as medidas
void updateAscelTime(float (&Ascel)[3], int &DeltaT){
  static unsigned long timeAtu, timeAnt = 0;//Variáveis para cálculo de DeltaT
  mpu6050.update();//Obtém as leituras do senssor
  //Calcula o tempo entre a medição atual e a anterior
  timeAtu = millis();
  DeltaT = timeAtu - timeAnt;
  timeAnt = timeAtu;
  //Faz uma correção preliminar removendo a constante gravitacional com base nos ângulos. 
  Ascel[0] = mpu6050.getAccX(); //+ g * sin(mpu6050.getAngleY()*PI/180);
  Ascel[1] = mpu6050.getAccY(); //- g * sin(mpu6050.getAngleX()*PI/180) * cos(mpu6050.getAngleY()*PI/180);
  Ascel[2] = mpu6050.getAccZ(); //- g * cos(mpu6050.getAngleX()*PI/180) * cos(mpu6050.getAngleY()*PI/180) * sig(mpu6050.getAccZ());
}

void setup() {
  pinMode(sda,INPUT);
  pinMode(scl,INPUT);
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  g = sqrt(pow(mpu6050.getAccX(),2)+pow(mpu6050.getAccY(),2)+pow(mpu6050.getAccZ(),2));//Obtem a asceleração da gravidade
}

void loop() {
  //Array para a leitura da asceleração
  static float ascel[3];

  //Variáveis integração
  static float v[3] ={0,0,0};//Armazenam a velocidade nos tres eixos
  static int deltaT;//Armazena o tempo entre a medida atual e a anterior
  
  //Variáveis filtro de média móvel:
  static float a[n][3], med[3], soma[3] = {0,0,0};
  static int counter = 0;
  static bool carregate = true;

  //Variáveis do calculo da potência de impacto:
  static float P, A, V, m;

  //FILTRO DE MÉDIA MÓVEL:

  //Carrega o buffer na primeira vez para eliminar o lixo da memória.
  //No futuro talvez mover para o setup(), mas isso tornaria "ascel", "deltaT" e "a" globais, tornando menos eficiente.
  if(carregate){
    updateAscelTime(ascel, deltaT);//Obtem os valores atuais de asceleração.
    for(int i =0; i<n; i++){//Repete para os n termos de "a" preenchendo a dimenssão temporal
      for(int j=0; j<3; j++){//Repete para os três eixos
        a[i][j] = ascel[j];
        soma[j] += a[i][j];
      }
    }
    carregate = false;//Desabilita para o restante do tempo de execução.
  }
  static float ascelA[3] = {0,0,0};
  updateAscelTime(ascel, deltaT);
  for (int i =0; i<3; i++){//Repete três vezes, uma para cada eixo
    soma[i] = soma[i] - a[counter][i] + ascel[i];//Substitui em "soma" a informação mais antiga pela mais nova
    a[counter][i] = ascel[i];//Substitui em "a" a informação mais antiga pela mais nova
    med[i] = soma[i]/n;//Calcula a media dividindo o total pelo numero de fatores
    ascel[i] -= med[i];
    ascel[i] /= 10;
    //if(ascel[i] < 0.01 && ascel[i] > -0.01){ascel[i] = 0;}
    v[i] += (ascel[i]+ascelA[i])*deltaT/2;
    ascelA[i] = ascel[i];
  }

  //Debug:
  
  Serial.print("AccX:");
  Serial.println(ascel[0]);
  Serial.print("AccY:");
  Serial.println(ascel[1]);
  Serial.print("AccZ:");
  Serial.println(ascel[2]);
  
  Serial.print("Velx:");
  Serial.println(v[0]);
  Serial.print("Vely:");
  Serial.println(v[1]);
  Serial.print("Velz:");
  Serial.println(v[2]);

  counter = (counter +1) % n;//Incrementa counter 
  
  //Cálculo da potência
  A = sqrt(pow(med[0],2)+pow(med[1],2)+pow(med[2],2));//Calcula a asceleração total com pitágoras
  V = sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));//Calcula a velocidade total com pitágoras
  P = m * A * V;//Calcula a potência cinética

  //Aqui tem que  implementar toda a mecânica do alarme.
}

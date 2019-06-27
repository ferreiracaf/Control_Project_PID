#include <Ultrasonic.h>

#define pinSH_CP  4   //Pino Clock  DIR_CLK
#define pinST_CP  12  //Pino Latch  DIR_LATCH
#define pinDS     8   //Pino Data   DIR_SER
#define pinEnable 7   //Pino Enable DIR_EN

#define pinMotor2PWM 3
#define pinMotor3PWM 5

#define qtdeCI   1

#define bitMotor2A 1
#define bitMotor2B 4
#define bitMotor3A 5
#define bitMotor3B 7

#define p_on 11

void ciWrite(byte pino, bool estado);

Ultrasonic ultrassom(10, 9);

float distancia;
float lastMeasure;

void setup() {
  pinMode(pinSH_CP, OUTPUT);
  pinMode(pinST_CP, OUTPUT);
  pinMode(pinEnable, OUTPUT);
  pinMode(pinDS, OUTPUT);

  pinMode(pinMotor2PWM, OUTPUT);
  pinMode(pinMotor3PWM, OUTPUT);

  digitalWrite(pinEnable, LOW);

  pinMode(p_on, INPUT);

  Serial.begin(9600);
}

void loop() {

  distancia = ultrassom.Ranging(CM);
  Serial.print(distancia); //imprime o valor da variável distancia
  Serial.println("cm");

  if(digitalRead(p_on)){
    Serial.println("PID");
    pid_control(distancia);    //controle PID, se chave p_on acionada
  }
  else{
    Serial.println("OnOff");
    onOff_control(distancia);
  }
  
  delay(65);
  
}

void onOff_control(float measure){

  if(measure > 20){
    //  Serial.println("1: A=HIGH B=LOW");
    ciWrite(bitMotor2A, HIGH);
    ciWrite(bitMotor3A, HIGH);
    ciWrite(bitMotor2B, LOW);
    ciWrite(bitMotor3B, LOW);  
  }
  else if(measure <= 20){
    //  Serial.println("3: A=LOW B=HIGH");
    ciWrite(bitMotor2A, LOW);
    ciWrite(bitMotor3A, LOW);
    ciWrite(bitMotor2B, HIGH);
    ciWrite(bitMotor3B, HIGH);  
  }

  analogWrite(pinMotor2PWM, 255);
  analogWrite(pinMotor3PWM, 255);

}

void pid_control(float measure)                   //Função para algorimo PID
{

  float    error_meas,                            //armazena o erro
           kp = 1.0,                              //constante kp
           ki = 0.02,                             //constante ki
           kd = 0.01,                              //constante kd
           proportional,                          //armazena valor proporcional
           integral,                              //armazena valor integral
           derivative,                            //armazena valor derivativo
           PID,                                   //armazena resultado PID
           ideal_value = 20.0;                           //armazena última medida
         
    error_meas = measure - ideal_value;           //calcula erro
    
    proportional = error_meas * kp;               //calcula proporcional
    
    integral += error_meas * ki;                  //calcula integral
    
    derivative = (lastMeasure - measure) * kd;    //calcula derivada
                                                  
    lastMeasure = measure;                        //atualiza medida
    
    PID = proportional + integral + derivative;   //calcula PID

 
    if(PID < 0)                                   //PID menor que zero?
    {                                             //sim
      PID = map(PID,  0, -20, 0, 255);            //normaliza para PWM de 8 bits
      // robot_back();                            //move robô para trás
      ciWrite(bitMotor2A, LOW);
      ciWrite(bitMotor3A, LOW);
      ciWrite(bitMotor2B, HIGH);
      ciWrite(bitMotor3B, HIGH); 

    } //end if PID
    
    else                                          //senão
    {                                             //PID maior ou igual a zero
      PID = map(PID,  0, 20, 0, 255);             //normaliza para PWM de 8 bits
      if(PID > 255) PID = 255;                    //se PID maior que 255, mantém em 255
      // robot_ahead();                              //move robô para frente
      
      ciWrite(bitMotor2A, HIGH);
      ciWrite(bitMotor3A, HIGH);
      ciWrite(bitMotor2B, LOW);
      ciWrite(bitMotor3B, LOW); 
    } //end else
    
    analogWrite(pinMotor3PWM, PID);                       //controla PWM de acordo com o PID (motor 2)
    analogWrite(pinMotor2PWM, PID);                       //controla PWM de acordo com o PID (motor 1)
    

    
} //end pid_control

void ciWrite(byte pino, bool estado) {
  static byte ciBuffer[qtdeCI];

  bitWrite(ciBuffer[pino / 8], pino % 8, estado);
  
  digitalWrite(pinST_CP, LOW); //Inicia a Transmissão
  
  digitalWrite(pinDS, LOW);    //Apaga Tudo para Preparar Transmissão
  
  digitalWrite(pinSH_CP, LOW);

  for (int nC = qtdeCI-1; nC >= 0; nC--) {
      for (int nB = 7; nB >= 0; nB--) {
          digitalWrite(pinSH_CP, LOW);  //Baixa o Clock      
          digitalWrite(pinDS,  bitRead(ciBuffer[nC], nB) );     //Escreve o BIT
          digitalWrite(pinSH_CP, HIGH); //Eleva o Clock
          digitalWrite(pinDS, LOW);     //Baixa o Data para Previnir Vazamento      
      }  
  }
  digitalWrite(pinST_CP, HIGH);  //Finaliza a Transmissão
}

/*
 * Projeto de Smarcar por Fabio Kimura
 * fkimura@gmail.com
 * 14/02/2020
 */
#include <Servo.h>

// -> D0=16
// -> D1=5
// -> D2=4
// -> D3=0
// -> D4=2
// -> D5=14
// -> D6=12
// -> D7=13
// -> D8=15
// -> D9=3
// -> D10=1
const int leftForward = D2; //5;
const int rightForward = D1; //4;

const int leftBackward = D4; //2; // D4
const int rightBackward = D3; //0; // D3


const int trigPin = D8; // isto poderia ir para i2c em um nano
const int echoPin = D7; // isto poderia ir para i2c em um nano
const int encoderEsqPin = D5; // isto poderia ir para i2c em um nano?
const int encoderDirPin = D6;// isto poderia ir para i2c em um nano? protocolo Wire.h com onReceive

const int servoPin = D0; 
const int servoDireita = 0;
const int servoMeiaDireita = 60;
const int servoEsquerda = 180;
const int servoMeiaEsquerda = 120;
const int servoFrente = 90;

volatile byte pulsosEsquerdo = 0;
volatile byte pulsosDireito = 0;
static volatile unsigned long debounceDir = 0;
static volatile unsigned long debounceEsq = 0;
void ICACHE_RAM_ATTR contadorEsquerdo();
void ICACHE_RAM_ATTR contadorDireito();

unsigned long timeDir = 0;
unsigned long timeEsq = 0;
unsigned int pulsesperturn = 20; // Número de furos do encoder.
const int wheel_diameter = 64;   // Diametro da roda [mm]

Servo servo;  // create servo object to control a servo

void setup() {

  // set control pins as Output
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input

  pinMode(encoderEsqPin, INPUT);
  pinMode(encoderDirPin, INPUT);

  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(servoFrente);
  attachInterrupt(encoderDirPin, contadorDireito, RISING);
  attachInterrupt(encoderEsqPin, contadorEsquerdo, RISING);
  delay(1000);
  Serial.println("comecando");
}

int contador = 0;
int distancia = 100;
const int DISTANCIA_MIN = 20;
const float VELOCIDADE_MIN = 40;
float velocidadeDir = 1000;
float velocidadeEsq = 1000;
int posicao = servoMeiaDireita;
boolean parado = true;

void loop() {
  distancia = lerDistancia();
  Serial.print("Distancia: ");
  Serial.println(distancia);
  if (parado) {
    frente();
    delay(100);
    distancia = lerDistancia();
    velocidadeDir = VELOCIDADE_MIN + 1;
    velocidadeEsq = VELOCIDADE_MIN + 1;
  } else {
    velocidadeDir = velocidadeDireita();
    velocidadeEsq = velocidadeEsquerda();
    Serial.println(velocidadeDir);
    Serial.println(velocidadeEsq);
  }
  if (distancia <= DISTANCIA_MIN || velocidadeDir <= VELOCIDADE_MIN || velocidadeEsq <= VELOCIDADE_MIN) {
    parar(500);
    if (velocidadeDir <= VELOCIDADE_MIN || velocidadeEsq <= VELOCIDADE_MIN) {
      tras(300);
      parar(0);
    }
    girarAteSair();
  } else {
    frente();
  }
  if (contador++ % 5 == 0) {
    oscilar();
  } else {
    delay(100);
  }
  if (contador > 100) {
    contador = 0;
    parar(300);
    girarAteSair();
  }
}

void oscilar() {
  servo.write(posicao);
  delay(250);
  distancia = lerDistancia();
  servo.write(servoFrente);
  if (distancia <= DISTANCIA_MIN * 2) {
    if (posicao == servoMeiaDireita) {
      virarEsquerda(10);
      frente();
    } else {
      virarDireita(10);
      frente();
    }
  }
  if (distancia <= DISTANCIA_MIN) {
    parar(300);
    girarAteSair();
  }
  if (posicao == servoMeiaDireita) {
    posicao = servoMeiaEsquerda;
  } else {
    posicao = servoMeiaDireita;
  }
}

void girarAteSair() {
  distancia = 0;
  while (distancia <= DISTANCIA_MIN) {
    girar();
    distancia = lerDistancia();
  }
  frente();
}


void girar() {
  int dir = lerDireita();
  int esq = lerEsquerda();
  if (esq > dir) {
    virarEsquerda(90);
    parar(100);
  } else {
    virarDireita(90);
    parar(100);
  }
}

int lerDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (pulseIn(echoPin, HIGH) / 58);
}

void girarAteSair2() {
  distancia = 0;
  while (distancia < DISTANCIA_MIN) {
    int grau = scan();
    if (grau > servoFrente) {
      virarEsquerda(grau - servoFrente);
    }
    if (grau < servoFrente) {
      virarDireita(servoFrente - grau);
    }
    distancia = lerDistancia();
    Serial.print("distancia=");
    Serial.println(distancia);
    if (distancia < DISTANCIA_MIN) {
      virarDireita(90);
    }
  }
}

int scan() {
  int distMaior = 0;
  int grau = 0;
  int d = 0;
  servo.write(servoDireita);
  delay(800);
  for (int i = servoDireita; i <= servoEsquerda; i = i + 5) {
    servo.write(i);
    delay(100);
    d = lerDistancia();
    if (d > distMaior) {
      distMaior = d;
      grau = i;
    }
  }
  servo.write(servoFrente);
  delay(500);
  Serial.print("grau=");
  Serial.println(grau);
  Serial.print("distancia=");
  Serial.println(distMaior);

  return grau;
}



int lerDireita() {
  servo.write(servoDireita);
  delay(500);
  int distancia = lerDistancia();
  servo.write(servoFrente);
  return distancia;
}

int lerEsquerda() {
  servo.write(servoEsquerda);
  delay(500);
  int distancia = lerDistancia();
  servo.write(servoFrente);
  return distancia;
}


float velocidadeEsquerda() {
  int rpm = (60 * 1000 / pulsesperturn ) / (millis() - timeEsq) * pulsosEsquerdo; // Calculamos las revoluciones por minuto
  float velocity = rpm * 3.1416 * wheel_diameter / 60; // Cálculo de la velocidad en [metros por s]
  timeEsq = millis(); // Almacenamos el tiempo actual.
  Serial.print("Esq: ");
  //  Serial.pring("segundos=");
  //  Serial.print(millis() / 1000); Serial.print("       "); // Se envia al puerto serie el valor de tiempo, de las rpm y los pulsos.
  //  Serial.print("rpm=");
  //  Serial.print(rpm, DEC); Serial.print("   ");
  Serial.print("pulsos=");
  Serial.print(pulsosEsquerdo, DEC); Serial.print("     ");
  Serial.print("velocidade=");
  Serial.println(velocity, 2);
  pulsosEsquerdo = 0;  // Inicializamos los pulsos.
  return velocity;
}
float velocidadeDireita() {
  int rpm = (60 * 1000 / pulsesperturn ) / (millis() - timeDir) * pulsosDireito; // Calculamos las revoluciones por minuto
  float velocity = rpm * 3.1416 * wheel_diameter / 60; // Cálculo de la velocidad en [Km/h]
  timeDir = millis(); // Almacenamos el tiempo actual.
  Serial.print("Dir: ");
  //  Serial.pring("segundos=");
  //  Serial.print(millis() / 1000); Serial.print("       "); // Se envia al puerto serie el valor de tiempo, de las rpm y los pulsos.
  //  Serial.print("rpm=");
  //  Serial.print(rpm, DEC); Serial.print("   ");
  Serial.print("pulsos=");
  Serial.print(pulsosDireito, DEC); Serial.print("     ");
  Serial.print("velocidade=");
  Serial.println(velocity, 2);
  pulsosDireito = 0;  // Inicializamos los pulsos.
  return velocity;
}


void contadorEsquerdo() {
  if (  digitalRead (encoderEsqPin) && (micros() - debounceEsq > 500)  ) {
    debounceEsq = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosEsquerdo++;
  }
}
void contadorDireito() {
  pulsosDireito++;
  if (  digitalRead (encoderDirPin) && (micros() - debounceDir > 500) ) {
    debounceDir = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
    pulsosDireito++;
  }
}



void frente() {
  Serial.println("frente");
  parado = false;
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);

  //  analogWrite(leftForward, 800);
  //  analogWrite(rightForward, 800);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);
}

void tras(int t) {
  Serial.println("para tras");
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, HIGH);
  delay(t);
}


void virarDireita(int t) {
  Serial.println("direita");
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, HIGH);
  delay(t);
}


void virarEsquerda(int t) {
  Serial.println("esquerda");
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  delay(t * 2);
}



void parar(int t) {
  parado = true;
  Serial.println("parar");
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(t * 2);
}

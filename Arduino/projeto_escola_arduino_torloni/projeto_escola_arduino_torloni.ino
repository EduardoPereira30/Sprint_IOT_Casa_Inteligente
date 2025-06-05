//bibliotecas
#include <DHT.h>
#include <DHT_U.h>
#include <DHT11.h>

#define DHTPINO A1 //definimos qual sera o pino
#define DHTTYPE DHT11 //variavel do tipo DHT

DHT dht( DHTPINO,DHT11);//pinMode

// variaveis da fução acerderLEDAoDETectarPresenca 
const int PIR = 2;  //pino digital que a PIR esta plugado
const int LEDV = 13;
//variaveis da fução verificarVazamentoDeGas
const int MQ135 = A0;
const int buzzer = 12;
void alarme_dois_tons() {
  int freqAlta = 2000;
  int freqBaixa = 800;
  int duracaoTom = 250;

  tone(buzzer, freqAlta, duracaoTom);
  delay(duracaoTom);
  tone(buzzer, freqBaixa, duracaoTom);
  delay(duracaoTom);
}
void verificarVazamentoDeGas() {
  int estadoMQ135 = analogRead(MQ135);

  //if  - veriaficar a intensidade de valor do estadoMQ135 >= 600
  //sim - ativar o alarme
  //não - desativar o alarme

  if (estadoMQ135 >= 600){
  alarme_dois_tons();

  } else {
  noTone(buzzer);
  }
}
void acerderLEDAoDETectarPresenca() {
  int estadoPir = digitalRead(PIR);  // le o pino digital 2
  //(Atalho) - Shift + Alt + F
  //HIGH  - valor 1
  //LOW - valor 0
  if (estadoPir == HIGH) {
    Serial.println("LED ligado");
    digitalWrite(LEDV, HIGH);

  } else {
    Serial.println("LED desligado");
    digitalWrite(LEDV, LOW);}
}
void verificarTemperaturaEUmiddade(){
 float umidade = dht.readHumidity(); //le a umidade
 float temperatura = dht.readTemperature();//le a temperatura

 Serial.println("umiddade" + String(umidade) + "%");
 Serial.println("temperatura" + String(temperatura) + "C");
}
void setup() {
  pinMode(buzzer,OUTPUT);
  pinMode(MQ135, INPUT);
  pinMode(LEDV, OUTPUT);

  //Inicializar o sensor DHT
  dht.begin();

  Serial.begin(9600);
  Serial.println(" Calibrado os sensores ");
  delay(1000);
  Serial.println(" Sesores calibrados !!! ");
}
void loop() {
  // as instruções no loop sera somente de função
  verificarVazamentoDeGas();
  acerderLEDAoDETectarPresenca();
}
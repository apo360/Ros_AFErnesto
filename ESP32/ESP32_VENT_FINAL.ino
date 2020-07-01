/*---------------------------------------------*/
#define MASTER
/*---------------------------------------------*/
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>              //protocolo SPI
#include <LoRa.h>             //comms chip SX1276 - LoRa (433MHz)
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*-----------------------------------------------------------------*/
#define BATTERY_PIN 35
#define INIT_C 0x2B           //código de início de mensagem
#define TIMEOUT 2000          //prevenção de bloqueio da rotina
#define MAX_MSG_LENGTH 128

#define REPLY_GPS 6
#define REPLY_IMU 7
#define REPLY_GPS_SATELITES 8
#define REPLY_LoRa 9

#define SCK 5
#define MISO 19
#define MOSI 27             //pins de ligação SPI SX1276
#define NSS 18
#define RST 14
#define DI0 26

#define BAND 433E6          //banda de frequência -> 433 MHz
#define PABOOST true        //escolha do amplificador de potência //(chip sx1276 possui dois amplificadores -> 20 ou 14 dBm)
/*-----------------------------------------------------------------*/
unsigned long t0;
bool checkSumXOR();
uint8_t calculateCheckSumXOR(const uint8_t* data, uint16_t n, uint8_t chkSum = 0);
uint8_t replyMessage[MAX_MSG_LENGTH];
uint8_t message[MAX_MSG_LENGTH];
uint8_t dataSize, dataCount;
/*----------------------------------------------------------------*/
const String GETDATA = "getdata"; //frases identificativas de
const String SETDATA = "setdata"; //pedido de transmissão de msg
/*----------------------------------------------------------------*/
long lastSendTime = 0;
/*----------------------------------------------------------------*/

TinyGPSPlus gps;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool status_lora = true, status_imu = true;

void setup() {
  Serial.begin(9600, SERIAL_8N1, 12, 15);  // Iniciar GPS
  Serial2.begin(9600, SERIAL_8N1, TX, RX); // Serial.begin(115200, SERIAL_8N1, RX, TX);
  SPI.begin(SCK, MISO, MOSI, NSS);
  LoRa.setPins(NSS, RST, DI0);
  if (!LoRa.begin(BAND)) {status_lora = false;}
  if (!bno.begin()){status_imu = false;}
  bno.setExtCrystalUse(true);
  t0 = millis(); //inicialização de todos os componentes
  replyMessage[0] = INIT_C;

// Iniciar o pin de medição da batéria
  adcAttachPin(BATTERY_PIN);
  adcStart(BATTERY_PIN);
  analogReadResolution(10);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
// ### Fim Pin Battery ### //
}

void loop() {
  
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) 
  {
    while (LoRa.available()) 
    {
      String Formato_lora = "$Lora:" + LoRa.readString() + '\0';
      
      message_struct(Formato_lora, REPLY_LoRa);
    }
    
  }
  else if(!packetSize)
  {
    if(Serial2.available()){
      
      String pedido = Serial2.readString();
  
      if(strcmp(pedido.substring(0, 3).c_str(),"RPY") == 0)
      {
        readData();
      } 
      else if (pedido.substring(0, 4) == "LoRa")
      {
        LoRa.beginPacket();
        LoRa.println(pedido);
        LoRa.endPacket();
      }
      else if (strcmp(pedido.c_str(),"restart") == 0)
      {
        delay(5000);
        ESP.restart();
      }
    }
  }
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}

bool checkSumXOR() {
return (calculateCheckSumXOR(message, 3 + dataSize) == message[3 + dataSize]);
}

uint8_t calculateCheckSumXOR(const uint8_t* data, uint16_t n, uint8_t chkSum) {
  while (n--)
chkSum ^= *data++; //cálculo do checksum
return chkSum;
}

void sendReplyMessage() {
  replyMessage[2 + replyMessage[1]] = calculateCheckSumXOR(replyMessage, 2 + replyMessage[1]);
  replyMessage[3 + replyMessage[1]] = '\n';
  Serial2.write(replyMessage, 4 + replyMessage[1]);
  //Serial2.flush();
}

void message_struct(String FinalValue, uint8_t REPLAY){
  int replySize1 = FinalValue.length(); // Tamanho da mensagem
  //replyMessage[2] = REPLAY;
  replyMessage[1] = replySize1; 
  for (int i = 0; i < replySize1; i++) 
        replyMessage[i + 3] = FinalValue.charAt(i);

  sendReplyMessage();
}

void Hardeware_Status(void){
  
  String estado, lora_estado, imu_estado;
  sensor_t sensor;
  bno.getSensor(&sensor);

  String battery = "Battery Voltage :" + String(analogRead(BATTERY_PIN)*2.0*(3.3/1024.0));
  
  imu_estado = String(sensor.name)+ ' ' + String(bno.getTemp()) + " Conectado";

  if(status_lora){lora_estado =("LoRa-" + String(BAND)+ ' ' + " Conectado");}
  else{lora_estado = ("LoRa-" + String(BAND)+ ' ' + " Desconectado");}

  estado = "$Hardware:" + imu_estado + ','+ lora_estado +','+battery+','+ '\n';

  message_struct(estado, REPLY_GPS_SATELITES);
}

void Imu_Status(void){
  
  sensors_event_t orient_, gravity_, gyroscope_, linearAccel_, magnetometer_;
  
  bno.getEvent(&orient_, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&gravity_, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&gyroscope_, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccel_, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometer_, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  String imuOrient_ = String(orient_.orientation.x) + ' ' + String(orient_.orientation.y) + ' ' + String(orient_.orientation.z);
  String imuGravity_ = String(gravity_.orientation.x) + ' ' + String(gravity_.orientation.y) + ' ' + String(gravity_.orientation.z);
  String imuGyroscope_ = String(gyroscope_.orientation.x) + ' ' + String(gyroscope_.orientation.y) + ' ' + String(gyroscope_.orientation.z);
  String imuLinearccel_ = String(linearAccel_.orientation.x) + ' ' + String(linearAccel_.orientation.y) + ' ' + String(linearAccel_.orientation.z);
  String imuMagnetometer_ = String(magnetometer_.orientation.x) + ' ' + String(magnetometer_.orientation.y) + ' ' + String(magnetometer_.orientation.z);

  String imu_ = "$HCHDT:" + imuOrient_ + ',' + imuGravity_ + ',' + imuGyroscope_ + ',' + imuLinearccel_ + ',' + imuMagnetometer_ + ',' + '\0';

  message_struct(imu_, REPLY_IMU);
}

void Gps_Status(void){
  
  //leitura da latitude e transformação em formato NMEA
  float latitude = gps.location.lat();
  if (latitude < 0) latitude = latitude * -1;

  //leitura da longitude e transformação em formato NMEA
  float longitude = gps.location.lng();
  if (longitude < 0) longitude = longitude * -1;

  String UTC = String((gps.time.value())/100) + "Z"; //Formato Grupo data e Hora
  
  String equador = (gps.location.rawLng().negative ? "N" : "S");
  
  String meridiano = (gps.location.rawLng().negative ? "W" : "E");
  
  double sog = (gps.speed.knots()); //leitura da velocidade em nós
  
  double cog = (gps.course.deg()); // Leitura do rumo
  
  String ddate = String(gps.date.month()) + F("/") + String(gps.date.day()) + F("/") + String(gps.date.year()); //leitura da hora
  
  String satelites = String(gps.satellites.value()); // leitura de satélites

  String gps_ = "$GPGGA:" + UTC + ',' + String(latitude,5) + ' ' + String(equador) + ',' + String(longitude, 5) + ' ' + String(meridiano) + ',' + String(sog) + ',' + String(cog) + ',' + String(ddate) + ',' + satelites +',' +'\0';

  message_struct(gps_, REPLY_GPS);

}

void readData(){

  Hardeware_Status();
  Imu_Status();
  Gps_Status();
  smartDelay(50);
}

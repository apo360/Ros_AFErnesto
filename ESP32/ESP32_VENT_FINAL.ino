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
/*---------------------------------------------------------------*/

TinyGPSPlus gps;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool status_lora = true, status_imu = true, status_gps = true;

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

  if(Serial2.available()){

    String pedido = Serial2.readString();

    if(pedido.substring(0, 3) == "RPY")
    {
      readData();
    } 
    else if (pedido.substring(0, 4) == "LoRa")
    {
      LoRa.beginPacket();
      LoRa.println(pedido);
      LoRa.endPacket();
    }
    else {Serial2.println("Erro na entrada de Pedidos");}
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
  
  String estado, lora_estado;

  if(status_lora){lora_estado =("LoRa " + String(BAND)+ ' ' + String(bno.getTemp()) + " Conectado");}
  else{lora_estado = ("LoRa " + String(BAND)+ ' ' + String(bno.getTemp()) + " Desconectado");}

  estado = "$1:" + ','+ lora_estado + '\n';

  message_struct(estado, REPLY_GPS_SATELITES);
}

void Imu_Status(void){

  double x, y, z; //dumb values, easy to spot problem
  String imu_estado;
  
  sensors_event_t orient_;
  sensor_t sensor;
  
  bno.getSensor(&sensor);
  bno.getEvent(&orient_, Adafruit_BNO055::VECTOR_EULER);

  if(status_imu){ imu_estado = (String(sensor.name)+ ' ' + String(bno.getTemp()) + " Conectado");}
  else{imu_estado = (String(sensor.name)+ ' ' + String(bno.getTemp()) + " Desconectado");}

    x = orient_.orientation.x;
    y = orient_.orientation.y;
    z = orient_.orientation.z;

    String imu_ = "$HCHDT:" + String(x) + ' ' + String(y) + ' ' + String(z) + ' ' + imu_estado + '\0';

  message_struct(imu_, REPLY_IMU);
}

void Gps_Status(void){

  String gps_estado;
  
  //leitura da latitude e transformação em formato NMEA
  float latitude = gps.location.lat();
  
  if (latitude < 0) latitude = latitude * -1;
  
  int latitudeD = floor(latitude);
  
  float latitudeDM = ((latitude - latitudeD) * 60);

  //leitura da longitude e transformação em formato NMEA
  float longitude = gps.location.lng();
  
  if (longitude < 0) longitude = longitude * -1;
  
  int longitudeD = floor(longitude);
  int longitudeD0 = longitudeD * 10;
  float longitudeDM = ((longitude - longitudeD) * 60);

  String UTC = String((gps.time.value())/100) + "Z"; //Formato Grupo data e Hora
  String equador = (gps.location.rawLng().negative ? "N" : "S");
  String meridiano = (gps.location.rawLng().negative ? "W" : "E");
  double sog = (gps.speed.knots()); //leitura da velocidade em nós
  double cog = (gps.course.deg()); // Leitura do rumo
  String ddate = String(gps.date.value()); //leitura da hora
  
  static const double GS_LAT = 38.6631, GS_LON = -9.146906;
  
  double distanceToGS = TinyGPSPlus::distanceBetween(
                          gps.location.lat(),
                          gps.location.lng(),
                          GS_LAT,
                          GS_LON);

  if (millis() > 5000 && gps.charsProcessed() > 10){gps_estado = String(gps.satellites.value())+ ' ' + "Conectado";}
  else{gps_estado = String(gps.satellites.value()) + ',' + "Conectado";}

  String gps_ = "$GPGGA:" + UTC + ',' + String(latitudeD) + String(latitudeDM,5) + ' ' + String(equador) + ',' +
  String(longitudeD0) + String(longitudeDM, 5) + ' ' + String(meridiano) + ',' + String(sog) + ',' + 
  String(cog) + ',' + String(ddate) + ',' + gps_estado + '\0';

  message_struct(gps_, REPLY_GPS);
}

void readData(){

  //Hardeware_Status();
  Imu_Status();
  Gps_Status();
  smartDelay(500);
}

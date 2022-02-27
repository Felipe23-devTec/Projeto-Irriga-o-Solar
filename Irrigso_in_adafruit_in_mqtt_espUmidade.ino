#include <PubSubClient.h>
#include <DHT.h>


//.......Inclusão das bibliotecas..........
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//.......Conexão com a internet...........

#define WIFI_SSID       "FELIPE" // nome de sua rede wifi
#define WIFI_PASS       "05101520"     // senha de sua rede wifi


//........Credenciais da adafruit io...........
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Irrigso1" // Seu usuario cadastrado na plataforma da Adafruit
#define AIO_KEY         "aio_VXhz69ktQZ97dA26HqVBvu3t4e1Z"       // Sua key da dashboard

//...........Variáveis globais..............
WiFiClient client;
WiFiClient wifiClient;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

long previousMillis = 0;
int pinAnalogico = A0;

//................Declaração dos Feeds..........
//feed responsavel por receber os dados da nossa dashboard 
//Adafruit_MQTT_Subscribe _rele1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/rele1", MQTT_QOS_1);
Adafruit_MQTT_Publish _umidade_solo = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade_do_solo", MQTT_QOS_1);

Adafruit_MQTT_Publish _temperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura", MQTT_QOS_1);
Adafruit_MQTT_Publish _umidade_Ar = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade_Ar", MQTT_QOS_1);


void initSerial();
void initPins();
void initWiFi();
void conectar_broker();

//MQTT Server
const char* BROKER_MQTT = "broker.mqtt-dashboard.com";
int BROKER_PORT = 1883;   
#define ID_MQTT  "SDHS2001" 
#define TOPIC_PUBLISH "SDHSensor120"
PubSubClient MQTT(wifiClient);
void mantemConexoes();       
void conectaMQTT();    
void enviaPacote(); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initSerial();
  initPins();
  initWiFi();
  dht.begin();
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
}

void loop() {
  // put your main code here, to run repeatedly:
  conectar_broker();
  mqtt.processPackets(5000);
  mantemConexoes();
  // Função responsável por ler e enviar o valor do sensor a cada 5 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 5000 && mqtt.connected()) {
    previousMillis = currentMillis;
    float temperatura = dht.readTemperature();
    float humidadeAr = dht.readHumidity();
    Serial.println(temperatura);
    Serial.println(humidadeAr);
    delay(200);
    int x = analogRead(pinAnalogico);
    int valUmidade = map(x,0,1023,0,100);
    delay(200);
    enviaValores(valUmidade);
    MQTT.loop();
    Serial.print("Temp: "); 
    Serial.print(valUmidade); 
    Serial.println("*Ct");
    if (! _umidade_solo.publish(valUmidade)) {
      Serial.println("Falha ao enviar o valor do sensor de umidade do solo.");
    }else{
      Serial.println("Envio do valor da umidade do solo com sucesso!");
    }
    
    if (! _temperatura.publish(temperatura)) {
      Serial.println("Falha ao enviar o valor da temperatura.");
    }else{
      Serial.println("Envio do valor da temperatura com sucesso!");
    }
    
    if (! _umidade_Ar.publish(humidadeAr)) {
      Serial.println("Falha ao enviar o valor da umidade do ar.");
    }else{
      Serial.println("Envio do valor da umidade do ar com sucesso!");
    } 
  }
}
void initSerial() {
  Serial.begin(115200);
  delay(10);
}
void initPins() {
  pinMode(pinAnalogico,INPUT);
}
void initWiFi() {
  Serial.print("Conectando-se na rede "); 
  Serial.println(WIFI_SSID);
 
  WiFi.begin(WIFI_SSID, WIFI_PASS);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
 
  Serial.println("Conectado à rede com sucesso"); 
  Serial.println("Endereço IP: "); 
  Serial.println(WiFi.localIP());
}
void conectar_broker() {
  int8_t ret;
 
  if (mqtt.connected()) {
    return;
  }
 
  Serial.println("Conectando-se ao broker mqtt...");
 
  uint8_t num_tentativas = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Falha ao se conectar. Tentando se reconectar em 5 segundos.");
    mqtt.disconnect();
    delay(5000);
    num_tentativas--;
    if (num_tentativas == 0) {
      Serial.println("Seu ESP será resetado.");
      while (1);
    }
  }
  Serial.println("Conectado ao broker com sucesso.");
}
void mantemConexoes() {
  if (!MQTT.connected()) {
     conectaMQTT(); 
  }
    
  initWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}
void enviaValores(int valUmidade) {
  long previousMillis = 0;
  delay(200);
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousMillis > 5000){
     previousMillis = currentMillis;
     if (valUmidade >= 70) {   
        MQTT.publish(TOPIC_PUBLISH, "1");
        Serial.println("Umidade baixa. Payload enviado.");
     } else {
        MQTT.publish(TOPIC_PUBLISH, "0");
        Serial.println("Umidade alta. Payload enviado.");
     }
     
  }
  delay(200);
}
void conectaMQTT() { 
  while (!MQTT.connected()) {
     Serial.print("Conectando ao Broker MQTT: ");
     Serial.println(BROKER_MQTT);
     if (MQTT.connect(ID_MQTT)) {
        Serial.println("Conectado ao Broker com sucesso!");
     } 
     else {
        Serial.println("Noo foi possivel se conectar ao broker.");
        Serial.println("Nova tentatica de conexao em 10s");
        delay(10000);
     }
  }
}

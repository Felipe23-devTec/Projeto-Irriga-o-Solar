 #include <ESP8266WiFi.h> 
#include <PubSubClient.h>

#define pinRELE1 5//D1

//WiFi
const char* SSID = "FELIPE";                
const char* PASSWORD = "05101520";   
WiFiClient wifiClient;                        
 
//MQTT Server
const char* BROKER_MQTT = "broker.mqtt-dashboard.com"; 
int BROKER_PORT = 1883;                      

#define ID_MQTT  "SDHS2002"             
#define TOPIC_SUBSCRIBE "SDHSensor120"   
PubSubClient MQTT(wifiClient);        

//Declaração das Funções
void mantemConexoes(); 
void conectaWiFi();  
void conectaMQTT();     
void recebePacote(char* topic, byte* payload, unsigned int length);

void setup() {
  pinMode(pinRELE1, OUTPUT);         

  Serial.begin(115200);

  conectaWiFi();
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);  
  MQTT.setCallback(recebePacote); 
}

void loop() {
  mantemConexoes();
  MQTT.loop();
}

void mantemConexoes() {
    if (!MQTT.connected()) {
       conectaMQTT(); 
    }
    
    conectaWiFi(); 
}

void conectaWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
     return;
  }
        
  Serial.print("Conectando-se na rede: ");
  Serial.print(SSID);
  Serial.println("  Aguarde!");

  WiFi.begin(SSID, PASSWORD);  
  while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Conectado com sucesso, na rede: ");
  Serial.print(SSID);  
  Serial.print("  IP obtido: ");
  Serial.println(WiFi.localIP()); 
}

void conectaMQTT() { 
    while (!MQTT.connected()) {
        Serial.print("Conectando ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) {
            Serial.println("Conectado ao Broker com sucesso!");
            MQTT.subscribe(TOPIC_SUBSCRIBE);
        } 
        else {
            Serial.println("Noo foi possivel se conectar ao broker.");
            Serial.println("Nova tentatica de conexao em 10s");
            delay(10000);
        }
    }
}

void recebePacote(char* topic, byte* payload, unsigned int length) 
{
    String msg;

    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload[i];
       msg += c;
    }

    if (msg == "0") {
       digitalWrite(pinRELE1, LOW);
       Serial.print("Mensagem recebida.Rele desligado!");
    }

    if (msg == "1") {
       digitalWrite(pinRELE1, HIGH);
       Serial.print("Mensagem recebida.Rele ligado!");
    }
}

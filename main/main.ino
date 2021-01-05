#include <SparkFun_SCD30_Arduino_Library.h>
#include <Wire.h>
#include <pins_arduino.h>

// Светодиод подключен к 5 пину
// Датчик температуры ds18b20 к 2 пину

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// resistance other resistor for divider (Ohm)
#define RES_DIVIDER         10000.0
#define MULT_VALUE          79838286//32017200.0
#define POW_VALUE           1.5832
#define SEND_INTERVAL       5

#define SAMPLE_TIMES        32
#define ADC_BIT             10
#define ADC_VALUE_MAX       pow(2, ADC_BIT)
#include <math.h>


//#include <DallasTemperature.h>
//
//#define ONE_WIRE_BUS 2
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

const char *ssid =  "KODE";  // Имя вайфай точки доступа
const char *pass =  "a1b2c3d4e5"; // Пароль от точки доступа

const char *mqtt_server = "mqtt.lividash.kode-t.ru"; // Имя сервера MQTT
const int  mqtt_port = 1884; // Порт для подключения к серверу MQTT
const char *mqtt_user = "device_light_noise"; // Логин от сервер
const char *mqtt_pass = "d6Vr3VdxJVZ36Bnf"; // Пароль от сервера

#define BUFFER_SIZE 100
#define ADC_SEL 15

int tm = SEND_INTERVAL;
float light_lux, temp, humid = 0;
int noise, co2 = 0;

// Создаем объект СО2 сенсора
SCD30 airSensor;

// Функция получения данных от сервера

void callback(const MQTT::Publish& pub)
{
  Serial.print(pub.topic());   // выводим в сериал порт название топика
  Serial.print(" => ");
  Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных
  
  String payload = pub.payload_string();
  
  if(String(pub.topic()) == "test/led") // проверяем из нужного ли нам топика пришли данные 
  {
  int stled = payload.toInt(); // преобразуем полученные данные в тип integer
  digitalWrite(5,stled);  //  включаем или выключаем светодиод в зависимоти от полученных значений данных
  }
}



WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

void setup() {
  
  Serial.begin(115200);
  
//  Wire.begin();
//
//  if (airSensor.begin() == false)
//  {
//    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
//    while (1)
//      ;
//  }
  
  delay(10);
  Serial.println();
  Serial.println();
  pinMode(5, OUTPUT);
  pinMode(ADC_SEL, OUTPUT);
  
}

void selectAdc (int channel){
  for (uint8_t i=0; i < channel; i++){
    digitalWrite(ADC_SEL, HIGH);
    delayMicroseconds(10);
    digitalWrite(ADC_SEL, LOW);
    delayMicroseconds(10);
    digitalWrite(ADC_SEL, HIGH);
    delayMicroseconds(10);
    digitalWrite(ADC_SEL, LOW);
    delayMicroseconds(10);
  }
}

void resetMux (void) {
  
  uint16_t resetDelay = 50;

  digitalWrite(ADC_SEL, HIGH);
  delay(resetDelay);
  //digitalWrite(ADC_SEL, LOW);
  //delay(resetDelay);  
}

float getLux (void){
    int sensorADC = 0;
    float sensorRatio = 0;
    float sensorResistance = 0;
    //for (int i = 0; i < SAMPLE_TIMES; i++) {
        sensorADC += analogRead(A0);
    //}
    //sensorADC = sensorADC >> 5;
    sensorRatio = (float)ADC_VALUE_MAX / (float)sensorADC - 1.0;
    sensorResistance  = RES_DIVIDER / sensorRatio;
    
    return MULT_VALUE / (float)pow(sensorResistance, POW_VALUE);
}

int getNoiseDb (void){
    int sensorADC = 0;
    
    //for (int i = 0; i < SAMPLE_TIMES; i++) {
        sensorADC += analogRead(A0);
    //}
    //sensorADC = sensorADC >> 5;
   
    
    return 20*log10(sensorADC);
}

void loop() {
  
  // подключаемся к wi-fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect("arduinoClient2")
                         .set_auth(mqtt_user, mqtt_pass))) {
        Serial.println("Connected to MQTT server");
        client.set_callback(callback);
        client.subscribe("test/led"); // подписывааемся по топик с данными для светодиода
      } else {
        Serial.println("Could not connect to MQTT server");   
      }
    }

    if (client.connected()){
      client.loop();
      TempSend();
    }
  }
  
  // Сброс мультиплексора
  resetMux();
  // считывание данных с датчика освещённости
  selectAdc(4);

  light_lux = getLux();
  // вывод показателей сенсора освещённости в люксах

  // Сброс мультиплексора
  delay(500);
  // считывание данных с датчика шума
  selectAdc(2);

  noise = getNoiseDb();

//  if (airSensor.dataAvailable())
//  {
//    co2 = airSensor.getCO2(); 
//    Serial.print("co2(ppm):");
//    Serial.print(co2);
//
//    temp = airSensor.getTemperature();
//    Serial.print(" temp(C):");
//    Serial.print(temp, 1);
//
//    humid = airSensor.getHumidity();
//    Serial.print(" humidity(%):");
//    Serial.print(humid, 1);
//
//    Serial.println();
//  }
//  else
//    Serial.println("Waiting for new data");

  
  delay(500);
  
} // конец основного цикла


// Функция отправки показаний с термодатчика
void TempSend(){
  if (tm==0)
  {
    //sensors.requestTemperatures();   // от датчика получаем значение температуры
    char buf[100];
    sprintf(buf,"{\"ID\":\"A0B1C2D3E4F5\",\"Lux\":\"%4.2f\",\"Noise\":\"%d\",\"CO2\":\"%d\",\"Temperature\":\"%4.2f\",\"Humidity\":\"%4.2f\"}",light_lux,noise,co2,temp,humid);
    client.publish("v1/devices/light_noise",buf); // отправляем в топик значение освещенности
    Serial.println("Sent data!");
    Serial.print(light_lux);
    Serial.print("  ");
    Serial.println(noise);
//    Serial.print("  ");
//    Serial.print(co2);
//    Serial.print("  ");
//    Serial.print(temp);
//    Serial.print("  ");
//    Serial.print(humid);

    tm = SEND_INTERVAL;  // пауза между отправками значений температуры  коло 3 секунд
  }
  tm--; 
  delay(10); 
}

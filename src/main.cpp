#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>
const uint8_t wire_bus = 1;
OneWire oneWire(wire_bus);
DallasTemperature DS18B20(&oneWire);

const uint8_t pinBMEminus = 3; //  питание датчика -
const uint8_t pinBMEplus = 19; //  питание датчика +
const uint8_t pinVcc = 4;      //  напряжение батареи

WiFiClient espClient;
PubSubClient client(espClient);

//------------------------------------------------
const char *ssid = "link";
// const char *ssid = "MikroTik-2";
const char *pass = "dkfgf#*12091997";

// const char *mqtt_client = "mqtt_esp32_c3-003";
const char *mqtt_client = "mqtt_esp32_c3-369929";

// const char *mqtt_client2 = "esp32_c3-test-3";
// const char *mqtt_client2 = "Villa_bme280_base";
// const char *mqtt_client2 = "Villa_bme280_yama";
const char *mqtt_client2 = "Home_bme280";
// const char *mqtt_client2 = "esp32c3-балкон";

const char *mqtt_user = "mqtt";
const char *mqtt_pass = "qwe#*1243";
const char *mqtt_server = "178.20.46.157";
const uint16_t mqtt_port = 1883;

const char *outTopicTemp = "/TempOut";
const char *outTopicVcc = "/VccOut";

//------------------------------------------------
uint8_t countConnect = 20;        //  кол-во попыток соединения
uint16_t countPause = 500;        //  пауза между попытками
uint32_t timeSleep = 10000000;    //  время сна
uint16_t TimeBeforeBedtime = 500; //  время до засыпания

RTC_DATA_ATTR struct
{
  float t = 0;    //  температура
  float v = 0;    // напряжение
} data;

float diffTemp = 0.25; //  разница температур
float diffVcc = 0.1;   //  разница зарядки акб

RTC_DATA_ATTR bool flagNotWork = false;

//-----------------------------------
inline bool mqtt_subscribe(PubSubClient &client, const String &topic)
{
  Serial.print("Subscribing to: ");
  Serial.println(topic);
  return client.subscribe(topic.c_str());
}

//-----------------------------------
inline bool mqtt_publish(PubSubClient &client, const String &topic, const String &value)
{
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(value);
  return client.publish(topic.c_str(), value.c_str());
}

//-----------------------------------
void mqttDataOut(float temp, float vcc)
{
  String topic = "/";
  topic += mqtt_client2;
  topic += outTopicTemp;
  while (!mqtt_publish(client, topic, (String)temp))
  {
    mqtt_publish(client, topic, (String)temp);
  }

  topic = "/";
  topic += mqtt_client2;
  topic += outTopicVcc;
  while (!mqtt_publish(client, topic, (String)vcc))
  {
    mqtt_publish(client, topic, (String)vcc);
  }
}

//-----------------------------------
bool reconnect()
{
  client.setServer(mqtt_server, mqtt_port);

  Serial.print("MQTT connect : ");
  Serial.println(mqtt_server);

  while (!(client.connect(mqtt_client, mqtt_user, mqtt_pass)) && countConnect--)
  {
    Serial.print(countConnect);
    Serial.print('>');
    delay(countPause);
  }

  if (client.connected())
  {
    Serial.println("MQTT connected - OK !");
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------
bool setupWiFi(const char *wifi_ssid, const char *wifi_pass)
{
  WiFi.begin(wifi_ssid, wifi_pass);

  Serial.print("Setup WiFi: ");
  Serial.println(ssid);

  while ((WiFi.status() != WL_CONNECTED) && countConnect--)
  {
    Serial.print(countConnect);
    Serial.print('>');
    delay(countPause);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    // индикация IP
    Serial.print("\nWiFi connected - OK !\n");
    Serial.println(WiFi.localIP());
    // индикация силы сигнала
    int8_t dBm = WiFi.RSSI();
    Serial.print("RSSI dBm = ");
    Serial.println(dBm);
    uint8_t quality_RSSI = 2 * (dBm + 100);
    if (quality_RSSI >= 100)
      quality_RSSI = 100;
    Serial.print("RSSI % = ");
    Serial.println(quality_RSSI);
    Serial.println("=================");
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------
// медиана на 3 значения со своим буфером
float medianRoom(float newValRoom)
{
  RTC_DATA_ATTR static float bufRoom[3] = {0, 0, 0};
  RTC_DATA_ATTR static byte countRoom = 0;

  Serial.println();
  Serial.print(countRoom);
  Serial.print(" - ");
  Serial.print(bufRoom[0]);
  Serial.print("...");
  Serial.print(bufRoom[1]);
  Serial.print("...");
  Serial.print(bufRoom[2]);
  Serial.println();

  if (!(bufRoom[0] + bufRoom[1] + bufRoom[2]))
    bufRoom[0] = bufRoom[1] = bufRoom[2] = newValRoom;

  bufRoom[countRoom] = newValRoom;
  if (countRoom++ >= 2)
    countRoom = 0;
  float dataRoom = (max(bufRoom[0], bufRoom[1]) == max(bufRoom[1], bufRoom[2]))
                          ? max(bufRoom[0], bufRoom[2])
                          : max(bufRoom[1], min(bufRoom[0], bufRoom[2]));
  // return expRunningAverage(data);
  return dataRoom;
}

//-----------------------------------
bool readData()
{
  DS18B20.begin();               // Инициализация DS18B20
  DS18B20.requestTemperatures(); // Запрос на считывание температуры

  float temp = DS18B20.getTempCByIndex(0);
  float vcc = 0;
  Serial.println("\n=================");

  uint16_t dataVcc = 0;
  for (uint8_t i = 0; i < 3; ++i)
  {
    uint16_t temp = analogRead(pinVcc);
    Serial.print(temp);
    if (i < 2)
      Serial.print("...");
    dataVcc += temp;
  }
  Serial.print(" = ");
  Serial.print(dataVcc / 3);
  vcc = 2.8 / 4095 * medianRoom(dataVcc / 3) * 3.6 / 2.8;
  Serial.println();

  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print("Vcc = ");
  Serial.print(vcc);
  Serial.println(" v");

  Serial.println("=================\n");

  if (abs(data.t - temp) >= diffTemp || abs(data.v - vcc) >= diffVcc || flagNotWork)
  {
    data.t = temp;
    data.v = vcc;

    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------
void setup()
{
  Serial.begin(115200);
  pinMode(pinBMEplus, OUTPUT);
  digitalWrite(pinBMEplus, HIGH);
  pinMode(pinBMEminus, OUTPUT);
  digitalWrite(pinBMEminus, LOW);
  delay(20);

  if (!readData())
  {
    esp_deep_sleep(timeSleep);
  }

  if (!setupWiFi(ssid, pass))
  {
    flagNotWork = true;
    esp_deep_sleep(timeSleep);
  }
  else
  {
    flagNotWork = false;
  }

  if (!reconnect())
  {
    flagNotWork = true;
    esp_deep_sleep(timeSleep);
  }
  else
  {
    flagNotWork = false;
  }

  mqttDataOut(data.t, data.v);

  Serial.println("=================");
  Serial.flush();

  delay(TimeBeforeBedtime);
  esp_deep_sleep(timeSleep);
}

void loop() {}
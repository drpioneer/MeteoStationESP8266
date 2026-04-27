/*--------------------------------------------------------------------------------------------
 *  Метеостанция на базе ESP8266 ( LoLin / WeMos D1 mini )
 *  http://arduino.ru/forum/proekty/meteostantsiya-dlya-narodnogo-monitoringa#comment-551457
 *  Станция собирает информацию от датчиков и передает информацию на сервер нородного мониторинга
 *  При первом запуске или отсутствии сети создается точка доступа на 5 секунд (нужно успеть
 *  подключиться). В WEB-интерфейсе можно просканировать сети и подключится к имеющейся либо
 *  вписать название сети вручную, после чего на сайт начинают отправлятся данные, а в SERIAL
 *  выводится уникальный ID станции, который потом привяжется к сайту. 
 *  Дополнительно станция отсылает на сервер информацию о напряжении питания контроллера (VCC) и уровне сигнала Wifi.
 *  Все показания и статистику на графиках можно смотреть в приложениях под разные платформы, в том числе Android, IOS.
 *-------------------------------------------------------------------------------------------- */
#include <FS.h>
#include "ESP8266WiFi.h"
#include "DNSServer.h"
#include "ESP8266WebServer.h"
#include "DallasTemperature.h"
#include "WiFiManager.h"                                    // источник библиотеки https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include "Adafruit_BMP280.h"                                // библиотека для работы с датчиком BMP280
#include "Adafruit_AHTX0.h"                                 // библиотека для работы с датчиком AHT20

#define PIN_1WIRE_VCC             D6                        // user defined 1WIRE sensor VCC-pin for DS18B20
#define PIN_1WIRE_DAT             D7                        // user defined 1WIRE sensor DAT-pin for DS18B20
#define PIN_1WIRE_GND             D8                        // user defined 1WIRE sensor GND-pin for DS18B20 (!!!для старта МК уровень на этом PINе должен быть LOW!!!)

#define PIN_I2C_SDA               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280 // датчик припаяный
#define PIN_I2C_CLK               D4                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280
#define PIN_I2C_GND               D3                        // user defined I2C sensor GND-pin for AHT20&BMP280

/*
#define PIN_I2C_SDA               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280 // датчик на проводе
#define PIN_I2C_CLK               D3                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280
#define PIN_I2C_GND               D4                        // user defined I2C sensor GND-pin for AHT20&BMP280
*/

#define COM_PORT_SPEED            115200                    // скорость COM-порта отладочного терминала
#define POSTING_INTERVAL          300000                    // интервал между отправками данных в миллисекундах (5 минут)
#define SLEEP_TIME                600e6                     // время сна 600*10^6 мс = 10мин
#define ATTEMPTS                  10                        // количество попыток инициализации датчиков
#define TEMPERATURE_PRECISION     10                        // точность бит для DS18B20. Если глючит или врет -> уменьшить до 9
#define HUMIDITY_CORREСTION       0                         // поправка в процентах для корявого датчика влажности AHT20 (если есть понимание на сколько нужно подкрутить)

ADC_MODE (ADC_VCC);                                         // настройка АЦП на измерение напряжения питания микроконтроллера
OneWire                           oneWire(PIN_1WIRE_DAT);
DallasTemperature                 ds18b20(&oneWire);        // объявление объекта для работы с DS18B20 с использованием библиотеки DallasTemperature
Adafruit_AHTX0                    aht;                      // объявление объекта для работы с датчиком AHT20
Adafruit_Sensor                   *aht_humidity, *aht_temperature;
Adafruit_BMP280                   bmx;                      // объявление объекта для работы с датчиком BMP280
String hostname                   = "";                     // уникальное имя метеостанции в формате: "ESP+MAC-адрес" (выглядит как ESPAABBCCDDEEFF)
unsigned long lastConnectionTime  = 0;                      // время последней передачи данных

// --------------------------------------------------------------------------------------------
void WifiManStart() {                                       // процедура начального подключения к wifi, если не знает к чему подцепить - создает точку доступа ESP8266 и настроечную таблицу
  WiFiManager wifiManager;                                  // подробнее: https://github.com/tzapu/WiFiManager
  wifiManager.setTimeout (1);                               // устанавливает время ожидания до выключения портала конфигурации полезно, чтобы все повторилось или пошло спать в секундах
  wifiManager.setDebugOutput(true);                         // вывод отладочных сообщений от wifi-менеджера
  wifiManager.setMinimumSignalQuality(1);                   // минимальное качество сигнала в % для попытки соединения
  if (!wifiManager.autoConnect("meteostation"))             // когда интернет отсутствует -> включаем точку доступа "meteostation" на 5 секунд и если не успели подключиться тогда ->
    Serial.println("Not connect in allotted time");
  Serial.println("Connected...");
}

// --------------------------------------------------------------------------------------------
void setup() {                                              // настройка оборудования
  Serial.begin (COM_PORT_SPEED);                            // инициализация последовательного порта для отладки на заданной скорости
  Serial.println("\r\n\r\n\r\n***** MeteoStation awaked ******");
  Serial.println("\r\n***** >>> WIFI STAGE <<< *****");
  WifiManStart();                                           // запуск wifi
  hostname="ESP"+WiFi.macAddress();                         // формирование уникального имени метеостанции
  hostname.replace(":","");
  WiFi.hostname(hostname);                                  // представление имени хоста метеостанции для wifi
  //WiFiManager(resetSettings());                           // сброс настроек wifi
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
  Serial.println("MeteoStation ID: "+hostname);
  lastConnectionTime = millis() - POSTING_INTERVAL + 15000; // передача на народный мониторинг через 15 сек.
}

String Measurement(void) {                                  // чтение показаний датчиков
  String            buf = "";                               // динамический текстовый буфер для подготовки сообщения
  uint8_t numberOf1Wire = 0;                                // счётчик доступных датчиков на шине 1Wire
  uint8_t   numberOfI2C = 0;                                // счётчик доступных датчиков на шине I2C
  uint8_t      attempts = 0;                                // счётчик попыток инициализации датчиков
  float        humidity = 0;                                // текущая влажность
  float        pressure = 100025;                           // текущее атмосферное давление
  float     temperature = 0;                                // текущая температура воздуха
  float     averageTemp = 0;                                // усреднённая температура воздуха с датчиков температуры
  bool             real = true;                             // признак реальных значений
  Serial.println("\r\n***** >>> MEASUREMENTS STAGE <<< *****");

// -----------------------значения DS18B20----------------------------
  pinMode      (PIN_1WIRE_GND, OUTPUT);
  digitalWrite (PIN_1WIRE_GND, LOW);                        // подключение 1WIRE датчиков к GND
  pinMode      (PIN_1WIRE_VCC, OUTPUT);
  digitalWrite (PIN_1WIRE_VCC, HIGH);                       // подключение 1WIRE датчиков к VCC

  ds18b20.begin();                                          // инициализация DS18B20
  DeviceAddress tempDeviceAddress;
  numberOf1Wire = ds18b20.getDeviceCount();                 // определение количества подключенных датчиков DS18B20 к шине 1WIRE
  if (numberOf1Wire > 0) {                                  // инициализация DS18B20
    Serial.println("Sensor DS18B20 detected");
    for (uint8_t i=0; i < numberOf1Wire; i++) {             // перебор всех обнаруженных датчиков DS18B20
      if (ds18b20.getAddress(tempDeviceAddress,i))
        ds18b20.setResolution(tempDeviceAddress,TEMPERATURE_PRECISION);  // настройка точности обнаруженных датчиков DS18B20
      ds18b20.requestTemperatures();                        // проведение измерений DS18B20
      float currentTemp = ds18b20.getTempCByIndex(i);
      if (!isnan(currentTemp)) {
        buf = buf + "#TMPD" + String(i+1) + "#" + String(currentTemp) + "#Температура (°C) DS18B20 id:"; // чтение температуры с конкретного датчика DS18B20
        for (uint8_t j = 0; j < 8; j++) {
          if (tempDeviceAddress[i] < 16)  buf = buf + "0";  // адрес конкретного датчика DS18B20
          buf = buf + String(tempDeviceAddress[j],HEX);
        }
        buf = buf + "\r\n";
      }
    }
  } else Serial.println("Sensor DS18B20 not detected, сheck 1-Wire interface");

  pinMode      (PIN_1WIRE_GND, INPUT);                      // обесточивание 1WIRE датчиков
  pinMode      (PIN_1WIRE_VCC, INPUT);

// -----------------------значения BMP280------------------------------
  pinMode      (PIN_I2C_GND, OUTPUT);
  digitalWrite (PIN_I2C_GND, LOW);                          // подключение I2C датчиков к GND
  pinMode      (PIN_I2C_VCC, OUTPUT);
  digitalWrite (PIN_I2C_VCC, HIGH);                         // подключение I2C датчиков к VCC 
  Wire.begin   (PIN_I2C_SDA, PIN_I2C_CLK);                  // инициализация шины I2C с пользовательскими параметрами

  attempts = ATTEMPTS;
  while (!bmx.begin() && (attempts == 0))                   // попытки инициализации BMP280
    attempts--;

  if (bmx.begin()) {                                        // инициализация BMP280
    Serial.println("Sensor BMP280 success init");
    numberOfI2C++;
    temperature = bmx.readTemperature();                    // чтение температуры в градусах Цельсия из BMP280
    pressure    = bmx.readPressure();                       // чтение давления в Па из BMP280

    if (!isnan(temperature)) {
      if ((temperature > -100) && (temperature < 100)) {
        buf = buf + "#TMPB#" + String(temperature) + "#Температура воздуха (°C) BMP280\r\n";
      } else {
        buf = buf + "#TMPB#" + String(temperature) + "#Температура воздуха (°C) BMP280 нереальная\r\n";
        real = false;                                       // запрет расчета параметров на основе полученных данных
      }
      averageTemp = (averageTemp + temperature);
    }
    
    if (!isnan(pressure)) {
      pressure = pressure / 133.33F;                        // пересчёт атмосферного давления в мм ртутного столба
      if ((pressure > 600) && (pressure < 800)) {
        buf = buf + "#PRSB#" + String(pressure) + "#Атмосферное давление (mmHg) BMP280\r\n";
      } else {
        buf = buf + "#PRSB#" + String(pressure) + "#Атмосферное давление (mmHg) BMP280 нереальное\r\n";
        real = false;                                       // запрет расчета параметров на основе полученных данных
      }
    }
  } else Serial.println("Sensor BMP280 not detected, сheck I2C interface & sensor address");


// -----------------------значения AHT20------------------------------
  attempts = ATTEMPTS;
  while (!aht.begin() && (attempts == 0))                   // попытки инициализации AHT20
    attempts--;

  if (aht.begin()) {                                        // инициализация AHT20
    Serial.println("Sensor AHT20 success init\r\nSensor AHT20 humidity correction value: " + String(HUMIDITY_CORREСTION));
    numberOfI2C++;
    sensors_event_t  humid, temp;
    aht.getEvent(&humid, &temp);                            // обновление данных об относительной влажности и температуре из AHT20
    temperature = temp.temperature;                         // извлечение температуры в градусах Цельсия
    humidity    = humid.relative_humidity;                  // извлечение относительной влажности в процентах

    if (!isnan(temperature)) {
      if (abs(averageTemp - temperature) < 1) {
        buf = buf + "#TMPA#" + String(temperature) + "#Температура воздуха (°C) AHT20\r\n";
      } else {
        buf = buf + "#TMPA#" + String(temperature) + "#Температура воздуха (°C) AHT20 нереальная\r\n";
        real = false;                                       // запрет расчета параметров на основе полученных данных
      }
      averageTemp = (averageTemp + temperature);
    }

    if (!isnan(humidity)) {
      humidity = constrain((humidity + HUMIDITY_CORREСTION), 0, 100);  // внесение поправки в показания датчика относительной влажности (если врёт на постоянную величину)
      if ((humidity > 0) && (humidity < 100)) {
        buf   = buf + "#HMDA#" + String(humidity)  + "#Относительная влажность(%rH) AHT20\r\n";
      } else {
        buf   = buf + "#HMDA#" + String(humidity)  + "#Относительная влажность (%rH) AHT20 нереальная\r\n";
        real = false;                                       // запрет расчета параметров на основе полученных данных
      }
    }
  } else Serial.println("Sensor AHT20 not detected, сheck I2C interface & sensor address");

  pinMode      (PIN_I2C_GND, INPUT);                        // обесточивание I2C датчиков
  pinMode      (PIN_I2C_VCC, INPUT);

// ------------------расчётные значения---------------------
  if ((numberOfI2C > 0) && real) {                          // когда на шине I2C присутствуют датчики и нет запрета расчетов на основе полученных данных ->
    averageTemp = averageTemp / numberOfI2C;                // расчёт средней температуры воздуха на основе температуры от AHT20 и BMP280
    buf = buf + "#TMPZ#" + String(averageTemp) + "#Температура воздуха (°C) AHT20&BMP280\r\n";

    if (averageTemp >= -40 && averageTemp <= +50 && humidity > 0) {  // при допустимых условиях для расчёта точки росы ->
      const float k = 273.15;                               // температура 0°C, выраженная в градусах Кельвина
      const float a = 17.62, b = 243.12;                    // коэффициенты зависимости удерживаемой в воздухе влаги при температуре от -40 до +50°C
      const float p = 6.112;                                // коэффициент зависимости давления насыщенного пара от температуры
      const float r = 461.52;                               // удельная газовая постоянная для водяного пара
      float       v = (a * averageTemp)/(b + averageTemp);  // влияние температуры воздуха на способность удерживать водяной пар
      float       g = v + log(humidity / 100);              // привязка температуры воздуха ко влажности для оценки точки росы
      float     tdp = (b * g) / (a - g);                    // расчёт точки росы по упрощённой формуле Магнуса
      //float absHmd = p * exp(v) * humidity * 2.1674 / (k + averageTemp);             // расчет абсолютной влажности v.1
      float absHmd = humidity * 10 * ((p * 100.0 * exp(v)) / (r * (averageTemp + k))); // расчет абсолютной влажности v.2
      buf = buf + "#TDPZ#" + String(tdp)    + "#Точка росы (°C) AHT20&BMP280\r\n";
      buf = buf + "#HMAZ#" + String(absHmd) + "#Абсолютная влажность (g/m3) AHT20&BMP280\r\n";
    }
  }

// ---------------------значения ESP8266-------------------------
  long dBm = WiFi.RSSI();
  if (dBm > 0)  buf = buf + "#WIFI#-120#Отсутствует WIFI (dBm)\r\n";
    else        buf = buf + "#WIFI#" + String(dBm) + "#Уровень WIFI(dBm) '" + String(WiFi.SSID()) + "'\r\n";

  float vcc = ESP.getVcc();
  if (!isnan(vcc)) {
    vcc = (vcc + 300) / 1000;                               // напряжения питания ESP8266
    if (vcc > 0)  buf = buf + "#VOLT#" + String(vcc) + "#Напряжение питания (V) ESP8266\r\n";
      else        buf = buf + "#VOLT#" + String(vcc) + "#Напряжение питания (V) ESP8266 нереальное\r\n";
  }

  buf = buf + "##\r\n";                                     // признак окончания посылки на narodmon.ru
  return buf;
}

// --------------------------------------------------------------------------------------------
bool SendToNarodmon() {                                     // формирование и отправка пакета
  WiFiClient client;
  String msg = "#" + hostname + "\r\n" + Measurement();     // id станции + показания датчиков
  Serial.print (msg);
  if (!client.connect("narodmon.ru",8283)) {                // когда попытка подключения не удалась ->
    Serial.println("connection failed"); 
    return false;                                           // возврат FALSE (неудачная посылка)
  } else {                                                  // когда подключение удалось ->
    Serial.println("\r\n***** >>> SENDING STAGE <<< *****");
    client.print(msg);                                      // отправка данных
    delay(100);
    while (client.available()) {                            // если прилетит ответ -> 
      String line=client.readStringUntil('\r');
      Serial.print("Answer from server: " + line);          // отобразить ответ в Serial
    }
  }
  return true;                                              // возврат TRUE (успешная посылка)
}

// --------------------------------------------------------------------------------------------
void loop() {
  if (WiFi.status()==WL_CONNECTED) {                        // при наличии wifi подключения к сети ->
    if (SendToNarodmon())                                   // формирование и отправка посылки на narodmon.ru
      Serial.println ("Successful sending!");
  } else {                                                  // при отсутствии wifi подключения к сети ->
    Serial.println("WIFI connection failed"); 
    String msg = "#" + hostname + "\r\n" + Measurement();   // формирование посылки
    Serial.print (msg);
  }
  Serial.print ("\r\n***** >>> SNOOZING STAGE <<< *****\r\n\r\n_ _ _ zzzzZZZZ _ _ _\r\n\r\n***** MeteoStation snoozing ******\r\n");
  ESP.deepSleep(SLEEP_TIME);                                // сон, по окончании которого активируется pin D0 -> RESET МК и программа стартует заново
}

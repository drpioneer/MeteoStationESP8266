/*------------------------------------------------------------------------------------------------------------------------------------
 *     Метеостанция на базе ESP8266 ( LoLin D1 mini / WeMos D1 mini ) ### https://github.com/drpioneer/MeteoStationESP8266
 *  
 *  Метеостанция собирает информацию от подключенных датчиков, отсылает её на сервер narodmon.ru и засыпает до следующего сеанса.
 *  При первом запуске или отсутствии сети кратковременно создается точка доступа по адресу (192.168.4.1) для возможности 
 *  настройки WIFI через WEB-интерфейс. После настройки на сервер народного мониторинга будут отправляться собранные данные, 
 *  а в виртуальный последовательный порт будет выводиться отладочная информация о работе метеостанции.
 *  Питание метеостанции осуществляется от литий-ионнного суперконденсатора, запитанного от солнечной панели через MPPT контроллер.
 *  Допустимое напряжение для литий-ионнного суперконденсатора обычно находится в пределах от +2.5 до +4.2В.
 *  Выход за границы допустимого напряжения приведёт к деградация литий-ионнного суперконденсатора!
 *  Для продолжительной автономной работы метеостанции в коде особое внимание уделено работе с суперконденсатором.
 *  Метеостанция умеет работать с несколькими типами метеодатчиков, подключенных к шинам I2C и 1Wire: BME(P)180/280, AHT20, SCD40, DS18B20.
 *  Для датчика DS18B20 требуется наличие внешнего подтягивающего резистора 4.7 кОм между линией данных и питанием датчика.
 *  Для снижения риска выхода из строя метеодатчиков по причине электрохимической коррозии питание датчиков производится только
 *  в момент их опроса. Остальное время датчики обесточены. Описание подключенных метеодатчиков выполнено в коде в дефайнах. 
 *------------------------------------------------------------------------------------------------------------------------------------*/
 
#include <FS.h>
#include "ESP8266WiFi.h"
#include "DNSServer.h"
#include "ESP8266WebServer.h"
#include "DallasTemperature.h"
#include "WiFiManager.h"                                    // источник библиотеки https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include "INA226_WE.h"                                      // библиотека для работы с энергомонитором INA226
#include "Adafruit_AHTX0.h"                                 // библиотека для работы с метеодатчиком   AHT20
#include "Adafruit_BME280.h"                                // библиотека для работы с метеодатчиком   BME280
#include "Adafruit_BMP280.h"                                // библиотека для работы с метеодатчиком   BMP280
#include "Adafruit_BMP085.h"                                // библиотека для работы с метеодатчиком   BMP180
#include "SparkFun_SCD4x_Arduino_Library.h"                 // библиотека для работы с метеодатчиком   SCD40  !!!Датчик прожорливый и тормозной!!!

#define PIN_1WIRE_VCC             D6                        // user defined 1WIRE sensor VCC-pin for DS18B20
#define PIN_1WIRE_DAT             D7                        // user defined 1WIRE sensor DAT-pin for DS18B20
#define PIN_1WIRE_GND             D8                        // user defined 1WIRE sensor GND-pin for DS18B20 (!!!для старта МК уровень на этом PINе должен быть LOW!!!)

#define PIN_I2C_SDA               D2                        // user defined I2C sensor SDA-pin for AHT20&BMP280 // AHT20+BMP280 припаяный
#define PIN_I2C_SCL               D4                        // user defined I2C sensor SCL-pin for AHT20&BMP280
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for AHT20&BMP280
#define PIN_I2C_GND               D3                        // user defined I2C sensor GND-pin for AHT20&BMP280

/*
#define PIN_I2C_SDA               D4                        // user defined I2C sensor SDA-pin for BMP180 // BMP180 припаяный
#define PIN_I2C_SCL               D3                        // user defined I2C sensor SCL-pin for BMP180
#define PIN_I2C_VCC               D1                        // user defined I2C sensor VCC-pin for BMP180
#define PIN_I2C_GND               D2                        // user defined I2C sensor GND-pin for BMP180
*/

#define COM_PORT_SPEED            115200                    // скорость COM-порта отладочного терминала
#define SLEEP_TIME                20                        // время сна метеостанции в минутах
#define ATTEMPTS                  5                         // количество попыток инициализации датчиков
#define TEMPERATURE_PRECISION     10                        // точность бит для DS18B20. Если глючит или врет -> уменьшить до 9

#define TEMP_MIN                  -40.0f                    // нижняя граница температуры для BMP180, BMP280, BME280 и AHT20 (°C)
#define TEMP_MAX                  +85.0f                    // верхняя граница температуры для BMP180, BMP280, BME280 и AHT20 (°C)
#define HUM_MIN                   0.0f                      // нижняя граница относительной влажности для BME280 и AHT20 (%)
#define HUM_MAX                   100.0f                    // верхняя граница относительной влажности для BME280 и AHT20 (%)
#define PRES_MIN                  30000.0f                  // нижняя граница давления для BMP180, BMP280 и BME280 (Па)
#define PRES_MAX                  110000.0f                 // верхняя граница давления для BMP180, BMP280 и BME280 (Па)
#define VCC_MIN                   +2.7f                     // нижняя граница напряжения питания ESP8266 с учётом использования литий-ионного суперконденсатора (В)
#define VCC_MAX                   +3.5f                     // верхняя граница напряжения питания ESP8266 (В)
#define INA_MIN                   -0.3f                     // нижняя граница измеряемого напряжения для INA226 (В)
#define INA_MAX                   +40.0f                    // верхняя граница измеряемого напряжения для INA226 (В)

ADC_MODE                          (ADC_VCC);                // объявление объекта для работы внутреннего АЦП ESP8266 с измерением напряжения питания МК
OneWire                           oneWire(PIN_1WIRE_DAT);   // объявление объекта для работы с 1Wire
DallasTemperature                 ds18b20(&oneWire);        // объявление объекта для работы с DS18B20
SCD4x                             scd40;                    // объявление объекта для работы с датчиком SCD40
Adafruit_AHTX0                    aht20;                    // объявление объекта для работы с датчиком AHT20
Adafruit_BME280                   bme280;                   // объявление объекта для работы с датчиком BME280
Adafruit_BMP280                   bmp280;                   // объявление объекта для работы с датчиком BMP280
Adafruit_BMP085                   bmp180;                   // объявление объекта для работы с датчиком BMP180
INA226_WE                         ina226 = INA226_WE(0x40); // объявление объекта для работы с датчиком INA226 (по указанному адресу)
String                            msg;
float                             vcc;

// ------------------ проверка напряжения питания МК с аварийным отключением ---------------------------------
float checkVcc() {
  float current=0;                                          // буфер для текущего значения напряжения МК
  float sum=0;                                              // буфер для усредненного значения напряжения МК
  uint8_t attempts = ATTEMPTS;                              // число замеров напряжения МК
  uint8_t valid = 0;                                        // счётчик успешных замеров напряжения МК
  while (attempts-- > 0) {
    current = (ESP.getVcc() + 300) / 1000.0f;               // замер напряжения питания МК с пересчётом в Вольты
    sum += current;                                         // накопление значения напряжения питания МК
    valid++;
  }
  if ((sum / valid) < VCC_MIN || (sum / valid) > VCC_MAX)   // когда напряжение питания МК вышло за допустимые границы ->
    ESP.deepSleep(SLEEP_TIME * 60000000ULL);                // аварийный сон МК с последующей перезагрузкой
  return (valid > 0) ? (sum / valid) : 0.0f;                // возврат усреднённого значения напряжения питания МК
}

// ------------------- начальная настройка -----------------------------
void setup() {
  WiFi.mode(WIFI_OFF);                                      // принудительное отключение WIFI для экономии заряда энергии
  WiFi.forceSleepBegin();
  vcc = checkVcc();                                         // проверка напряжения питания МК
  Serial.begin (COM_PORT_SPEED);                            // инициализация последовательного порта на указанной скорости для вывода отладочных сообщений
  Serial.println("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n   MeteoStation awaked");
}

// ------------------ расчёт значений ---------------------
String Calculate(char chr, float temper, float humid) {
  if (!isnan(temper) && !isnan(humid) && temper >= -40 && temper <= +50 && humid > 0) {   // при допустимых условиях для расчёта точки росы ->
    const float k = 273.15;                                 // температура 0°C, выраженная в градусах Кельвина
    const float a = 17.62, b = 243.12;                      // коэффициенты зависимости удерживаемой в воздухе влаги при температуре от -40 до +50°C
    const float p = 6.112;                                  // коэффициент зависимости давления насыщенного пара от температуры
    const float r = 461.52;                                 // удельная газовая постоянная для водяного пара
    float       v = (a * temper)/(b + temper);              // влияние температуры воздуха на способность удерживать водяной пар
    float       g = v + log(humid / 100);                   // привязка температуры воздуха ко влажности для оценки точки росы
    float     tdp = (b * g) / (a - g);                      // расчёт точки росы по упрощённой формуле Магнуса
    float  absHmd = p * exp(v) * humid * 2.1674 / (k + temper);                  // расчет абсолютной влажности v.1
    //float  absHmd = humid * 10 * ((p * 100.0 * exp(v)) / (r * (temper + k)));  // расчет абсолютной влажности v.2
    String   text = "";
    text = "#D" + String(chr) + "#" + String(tdp)    + "#°C Dew point\r\n#H";
    text = text + String(chr) + "#" + String(absHmd) + "#g/m3 Absolute humidity\r\n";
    return (text);
  }
  return "";
}

String Measure(void) {
  String            buf = "";                               // динамический текстовый буфер для подготовки сообщения
  buf.reserve(2048);                                        // предварительное выделение памяти динамическому буферу
  uint8_t      attempts = 0;                                // счётчик попыток инициализации датчиков
  float          humAHT = NAN;                              // буфер для значения влажности от AHT20
  float          tmpAHT = NAN;                              // буфер для значения температуры воздуха от AHT20
  float          tmpBMx = NAN;                              // буфер для значения температуры воздуха от BMP180, BMP280 или BME280
  uint8_t validTmpCount = 0;
  float          prsBMx = NAN;                              // буфер для значения атмосферного давления от BMP180, BMP280 или BME280
  float          humBMx = NAN;                              // буфер для значения влажности от BME280
  float          tmpAVR = 0.0f;                             // буфер для значения усреднённой температуры воздуха
  float         inaVolt = NAN;
  uint16_t       co2SCD = 0;
  float          tmpSCD = NAN;
  float          hmdSCD = NAN;
  bool             real = true;                             // признак реальных значений
  bool           inited = false;                            // признак инициализации датчика

  Serial.println("\r\n***** >>> MEASUREMENTS STAGE <<< *****");

// --------------------- значение напряжения питания МК-------------------------
  buf += "#VCC#" +String(vcc)+ "#V Supply voltage ESP8266\r\n";

// -----------------------начало работы датчиками I2C------------------------------
  pinMode      (PIN_I2C_GND, OUTPUT);
  pinMode      (PIN_I2C_VCC, OUTPUT);

  digitalWrite (PIN_I2C_GND, LOW);                          // подключение I2C датчиков к GND
  digitalWrite (PIN_I2C_VCC, HIGH);                         // подключение I2C датчиков к VCC 

  Wire.begin   (PIN_I2C_SDA, PIN_I2C_SCL);                  // инициализация шины I2C с пользовательскими параметрами

// -----------------------опрос INA226------------------------------
  attempts = ATTEMPTS; inited = false;
  while (attempts-- > 0) {                                  // попытки инициализации энергомонитора INA226
    if (ina226.init()) { inited = true; break; }
    delay(1);                                               // время старта INA226 после подачи питания: ~1 мс
  }
  if (inited) {                                             // когда энергомонитор INA226 успешно инициализирован -> 
    Serial.println("Sensor INA226 success init");
    ina226.enableConvReadyAlert();
    ina226.startSingleMeasurement();                        // старт замеров энергомонитора INA226
    uint8_t inaWait = 50;
    while (!ina226.convAlert && --inaWait > 0) { delay(1); ina226.readAndClearFlags(); }  // ожидание результатов замеров INA226
    if (inaWait > 0) {
      inaVolt = ina226.getBusVoltage_V();                   // чтение значения напряжения из INA226
      ina226.powerDown();                                   // усыпление INA226 для экономии заряда суперконденсатора
      if (!(isnan(inaVolt) || inaVolt < INA_MIN || inaVolt > INA_MAX))  // когда результат работы INA226 корректен ->
        buf += "#VI#" + String(inaVolt) + "#V Solar panel voltage INA226\r\n";
      } else { ina226.powerDown(); Serial.println("Sensor INA226 timeout or read error"); }
    } else { Serial.println("Sensor INA226 not detected, check I2C interface & sensor address"); }

// -----------------------опрос BME280------------------------------
  attempts = ATTEMPTS; inited = false;
  while (attempts-- > 0) {                                  // попытки инициализации BME280
    if (bme280.begin()) { inited = true; break; }
    delay(1);                                               // время старта BME280 после подачи питания: ~2 мс
  }
  if (inited) {                                             // когда BME280 успешно инициализирован -> 
    Serial.println("Sensor BME280 success init");
    tmpBMx = bme280.readTemperature();                      // чтение температуры в градусах Цельсия из BME280
    prsBMx = bme280.readPressure();                         // чтение давления в Паскалях из BME280
    humBMx = bme280.readHumidity();                         // чтение относительной влажности в процентах из BME280

    if (isnan(prsBMx) || prsBMx < PRES_MIN || prsBMx > PRES_MAX) real = false;  // когда проблемы с датчиком давления BME280 -> запрет дальнейших расчетов
    else {
      prsBMx = prsBMx / 133.33f;                            // пересчёт атмосферного давления из Паскалей в мм ртутного столба
      buf += "#PB#" + String(prsBMx) + "#mmHg Atmospheric pressure BME280\r\n";
    }

    if (isnan(tmpBMx) || tmpBMx < TEMP_MIN || tmpBMx > TEMP_MAX) real = false;  // когда проблемы с датчиком температуры BME280 -> запрет дальнейших расчетов
    else {
      tmpAVR = (tmpAVR + tmpBMx);                           // накопление данных о температуре воздуха
      validTmpCount++;
      buf += "#TB#" + String(tmpBMx) + "#°C Air temperature BME280\r\n";
    }

    if (isnan(humBMx) || humBMx < HUM_MIN || humBMx > HUM_MAX) real = false;    // когда проблемы с датчиком влажности BME280 -> запрет дальнейших расчетов
    else {
      buf += "#HB#" + String(humBMx) + "#%rH Relative humidity BME280\r\n";
    }
  } else {
    real = false;                                           // запрет дальнейших расчетов на основе полученных данных
    Serial.println("Sensor BME280 not detected, check I2C interface & sensor address");

// -----------------------опрос BMP280------------------------------
    attempts = ATTEMPTS; inited = false;
    while (attempts-- > 0) {                                // попытки инициализации BMP280
      if (bmp280.begin()) { inited = true; break; }
      delay(1);                                             // время старта BMP280 после подачи питания: ~2 мс
    }
    if (inited) {                                           // когда BMP280 успешно инициализирован -> 
      real = true;
      Serial.println("Sensor BMP280 success init");
      tmpBMx = bmp280.readTemperature();                    // чтение температуры в градусах Цельсия из BMP280
      prsBMx = bmp280.readPressure();                       // чтение давления в Паскалях из BMP280

      if (isnan(prsBMx) || prsBMx < PRES_MIN || prsBMx > PRES_MAX) real = false;  // когда проблемы с датчиком давления BMP280 -> запрет дальнейших расчетов
      else {
        prsBMx = prsBMx / 133.33f;                          // пересчёт атмосферного давления из Паскалей в мм ртутного столба
        buf += "#PB#" + String(prsBMx) + "#mmHg Atmospheric pressure BMP280\r\n";
      }

      if (isnan(tmpBMx) || tmpBMx < TEMP_MIN || tmpBMx > TEMP_MAX) real = false;  // когда проблемы с датчиком температуры BMP280 -> запрет дальнейших расчетов
      else {
        tmpAVR = (tmpAVR + tmpBMx);                         // накопление данных о температуре воздуха
        validTmpCount++;
        buf += "#TB#" + String(tmpBMx) + "#°C Air temperature BMP280\r\n";
      }
    } else {
      real = false;                                         // запрет дальнейших расчетов на основе полученных данных
      Serial.println("Sensor BMP280 not detected, check I2C interface & sensor address");

// -----------------------опрос BMP180------------------------------
      attempts = ATTEMPTS; inited = false;
      while (attempts-- > 0) {                              // попытки инициализации BMP180
        if (bmp180.begin()) { inited = true; break; }
        delay(3);                                           // время старта BMP180 после подачи питания: ~10 мс
      }
      if (inited) {                                         // когда BMP180 успешно инициализирован ->
        real = true;
        Serial.println("Sensor BMP180 success init");
        tmpBMx = bmp180.readTemperature();                  // чтение температуры в градусах Цельсия из BMP180
        prsBMx = bmp180.readPressure();                     // чтение давления в Паскалях из BMP180

        if (isnan(prsBMx) || prsBMx < PRES_MIN || prsBMx > PRES_MAX) real = false;  // когда проблемы с датчиком давления BMP180 -> запрет дальнейших расчетов
        else {
          prsBMx = prsBMx / 133.33f;                        // пересчёт атмосферного давления из Паскалей в мм ртутного столба
          buf += "#PB#" + String(prsBMx) + "#mmHg Atmospheric pressure BMP180\r\n";
        }

        if (isnan(tmpBMx) || tmpBMx < TEMP_MIN || tmpBMx > TEMP_MAX) real = false;  // когда проблемы с датчиком температуры BMP180 -> запрет дальнейших расчетов
        else {
          tmpAVR = (tmpAVR + tmpBMx);                       // накопление данных о температуре воздуха
          validTmpCount++;
          buf += "#TB#" + String(tmpBMx) + "#°C Air temperature BMP180\r\n";
        }
      } else {
        real = false;                                       // запрет дальнейших расчетов на основе полученных данных
        Serial.println("Sensor BMP180 not detected, check I2C interface & sensor address");
      }
    }
  }

// -----------------------опрос AHT20------------------------------
  attempts = ATTEMPTS; inited = false;
  while (attempts-- > 0) {                                  // попытки инициализации AHT20
    if (aht20.begin()) { inited = true; break; }
    delay(5);                                               // время старта AHT20 после подачи питания: ~10-20 мс
  }
  if (inited) {                                             // когда AHT20 успешно инициализирован -> 
    Serial.println("Sensor AHT20 success init");
    sensors_event_t humid, temp;
    aht20.getEvent(&humid, &temp);                          // обновление данных об относительной влажности и температуре из AHT20
    tmpAHT = temp.temperature;                              // чтение температуры в градусах Цельсия
    humAHT = humid.relative_humidity;                       // чтение относительной влажности в процентах

    if (isnan(tmpAHT) || tmpAHT < TEMP_MIN || tmpAHT > TEMP_MAX) real = false;  // когда проблемы c датчиком температуры AHT20 -> запрет дальнейших расчетов
    else {
      tmpAVR = (tmpAVR + tmpAHT);                           // накопление данных о температуре воздуха
      validTmpCount++;
      buf += "#TA#" + String(tmpAHT) + "#°C Air temperature AHT20\r\n";
    }

    if (isnan(humAHT) || humAHT < HUM_MIN || humAHT > HUM_MAX) real = false;    // когда проблемы с датчиком влажности AHT20 -> запрет дальнейших расчетов
    else { buf += "#HA#" + String(humAHT) + "#%rH Relative humidity AHT20\r\n"; }
  } else {
    real = false;                                           // запрет дальнейших расчетов на основе полученных данных
    Serial.println("Sensor AHT20  not detected, check I2C interface & sensor address");
  }

// -----------------опрос SCD40 (этот датчик прожорливый и тормозной: 1) его нельзя питать от GPIO МК! 2. быстро высадит суперконденсатор!)-----------------
  attempts = ATTEMPTS; inited = false;
  while (attempts-- > 0) {                                  // попытки инициализации SCD40
    if (scd40.begin()) { inited = true; break; }
    delay(25);                                              // время старта SCD40 после подачи питания: ~30-50 мс
  }
  if (inited) {
    Serial.println("Sensor SCD40 success init");
    scd40.measureSingleShot();                              // отправка команды для SCD40 на однократный замер
    uint8_t waitAttempts = 60;                              // ожидание до 6 секунд (60 * 100мс)
    bool dataReady = false;                                 // флаг готовности данных от датчика
    while (waitAttempts > 0) {                              // попытки чтения параметров из SCD40
      if (scd40.readMeasurement()) {
        co2SCD = scd40.getCO2();
        tmpSCD = scd40.getTemperature();
        hmdSCD = scd40.getHumidity();
        buf += "#DS#" + String(co2SCD) + "#ppm CO2 concentration SCD40\r\n";
        buf += "#TS#" + String(tmpSCD) + "#°C Air temperature SCD40\r\n";
        buf += "#HS#" + String(hmdSCD) + "#%rH Relative humidity SCD40\r\n";
        dataReady = true;
        break;
      }
      delay(100);
      waitAttempts--;
      Serial.print(".");
    }
    if (dataReady) { Serial.println("SCD40 data read successfully."); } else { Serial.println("SCD40 timeout!"); }
  } else { Serial.println("Sensor SCD40  not detected, check I2C interface & sensor address"); }

// -----------------------окончание работы с шиной I2C + обесточивание датчиков------------------------------
  digitalWrite (PIN_I2C_VCC, LOW);                          // принудительное обесточивание I2C датчиков
  pinMode (PIN_I2C_GND, INPUT);                             // принудительный перевод линий I2C в Z-состояние
  pinMode (PIN_I2C_VCC, INPUT);
  pinMode (PIN_I2C_SCL, INPUT);
  pinMode (PIN_I2C_SDA, INPUT);
    
// -----------------------расчётные значения------------------------------
  if (!isnan(tmpBMx) && !isnan(tmpAHT) && fabs(tmpBMx - tmpAHT) > 1) real = false; // при существенной разнице температур AHT20 и BMP280 -> запрет расчетов
  if (real && validTmpCount > 0) {                          // когда все значения от AHT20 и BMP280 корректны ->
    tmpAVR = tmpAVR / validTmpCount;                        // усреднение температуры воздуха
    buf += "#TZ#" + String(tmpAVR) + "#°C Air temperature average\r\n";
    float humAct = !isnan(humAHT) ? humAHT : humBMx;
    buf += Calculate('Z', tmpAVR, humAct);
  } else {                                                  // используем значения от SCD40, когда значения от AHT20 + BMP280 НЕ корректны ->
    buf += (Calculate('Z', tmpSCD, hmdSCD));
  }

// -----------------------работа с DS18B20----------------------------
  pinMode      (PIN_1WIRE_GND, OUTPUT);
  pinMode      (PIN_1WIRE_VCC, OUTPUT);

  digitalWrite (PIN_1WIRE_GND, LOW);                        // подключение 1WIRE датчиков к GND
  digitalWrite (PIN_1WIRE_VCC, HIGH);                       // подключение 1WIRE датчиков к VCC

  ds18b20.begin();                                          // инициализация DS18B20
  DeviceAddress tempDeviceAddress;
  uint8_t numberOf1Wire = ds18b20.getDeviceCount();         // определение количества подключенных датчиков DS18B20 к шине 1WIRE
  if (numberOf1Wire > 0) {                                  // инициализация DS18B20
    Serial.println("Sensor DS18B20 detected");
    for (uint8_t i=0; i < numberOf1Wire; i++) {             // перебор обнаруженных датчиков DS18B20
      if (ds18b20.getAddress(tempDeviceAddress,i))
        ds18b20.setResolution(tempDeviceAddress,TEMPERATURE_PRECISION);  // настройка точности обнаруженных датчиков DS18B20
      ds18b20.requestTemperatures();                        // проведение измерений DS18B20
      float tmpDS18 = ds18b20.getTempCByIndex(i);
      if (!isnan(tmpDS18))
        buf += "#T" + String(i+1) + "#" + String(tmpDS18) + "#°C Temperature DS18B20\r\n"; // чтение температуры с конкретного датчика DS18B20
    }
  } else { Serial.println("Sensor DS18B20 not detected, check 1-Wire interface"); }

// -----------------------окончание работы с шиной 1Wire + обесточивание датчиков------------------------------
  digitalWrite (PIN_1WIRE_VCC, LOW);                        // принудительное обесточивание 1WIRE датчиков
  pinMode (PIN_1WIRE_GND, INPUT);                           // принудительный перевод линий 1WIRE в Z-состояние
  pinMode (PIN_1WIRE_VCC, INPUT);
  pinMode (PIN_1WIRE_DAT, INPUT);

  return buf;
}

// -------------------------начальное подключение к wifi-------------------------------------------------------------------
void WifiManStart() {
  Serial.println("\r\n***** >>> WIFI INIT STAGE <<< *****");

  WiFi.forceSleepWake();                                    // принудительное пробуждение Wifi
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(11.0);                                // понижение выходной мощности передатчика Wifi (dBm)

  WiFiManager wifiManager;                                  // подробнее: https://github.com/tzapu/WiFiManager
  wifiManager.setDebugOutput(false);                        // вывод отладочных сообщений от wifi-менеджера
  wifiManager.setMinimumSignalQuality(1);                   // минимальное качество сигнала в % для попытки соединения
  wifiManager.setTimeout(1);                                // время ожидания до выключения портала конфигурации
  if (wifiManager.autoConnect("meteostation")) Serial.println("Connected...");  // когда интернет-соединение НЕ установлено -> активация ТД "meteostation"
  else Serial.println("Not connect in allotted time");
  String hostname="ESP"+WiFi.macAddress();                  // формирование уникального имени метеостанции в формате: "ESP+MAC-адрес" (выглядит как ESPAABBCCDDEEFF)
  hostname.replace(":","");
  WiFi.hostname(hostname);                                  // представление имени хоста метеостанции для wifi
  //WiFiManager(resetSettings());                           // сброс настроек wifi
  msg = "#" + hostname + "\r\n" + msg;                      // подготовка пакета данных для отсылки на сервер narodmon.ru
  if (WiFi.status() == WL_CONNECTED) {                      // при наличии wifi подключения к сети ->
    long dBm = WiFi.RSSI();
    msg += "#WF#" + String(dBm) + "#dBm Wifi level\r\n##\r\n";
    if (SendToNarodmon())                                   // отправка посылки на narodmon.ru
      Serial.println ("Successful sending!");
  } else {
    msg += "#WF#-120#dBm Wifi is missing\r\n##\r\n";
    Serial.println("WIFI connection failed");
  }
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();                                   // принудительное усыпление Wifi
}

// -----------------------отправка пакета на narodmon.ru--------------------------------
bool SendToNarodmon(){
  WiFiClient client;
  if (!client.connect("narodmon.ru",8283)) {                // когда попытка подключения не удалась ->
    Serial.println("connection failed"); 
    return false;                                           // возврат FALSE (неудачная посылка)
  } else {                                                  // когда подключение удалось ->
    Serial.println("\r\n***** >>> WIFI SENDING STAGE <<< *****");
    client.print(msg);                                      // отправка данных
    delay(50);
    while (client.available()) {                            // если прилетит ответ -> 
      String line=client.readStringUntil('\r');
      Serial.print("Answer from server: " + line);          // вывод ответа в Serial
    }
    client.stop();                                          // явное и немедленное закрытие TCP сокета для экономии заряда суперконденсатора
  }
  return true;                                              // возврат TRUE (успешная посылка)
}

// ------------------------основной цикл работы---------------------------
void loop() {
  msg = Measure();                                          // опрос датчиков
  WifiManStart();                                           // работа через wifi
  Serial.println("\r\n+++++ MESSAGE: +++++\r\n" + msg);
  Serial.println("***** >>> SNOOZING STAGE <<< *****\r\n\r\n_ _ _ ... zzzzZZZZ ... _ _ _\r\n\r\n   MeteoStation snoozing\r\n");
  ESP.deepSleep(SLEEP_TIME*60000000ULL);                    // усыпление МК, после сна дёргается pin 'D0' связанный с RESET -> программа стартует заново
}

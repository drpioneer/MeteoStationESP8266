# ESP8266-Meteo-Station
Метеостанция на базе отладочной платы WeMos D1R2 & mini (Lolin D1R2 & mini) на основе ESP8266.
http://arduino.ru/forum/proekty/meteostantsiya-dlya-narodnogo-monitoringa#comment-551457

Станция получает информацию от датчиков: 
* температуры воздуха - DS18B20
* атмосферного давления - BME280/BMP280
* направления ветра (магнитный энкодер) - AS5600
* скорости ветра (датчик Холла) - OH137

Программа автоматически определяет наличие датчиков и шлет от них информацию на сервер. При первом запуске или отсутствии сети создается точка доступа на 5 секунд (нужно успеть подключиться). В WEB-интерфейсе можно просканировать сети и подключиться к имеющейся либо вписать название сети вручную, после чего на сайт начинают отправлятся данные, а в SERIAL выводится уникальный ID станции, который потом привяжется к сайту. Дополнительно станция отсылает на сервер информацию о напряжении VCC и уровне сигнала Wifi в %. Все показания и статистику на графиках можно смотреть в приложениях под разные платформы, в том числе Android, IOS. Для отладки без Wifi удобно закоментировать 63 и 208 строки.

Корпусные детали метеостанции приобретал тут:
https://r2akt.ru/%d0%bf%d1%80%d0%be%d0%b5%d0%ba%d1%82%d1%8b/%d0%bc%d0%b5%d1%82%d0%b5%d0%be%d0%be%d1%81%d1%82%d0%b0%d0%bd%d1%86%d0%b8%d1%8f/

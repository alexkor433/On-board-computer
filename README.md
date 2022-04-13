# On-board-computer
Программа для бортового компьютера с использованием Arduino для мотоцикла.

![Снимок экрана (269)](https://user-images.githubusercontent.com/98914596/152302594-038accc3-8d7a-41bd-a859-1d5ee7e6715a.png)

![Снимок экрана (315)](https://user-images.githubusercontent.com/98914596/152304932-b542c358-555f-4f2a-b90a-e0459858ce10.png)

![Снимок экрана (322)](https://user-images.githubusercontent.com/98914596/152306161-d93440b4-e3fa-4f79-a25c-1ad9fa971339.png)


## Версии
`старые версии находятся в LOG-versions.md. Они не предназначены для системы на плате`

- v2.0 версия для системы на печатной плате с мк-к Arduino Pro Mini, добавлена поддержка раздельного питания системы (с буферным аккумулятором, отключается от основной системы
при включении зажигания, при выключенном зажигании заряжается от основного аккумулятора), показание напряжения буферного аккумулятора при 1 нажатии на кнопку, 
предупреждение о низком заряде аккумулятора, небольшие правки кода;
- v2.1 добавлены новые символы для LCD 1602, доработан фильтр на сравнении - сравнение делается по модулю, усредняется, ускораена его работа, добавлены удобные макросы,
некоторые части кода переписаны в универсальные библиотеки, исправление недочётов; //15.10.21
- v2.2 Применена библиотека Tachometer (https://github.com/GyverLibs/Tachometer) для более оптимального считывания количества RPM, имеет встроенный медианный фильтр,
убраны все фильтры RPM (необходимо фильтровать не RPM, а резкие изменения количества времени между сигналами), изменены действия после нажатия кнопки, 
оптимизация кода; //20.12.21
- v2.3 применена библиотека EncButton (https://github.com/GyverLibs/EncButton) для работы с энкодером и кнопкой, написано меню с настройками (пока только яркость TM1637) в 
LCD1602, изменены действия кнопки (1 нажатие - выводится время поездки и версия программы, 2 нажатия - количество моточасов и температура процессора, долгое нажатие - переход
в меню настроек), оптимизация кода, исправлено много багов; //23.10.21
- v2.3.1 Исправление багов (не работал счётчик моточасов), добавлена функция вывода на индикатор оставшейся оперативной памяти при двойном нажатии на кнопку; //26.11.21
- v2.3.2 Исправление багов (некорректно работал счётчик моточасов и настройки из-за неправильной работы с EEPROM), добавлена функция обнуления счётчика моточасов(последний пункт меню), 
теперь все параметры в меню сохраняют значения после выключения платы, в меню настроек на TM1637 отображается "tune"; //30.11.21
- v2.3.2.1 Оптимизированы некоторые условия if, ускорен вывод температуры после включения, добавлен дефайн, при подключении которого становится возможной работа с некачественными и убитыми энкодерами (средствами библиотеки v1.12, EB_BETTER_ENC); //2.12.21
- v2.3.3 Функция изменения яркости светодиода, исправлен баг с быстрым поворотом энкодера; //6.12.21
- v2.3.4 Добавлен динамический вывод размерности температуры, режим для полушаговых энкодеров; //12.01.22
- v2.3.5 Для строк добавлен макрос F, сэкономлено 80 байт оперативки; //13.01.22
- v2.3.6 Исправление багов (режим полушаговых энкодеров EB_HALFSTEP_ENC подключается только с EB_BETTER_ENC, неправильно работал светодиод).
Вывод строк через функцию printFromPGM (тех, которые часто повторяютя), макросы для вывода строк, сэкономлено пару байт; //16.01.22
- v2.3.7 Настройка минимального и максимального напряжения! (полезно при настройке показа предупреждений о низком или высоком напряжении,
мигании светодиода, также уровень минимального напряжения участвует в сохранении параметров в EEPROM при отключении), оптимизация записи 
и чтения EEPROM (теперь настройки сохраняются даже при отключении напряжения (из меню), тратится меньше процессорного времени, параметры меняются быстрее, 
меньше изнашивается память) //22.01.21
- v2.4.0 Добавлена поддержка активной пищалки (buzzer), добавлены директивы препроцессора ((#ifdef ... #endif) bufferBatt и RPMwarning, которые
включают код с обработкой буферного аккумулятора и предупреждения о высоких оборотах), убран серьёзный баг, который мог негативно влиять на
перезагрузку контроллера сторожевым таймером WDT, т.е. контроллер уходил в bootloop.	Это связано с тем, что контроллер автоматически ставит таймаут WDT на 16 мс и, 
если функция watchdog.enable() стоит не в начале, код до неё может выполняться дольше 16 мс (особенно со старым загрузчиком) и контроллер уходит в bootloop, 
вернее WDT перезагружает контроллер каждые 16 мс. Может случиться, что даже если сбросили таймер в начале сетапа, контроллер всё равно уходит в bootloop, 
тогда необходимо перепрошить загрузчик на optiboot или убрать его совсем. //2.02.2022 
- v2.4.1 Ускорена работа программы, количество тиков уменьшено с 51211 до 44237 (через бенчмарк AlexGyver`а). Применена библиотека GyverMAX6675, исправлен баг. //4.02.2022

![Снимок экрана (338)](https://user-images.githubusercontent.com/98914596/152585565-215a4ede-800a-409a-a3e9-cb3105b1311e.png)
![Снимок экрана (340)](https://user-images.githubusercontent.com/98914596/152585400-2c122783-a323-41d0-a48a-5dc0df8fae0f.png)

- v2.4.2 Оптимизация некоторых условий, вырезаны лишние действия, исправлены баги, функция настройки максимальной температуры цилиндров (при привышении этой температуры
показывается предупреждение о перегреве цилиндров); //23.02.22 
- v2.4.3 Режим для активного/пассивного зуммера, переключается дефайном, режим теста зуммера в настройках, отлажено поведение светодиода в настройках; //5.03.22 

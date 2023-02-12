/*версия для бортового компьютера на плате
  Настройки: энкодер (EB_BETTER_ENC (установлен по умолчанию с версии 2.0 библиотеки), EB_HALFSTEP_ENC, EB_FAST, keypin, A, B (пины энкодера)),
  указатели поворота (turnpin1, turnpin2), TM1637 (CLK, DIO), аналоговые преобразования (analogpin1, analogpin2, сопротивление резисторов r1, r2,
  r3, r4 в делителе напряжения, калибровка calibration1, calibration2), калибровка температуры процессора (tempsizing), пин пищалки (buzz),
  LiquidCrystal_I2C (настройка адреса), MAX6675_DELAY (задержка переключения CLK в микросекундах для улучшения связи по длинным проводам),
  настройка меню (SETTINGS_AMOUNT, FAST_STEP), настройки препроцессором (bufferBatt, RPMwarning, buzzPassive, buzzActive, OneCylinder, TwoCylinders,
  switchonanimation). Остальные настройки производятся через меню бортового компьютера.
  Можно искать настройки по тексту программы через Ctrl + F*/

#pragma message "Version 2.9.4.2"
#include <EEPROM.h>
#include <GyverWDT.h> // библиотека сторожевого таймера
#include <LiquidCrystal_I2C.h>
#define MAX6675_DELAY 15 // задержка переключения CLK в микросекундах (для улучшения связи по длинным проводам)
#include <GyverMAX6675.h>
#include <GyverTM1637.h>
#include <GetVolt.h>// библиотека для получения напряжения
#include <CPUTemperature.h>// пин А7 невозможно больше использовать
#include <Tachometer.h>
#define EB_HALFSTEP_ENC // режим для полушаговых энкодеров
#define EB_FAST 60      // таймаут быстрого поворота, мс
#include <EncButton.h>
#include <GyverTimers.h>// бибилиотека для управления системными таймерами


/*-----------------------Предустановки-----------------------*/
//#define bufferBatt  // включение обработки буферного аккумулятора
//#define RPMwarning  // включение предупреждения о высоких оборотах
//#define buzzPassive // дефайнить, если пищалка пассивная
#define buzzActive   // дефайнить, если пищалка активная
//#define OneCylinder // Настройка количества цилиндров двигателя
#define TwoCylinders
//#define switchonanimation // Анимация при включении
/*-----------------------------------------------------------*/


/*--Предупраждения о несовместимости определений--*/
/*--------------------------------------------------------!НЕ КОММЕНТИРОВАТЬ!----------------------------------------------*/
#if defined buzzPassive && defined buzzActive
# error "incompatible definitions buzzActive & buzzPassive" // нельзя одновременно дефайнить buzzActive и buzzPassive
#endif
#if ! defined buzzPassive && ! defined buzzActive
# define noPiezo // без пьезоэлемента - пищалки
# pragma message "Warning! buzzPassive or buzzActive are not defined"
#endif
#if defined OneCylinder && defined TwoCylinders
# error "incompatible definitions OneCylinder & TwoCylinders" // нельзя одновременно дефайнить OneCylinder и TwoCylinders
#endif
#if ! defined OneCylinder && ! defined TwoCylinders
# error "OneCylinder or TwoCylinders are not defined" // необходимо задефайнить хотя бы одну из настроек OneCylinder и TwoCylinders
#endif

#if defined buzzPassive || defined buzzActive // с пьезоэлементом
# define withPiezo
#endif
/*-------------------------------------------------------------------------------------------------------------------------*/

#define thermoSO 6  // Определяем константу с указанием № вывода Arduino к которому подключён вывод DO  ( SO, MISO ) модуля на чипе MAX6675
#define thermoCS 5  // Определяем константу с указанием № вывода Arduino к которому подключён вывод CS  ( SS )       модуля на чипе MAX6675
#define thermoSCK 4 // Определяем константу с указанием № вывода Arduino к которому подключён вывод CLK ( SCK )      модуля на чипе MAX6675

#ifdef TwoCylinders
# define thermoSO2 7 // для правого датчика
# define thermoCS2 8
# define thermoSCK2 9
#endif

#define turnpin1 12 // пины указателей поворота
#define turnpin2 11

#define CLK 0 // tm1637
#define DIO 1
#define keypin 13 // кнопка
#define ledpin 3 // светодиод
#define buzz 10 // пин пищалки
#define A 16    // A2 пины энкодера
#define B 17    // A3

#define analogpin1 14 // A0
#define r1 22700.0    // сопротивление резистора r1
#define r2 2710.0     // сопротивление резистора r2
#define calibration1 1.126

#define analogpin2 15 // A1
#define r3 46450.0
#define r4 5580.0
#define calibration2 1.12

#define tempsizing 296.89 // калибровочное значение для измерения температуры процессора
#define tempGain 0.94

/*для меню настроек*/
#define LINES 2       // количество строк дисплея
#if defined withPiezo
# define SETTINGS_AMOUNT 8  // количество настроек (16 ячеек EEPROM int)
#elif defined noPiezo
# define SETTINGS_AMOUNT 6 // количество настроек - 2
#endif
#define FAST_STEP 10   // скорость изменения при быстром повороте

CPUTemperature temperature(tempsizing, tempGain); // прописывем конструктор с передачей калибровочных параметров для получения температуры процессора
// указываем пины в порядке SCK SO CS
GyverMAX6675<thermoSCK, thermoSO, thermoCS> thermo;
#ifdef TwoCylinders
GyverMAX6675<thermoSCK2, thermoSO2, thermoCS2> thermo2;
#endif
LiquidCrystal_I2C lcd(0x27, 16, LINES); // адрес 0x27, сегменты и строки дисплея
GyverTM1637 disp(CLK, DIO);
Tachometer tacho;
EncButton<EB_TICK, A, B, keypin> enc(INPUT_PULLUP);   // энкодер с кнопкой <A, B, KEY> (A, B, KEY - номера пинов)
GetVolt firstbatt (r1, r2, calibration1);
#ifdef bufferBatt
GetVolt secondbatt (r3, r4, calibration2);
#endif

float input_volt = 0.0, buff_input_volt = 0.0;

boolean z, j, Hold, L;  // z - для мигания текстом и светодиодом, j - для мигания светодиода при высоких оборотах,
boolean ledState = LOW; // Hold - в меню неастроек, L - обновление значений счётчика моточасов
int t1, t2;          // t1, t2 - температура с термопар
uint16_t R;             // R - для работы с RPM
float e_hours, maxV, minV, minVMH; // моточасы, максимальное, минимальное напряжение, напряжение сохранения моточасов
uint8_t bv;             // bv - для показа напряжения буферного аккумулятора;
uint32_t myTimer4;
volatile uint8_t m, h;  // время поездки - минуты, часы объявляем volatile, т.к. обрабатываются прерыванием

// для показа свободной оперативки
extern int __bss_end;
extern void *__brkval;

const byte rightcursor[8] = {B11000, B11100, B11110, B11111, B11111, B11110, B11100, B11000}; // стрелка направо >
const byte leftcursor[8] = {B00011, B00111, B01111, B11111, B11111, B01111, B00111, B00011}; // стрелка налево <
const byte degree[8] = {140, 146, 146, 140, 128, 128, 128, 128}; // символ градуса
const byte battL[8] = {B00100, B11111, B10000, B10010, B10111, B10010, B10000, B11111}; //  _-____-_
const byte battR[8] = {B00100, B11111, B00001, B00001, B11101, B00001, B00001, B11111}; // | +    - |
//                                                                                         |________|
static const uint8_t CRTgammaPGM[32] PROGMEM = {
  0, 1, 3, 5, 6, 8, 12, 14, 17, 21, 25, 30, 35, 40, 46, 53,
  59, 67, 75, 84, 94, 103, 115, 127, 139, 153, 167, 182, 199, 216, 235, 255
};

/*для обработки энкодера и меню в LCD1602*/
// названия параметров (max 12 букв)
const char name1[] PROGMEM = "DSBrightness";
const char name2[] PROGMEM = "MaxCylTemp";
const char name3[] PROGMEM = "MinVoltage";
const char name4[] PROGMEM = "MaxVoltage";
#if defined withPiezo
const char name5[] PROGMEM = "Buzz Enable";
const char name6[] PROGMEM = "Buzzer Test";
#endif
const char name7[] PROGMEM = "LED-PWM";
const char name8[] PROGMEM = "MotorH-to-0";

// объявляем таблицу ссылок на параметры
#if defined withPiezo
const char* const names[] PROGMEM = {
  name1, name2, name3, name4, name5, name6, name7, name8
};
#elif defined noPiezo
const char* const names[] PROGMEM = {
  name1, name2, name3, name4, name7, name8
};
#endif

int vals[SETTINGS_AMOUNT];  // массив параметров для сохранения настроек
int8_t arrowPos = 0;
bool controlState = 0; // для изменения режима в меню

/* ---Описание функций--- */
// код скомпилируется быстрее
inline __attribute__((always_inline)) void thermocouple();
inline __attribute__((always_inline)) void isButtonSingle();
inline __attribute__((always_inline)) void isButtonDouble();
void lcdUpdate();
void printGUI();
inline __attribute__((always_inline)) void printFromPGM(int charMap);
inline __attribute__((always_inline)) void smartArrow(bool state1);
inline __attribute__((always_inline)) uint16_t memoryFree();


void setup() {
  // !!!Обязательно размещается в начале setup, иначе уходит в bootloop!!!
  Watchdog.enable(RESET_MODE, WDT_PRESCALER_512);// режим сброса при зависании, таймаут 4 сек.
  // Либо размещается в любом месте сетапа, но с условием отключения WDT в начале сeтапа функцией watchdog.disable()
  // Это связано с тем, что контроллер автоматически ставит таймаут WDT на 16 мс, и, если функция watchdog.enable() стоит не в начале, код до неё может
  // выполняться дольше 16 мс - контроллер уходит в bootloop, (WDT перезагружает контроллер каждые 16 мс). Если контроллер всё равно уходит в bootloop,
  // даже когда таймер сброшен, необходимо перепрошить загрузчик или убрать его совсем.

  lcd.init();// инициализация lcd1602
  //pinMode(turnpin1, INPUT); // для указателей поворота | при загрузке скетча через USBasp отсоединить
  //pinMode(turnpin2, INPUT); // для указателей поворота | провода от D11, D12, D13 (MOSI, MISO, SCK)
  pinMode(ledpin, OUTPUT);
#if defined withPiezo
  pinMode(buzz, OUTPUT);
#endif

  analogPrescaler(128);// !!!ВНИМАНИЕ предделитель АЦП 128 - наивысшая точность (при использовании GyverCore)

  analogReference(INTERNAL);// опорное напряжения для аналоговых измерений 1.1вольт
  attachInterrupt(0, sens, FALLING); // прерывание на 2 пин(2пин-0, 3пин-1)
  tacho.setWindow(5);// установка количества тиков для счёта времени (по умолч 10)
  EEPROM.get(4, vals);// получаем весь массив из EEPROM
  disp.brightness(vals[0]);// Яркость индикатора (0-7)
  minV = float(vals[2]) * 0.1;
  minVMH = minV - 1.5; // из мин. напряжения вычитаем 1.5 вольта, чтобы моточасы записывались только при выключении
  maxV = float(vals[3]) * 0.1;
  // Если перенастроить 0 таймер, не будут работать delay(), millis(), micros() и т.д.
  // На 1 таймере может некорректно работать ШИМ на 9 и 10 пинах, а также библиотека Servo!
  // На 2 таймере отключится tone()
  Timer1.setPeriod(1000000);// устанавливем период для счёта времени в мкс с момента включения м-к (1 секунда)
  Timer1.enableISR();// с этого момента начнётся счёт

  lcd.createChar(1, degree);// Загружаем массив с символом градуса «°» в 1 ячейку ОЗУ дисплея
  lcd.createChar(2, rightcursor);
  lcd.createChar(3, leftcursor);
  lcd.createChar(4, battL);
  lcd.createChar(5, battR);
  // Важный момент: обёрнутые в F() строки оптимизируются, то есть одинаковые строки не дублируются в памяти!
  // Поэтому можно использовать макрос в разных участках программы, одинаковые строки не нужно выносить глобально и делать их общими – это сделает компилятор
  lcd.backlight();// подсветка lcd1602
  thermocouple();
  lcdUpdate();
#ifdef switchonanimation
  byte ON[4] = {0, 0, 0, 0};
  disp.twist(ON, 27);// анимация при включении
  disp.clear();
#endif
  myTimer4 = millis();
}


void sens() {
  tacho.tick(); // в прерывании вазываем tick для обработки RPM (Библиотека Tachometer)
}


ISR(TIMER1_A) {// прерывание для счёта времени
  static volatile uint8_t sec;
  if (++sec > 59) {// если после инкрементирования секунд больше, чем 59
    sec = 0;
    if (++m > 59) {// проверяем, после инкрементирования минут больше ли 59
      m = 0;
      ++h;// и прибавляем часы
    }
  }
}


void loop() {
  /*--обработка энкодера с кнопкой--*/
  enc.tick();// обработчик энкодера с кнопкой

  // смена режимов показа на дисплее
  if (enc.held()) {
    Hold = !Hold;
    switch (Hold) {
      case 0:
        EEPROM.put(4, vals); // запись при переходе ИЗ режима настроек
        minV = float(vals[2]) * 0.1;// делим на 10, чтобы получить флоат с 1 знаком после точки
        maxV = float(vals[3]) * 0.1;
        minVMH = minV - 1.5; // из мин. напряжения вычитаем 1.5 вольта, чтобы моточасы записывались только при выключении
        digitalWrite (ledpin, LOW);
        lcdUpdate();// Hold == 0, очищаем дисплей
        break;
      case 1: // при переходе в режим настройки
#if defined withPiezo
        analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[6]])));
#elif deifned noPiezo
        analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[4]])));
#endif
        disp.displayByte(_t, _u, _n, _e);
        lcd.clear();
        printGUI();// выводим интерфейс меню
        break;
    }
  }


  switch (Hold) {
    /* --обработка в режиме настроек-- */
    case 1: {
        // если отключили питание в меню, то сохраняем настройки
        if (input_volt < minV) EEPROM.put(4, vals);

        if (enc.click()) {
          controlState = !controlState;
          printGUI(); // печатаем на дисплее (названия настроек)
        }

        if (enc.turn()) { // если повернули (факт поворота)
          switch (controlState) {
            case 0: // управляем ВЫБОРОМ НАСТРОЕК
              arrowPos += enc.dir();  // двигаем курсор. dir возвращает 1 или -1
              arrowPos = constrain(arrowPos, 0, SETTINGS_AMOUNT - 1); // ограничиваем позицию стрелки
              break;

            case 1: // управляем ПАРАМЕТРАМИ
              // меняем параметры по позиции стрелки
              switch (enc.dir()) { // если поворот быстрый, прибавляем FAST_STEP, иначе прибавляем 1
                case 1:  vals[arrowPos] += enc.fast() ? FAST_STEP : 1; break;
                case -1: vals[arrowPos] += enc.fast() ? -FAST_STEP : -1; break;
              }
#if defined withPiezo // для системы с пьезоэлементом

              switch (arrowPos) {// ограничиваем только изменённые настройки
                case 0: vals[0] = constrain(vals[0], 0, 7); break; //ограничиваем парметр яркости дисплея
                case 1: vals[1] = constrain(vals[1], 0, 800); break; // параметр максимальной температуры цилиндров
                case 2: vals[2] = constrain(vals[2], 0, vals[3]); break; //  minV   параметры int - потом делятся на 10 и получаются float
                case 3: vals[3] = constrain(vals[3], vals[2], 999); break; //maxV
                case 4: vals[4] = constrain(vals[4], 0, 1); break; // пищалка указателей поворота
                case 5: vals[5] = constrain(vals[5], 0, 1); // тест пищалки
                  switch (vals[5]) {
                    case 1:
# ifdef buzzPassive
                      tone (buzz, 2000);
# elif defined buzzActive
                      digitalWrite (buzz, HIGH);
# endif
                      break;
                    case 0:
# ifdef buzzPassive
                      noTone (buzz);
# elif defined buzzActive
                      digitalWrite (buzz, LOW);
# endif
                      break;
                  }
                  break;
                case 6: vals[6] = constrain(vals[6], 0, 31); // параметр яркости светодиода
                  analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[6]]))); // CRT коррекция с 32 уровнями
                  break;
                case 7: if (vals[7]) { // если равно единице обнуляем счётчик моточасов (покрутить энкодер)
                    vals[7] = 0; // параметр обнуления моточасов
                    e_hours = 0;
                    EEPROM.put(0, e_hours); // запись моточасов в нулевую ячейку памяти
                  }
                  break;
              }

#elif defined noPiezo// для системы БЕЗ пьезоэлемента:

              switch (arrowPos) {
                case 0: vals[0] = constrain(vals[0], 0, 7); break;    // ограничиваем парметр яркости дисплея
                case 1: vals[1] = constrain(vals[1], 0, 800); break;  // параметр максимальной температуры цилиндров
                case 2: vals[2] = constrain(vals[2], 0, vals[3]); break;    // minV
                case 3: vals[3] = constrain(vals[3], vals[2], 999); break;  // maxV
                case 4: vals[4] = constrain(vals[4], 0, 31); // параметр яркости светодиода
                  analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[4]])));
                  break;
                case 5: if (vals[5]) {
                    vals[5] = 0;
                    e_hours = 0;
                    EEPROM.put(0, e_hours);
                  }
                  break;
              }
#endif
              break;
          } // switch (controlState)
          printGUI();
        }
      }
      break; //!конец switch-case: 1!


    case 0: {// действия, которые выполняются не в режиме настроек !Hold
        /* --обработка одинарного и двойного нажатий-- */
        if (enc.hasClicks(1)) isButtonSingle();
        else if (enc.hasClicks(2)) {
          L = true;// для обновления моточасов при двойном нажатии
          isButtonDouble();
        }

        /* --обработка указателей поворота-- */
        static bool ls, TurnOff;
        bool l = digitalRead(turnpin1);// переменная флага включения левого указателя. Потом по флагу работаем с проецированием на экран
        if (l || digitalRead(turnpin2) == HIGH) {// если какой-то из указателей загорелся
          myTimer4 = millis(); // запоминаем время для избежания наложения включений светодиода
          ls = l;// запоминаемм, чтобы потом выключить нужную стрелку
          // включаем светодиод и пищим
#if defined withPiezo
          if (vals[4]) {
# ifdef buzzActive
            digitalWrite (buzz, HIGH);// пищим если разрешено в настройках
# elif defined buzzPassive
            tone (buzz, 2000);
# endif
          }
#endif
          digitalWrite(ledpin, HIGH);
          switch (l) {
            case 1:
              lcd.setCursor(8, 1);
              lcd.print(char(3));
              break;
            case 0:
              lcd.setCursor(9, 1);
              lcd.print(char(2));
              break;
          }
          TurnOff = true;// флаг для однократного выключения
        }
        else if (TurnOff) {// чтобы постоянно не выключался светодиод, выключаем по флагу
          TurnOff = false;
#if defined withPiezo
          if (vals[4]) {
# ifdef buzzActive
            digitalWrite (buzz, LOW);
# elif defined buzzPassive
            noTone(buzz);
# endif
          }
#endif
          digitalWrite(ledpin, LOW);
          switch (ls) {
            case 1: lcd.setCursor(8, 1); break;
            case 0: lcd.setCursor(9, 1); break;
          }
          lcd.print(F(" "));// выключаем нужную стрелку
        }

        /* --вывод ошибок и неисправностей-- */
        static uint16_t myTimer3;
        uint16_t ms3 = (uint16_t)millis();
        if (ms3 - myTimer3 >= 1500) { // для мигания текстом LCD 1602
          myTimer3 = ms3;
          z = !z;
#if defined bufferBatt && defined TwoCylinders
          if ((buff_input_volt < minV) && (++bv > 2)) bv = 0; // переключаем индекс для предупреждений буферного акб: 0 - 1 - 2 - 0
#endif
        }

        static bool flag; // флаг для однократной очистки дисплея, если выводились какие-либо строки
        lcd.setCursor(8, 0);
#ifdef TwoCylinders
        if (t1 > vals[1] || t2 > vals[1]) // если температура больше заданной
#elif defined OneCylinder
        if (t1 > vals[1])
#endif
        {
          switch (z) {
            case 1:
              lcd.print(F("OVERheat"));
              flag = true;
              break;
            case 0:
              if (input_volt < minV) lcd.print(F("LOWvolt "));
              else if (input_volt > maxV) lcd.print(F("OVERvolt"));
              else if (flag) {
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }
        }
        else if (input_volt < minV) {
          switch (z) {
            case 1:
              lcd.print(F("LOWvolt "));
              flag = true;
              break;
            case 0:
              if (flag) {
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }
        }
        else if (input_volt > maxV) {
          switch (z) {
            case 1:
              lcd.print(F("OVERvolt"));
              flag = true;
              break;
            case 0:
              if (flag) {
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }
        }
        else if (R < 500) {
          lcd.print(F("  Hello "));
          flag = true;
        }
        else if (flag) {
          lcd.print(F("        "));
          flag = false;
        }

        /* --вывод информации о буферном аккумуляторе-- */
#ifdef bufferBatt
# ifdef TwoCyliners
        if (buff_input_volt < minV) {
          lcd.setCursor(10, 1);
          switch (bv) {
            case 0:
              lcd.print(F("LOW Bv"));
              break;
            case 1:
              lcd.print(F("BV"));
              lcd.print(buff_input_volt);
              break;
            case 2:
              lcd.print(F("V "));
              lcd.print(input_volt);
              break;
          }
          buff_input_volt = secondbatt.getVolt(analogRead(analogpin2));
          if (buff_input_volt >= minV) { // при выходе из цикла напечатать значок
            lcd.setCursor(10, 1);
            lcd.print(char(4));
            lcd.print(char(5));
          }
        }
# endif

# ifdef OneCylinder
        if (buff_input_volt < minV) {
          lcd.setCursor(0, 1);
          switch (z) {
            case 0:
              lcd.print(F("BV="));
              lcd.print(buff_input_volt);
              break;
            case 1:
              lcd.print(F("LOW Bv  "));
              break;
          }
          buff_input_volt = secondbatt.getVolt(analogRead(analogpin2));
          if (buff_input_volt >= minV) {// при выходе из цикла напечатать значок
            lcd.setCursor(0, 1);
            lcd.print(F("BV="));
          }
        }
# endif
#endif

        /* --управление светодиодом-- */
        static bool le;
#if defined TwoCylinders
        if ((millis() - myTimer4 > 1200) && ((t1 > vals[1]) ||  (t2 > vals[1]) || (input_volt < minV) || (input_volt > maxV)))
#elif defined OneCylinder
        if ((millis() - myTimer4 > 1200) && ((t1 > vals[1]) || (input_volt < minV) || (input_volt > maxV)))
#endif
        {
          // управляем светодиодом если прошло больше секунды с момента включения указателей поворота
          switch (z) {
            case 1:
              ledState = HIGH;
              le = true;
              break;
            case 0:
              if ((digitalRead(turnpin1) == LOW) || (digitalRead(turnpin2) == LOW)) {
                ledState = LOW;
                le = false;
              }
              break;
          }
          digitalWrite(ledpin, ledState);
        } else if (le) { // при определённых условиях светодиод может не выключиться, этот код предотвратит это
          digitalWrite(ledpin, LOW);// если произойдёт выход из прошлого if, и светодиод не выключится, этот код однократно выключит светодиод
          le = false;
        }

        /* --если кол-во оборотов больше 5800, то включать, выключать светодиод-- */
#ifdef RPMwarning
        if ((millis() - myTimer4 > 1200) && R >= 5800) {
          static uint16_t myTimer;
          uint16_t ms = (uint16_t)millis();
          if (ms - myTimer >= 400) {
            myTimer = ms;
            j = !j;
          }
          switch (j) {
            case 1: ledState = HIGH; break;
            case 0: ledState = LOW; break;
          }
          digitalWrite(ledpin, ledState);
        }
#endif
      }
      break; //!Конец switch-case: 0!
  }


  /* --вывод значения тахометра и получение напряжения-- */
  static uint16_t myTimer2;
  uint16_t ms2 = (uint16_t)millis();
  if (ms2 - myTimer2 > 100) {
    myTimer2 = ms2;
#if defined TwoCylinders
    if (!Hold) disp.displayInt(R = (tacho.getRPM() >> 1));// делим на 2
#elif defined OneCylinder
    if (!Hold) disp.displayInt(R = tacho.getRPM());
#endif
    /* --обработка вольтметра-- */
    input_volt = firstbatt.getVolt(analogRead(analogpin1));// передаём параметры в функцию получения напряжения
#ifdef bufferBatt
    buff_input_volt = secondbatt.getVolt(analogRead(analogpin2));
#endif
  }

  /*записываем значание моточасов в память при выключении м-к или при нажатии на кнопку*/
  if (input_volt < minVMH || L) {
    static uint32_t sec1;// sec2 делаем просто локальной, а sec1 - static, чтобы сохраняла значение между вызовами функции
    uint32_t sec2;
    L = false;
    EEPROM.get(0, e_hours);// читаем из памяти
    sec2 = millis();// запоминаем текущее время
    // разницу настоящего значения времени и предыдущего в миллисекундах преобразуем в десятичные часы
    e_hours += (sec2  - sec1) / 3.6E6; // 3600000
    EEPROM.put(0, e_hours);// записываем в ЭСППЗУ
    sec1 = sec2;// запоминаем время для следующей итерации
  }

  /* --вывод информации на дисплей-- */
  static uint16_t myTimer1;
  uint16_t ms1 = (uint16_t)millis();
  if (ms1 - myTimer1 >= 1000) {
    myTimer1 = ms1;
    Watchdog.reset();// защита от зависания - сбрасываем таймер Watchdog раз в секунду
    thermocouple();
    if (!Hold) {
      lcd.setCursor(3, 0);
      lcd.print(t1);
      if (t1 == 10 || t1 == 100) lcd.print(F("\1C"));// печатаем символ градуса, если температура увеличилась на разряд
      else if (t1 == 9 || t1 == 99) lcd.print(F("\1C "));// если температура уменьшилась на разряд, печатаем символ градуса, а предыдущий очищаем

#ifdef TwoCylinders
      lcd.setCursor(3, 1);
      lcd.print(t2);
      if (t2 == 10 || t2 == 100) lcd.print(F("\1C"));
      else if (t2 == 9 || t2 == 99) lcd.print(F("\1C "));

#endif

#if defined bufferBatt && defined TwoCylinders
      if (buff_input_volt >= minV) {
#endif
        // напряжение питания выводим по умолчанию
        lcd.setCursor(12, 1);
        lcd.print(input_volt);

#if defined bufferBatt && defined TwoCylinders
      }
#endif

#if defined bufferBatt && defined OneCylinder
      if (buff_input_volt >= minV) {
        lcd.setCursor(3, 1); // выводим напряжение буферного аккумулятора под температурой
        lcd.print(buff_input_volt);
      }
#endif
    }
  }
}

/* --получение температуры термопар-- */
void thermocouple() {
  t1 = thermo.readTemp() ? (thermo.getTempInt() - 2) : NAN;
#ifdef TwoCylinders
  t2 = thermo2.readTemp() ? (thermo2.getTempInt() - 2) : NAN;
#endif
}


/* --выводим версию программы, напряжение буферного аккумулятора (если есть) и время поездки-- */
void isButtonSingle() { // действия после одиночного нажатия кнопки
  uint32_t myTimer = millis();
  digitalWrite(ledpin, LOW);
  disp.displayByte(_U, _2, _9, _4);// выводим версию программы на дисплей
  lcd.clear();
  Watchdog.reset();// сбрасываем таймер перед циклом
  while (millis() - myTimer < 3650) {
    lcd.home();
    lcd.print(F("Elapsed T: "));
    lcd.print(h);
    lcd.print(F(":"));
    lcd.print(m);
#if defined bufferBatt && defined TwoCylinders
    lcd.setCursor(0, 1);
    lcd.print(F("Buff Voltage: "));
    lcd.print(buff_input_volt);
#endif
  }
  lcdUpdate();
  disp.clear();
}

/* --выводим температуру процессора и моточасы-- */
void isButtonDouble() { // действия после двойного нажатия кнопки
  uint32_t myTimer = millis();
  digitalWrite(ledpin, LOW);
  disp.displayInt(memoryFree());
  lcd.clear();
  float CPUt = temperature.getCPUTemp();
  EEPROM.get(0, e_hours);
  Watchdog.reset();// сбрасываем таймер перед циклом
  while (millis() - myTimer < 3650) { // время не должно быть больше периода Watchdog
    lcd.home();
    lcd.print(F("motor hours:"));
    lcd.print(e_hours);
    lcd.setCursor(0, 1);
    lcd.print(F("CPU temp:")); // выводим температуру процессора
    lcd.print(CPUt);
  }
  lcdUpdate();
  disp.clear();
}

/* --обновление LCD при переходе на другие экраны-- */
void lcdUpdate() {
  lcd.clear();
#ifdef TwoCylinders
  lcd.print(F("tL="));
  lcd.print(t1);
  lcd.print(F("\1C"));//символ градуса
  lcd.setCursor(0, 1);
  lcd.print(F("tR="));
  lcd.print(t2);
  lcd.print(F("\1C"));
#elif defined OneCylinder
  lcd.print(F("tc="));
  lcd.print(t1);
  lcd.print(F("\1C"));
# ifdef bufferBatt
  lcd.setCursor(0, 1);
  lcd.print(F("BV="));
# endif
#endif
  lcd.setCursor(10, 1);
  lcd.print(char(4));// левая половина значка аккумулятора
  lcd.print(char(5));// правая половина
}

/* --печать интерфейса в меню настроек-- */
void printGUI() {
  static int8_t screenPos = 0; // номер "экрана"
  static int8_t lastScreen = 0; // предыдущий номер "экрана"

  screenPos = arrowPos / LINES;   // ищем номер экрана (0..1 - 0, 2..4 - 1)
  if (lastScreen != screenPos) lcd.clear(); // если экран сменился - очищаем
  lastScreen = screenPos;

  for (byte i = 0; i < LINES; i++) {  // для всех строк
    lcd.setCursor(0, i);              // курсор в начало

    // если курсор находится на выбранной строке
    smartArrow(arrowPos == LINES * screenPos + i);  // рисуем стрелку или пробел

    // если пункты меню закончились, покидаем цикл for
    if (LINES * screenPos + i == SETTINGS_AMOUNT) break;

    // выводим имя и значение пункта меню
    printFromPGM(&names[LINES * screenPos + i]);// выводим названия пунктов меню через функцию
    lcd.print(F(": "));
    lcd.print(vals[LINES * screenPos + i]);// выводим значение параметров
    lcd.print(F("  ")); // пробелы для очистки
  }
}

// очень хитрая функция для печати из PROGMEM
void printFromPGM(int charMap) {
  uint8_t ptr = pgm_read_word(charMap);    // получаем адрес из таблицы ссылок
  while (pgm_read_byte(ptr) != NULL) {      // всю строку до нулевого символа
    lcd.print(char(pgm_read_byte(ptr)));    // выводим названия пунктов меню
    ptr++;                                  // следующий символ
  }
}


void smartArrow(bool state1) {  // рисует стрелку, галку или пробел
  lcd.write(state1 ? (controlState ? 62 : 126) : 32);
}


uint16_t memoryFree() {// функция вывода свободной оперативки
  uint16_t freeValue;
  if ((uint16_t)__brkval == 0)
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)&__bss_end);
  else
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)__brkval);
  return freeValue;
}

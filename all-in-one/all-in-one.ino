/*версия для бортового компьютера на плате
  Настройки библиотек: setEncType(), EB_FAST_TIME, MAX6675_DELAY
  Пользовательские настройки находятся в Confihuration.h */

#pragma message "Version 3.1.4"
#include <EEPROM.h>
#include <GyverWDT.h>
#include <LiquidCrystal_I2C.h>
#define MAX6675_DELAY 16  // задержка переключения CLK в микросекундах (для улучшения связи по длинным проводам)
#include <GyverMAX6675.h>
#include <GyverTM1637.h>
#include <GetVolt.h>
#include <CPUTemperature.h>  // пин А7 невозможно больше использовать
#include <Tachometer.h>
#define EB_NO_FOR        // отключить поддержку pressFor/holdFor/stepFor и счётчик степов
#define EB_NO_CALLBACK   // отключить обработчик событий attach
#define EB_NO_COUNTER    // отключить счётчик энкодера
#define EB_NO_BUFFER     // отключить буферизацию (т.к. обработка энкодера осуществляется не в прерывании)
#define EB_FAST_TIME 60  // таймаут быстрого поворота, мс
#include <EncButton.h>
#include <GyverTimers.h>
#include <GyverIO.h>

#include "Configuration.h"

/*--Предупраждения о несовместимости определений--*/
/*==================================================!НЕ КОММЕНТИРОВАТЬ!==================================================*/
#if defined BUZZER_PASSIVE && defined BUZZER_ACTIVE
#error "incompatible definitions BUZZER_PASSIVE & BUZZER_ACTIVE" // нельзя одновременно дефайнить BUZZER_ACTIVE и BUZZER_PASSIVE
#elif !defined BUZZER_PASSIVE && !defined BUZZER_ACTIVE
#define NO_PIEZO
#pragma message "Warning! BUZZER_PASSIVE or BUZZER_ACTIVE are not defined"
#endif
#if defined ONE_CYLINDER && defined TWO_CYLINDERS
#error "incompatible definitions ONE_CYLINDER & TWO_CYLINDERS" // нельзя одновременно дефайнить ONE_CYLINDER и TWO_CYLINDERS
#elif !defined ONE_CYLINDER && !defined TWO_CYLINDERS
#error "ONE_CYLINDER or TWO_CYLINDERS are not defined"         // необходимо задефайнить хотя бы одну из настроек ONE_CYLINDER и TWO_CYLINDERS
#endif

#if defined BUZZER_PASSIVE || defined BUZZER_ACTIVE  // с пьезоэлементом
#define WITH_PIEZO
#endif
/*------------------------------------------------служебные определения--------------------------------------------------*/
#if defined WITH_PIEZO

#define CRT_VALS 5  // номер параметра яркости светодиода в массиве vals
#if defined ECO_RPM
#define SETTINGS_AMOUNT 8
#define ECO_RPM_VALS 7  // номер параметра включения eco-оборотов
#else
#define SETTINGS_AMOUNT 7
#endif

#elif defined NO_PIEZO

#define CRT_VALS 4
#if defined ECO_RPM
#define SETTINGS_AMOUNT 7
#define ECO_RPM_VALS 6
#else
#define SETTINGS_AMOUNT 6
#endif

#endif

/*--------- для работы с PGM строками -----------*/
#define FPSTR(pstr) (const __FlashStringHelper*)(pstr)

/*=======================================================================================================================*/

/*-------------- для меню настроек --------------*/
#define LINES 2       // количество строк lcd
#define FAST_STEP 10  // скорость изменения при быстром повороте

/*================= БИБЛИОТЕКИ ==================*/
CPUTemperature temperature(TEMP_OFFSET, TEMP_GAIN);
GyverMAX6675<SCK_PIN, SO_PIN, CS_PIN> thermo;  // указываем пины в порядке SCK SO CS
#ifdef TWO_CYLINDERS
GyverMAX6675<SCK_PIN_2, SO_PIN_2, CS_PIN_2> thermo2;
#endif
LiquidCrystal_I2C lcd(LCD_ADDR, 16, LINES);  // адрес, сегменты и строки дисплея
GyverTM1637 disp(CLK, DIO);
Tachometer tacho;
EncButtonT<A, B, KEY_PIN> enc(INPUT, INPUT);  // энкодер с кнопкой <A, B, KEY> (A, B, KEY - номера пинов)
GetVolt firstbatt(R1, R2, CALIBRATION_1);
#ifdef BUFFER_BATTERY
GetVolt secondbatt(R3, R4, CALIBRATION_2);
#endif
/*================= ПЕРЕМЕННЫЕ ==================*/
float input_volt = 0.0, buff_input_volt = 0.0;

bool Hold, bp;
uint16_t R, t1, t2;                 // R - для работы с RPM, t1, t2 - температура с термопар
float e_hours, maxV, minV, minVMH;  // моточасы, максимальное, минимальное напряжение, напряжение сохранения моточасов
uint16_t myTimer4;
volatile uint8_t m, h;  // время поездки - минуты, часы объявляем volatile, т.к. обрабатываются прерыванием

/*------- для показа свободной оперативки --------*/
extern int __bss_end;
extern void* __brkval;

/*-------------- для загрузки в LCD --------------*/
const byte rightcursor[8] = { B01000, B01100, B01110, B11111, B11111, B01110, B01100, B01000 };  // стрелка вправо >
const byte leftcursor[8] = { B00010, B00110, B01110, B11111, B11111, B01110, B00110, B00010 };   // стрелка влево <
const byte degree[8] = { 140, 146, 146, 140, 128, 128, 128, 128 };                               // символ градуса     _-____-_
const byte battL[8] = { B00100, B11111, B10000, B10010, B10111, B10010, B10000, B11111 };        // | +    - |
const byte battR[8] = { B00100, B11111, B00001, B00001, B11101, B00001, B00001, B11111 };        // |________|
/*-------- для избежания повторения строк --------*/
const char voidStr[] PROGMEM = "        ";
const char lowVolt[] PROGMEM = "LOWvolt ";
const char overVolt[] PROGMEM = "OVERvolt";
const char degC[] PROGMEM = "\1C";
const char degCgap[] PROGMEM = "\1C ";
/*---------- гамма-коррекция светодиода ----------*/
static const uint8_t CRTgammaPGM[32] PROGMEM = {
  0, 1, 3, 5, 6, 8, 12, 14, 17, 21, 25, 30, 35, 40, 46, 53,
  59, 67, 75, 84, 94, 103, 115, 127, 139, 153, 167, 182, 199, 216, 235, 255
};

/*--- для обработки энкодера и меню в LCD1602 ---*/
// названия параметров (max 12 букв)
const char name1[] PROGMEM = "DSBrightness";
const char name2[] PROGMEM = "MaxCylTemp";
const char name3[] PROGMEM = "MinVoltage";
const char name4[] PROGMEM = "MaxVoltage";
#if defined WITH_PIEZO
const char name5[] PROGMEM = "Buzz Enable";
#endif
const char name6[] PROGMEM = "LED-PWM";
const char name7[] PROGMEM = "MotorH-to-0";
#if defined ECO_RPM
const char name8[] PROGMEM = "ecoRPM note";
#endif

/*--------- таблица ссылок на параметры ---------*/
const char* const names[] PROGMEM = {
  name1, name2, name3, name4
#if defined WITH_PIEZO
  ,
  name5
#endif
  ,
  name6, name7
#if defined ECO_RPM
  ,
  name8
#endif
};

/*---------------- меню настроек ----------------*/
int vals[SETTINGS_AMOUNT];  // массив параметров для сохранения настроек
int8_t arrowPos = 0;
bool controlState = 0;  // для изменения режима в меню

/*--- структуры для удобной работы с флагами ---*/
struct {
  bool high;
  bool low;
  bool lowMH;
} volt;

struct {
  bool low;
} bufVolt;

struct {
  bool high;
} temp;

/*============== ОПИСАНИЕ ФУНКЦИЙ ===============*/
inline __attribute__((always_inline)) void sensorsProcessing();
inline __attribute__((always_inline)) void mainGUI();
inline __attribute__((always_inline)) void thermocouple();
inline __attribute__((always_inline)) void isButtonSingle();
inline __attribute__((always_inline)) void isButtonDouble();
void lcdUpdate();
inline __attribute__((always_inline)) void menuHandler();
void menuGUI();
inline __attribute__((always_inline)) void printFromPGM(const char* const* charMap);
inline __attribute__((always_inline)) void smartArrow(bool state1);
void beep(uint8_t* ms);
inline __attribute__((always_inline)) void beepTick(uint8_t* ms);
inline __attribute__((always_inline)) uint16_t memoryFree();


void setup() {
  // !!!Обязательно размещается в НАЧАЛЕ setup, иначе уходит в bootloop!!!
  Watchdog.enable(RESET_MODE, WDT_PRESCALER_512);  // режим сброса при зависании, таймаут 4 сек.

  lcd.init();
  gio::mode(LED_PIN, OUTPUT);
#ifdef ECO_RPM
  gio::mode(ECO_LED_PIN, OUTPUT);
#endif
#ifdef BUZZER_ACTIVE
  gio::mode(BUZZER_PIN, OUTPUT);
#endif

  analogPrescaler(128);  // !!!ВНИМАНИЕ предделитель АЦП 128 - наивысшая точность (при использовании GyverCore)

  analogReference(INTERNAL);
  attachInterrupt(0, sens, FALLING);  // прерывание на 2 пин(2 пин-0, 3 пин-1)
  tacho.setWindow(5);                 // установка количества тиков для счёта времени (по умолч 10)
  EEPROM.get(4, vals);                // получаем весь массив
  disp.brightness(vals[0]);           // Яркость индикатора (0-7)
  minV = float(vals[2]) * 0.1;
  minVMH = minV - 0.6;  // из мин. напряжения вычитаем 0.8 вольт, чтобы моточасы записывались только при выключении
  maxV = float(vals[3]) * 0.1;
  /* Если перенастроить 0 таймер, не будут работать delay(), millis(), micros() и т.д.
    На 1 таймере может некорректно работать ШИМ на 9 и 10 пинах, а также библиотека Servo!
    На 2 таймере отключится tone()*/
  Timer1.setPeriod(1000000);  // устанавливем период для счёта времени в мкс с момента включения м-к (1 секунда)
  Timer1.enableISR();         // с этого момента начнётся счёт

  lcd.createChar(1, degree);  // Загружаем массив с символом градуса «°» в 1 ячейку ОЗУ дисплея
  lcd.createChar(2, rightcursor);
  lcd.createChar(3, leftcursor);
  lcd.createChar(4, battL);
  lcd.createChar(5, battR);

  thermocouple();
  lcdUpdate();
  lcd.backlight();
#ifdef SWITCH_ON_ANIMATION
  byte ON[4] = { 0, 0, 0, 0 };
  disp.twist(ON, 27);  // анимация при включении
  disp.clear();
#endif
  enc.setEncType(EB_STEP2);  // установка типа энкодера
  myTimer4 = (uint16_t)millis();
}


void sens() {
  tacho.tick();  // в прерывании вазываем tick для обработки RPM (Библиотека Tachometer)
}


ISR(TIMER1_A) {  // прерывание для счёта времени
  static volatile uint8_t sec;
  if (++sec > 59) {  // если после инкрементирования секунд больше, чем 59
    sec = 0;
    if (++m > 59) {  // проверяем, после инкрементирования минут больше ли 59
      m = 0;
      ++h;  // и прибавляем часы
    }
  }
}


void loop() {
  /*--ОБРАБОТКА ЭНКОДЕРА С КНОПКОЙ--*/
  static bool L;  // L - обновление значений счётчика моточасов

  if (enc.tick()) {    // обработчик энкодера с кнопкой
    if (enc.hold()) {  // смена режимов показа на дисплее
      Hold = !Hold;
      switch (Hold) {
        case 0:
          EEPROM.put(4, vals);          // запись при переходе ИЗ режима настроек
          minV = float(vals[2]) * 0.1;  // делим на 10, чтобы получить флоат с 1 знаком после точки
          maxV = float(vals[3]) * 0.1;
          minVMH = minV - 0.6;  // из мин. напряжения вычитаем 0.6 вольт, чтобы моточасы записывались только при выключении
          gio::write(LED_PIN, LOW);
          disp.clear();
#ifdef BUFFER_BATTERY
          bufVolt.low = false;
#endif
          volt.low = false;
          volt.high = false;
          temp.high = false;
          lcdUpdate();
          break;
        case 1:  // при переходе в режим настройки
          analogWrite(LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[CRT_VALS]])));
#if defined ECO_RPM
          analogWrite(ECO_LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[CRT_VALS]])));
#endif
          disp.displayByte(_t, _u, _n, _e);
          lcd.clear();
          menuGUI();  // выводим интерфейс меню
          break;
      }
    }
  }


  switch (Hold) {
    case 1:           // обработка в режиме настроек
      menuHandler();  // фукция обработки меню
      break;

    case 0:
      {  // действия, которые выполняются не в режиме настроек !Hold
        /* --ОБРАБОТКА ОДИНАРНОГО И ДВОЙНОГО НАЖАТИЙ-- */
        if (enc.hasClicks()) {
          switch (enc.getClicks()) {
            case 1: isButtonSingle(); break;
            case 2:
              L = true;  // для обновления моточасов при двойном нажатии
              isButtonDouble();
              break;
          }
        }

        /* --ОБРАБОТКА УКАЗАТЕЛЕЙ ПОВОРОТА-- */
        static bool TurnOff;
        // флаг включения левого указателя
        bool l = gio::read(TURN_PIN_1);
        uint8_t Ind = l + gio::read(TURN_PIN_2);
        if (Ind) {                          // если какой-то из указателей загорелся
          if (!TurnOff) {                   // для однократного выполнения кода:
            myTimer4 = (uint16_t)millis();  // запоминаем время для избежания наложения включений светодиода
            gio::write(LED_PIN, HIGH);
            switch (Ind) {
              case 2:  // включение обоих указателей поворота (режим аварийки)
                lcd.setCursor(8, 1);
                lcd.print(char(3));
                lcd.print(char(2));
                break;
              default:
#if defined WITH_PIEZO
                if (vals[4])
#ifdef BUZZER_ACTIVE
                  gio::write(BUZZER_PIN, HIGH);  // пищим если разрешено в настройках
#elif defined BUZZER_PASSIVE
                  tone(BUZZER_PIN, BUZZER_FREQUENCY);
#endif
#endif
                if (l) {
                  lcd.setCursor(8, 1);
                  lcd.print(char(3));
                } else {
                  lcd.setCursor(9, 1);
                  lcd.print(char(2));
                }
                break;
            }
            TurnOff = true;  // флаг для однократного выключения
          }
        } else if (TurnOff) {  // чтобы постоянно не выключался светодиод, выключаем по флагу
          TurnOff = false;
#if defined WITH_PIEZO
          if (vals[4])
#ifdef BUZZER_ACTIVE
            gio::write(BUZZER_PIN, LOW);
#elif defined BUZZER_PASSIVE
            noTone(BUZZER_PIN);
#endif
#endif
          gio::write(LED_PIN, LOW);
          lcd.setCursor(8, 1);
          lcd.print(F("  "));  // выключаем нужную стрелку
        }

        /* --ВЫВОД ПРЕДУПРЕЖДЕНИЙ-- */
        static bool z;  // для мигания текстом и светодиодом
        static uint16_t myTimer3;
        uint16_t ms3 = (uint16_t)millis();
        if (ms3 - myTimer3 >= 1500) {  // для мигания текстом LCD 1602
          myTimer3 = ms3;
          z = !z;
          /* --управление светодиодом-- */
          static bool le;
          if ((uint16_t)millis() - myTimer4 > 2000 && (volt.low || volt.high || temp.high)) {  // если какой-то из показателей превысил норму
            // управляем светодиодом, если прошло больше секунды с момента включения указателей поворота
            gio::toggle(LED_PIN);
            le = z;
          } else if (le) {
            gio::write(LED_PIN, LOW);  // если произойдёт выход из прошлого if, и светодиод не выключится, этот код однократно выключит светодиод
            le = false;
          }

          /* --ВЫВОД ИНФОРМАЦИИ О БУФЕРНОМ АККУМУЛЯТОРЕ-- */
#ifdef BUFFER_BATTERY
          static bool flagBv;
          if (bufVolt.low) {
#if defined TWO_CYLINDERS
            flagBv = true;         // флаг выключения
            static uint8_t bv;     // для показа напряжения буферного аккумулятора;
            if (++bv > 2) bv = 0;  // переключаем индекс для предупреждений буферного акб: 0 - 1 - 2 - 0
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
                lcd.print(F("V="));
                lcd.print(input_volt);
                break;
            }

#elif defined ONE_CYLINDER
            lcd.setCursor(0, 1);
            switch (z) {
              case 0:
                lcd.print(F("BV="));
                lcd.print(buff_input_volt);
                break;
              case 1:
                lcd.print(F("LOW Bv  "));
                flagBv = true;
                break;
            }
#endif
          } else if (flagBv) {  // при нормализации напряжения напечатать исходные символы
            flagBv = false;
#if defined TWO_CYLINDERS
            lcd.setCursor(10, 1);
            lcd.print(char(4));
            lcd.print(char(5));
#elif defined ONE_CYLINDER
            lcd.setCursor(0, 1);
            lcd.print(F("BV="));
#endif
          }
#endif
        }

        static bool flag;  // флаг для однократной очистки дисплея, если выводились какие-либо строки
        if (temp.high) {
          static bool f;  // флаг для однократной печати на дисплее
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag || f) {  // печатаем один раз (если не печатали до этого)
                lcd.print(F("OVERheat"));
                flag = true;
                f = false;
              }
              break;
            case 0:
              if (!f) {  // печатаем один раз или если до этого выводилось OVERheat (при z == 1)
                if (volt.low) {
                  lcd.print(FPSTR(lowVolt));
                  flag = true;  // подняли, чтобы стереть в конце
                } else if (volt.high) {
                  lcd.print(FPSTR(overVolt));
                  flag = true;
                } else if (flag) {
                  lcd.print(FPSTR(voidStr));
                  flag = false;  // опустили, чтобы не стирать в конце
                }
                f = true;  // подняли, чтобы вывести OVERheat при z == 1
              }
              break;
          }
        } else if (volt.low) {
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag) {
                lcd.print(FPSTR(lowVolt));
                flag = true;
              }
              break;
            case 0:
              if (flag) {
                lcd.print(FPSTR(voidStr));
                flag = false;
              }
              break;
          }
        } else if (volt.high) {
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag) {
                lcd.print(FPSTR(overVolt));
                flag = true;
              }
              break;
            case 0:
              if (flag) {
                lcd.print(FPSTR(voidStr));
                flag = false;
              }
              break;
          }
        } else switch (R) {
            case 0 ... 500:  // если RPM < 500
              if (!flag) {
                lcd.setCursor(8, 0);
                lcd.print(F("  Hello "));
                flag = true;
              }
              break;
            default:
              if (flag) {
                lcd.setCursor(8, 0);
                lcd.print(FPSTR(voidStr));
                flag = false;
              }
              break;
          }

            /* --если кол-во оборотов больше 5500, то включать, выключать светодиод-- */
#ifdef RPM_WARNING
        if ((uint16_t)millis() - myTimer4 > 2000 && R >= 5500) {
          static uint16_t myTimer;
          uint16_t ms = (uint16_t)millis();
          if (ms - myTimer >= 400) {
            myTimer = ms;
            gio::toggle(LED_PIN);
          }
        }
#endif
      }
      break;
      // !Конец switch(Hold)-case: 0!
  }


  /* --ВЫВОД ЗНАЧЕНИЯ ТАХОМЕТРА И ОБРАБОТКА НАПРЯЖЕНИЯ-- */
  sensorsProcessing();

  /*записываем значание моточасов в память при выключении м-к или при нажатии на кнопку*/
  if (volt.lowMH || L) {
    static uint32_t sec1;  // sec2 делаем просто локальной, а sec1 - static, чтобы сохраняла значение между вызовами функции
    uint32_t sec2;
    L = false;
    EEPROM.get(0, e_hours);  // читаем из памяти
    sec2 = millis();         // запоминаем текущее время
    // разницу настоящего значения времени и предыдущего в миллисекундах преобразуем в десятичные часы
    e_hours += (sec2 - sec1) / 3.6E6;  // 3'600'000
    EEPROM.put(0, e_hours);
    sec1 = sec2;  // запоминаем время для следующей итерации
  }

  /* --ВЫВОД ИНФОРМАЦИИ НА ДИСПЛЕЙ-- */
  mainGUI();
}


void sensorsProcessing() {
  /* --ОБРАБОТКА ТАХОМЕТРА-- */
  if (!Hold) {  // если не в режиме настроек
    static uint8_t tmr;
    uint8_t ms = (uint8_t)millis();
    if (uint8_t(ms - tmr) > 180) {
      tmr = ms;
      static uint16_t prevR;
#if defined TWO_CYLINDERS
      R = uint16_t(tacho.getRPM()) >> 1;
#elif defined ONE_CYLINDER
      R = tacho.getRPM();
#endif

      // если предыдущее значение RPM не равно настроящему, выводим на индикатор
      if (prevR != R) {
#if defined ECO_RPM
        if (vals[ECO_RPM_VALS])  // если разрешено в настройках
          switch (R) {           // включаем светотодиод в эко-интервале
            case ECO_INTERVAL: gio::write(ECO_LED_PIN, HIGH); break;
            default: gio::write(ECO_LED_PIN, LOW); break;
          }
#endif
        disp.displayInt(R);
        prevR = R;  // запоминаем новое значение
      }
    }
  }

  /* --ОБРАБОТКА ВОЛЬТМЕТРА-- */
  static uint8_t tmrv;
  uint8_t msv = (uint8_t)millis();
  if (uint8_t(msv - tmrv) > 50) {
    tmrv = msv;
    input_volt = firstbatt.getVolt(analogRead(ANALOG_PIN_1));
#ifdef BUFFER_BATTERY
    buff_input_volt = secondbatt.getVolt(analogRead(ANALOG_PIN_2));
    bufVolt.low = (buff_input_volt < minV) ? true : false;
#endif

    // поднимаем флаги высокого/низкого напряжения и напряжения lowMH (записи в EEPROM)
    if (input_volt < minV) {
      volt.low = true;
      volt.lowMH = (input_volt < minVMH) ? true : false;
    } else {
      volt.low = false;
      volt.high = (input_volt > maxV) ? true : false;
    }
  }
}

/* ---ОТРИСОВКА ОСНОВНОГО ИНТЕРФЕЙСА--- */
void mainGUI() {
  static uint16_t tmr;
  uint16_t ms = (uint16_t)millis();
  if (ms - tmr >= 1000) {
    tmr = ms;
    Watchdog.reset();  // защита от зависания - сбрасываем таймер Watchdog раз в секунду
    if (!Hold) {
      thermocouple();
      static uint16_t prevT1, prevT2;
      if (prevT1 != t1) {  // печатаем на дисплее только если изменилась температура
        lcd.setCursor(3, 0);
        lcd.print(t1);
        switch (t1) {
          case 10:
          case 100: lcd.print(FPSTR(degC)); break;  // печатаем символ градуса, если температура увеличилась на разряд
          case 9:
          case 99: lcd.print(FPSTR(degCgap)); break;  // если температура уменьшилась на разряд, печатаем символ градуса, а предыдущий очищаем
        }
        prevT1 = t1;
      }
#ifdef TWO_CYLINDERS
      if (prevT2 != t2) {
        lcd.setCursor(3, 1);
        lcd.print(t2);
        switch (t2) {
          case 10:
          case 100: lcd.print(FPSTR(degC)); break;
          case 9:
          case 99: lcd.print(FPSTR(degCgap)); break;
        }
        prevT2 = t2;
      }
#endif

#ifdef BUFFER_BATTERY

#if defined ONE_CYLINDER
      lcd.setCursor(12, 1);
      lcd.print(input_volt);
      if (!bufVolt.low) {     // если напряж. в норме
        lcd.setCursor(3, 1);  // выводим напряжение буферного аккумулятора под температурой
        lcd.print(buff_input_volt);
      }
#elif defined TWO_CYLINDERS
      if (!bufVolt.low) {
        lcd.setCursor(12, 1);
        lcd.print(input_volt);
      }
#endif

#else
      lcd.setCursor(12, 1);
      lcd.print(input_volt);
#endif
    }
  }
}


/* --ПОЛУЧЕНИЕ ТЕМПЕРАТУРЫ ТЕРМОПАР-- */
void thermocouple() {
  t1 = thermo.readTemp() ? thermo.getTempInt() : NAN;
#ifdef TWO_CYLINDERS
  t2 = thermo2.readTemp() ? thermo2.getTempInt() : NAN;
  /*если температура больше заданной, поднимаем флаг*/
  temp.high = (t1 > vals[1] || t2 > vals[1]) ? true : false;
#else
  temp.high = (t1 > vals[1]) ? true : false;
#endif
}


/* --выводим версию программы, напряжение буферного аккумулятора (если есть) и время поездки-- */
void isButtonSingle() {  // действия после ОДИНОЧНОГО нажатия кнопки
  gio::write(LED_PIN, LOW);
  disp.displayByte(_U, _3, _1, _4);  // выводим версию программы на дисплей
  lcd.clear();
  lcd.print(F("Elapsed T: "));
  lcd.print(h);
  lcd.print(F(":"));
  lcd.print(m);
#if defined BUFFER_BATTERY && defined TWO_CYLINDERS
  lcd.setCursor(0, 1);
  lcd.print(F("Buff Volt: "));
  lcd.print(buff_input_volt);
#endif
  Watchdog.reset();  // сбрасываем таймер перед циклом
  delay(3650);
  lcdUpdate();
  disp.clear();
}

/* --выводим температуру процессора и моточасы-- */
void isButtonDouble() {  // действия после ДВОЙНОГО нажатия кнопки
  gio::write(LED_PIN, LOW);
  disp.displayInt(memoryFree());
  lcd.clear();
  EEPROM.get(0, e_hours);
  lcd.print(F("motor hours:"));
  lcd.print(e_hours);
  lcd.setCursor(0, 1);
  lcd.print(F("CPU temp:"));
  delay(100);  // доп. стабилизация напряжения
  lcd.print(temperature.getCPUTemp());
  Watchdog.reset();  // сбрасываем таймер перед циклом
  delay(3650);       // время не должно быть больше периода Watchdog
  lcdUpdate();
  disp.clear();
}

/* --ОБНОВЛЕНИЕ LCD ПРИ ПЕРЕХОДЕ НА ДРУГИЕ ЭКРАНЫ-- */
void lcdUpdate() {
  lcd.clear();
#ifdef TWO_CYLINDERS
  lcd.print(F("tL="));
  lcd.print(t1);
  lcd.print(FPSTR(degC));  // символ градуса
  lcd.setCursor(0, 1);
  lcd.print(F("tR="));
  lcd.print(t2);
  lcd.print(FPSTR(degC));
#elif defined ONE_CYLINDER
  lcd.print(F("tc="));
  lcd.print(t1);
  lcd.print(FPSTR(degC));
#ifdef BUFFER_BATTERY
  lcd.setCursor(0, 1);
  lcd.print(F("BV="));
#endif
#endif
  lcd.setCursor(10, 1);
  lcd.print(char(4));  // левая половина значка аккумулятора
  lcd.print(char(5));  // правая половина
  lcd.print(input_volt);
}

/* ---ОБРАБОТЧИК МЕНЮ НАСТРОЕК--- */
void menuHandler() {
  // если отключили питание в меню, то сохраняем настройки
  if (volt.lowMH) {
    noInterrupts();  // запрет прерываний для максимально быстрой записи в EEPROM
    EEPROM.put(4, vals);
    interrupts();  // разрешаем прерывания
  }
  static uint8_t buzzTmr;

#if defined WITH_PIEZO
  // если произошло какое-то действие с энкодером или кнопкой
  if (bp) beepTick(&buzzTmr);  // вызываем функцию отключения пьезоэлемента
#endif

  switch (enc.action()) {
    case EB_CLICK:
#if defined WITH_PIEZO
      beep(&buzzTmr);
#endif
      controlState = !controlState;
      menuGUI();  // печатаем на дисплее (названия настроек)
      break;

    case EB_TURN:  // если повернули (факт поворота)
#if defined WITH_PIEZO
      beep(&buzzTmr);
#endif
      switch (controlState) {
        case 0:                                                    // управляем ВЫБОРОМ НАСТРОЕК
          arrowPos += enc.dir();                                   // двигаем курсор. dir возвращает 1 или -1
          arrowPos = constrain(arrowPos, 0, SETTINGS_AMOUNT - 1);  // ограничиваем позицию стрелки
          break;

        case 1:  // управляем ПАРАМЕТРАМИ
          // меняем параметры по позиции стрелки
          switch (enc.dir()) {  // если поворот быстрый, прибавляем FAST_STEP, иначе прибавляем 1
            case 1: vals[arrowPos] += enc.fast() ? FAST_STEP : 1; break;
            case -1: vals[arrowPos] += enc.fast() ? -FAST_STEP : -1; break;
          }

#if defined WITH_PIEZO                                                  // для системы с пьезоэлементом
          switch (arrowPos) {                                           // ограничиваем только изменённые настройки
            case 0: vals[0] = constrain(int8_t(vals[0]), 0, 7); break;  // ограничиваем парметр яркости дисплея
            case 1: vals[1] = constrain(vals[1], 0, 800); break;        // параметр максимальной температуры цилиндров
            case 2: vals[2] = constrain(vals[2], 0, vals[3]); break;    // minV   параметры int - потом делятся на 10 и получаются float
            case 3: vals[3] = constrain(vals[3], vals[2], 999); break;  // maxV
            case 4: vals[4] = bool(vals[4]); break;                     // пищалка указателей поворота
            case 5:
              vals[5] = constrain(int8_t(vals[5]), 0, 31);                   // параметр яркости светодиода
              analogWrite(LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[5]])));  // CRT коррекция с 32 уровнями
#ifdef ECO_RPM
              analogWrite(ECO_LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[5]])));
#endif
              break;
            case 6:
              if (vals[6]) {  // если равно единице обнуляем счётчик моточасов
                vals[6] = 0;
                e_hours = 0;
                EEPROM.put(0, e_hours);
              }
              break;
#ifdef ECO_RPM
            case 7: vals[7] = bool(vals[7]); break;  // включение предупреждения eco-оборотов
#endif
          }

#elif defined NO_PIEZO  // для системы БЕЗ пьезоэлемента:
          switch (arrowPos) {
            case 0: vals[0] = constrain(int8_t(vals[0]), 0, 7); break;
            case 1: vals[1] = constrain(vals[1], 0, 800); break;
            case 2: vals[2] = constrain(vals[2], 0, vals[3]); break;
            case 3: vals[3] = constrain(vals[3], vals[2], 999); break;
            case 4:
              vals[4] = constrain(int8_t(vals[4]), 0, 31);  // параметр яркости светодиода
              analogWrite(LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[4]])));
#ifdef ECO_RPM
              analogWrite(ECO_LED_PIN, pgm_read_byte(&(CRTgammaPGM[vals[4]])));
#endif
              break;
            case 5:
              if (vals[5]) {
                vals[5] = 0;
                e_hours = 0;
                EEPROM.put(0, e_hours);
              }
              break;
#ifdef ECO_RPM
            case 6: vals[6] = bool(vals[6]); break;
#endif
          }
#endif
          break;
      }  // switch (controlState)
      menuGUI();
      break;
  }  // switch (enc.action())
}


/* ----ОТРИСОВКА ИНТЕРФЕЙСА В МЕНЮ НАСТРОЕК---- */
void menuGUI() {
  static int8_t screenPos = 0;   // номер "экрана"
  static int8_t lastScreen = 0;  // предыдущий номер "экрана"

  screenPos = arrowPos / LINES;              // ищем номер экрана (0..1 - 0, 2..4 - 1)
  if (lastScreen != screenPos) lcd.clear();  // если экран сменился - очищаем
  lastScreen = screenPos;

  for (byte i = 0; i < LINES; i++) {  // для всех строк
    lcd.setCursor(0, i);              // курсор в начало

    // если курсор находится на выбранной строке
    smartArrow(arrowPos == LINES * screenPos + i);  // рисуем стрелку или пробел

    // если пункты меню закончились, покидаем цикл for
    if (LINES * screenPos + i == SETTINGS_AMOUNT) break;

    // выводим имя и значение пункта меню
    printFromPGM(&names[LINES * screenPos + i]);  // выводим названия пунктов меню через функцию
    lcd.print(F(": "));
    lcd.print(vals[LINES * screenPos + i]);  // выводим значение параметров
    lcd.print(F("  "));                      // пробелы для очистки
  }
}

/* ---очень хитрая ФУНКЦИЯ ДЛЯ ПЕЧАТИ ИЗ PROGMEM--- */
void printFromPGM(const char* const* charMap) {
  uint8_t ptr = pgm_read_word(charMap);   // получаем адрес из таблицы ссылок
  while (pgm_read_byte(ptr) != NULL) {    // всю строку до нулевого символа
    lcd.print(char(pgm_read_byte(ptr)));  // выводим названия пунктов меню
    ptr++;                                // следующий символ
  }
}


void smartArrow(bool state1) {  // рисует стрелку, галку или пробел
  lcd.write(state1 ? (controlState ? 62 : 126) : 32);
}

/* ---ФУНКЦИИ ЗВУКОВОЙ ИНДИКАЦИИ В МЕНЮ НАСТРОЕК--- */
void beep(uint8_t* ms) {
  bp = true;  // флаг выключения
  *ms = (uint8_t)millis();

#if defined BUZZER_ACTIVE
  gio::write(BUZZER_PIN, HIGH);

#elif defined BUZZER_PASSIVE
  tone(BUZZER_PIN, BUZZER_FREQUENCY);
#endif
}

void beepTick(uint8_t* ms) {
  if (uint8_t((uint8_t)millis() - *ms) > 20) {
    bp = false;

#if defined BUZZER_ACTIVE
    gio::write(BUZZER_PIN, LOW);

#elif defined BUZZER_PASSIVE
    noTone(BUZZER_PIN);
#endif
  }
}

/* ---ФУНКЦИЯ ВЫВОДА СВОБОДНОЙ ОПЕРАТИВКИ--- */
uint16_t memoryFree() {
  uint16_t freeValue;
  if ((uint16_t)__brkval == 0)
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)&__bss_end);
  else
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)__brkval);
  return freeValue;
}

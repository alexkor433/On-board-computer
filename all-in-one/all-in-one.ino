/*версия для бортового компьютера на плате
  Настройки библиотек: EB_BETTER_ENC (установлен по умолчанию с версии 2.0 библиотеки), EB_HALFSTEP_ENC, EB_FAST, MAX6675_DELAY
  Пользовательские настройки находятся в Confihuration.h */

#pragma message "Version 2.9.5.2"
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

#include "Configuration.h"

/*--Предупраждения о несовместимости определений--*/
/*--------------------------------------------------------!НЕ КОММЕНТИРОВАТЬ!----------------------------------------------*/
#if defined buzzPassive && defined buzzActive
# error "incompatible definitions buzzActive & buzzPassive" // нельзя одновременно дефайнить buzzActive и buzzPassive
#elif ! defined buzzPassive && ! defined buzzActive
# define noPiezo // без пьезоэлемента - пищалки
# pragma message "Warning! buzzPassive or buzzActive are not defined"
#endif
#if defined OneCylinder && defined TwoCylinders
# error "incompatible definitions OneCylinder & TwoCylinders" // нельзя одновременно дефайнить OneCylinder и TwoCylinders
#elif ! defined OneCylinder && ! defined TwoCylinders
# error "OneCylinder or TwoCylinders are not defined" // необходимо задефайнить хотя бы одну из настроек OneCylinder и TwoCylinders
#endif

#if defined buzzPassive || defined buzzActive // с пьезоэлементом
# define withPiezo
#endif
/*-------------------------------------------------------------------------------------------------------------------------*/

/*для меню настроек*/
#define LINES 2       // количество строк дисплея
#if defined withPiezo
# define SETTINGS_AMOUNT 8  // количество настроек (16 ячеек EEPROM int)
#elif defined noPiezo
# define SETTINGS_AMOUNT 6 // количество настроек - 2
#endif
#define FAST_STEP 10   // скорость изменения при быстром повороте

CPUTemperature temperature(tempsizing, tempGain);
// указываем пины в порядке SCK SO CS
GyverMAX6675<thermoSCK, thermoSO, thermoCS> thermo;
#ifdef TwoCylinders
GyverMAX6675<thermoSCK2, thermoSO2, thermoCS2> thermo2;
#endif
LiquidCrystal_I2C lcd(lcdAddr, 16, LINES); // адрес 0x27, сегменты и строки дисплея
GyverTM1637 disp(CLK, DIO);
Tachometer tacho;
EncButton<EB_TICK, A, B, keypin> enc(INPUT_PULLUP);   // энкодер с кнопкой <A, B, KEY> (A, B, KEY - номера пинов)
GetVolt firstbatt (r1, r2, calibration1);
#ifdef bufferBatt
GetVolt secondbatt (r3, r4, calibration2);
#endif

float input_volt = 0.0, buff_input_volt = 0.0;

bool Hold;
uint16_t R, t1, t2; // R - для работы с RPM, t1, t2 - температура с термопар
float e_hours, maxV, minV, minVMH; // моточасы, максимальное, минимальное напряжение, напряжение сохранения моточасов
uint16_t myTimer4;
volatile uint8_t m, h;  // время поездки - минуты, часы объявляем volatile, т.к. обрабатываются прерыванием

// для показа свободной оперативки
extern int __bss_end;
extern void *__brkval;

const byte rightcursor[8] = {B11000, B11100, B11110, B11111, B11111, B11110, B11100, B11000}; // стрелка направо >
const byte leftcursor[8] = {B00011, B00111, B01111, B11111, B11111, B01111, B00111, B00011}; // стрелка налево <
const byte degree[8] = {140, 146, 146, 140, 128, 128, 128, 128}; // символ градуса          _-____-_
const byte battL[8] = {B00100, B11111, B10000, B10010, B10111, B10010, B10000, B11111}; // | +    - |
const byte battR[8] = {B00100, B11111, B00001, B00001, B11101, B00001, B00001, B11111}; // |________|

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

int vals[SETTINGS_AMOUNT]; // массив параметров для сохранения настроек
int8_t arrowPos = 0;
bool controlState = 0; // для изменения режима в меню

// структуры для удобной работы с флагами
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

/* ---Описание функций--- */
// код скомпилируется быстрее
inline __attribute__((always_inline)) void sensorsProcessing();
inline __attribute__((always_inline)) void mainGUI();
inline __attribute__((always_inline)) void thermocouple();
inline __attribute__((always_inline)) void isButtonSingle();
inline __attribute__((always_inline)) void isButtonDouble();
void lcdUpdate();
inline __attribute__((always_inline)) void menuHandler();
void menuGUI();
inline __attribute__((always_inline)) void printFromPGM(int charMap);
inline __attribute__((always_inline)) void smartArrow(bool state1);
inline __attribute__((always_inline)) uint16_t memoryFree();


void setup() {
  // !!!Обязательно размещается в начале setup, иначе уходит в bootloop!!!
  Watchdog.enable(RESET_MODE, WDT_PRESCALER_512);// режим сброса при зависании, таймаут 4 сек.
  /* Либо размещается в любом месте сетапа, но с условием отключения WDT в начале сeтапа функцией watchdog.disable()
    Это связано с тем, что контроллер автоматически ставит таймаут WDT на 16 мс, и, если функция watchdog.enable() стоит не в начале, код до неё может
    выполняться дольше 16 мс - контроллер уходит в bootloop, (WDT перезагружает контроллер каждые 16 мс). Если контроллер всё равно уходит в bootloop,
    даже когда таймер сброшен, необходимо перепрошить загрузчик или убрать его совсем.*/

  lcd.init();// инициализация lcd1602
  pinMode(ledpin, OUTPUT);
#ifdef buzzActive
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
  /* Важный момент: обёрнутые в F() строки оптимизируются, то есть одинаковые строки не дублируются в памяти!
    Поэтому можно использовать макрос в разных участках программы, одинаковые строки не нужно выносить глобально и делать их общими – это сделает компилятор*/
  lcd.backlight();// подсветка lcd1602
  thermocouple();
  lcdUpdate();
#ifdef switchonanimation
  byte ON[4] = {0, 0, 0, 0};
  disp.twist(ON, 27);// анимация при включении
  disp.clear();
#endif
  myTimer4 = (uint16_t)millis();
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
  static bool L; // L - обновление значений счётчика моточасов

  if (enc.tick()) { // обработчик энкодера с кнопкой
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
          disp.clear();
          lcdUpdate();
          break;
        case 1: // при переходе в режим настройки
#if defined withPiezo
          analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[6]])));
#elif defined noPiezo
          analogWrite (ledpin, pgm_read_byte(&(CRTgammaPGM[vals[4]])));
#endif
          disp.displayByte(_t, _u, _n, _e);
          lcd.clear();
          menuGUI();// выводим интерфейс меню
          break;
      }
    }
  }


  switch (Hold) {
    case 1: // обработка в режиме настроек
      menuHandler(); // фукция обработки меню
      break;

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
          myTimer4 = (uint16_t)millis(); // запоминаем время для избежания наложения включений светодиода
          ls = l;// запоминаемм, чтобы потом выключить нужную стрелку
          if (!TurnOff) { // для однократного выполнения кода:
#if defined withPiezo
            if (vals[4])
# ifdef buzzActive
              digitalWrite (buzz, HIGH);// пищим если разрешено в настройках
# elif defined buzzPassive
              tone (buzz, buzzFrq);
# endif
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
        }
        else if (TurnOff) {// чтобы постоянно не выключался светодиод, выключаем по флагу
          TurnOff = false;
#if defined withPiezo
          if (vals[4])
# ifdef buzzActive
            digitalWrite (buzz, LOW);
# elif defined buzzPassive
            noTone(buzz);
# endif
#endif
          digitalWrite(ledpin, LOW);
          switch (ls) {
            case 1: lcd.setCursor(8, 1); break;
            case 0: lcd.setCursor(9, 1); break;
          }
          lcd.print(F(" "));// выключаем нужную стрелку
        }

        /* --вывод ошибок и неисправностей-- */
        static uint8_t bv; // для показа напряжения буферного аккумулятора;
        static bool z; // для мигания текстом и светодиодом
        static uint16_t myTimer3;
        uint16_t ms3 = (uint16_t)millis();
        if (ms3 - myTimer3 >= 1500) { // для мигания текстом LCD 1602
          myTimer3 = ms3;
          z = !z;
          /* --управление светодиодом-- */
          static bool le;
          if ((uint16_t)millis() - myTimer4 > 1200 && (volt.low || volt.high || temp.high)) { // если какой-то из показателей превысил норму
            // управляем светодиодом, если прошло больше секунды с момента включения указателей поворота
            switch (z) {
              case 1: digitalWrite(ledpin, HIGH); break;
              case 0: if ((digitalRead(turnpin1) == LOW) || (digitalRead(turnpin2) == LOW)) digitalWrite(ledpin, LOW); break;
            }
            le = z;
          } else if (le) { // при определённых условиях светодиод может не выключиться, этот код предотвратит это
            digitalWrite(ledpin, LOW);// если произойдёт выход из прошлого if, и светодиод не выключится, этот код однократно выключит светодиод
            le = false;
          }
          /*----------------------------*/
#if defined bufferBatt && defined TwoCylinders
          if ((bufVolt.low) && (++bv > 2)) bv = 0; // переключаем индекс для предупреждений буферного акб: 0 - 1 - 2 - 0
#endif
        }

        static bool flag; // флаг для однократной очистки дисплея, если выводились какие-либо строки
        if (temp.high) { // если температура больше заданной
          static bool f; // флаг для однократной печати на дисплее
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag || f) { // печатаем один раз (если не печатали до этого)
                lcd.print(F("OVERheat"));
                flag = true;
                f = false;
              }
              break;
            case 0:
              if (!f) { // печатаем один раз или если до этого выводилось OVERheat (при z == 1)
                if (volt.low) {
                  lcd.print(F("LOWvolt "));
                  flag = true; // подняли, чтобы стереть в конце
                }
                else if (volt.high) {
                  lcd.print(F("OVERvolt"));
                  flag = true;
                }
                else if (flag) {
                  lcd.print(F("        "));
                  flag = false; // опустили, чтобы не стирать в конце
                }
                f = true; // подняли, чтобы вывести OVERheat при z == 1
              }
              break;
          }
        }
        else if (volt.low) {
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag) {
                lcd.print(F("LOWvolt "));
                flag = true;
              }
              break;
            case 0:
              if (flag) {
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }
        }
        else if (volt.high) {
          lcd.setCursor(8, 0);
          switch (z) {
            case 1:
              if (!flag) {
                lcd.print(F("OVERvolt"));
                flag = true;
              }
              break;
            case 0:
              if (flag) {
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }
        }
        else switch (R) {
            case 0 ... 500: // если RPM < 500
              if (!flag) {
                lcd.setCursor(8, 0);
                lcd.print(F("  Hello "));
                flag = true;
              }
              break;
            default:
              if (flag) {
                lcd.setCursor(8, 0);
                lcd.print(F("        "));
                flag = false;
              }
              break;
          }

        /* --вывод информации о буферном аккумуляторе-- */
#ifdef bufferBatt
# ifdef TwoCyliners
        if (bufVolt.low) {
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

# elif defined OneCylinder
        if (bufVolt.low) {
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

        /* --если кол-во оборотов больше 5800, то включать, выключать светодиод-- */
#ifdef RPMwarning
        if ((uint16_t)millis() - myTimer4 > 1200 && R >= 5800) {
          static uint16_t myTimer;
          uint16_t ms = (uint16_t)millis();
          if (ms - myTimer >= 400) {
            static bool j;
            myTimer = ms;
            j = !j;
            digitalWrite(ledpin, j);
          }
        }
#endif
      }
      break; //!Конец switch(Hold)-case: 0!
  }


  /* --вывод значения тахометра и получение напряжения-- */
  sensorsProcessing();

  /*записываем значание моточасов в память при выключении м-к или при нажатии на кнопку*/
  if (volt.lowMH || L) {
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
  mainGUI();
}


void sensorsProcessing() {
  static uint8_t tmr;
  uint8_t ms = (uint8_t)millis();
  if (uint8_t(ms - tmr) > 100) {
    tmr = ms;
    static uint16_t prevR;
#if defined TwoCylinders
    R = tacho.getRPM() >> 1;
#elif defined OneCylinder
    R = tacho.getRPM();
#endif
    // если не в режиме настроек и предыдущее значение RPM не равно настроящему, выводим на индикатор
    if (!Hold && prevR != R) {
      disp.displayInt(R);
      prevR = R; // запоминаем новое значение;
    }
    /* --обработка вольтметра-- */
    input_volt = firstbatt.getVolt(analogRead(analogpin1));// передаём параметры в функцию получения напряжения
#ifdef bufferBatt
    buff_input_volt = secondbatt.getVolt(analogRead(analogpin2));
    bufVolt.low = (buff_input_volt < minV) ? true : false;
#endif
    // поднимаем флаги высокого/низкого напряжения и напряжения lowMH(записи в EEPROM)
    if (input_volt < minV) {
      volt.low = true;
      volt.lowMH = (input_volt < minVMH) ? true : false;
    } else {
      volt.low = false;
      volt.high = (input_volt > maxV) ? true : false;
    }
  }
}


void mainGUI() {
  static uint16_t tmr;
  uint16_t ms = (uint16_t)millis();
  if (ms - tmr >= 1000) {
    tmr = ms;
    Watchdog.reset();// защита от зависания - сбрасываем таймер Watchdog раз в секунду
    static uint16_t prevT1, prevT2;
    thermocouple();
    if (!Hold) {
      if (prevT1 != t1) { // печатаем на дисплее только если изменилась температура
        lcd.setCursor(3, 0);
        lcd.print(t1);
        switch (t1) {
          case 10: case 100: lcd.print(F("\1C")); break; // печатаем символ градуса, если температура увеличилась на разряд
          case 9: case 99: lcd.print(F("\1C ")); break; // если температура уменьшилась на разряд, печатаем символ градуса, а предыдущий очищаем
        }
        prevT1 = t1;
      }
#ifdef TwoCylinders
      if (prevT2 != t2) {
        lcd.setCursor(3, 1);
        lcd.print(t2);
        switch (t2) {
          case 10: case 100: lcd.print(F("\1C")); break;
          case 9: case 99: lcd.print(F("\1C ")); break;
        }
        prevT2 = t2;
      }
#endif

#if defined bufferBatt
      if (!bufVolt.low) { // если напряж. в норме
#endif
        // напряжение питания выводим по умолчанию
        lcd.setCursor(12, 1);
        lcd.print(input_volt);

#if defined bufferBatt && defined OneCylinder
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
  /*если температура больше заданной, поднимаем флаг*/
  temp.high = (t1 > vals[1] || t2 > vals[1]) ? true : false;
#else
  temp.high = (t1 > vals[1]) ? true : false;
#endif
}


/* --выводим версию программы, напряжение буферного аккумулятора (если есть) и время поездки-- */
void isButtonSingle() { // действия после одиночного нажатия кнопки
  digitalWrite(ledpin, LOW);
  disp.displayByte(_U, _2, _9, _5); // выводим версию программы на дисплей
  lcd.clear();
  lcd.print(F("Elapsed T: "));
  lcd.print(h);
  lcd.print(F(":"));
  lcd.print(m);
#if defined bufferBatt && defined TwoCylinders
  lcd.setCursor(0, 1);
  lcd.print(F("Buff Voltage: "));
  lcd.print(buff_input_volt);
#endif
  Watchdog.reset();// сбрасываем таймер перед циклом
  delay(3650);
  lcdUpdate();
  disp.clear();
}

/* --выводим температуру процессора и моточасы-- */
void isButtonDouble() { // действия после двойного нажатия кнопки
  digitalWrite(ledpin, LOW);
  disp.displayInt(memoryFree());
  lcd.clear();
  EEPROM.get(0, e_hours);
  lcd.print(F("motor hours:"));
  lcd.print(e_hours);
  lcd.setCursor(0, 1);
  lcd.print(F("CPU temp:")); // выводим температуру процессора
  delay(100); // доп. стабилизация напряжения
  lcd.print(temperature.getCPUTemp());
  Watchdog.reset();// сбрасываем таймер перед циклом
  delay(3550); // время не должно быть больше периода Watchdog
  lcdUpdate();
  disp.clear();
}

/* --обновление LCD при переходе на другие экраны-- */
void lcdUpdate() {
  lcd.clear();
#ifdef TwoCylinders
  lcd.print(F("tL="));
  lcd.print(t1);
  lcd.print(F("\1C"));// символ градуса
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


void menuHandler() {
  // если отключили питание в меню, то сохраняем настройки
  if (volt.low) EEPROM.put(4, vals);

  if (enc.click()) {
    controlState = !controlState;
    menuGUI(); // печатаем на дисплее (названия настроек)
  }

  else if (enc.turn()) { // если повернули (факт поворота)
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
# if defined buzzActive
            digitalWrite(buzz, vals[5]);
# elif defined buzzPassive
            switch (vals[5]) {
              case 1: tone (buzz, buzzFrq); break;
              case 0: noTone (buzz); break;
            }
# endif
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
    menuGUI();
  }
}


/* --печать интерфейса в меню настроек-- */
void menuGUI() {
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

//версия для бортового компьютера на плате
//Настройки: энкодер (EB_BETTER_ENC (установлен по умолчанию с версии 2.0), EB_HALFSTEP_ENC, EB_FAST, buttpin, A, B (пины энкодера)),
//указатели поворота (turnpin1, turnpin2), TM1637 (CLK, DIO), аналоговые преобразования (analogpin1, analogpin2, сопротивление резисторов r1, r2,
//r3, r4 в делителе напряжения, калибровка calibration1, calibration2), калибровка температуры процессора (tempsizing), пин пищалки (buzz),
//LiquidCrystal_I2C (настройка адреса), MAX6675_DELAY (задержка переключения CLK в микросекундах для улучшения связи по длинным проводам),
//настройка меню (SETTINGS_AMOUNT, FAST_STEP), настройки препроцессором (bufferBatt, RPMwarning, buzzPassive, buzzActive). Остальные настройки
//производятся через меню бортового компьютера.
//Можно искать настройки по тексту программы через Ctrl + F

#pragma message "Version 2.7.0"
#include <EEPROM.h>
#include <GyverWDT.h> //библиотека сторожевого таймера
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define MAX6675_DELAY 10 //задержка переключения CLK в микросекундах (для улучшения связи по длинным проводам)
#include <GyverMAX6675.h>
#include <GyverTM1637.h>
#include <GetVolt.h>//библиотека для получения напряжения
#include <GetCPUTemp.h>//пин А7 невозможно больше использовать
#include <Tachometer.h>
#define EB_HALFSTEP_ENC //режим для полушаговых энкодеров
#define EB_FAST 65     // таймаут быстрого поворота, мс
#include <EncButton.h>
#include <GyverTimers.h>//бибилиотека для управления системными таймерами

//#define bufferBatt//включение обработки буферного аккумулятора
//#define RPMwarning//включение предупреждения о высоких оборотах
//#define buzzPassive //дефайнить, если пищалка пассивная
#define buzzActive //дефайнить, если пищалка активная

#ifdef buzzPassive && buzzActive //нельзя одновременно дефайнить buzzActive и buzzPassive
#error "incompatible definitions buzzActive & buzzPassive"
#endif
// Переменные, создаваемые процессом сборки,
// когда компилируется скетч для показа свободной оперативки
extern int __bss_end;
extern void *__brkval;

const byte rightcursor[8] = {B11000, B11100, B11110, B11111, B11111, B11110, B11100, B11000}; //стрелка направо >
const byte leftcursor[8] = {B00011, B00111, B01111, B11111, B11111, B01111, B00111, B00011}; //стрелка налево <
const byte degree[8] = {140, 146, 146, 140, 128, 128, 128, 128}; // Определяем массив хрянящий биты символа градуса
const byte battL[8] = {B00100, B11111, B10000, B10010, B10111, B10010, B10000, B11111}; //  _-____-_
const byte battR[8] = {B00100, B11111, B00001, B00001, B11101, B00001, B00001, B11111}; // | +    - |
//                                                                                         |________|
#define thermoSO 6  // Определяем константу с указанием № вывода Arduino к которому подключён вывод DO  ( SO, MISO ) модуля на чипе MAX6675
#define thermoCS 5  // Определяем константу с указанием № вывода Arduino к которому подключён вывод CS  ( SS )       модуля на чипе MAX6675
#define thermoSCK 4 // Определяем константу с указанием № вывода Arduino к которому подключён вывод CLK ( SCK )      модуля на чипе MAX6675

#define thermoSO2 7 //для правого датчика
#define thermoCS2 8
#define thermoSCK2 9

#define turnpin1 12 //пины указателей поворота
#define turnpin2 11

#define CLK 0 //tm1637
#define DIO 1
#define analogpin1 14 //A0 пины для измерения
#ifdef bufferBatt
#define analogpin2 15 //A1 напряжения аккумуляторов
#endif
#define buttpin 13
#define ledpin 3
#define buzz 10//пин пищалки
#define A 16 //A2 пины энкодера
#define B 17 //A3
#define tempsizing 289.0 //калибровочное значение для измерения температуры процессора

GetCPUTemp temperature (tempsizing); //прописывем конструктор с передачей калибровочного параметра для получения температуры процессора
// указываем пины в порядке SCK SO CS
GyverMAX6675<thermoSCK, thermoSO, thermoCS> thermo;
GyverMAX6675<thermoSCK2, thermoSO2, thermoCS2> thermo2;
LiquidCrystal_I2C lcd(0x27, 16, 2); // устанавливаем адрес 0x27 LCD для 16 сегментного и 2 строчного дисплея
GyverTM1637 disp(CLK, DIO);
Tachometer tacho;
/*для меню настроек:*/
#define LINES 2       // количество строк дисплея
#define SETTINGS_AMOUNT 8  // количество настроек
#define FAST_STEP 10   // скорость изменения при быстром повороте

EncButton<EB_TICK, A, B, buttpin> enc;   // энкодер с кнопкой <A, B, KEY> (A, B, KEY - номера пинов)

#define r1 22700.0 //сопротивление резистора r1
#define r2 2710.0 // сопротивление резистора r2
#define calibration1 1.126
GetVolt firstbatt (r1, r2, calibration1);

#ifdef bufferBatt
#define r3 46450.0
#define r4 5580.0
#define calibration2 1.12
GetVolt secondbatt (r3, r4, calibration2);
#endif

float input_volt = 0.0;
float buff_input_volt = 0.0;

boolean z, j, Hold, L, P; //z - для мигания текстом и светодиодом, j - для мигания светодиода при высоких оборотах, P - смотреть на код меню
boolean ledState = LOW;//Hold - в меню неастроек, L - обновление значений счётчика моточасов
int t1, t2, R; //t1, t2 - температура с термопар, R - для работы с RPM
float e_hours, maxV, minV; //моточасы, максимальное и минимальное напряжение
uint8_t m, h, bv; //время поездки - минуты, часы, bv - для показа напряжения буферного аккумулятора;
uint32_t myTimer4;

/*для обработки энкодера и меню в LCD1602*/
// названия параметров (max 12 букв)
const char name1[] PROGMEM = "DSBrightness";
const char name2[] PROGMEM = "MinVoltage";
const char name3[] PROGMEM = "MaxVoltage";
const char name4[] PROGMEM = "Buzz Enable";
const char name5[] PROGMEM = "LED-PWM";
const char name6[] PROGMEM = "MotorH-to-0";
const char name7[] PROGMEM = "MaxCylTemp";
const char name8[] PROGMEM = "Buzzer Test";

// объявляем таблицу ссылок на параметры
const char* const names[] PROGMEM = {
  name1, name2, name3, name4, name5, name6, name7, name8
};

int vals[SETTINGS_AMOUNT];  // массив параметров для сохранения настроек
int8_t arrowPos = 0;
bool controlState = 0; //для изменения режима в меню

/* ---Описание функций--- */
//код скомпилируется быстрее
void thermocouple();
void isButtonSingle();
void isButtonDouble();
void lcdUpdate();
void printGUI();
void printFromPGM(int charMap);
void smartArrow(bool state1);
uint16_t memoryFree();

void setup() {
  //!!!Обязательно размещается в начале setup, иначе уходит в bootloop!!!
  Watchdog.enable(RESET_MODE, WDT_PRESCALER_1024);//режим сброса при зависании, таймаут 8 сек.
  //Либо размещается в любом месте сетапа, но с условием отключения WDT в начале сэтапа функцией watchdog.disable()
  //Это связано с тем, что контроллер автоматически ставит таймаут WDT на 16 мс, и, если функция watchdog.enable() стоит не в начале, код до неё может
  //выполняться дольше 16 мс и контроллер уходит в bootloop, вернее WDT перезагружает контроллер каждые 16 мс. Может случиться, что даже, если сбросили
  //таймер в начале сетапа, контроллер всё равно уходит в bootloop, тогда необходимо перепрошить загрузчик или убрать его совсем.

  lcd.init();//инициализация lcd1602
  //pinMode(turnpin1, INPUT); //для указателей поворота | при загрузке скетча через USBasp отсоединить
  //pinMode(turnpin2, INPUT); //для указателей поворота | провода от D11, D12, D13 (MOSI, MISO, SCK)
  pinMode(ledpin, OUTPUT);
  pinMode(buzz, OUTPUT);

  analogPrescaler(128);//!!!ВНИМАНИЕ предделитель АЦП 128 - наивысшая точность
  analogReference(INTERNAL);//опорное напряжения для аналоговых измерений 1.1вольт
  attachInterrupt(0, sens, FALLING); // прерывание на 2 пин(2пин-0, 3пин-1)
  tacho.setWindow(5);// установка количества тиков для счёта времени (по умолч 10)
  EEPROM.get(4, vals);//получаем весь массив из EEPROM
  disp.brightness(vals[0]);// Яркость индикатора (0-7)
  minV = float(vals[1]) / 10;
  maxV = float(vals[2]) / 10;
  //Если перенастроить 0 таймер, не будут работать delay(), millis(), micros() и т.д.
  //На 1 таймере может некорректно работать ШИМ на 9 и 10 пинах, а также библиотека Servo!
  //На 2 таймере отключится tone()
  Timer1.setPeriod(1000000);//устанавливем период для счёта времени в  мкс с момента включения м-к (1 секунда)
  Timer1.enableISR();//с этого момента начнётся счёт

  lcd.createChar(1, degree);// Загружаем массив с символом градуса «°» в 1 ячейку ОЗУ дисплея
  lcd.createChar(2, rightcursor);
  lcd.createChar(3, leftcursor);
  lcd.createChar(4, battL);
  lcd.createChar(5, battR);
  //Важный момент: обёрнутые в F() строки оптимизируются, то есть одинаковые строки не дублируются в памяти!
  //Поэтому можно использовать макрос в разных участках программы, одинаковые строки не нужно выносить глобально и делать их общими – это сделает компилятор
  lcd.backlight();//подсветка lcd1602
  lcd.home();
  lcd.print(F("t1=  "));
  lcd.print(F("\1C"));
  lcd.setCursor(0, 1);
  lcd.print(F("t2=  "));
  lcd.print(F("\1C"));
  lcd.setCursor(10, 1);
  lcd.print(char(4)); //левая часть значка аккумулятора
  lcd.print(char(5));// правая часть

  byte ON[4] = {0, 0, 0, 0};
  disp.twist(ON, 30);//анимация при включении
  //delay(150);//стабилизация max6675 (время конверсии чипа - 170-220 мс)
  disp.clear();
  //float(+4) необходимо очистить значения в каждом адресе памяти, затем этот код убрать или закомментировать
  //  for (int i = 0; i < sizeof(vals) + 4 ; i++) {
  //    EEPROM.put (i, 0);
  //  }
  myTimer4 = millis();
}


void sens() {
  tacho.tick(); //в прерывании вазываем tick для обработки RPM (Библиотека Tachometer)
}


ISR(TIMER1_A) {//прерывание для счёта времени
  static uint8_t sec;
  if (++sec > 59) {//если после инкрементирования секунд больше, чем 59
    sec = 0;
    if (++m > 59) {//проверяем, после инкрементирования минут больше ли 59
      m = 0;
      ++h;// и прибавляем часы
    }
  }
}


void loop() {
  /*--обработка энкодера с кнопкой--*/
  enc.tick();// обработчик энкодера с кнопкой

  if (enc.held()) {
    P = true;//меняем флаг для однократного чтения из EEPROM при запуске настроек
    Hold = !Hold; //переключаем в режим настройки
    digitalWrite (ledpin, LOW);
    if (!Hold) lcdUpdate();//Hold == 0, очищаем дисплей
    else {
      lcd.clear();//очищаем дисплей для показа параметров
      printGUI();// выводим интерфейс
    }
  }

  switch (Hold) {
    case 1: {//Hold == true
        // P - флаг для однократного получения параметров при входе в меню
        if (P) {//получили переменные 1 раз
          EEPROM.get(4, vals);
          P = false;//получили и опустили флаг
        }
        disp.displayByte(_t, _u, _n, _e);
        if (enc.click()) {
          controlState = !controlState; // изменяем состояние контроля из 0 в 1
          if (!controlState)//если controlState не в режиме настроек (на выходе из него изменился на режим прокрутки в прошлой строчке)
            EEPROM.put(4, vals);//при клике и переходе из режима настроек

          printGUI(); //печатаем на дисплее (названия настроек)
        }
        /*или если не перешёл в режим прокрутки меню (и при выключении), а просто вышел, тоже сохраняем*/
        if (input_volt < minV || (controlState && enc.press())) EEPROM.put(4, vals);

        if (enc.turn()) { //если повернули (факт поворота)
          int increment = 0;  // локальная переменная направления
          // получаем направление
          if (!controlState) { //controlState == 0 (если не равно 1, управляем ВЫБОРОМ НАСТРОЕК)
            if (enc.right()) increment = 1;
            if (enc.left()) increment = -1;
            arrowPos += increment;  // двигаем курсор
            arrowPos = constrain(arrowPos, 0, SETTINGS_AMOUNT - 1); // ограничиваем позицию стрелки
          }

          increment = 0;  // обнуляем инкремент
          if ((controlState && enc.right())) increment = 1;// если controlState == 0 управляем ПАРАМЕТРАМИ
          if ((controlState && enc.left())) increment = -1;

          if (controlState && enc.fast() && (enc.dir() == 1)) increment = FAST_STEP;//быстрый поворот и направление вправо
          if (controlState && enc.fast() && (enc.dir() == -1)) increment = -FAST_STEP;// влево
          vals[arrowPos] += increment;  // меняем параметры по позиции стрелки
          vals[0] = constrain(vals[0], 0, 7);//ограничиваем парметр яркости дисплея
          vals[1] = constrain(vals[1], 0, vals[2]);//  minV   параметры int`овые - потом делятся на 10 и получаются float
          vals[2] = constrain(vals[2], vals[1], 999);//maxV
          vals[3] = constrain(vals[3], 0, 1);//пищалка указателей поворота
          vals[4] = constrain(vals[4], 0, 255);//параметр яркости светодиода
          vals[5] = constrain(vals[5], 0, 1);//параметр обнуления моточасов
          vals[6] = constrain(vals[6], 0, 800);//параметр максимальной температуры цилиндров
          vals[7] = constrain(vals[7], 0, 1);//тест пищалки
          minV = float(vals[1]) / 10;//делим на 10, чтобы получить флоат с 1 знаком после точки
          maxV = float(vals[2]) / 10;
          if (vals[5]) {//если равно единице обнуляем счётчик моточасов (покрутить энкодер)
            e_hours = 0;
            EEPROM.put(0, e_hours);
            vals[5] = 0;
          }
          printGUI();
        }
        if (vals[7]) {//Тест пищалки
#ifdef buzzPassive
          tone (buzz, 2000);
#endif

#ifdef buzzActive
          digitalWrite (buzz, HIGH);
#endif
        }
        else {
#ifdef buzzPassive
          noTone (buzz);
#endif

#ifdef buzzActive
          digitalWrite (buzz, LOW);
#endif
        }
        analogWrite (ledpin, vals[4]);
      }
      break;

    case 0: {//действия, которые выполняются не в режиме настроек !Hold
        /* --обработка одинарного и двойного нажатий-- */
        if (enc.hasClicks(1)) isButtonSingle();
        else if (enc.hasClicks(2)) {
          L = true;//для обновления моточасов при двойном нажатии
          isButtonDouble();
        }

        /* --обработка указателей поворота-- */
        static bool ls, TurnOff; //переменные для правильного выключения повторителей
        bool l = digitalRead(turnpin1);//переменная флага включения левого указателя. Потом по флагу работаем с проецированием на экран
        if (l || digitalRead(turnpin2) == HIGH) {//если какой-то из указателей загорелся
          myTimer4 = millis(); //запоминаем время для избежания наложения включений светодиода
          ls = l;//запоминаемм, чтобы потом выключить нужную стрелку
          //включаем светодиод и пищим
          if (vals[3]) {
#ifdef buzzActive
            digitalWrite (buzz, HIGH);//пищим если разрешено в настройках
#endif
#ifdef buzzPassive
            tone (buzz, 2000);
#endif
          }
          digitalWrite(ledpin, HIGH);
          if (l) {
            lcd.setCursor(8, 1);
            lcd.print(char(3));
          }
          else {
            lcd.setCursor(9, 1);
            lcd.print(char(2));
          }
          TurnOff = true;//флаг для однократного выключения
        }
        else if (TurnOff) {//чтобы постоянно не выключался светодиод, выключаем по флагу
          TurnOff = false;
          if (vals[3]) {
#ifdef buzzActive
            digitalWrite (buzz, LOW);//пищим если разрешено в настройках
#endif
#ifdef buzzPassive
            noTone(buzz);
#endif
          }
          digitalWrite(ledpin, LOW);
          if (ls) lcd.setCursor(8, 1);
          else lcd.setCursor(9, 1);
          lcd.print(F(" "));
        }

        /* --вывод ошибок и неисправностей-- */
        static uint16_t myTimer3;
        uint16_t ms3 = millis() & 0xFFFF;//берём из миллис остаток от деления битовой маской (0xFF - 1 байт (255), 0xFFFF - 2 байта (65 535))>
        if (ms3 - myTimer3 >= 1500) { //для мигания текстом LCD 1602 || > т.е. не больше, чем 1 или 2 байта в зависимости от периода
          myTimer3 = ms3;
          z = !z;
#ifdef bufferBatt
          if ((buff_input_volt < minV) && (++bv > 2)) bv = 0; //переключаем индекс для предупреждений буферного акб: 0, 1, 2, 0
#endif
        }

        static bool flag; //флаг для однократной очистки дисплея, если выводились какие-либо строки
        lcd.setCursor(8, 0);
        if (t1 > vals[6] || t2 > vals[6]) {
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

        /*--вывод информации о буферном аккумуляторе--*/
#ifdef bufferBatt
        if (buff_input_volt < minV) {
          switch (bv) {
            case 0:
              lcd.setCursor(9, 1);
              lcd.print(F("LOWBvol"));
              break;
            case 1:
              lcd.setCursor(9, 1);
              lcd.print(F("BV="));
              lcd.print(buff_input_volt);
              break;
            case 2:
              lcd.setCursor(10, 1);
              lcd.print(F("V="));
              lcd.print(input_volt);
              break;
          }
          if (buff_input_volt >= minV) {//при выходе из цикла напечатать значок
            lcd.setCursor(10, 1);
            lcd.print(char(4));
            lcd.print(char(5));
          }
        }
#endif

        /*--управление светодиодом--*/
        static bool le;
        if ((millis() - myTimer4 > 1200) && ((t1 > vals[6]) ||  (t2 > vals[6]) || (input_volt < minV) || (input_volt > maxV))) {
          //управляем светодиодом если прошло больше секунды с момента включения указателей поворота
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
        } else if (le) { //при определённых условиях светодиод может не выключиться, этот код предотвратит это
          digitalWrite(ledpin, LOW);//если произойдёт выход из прошлого if, и светодиод не выключится, этот код однократно выключит светодиод
          le = false;
        }

        /*--если кол-во оборотов больше 5800, то включать, выключать светодиод--*/
#ifdef RPMwarning
        if ((millis() - myTimer4 > 1200) && R >= 5800) {
          static uint16_t myTimer;
          uint16_t ms = millis() & 0xFFFF;
          if (ms - myTimer >= 400) {
            myTimer = ms;
            j = !j;
          }
          switch (j) {
            case 1:
              ledState = HIGH;
              break;
            case 0:
              ledState = LOW;
              break;
          }
          digitalWrite(ledpin, ledState);
        }
#endif
      }
      break;
  }

  /* --вывод значения тахометра-- */
  static uint16_t myTimer2;
  uint16_t ms2 = millis() & 0xFFFF;//остаток от деления битовой маской
  if (ms2 - myTimer2 >= 100) {
    myTimer2 = ms2;
    if (!Hold) disp.displayInt(R = tacho.getRPM() >> 1);//делим на 2
    /* --обработка вольтметра-- */
    input_volt = firstbatt.getVolt(analogRead(analogpin1));//передаём параметры в функцию получения напряжения
#ifdef bufferBatt
    buff_input_volt = firstbatt.getVolt(analogRead(analogpin2));
#endif
  }

  /*записываем значание моточасов в память при выключении м-к или при нажатии на кнопку*/
  if (L || ((input_volt < minV) && (input_volt != 0.0))) {
    static uint32_t sec1;//sec2 делаем просто локальной, а sec1 - static, чтобы сохраняла значение между вызовами функции
    uint32_t sec2;
    L = false;
    EEPROM.get(0, e_hours);//читаем из памяти
    sec2 = millis();//запоминаем текущее время
    //разницу настоящего значения времени и предыдущего в миллисекундах преобразуем в десятичные часы
    e_hours += (sec2  - sec1) / 3600000.0;
    EEPROM.put(0, e_hours);//записываем в ЭСППЗУ
    sec1 = sec2;//запоминаем время для следующей итерации
  }

  /*--вывод информации на дисплей--*/
  static uint16_t myTimer1;
  uint16_t ms1 = millis() & 0xFFFF;
  if (ms1 - myTimer1 >= 1000) {
    myTimer1 = ms1;
    Watchdog.reset();//защита от зависания - сбрасываем таймер Watchdog раз в секунду
    thermocouple();
    if (!Hold) {
      lcd.setCursor(3, 0);
      lcd.print(t1);
      if (t1 == 10 || t1 == 100) lcd.print(F("\1C"));//печатаем символ градуса, если температура увеличилась на разряд
      else if (t1 == 9 || t1 == 99) {//если температура уменьшилась на разряд, печатаем символ градуса, а предыдущий очищаем
        lcd.print(F("\1C"));//т.е. если первое условие выполнилось, второе не будет проверяться
        lcd.print(F(" "));
      }
      lcd.setCursor(3, 1);
      lcd.print(t2);
      if (t2 == 10 || t2 == 100) lcd.print(F("\1C"));//символ градуса
      else if (t2 == 9 || t2 == 99) {
        lcd.print(F("\1C"));
        lcd.print(F(" "));
      }
#ifdef bufferBatt
      if (buff_input_volt >= minV) {
#endif
        lcd.setCursor(12, 1);
        lcd.print(input_volt);
#ifdef bufferBatt
      }
#endif
    }
  }
}


void thermocouple() {
  if (thermo.readTemp()) t1 = thermo.getTempInt() - 2;
  else t1 = NAN;
  if (thermo2.readTemp()) t2 = thermo2.getTempInt() - 2;
  else t2 = NAN;
}


/*--выводим версию программы, напряжение буферного аккумулятора (если есть) и время поездки--*/
void isButtonSingle() { // действия после одиночного нажатия кнопки
  uint32_t myTimer = millis();
  disp.displayByte(_U, _2, _7, _0);//выводим версию программы на дисплей
  lcd.clear();//очищаем дисплей для показа параметров

  while (millis() - myTimer < 3000) {
    lcd.home();
    lcd.print(F("trip time: "));
    lcd.print(h);
    lcd.print(F(":"));
    lcd.print(m);
#ifdef bufferBatt
    lcd.setCursor(0, 1);
    lcd.print(F("Buff Voltage: "));
    lcd.print(buff_input_volt);
#endif
  }
  lcdUpdate();
  disp.clear();
}

/*--выводим температуру процессора и моточасы--*/
void isButtonDouble() { // действия после двойного нажатия кнопки
  uint32_t myTimer = millis();
  disp.displayInt(memoryFree());
  lcd.clear();//очищаем дисплей для показа параметров
  float CPUt = temperature.getCPUTemp();
  EEPROM.get(0, e_hours); //читаем значение моточасов из памяти
  while (millis() - myTimer < 5000) { //время не должно быть больше периода Watchdog
    lcd.home();
    lcd.print(F("motor hours:"));
    lcd.print(e_hours);
    lcd.setCursor(0, 1);
    lcd.print(F("CPU temp:")); //выводим температуру процессора
    lcd.print(CPUt);
  }
  lcdUpdate();
  disp.clear();
}


void lcdUpdate() { //обновляем экран на LCD
  lcd.clear();//очищаем дисплей для показа параметров
  lcd.print(F("t1="));
  lcd.print(t1);
  lcd.print(F("\1C"));//символ градуса
  lcd.setCursor(0, 1);
  lcd.print(F("t2="));
  lcd.print(t2);
  lcd.print(F("\1C"));//символ градуса
  lcd.setCursor(10, 1);
  lcd.print(char(4));//левая половина значка аккумулятора
  lcd.print(char(5));// правая половина
}


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
    lcd.print(vals[LINES * screenPos + i]);//выводим значение параметров
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


uint16_t memoryFree() {//функция вывода свободной оперативки
  uint16_t freeValue;
  if ((uint16_t)__brkval == 0)
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)&__bss_end);
  else
    freeValue = ((uint16_t)&freeValue) - ((uint16_t)__brkval);
  return freeValue;
}

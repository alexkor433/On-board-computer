//версия для бортового компьютера на плате
//Настройки: энкодер (EB_BETTER_ENC, EB_HALFSTEP_ENC, EB_FAST, buttpin, A, B (пины энкодера)), указатели поворота (turnpin1, turnpin2),
//TM1637 (CLK, DIO), аналоговые преобразования (analogpin1, analogpin2, сопротивление резисторов r1, r2, r3, r4 в делителе напряжения,
//калибровка calibration1, calibration2), калибровка температуры процессора (tempsizing), пин пищалки (buzz) LiquidCrystal_I2C (настройка адреса),
//MAX6675_DELAY (задержка переключения CLK в микросекундах для улучшения связи по длинным проводам),
//настройка меню (SETTINGS_AMOUNT, FAST_STEP), настройки препроцессором (bufferBatt, RPMwarning, buzzPassive, buzzActive). Остальные настройки
//производятся через меню бортового компьютера.
//Можно искать настройки по тексту программы через Ctrl + F

//Version 2.4.3
#include <EEPROM.h>
#include <GyverWDT.h> //библиотека сторожевого таймера
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define MAX6675_DELAY 10 //задержка переключения CLK в микросекундах (для улучшения связи по длинным проводам)
#include <GyverMAX6675.h>
#include <GyverTM1637.h>
#include <GetVolt.h>
#include <GetCPUTemp.h>//пин А7 невозможно больше использовать
#include <Tachometer.h>
#define EB_BETTER_ENC  // улучшенный алгоритм опроса энкодера. Добавит 16 байт SRAM при подключении библиотеки
//подключать в зависимости от качества энкодера или при подключении режима полушаговых энк`ов
#define EB_HALFSTEP_ENC //режим для полушаговых энкодеров
#define EB_FAST 60     // таймаут быстрого поворота, мс
#include <EncButton.h>

//#define bufferBatt//включение обработки буферного аккумулятора
//#define RPMwarning//включение предупреждения о высоких оборотах
//#define buzzPassive
#define buzzActive

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

#define turnpin1 11 //пины указателей поворота
#define turnpin2 12

#define CLK 0 //tm1637
#define DIO 1
#define analogpin1 14 //A0 пины для измерения
#ifdef bufferBatt
#define analogpin2 15 //A1 напряжения аккумуляторов
#endif
#define buttpin 13
#define ledpin 3
#define buzz 10//пин пищалки
#define A 16 //A2
#define B 17 //A3
#define tempsizing 289.0 //калибровочное значение для измерения температуры процессора

GetCPUTemp temperature (tempsizing);
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
#define calibration1 1.12
GetVolt firstbatt (r1, r2, calibration1);

#ifdef bufferBatt
#define r3 46450.0
#define r4 5580.0
#define calibration2 1.12
GetVolt secondbatt (r3, r4, calibration2);
#endif

float input_volt = 0.0;
float buff_input_volt = 0.0;

uint32_t myTimer1, myTimer2, myTimer3, myTimer4, myTimer7, myTimer9, myTimer10;
boolean z, j, bv, Hold, L, P;//z - для мигания текстом и светодиодом, j - для мигания светодиода при высоких оборотах,P - смотри на код меню
boolean ledState = LOW;//bv - для показа напряжения буферного аккумулятора, Hold - в меню неастроек, L - обновление значений счётчика моточасов
int t1, t2, R;
float e_hours, maxV, minV;
uint8_t m, h, p1, p2;

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

// объявляем таблицу ссылок
const char* const names[] PROGMEM = {
  name1, name2, name3, name4, name5, name6, name7, name8
};

int vals[SETTINGS_AMOUNT];  // массив параметров
int8_t arrowPos = 0;
bool controlState = 0;  // клик

#define cels &vars[0]//макросы параметров для передачи в функцию printFromPGM для уменьшения
#define _t1 &vars[1]
#define _t2 &vars[2]

const char var1[] PROGMEM = "\1C";
const char var2[] PROGMEM = "t1=";
const char var3[] PROGMEM = "t2=";

const char* const vars[] PROGMEM = {
  var1, var2, var3
};


void setup() {
  //!!!Обязательно размещается в начале setup, иначе уходит в bootloop!!!
  Watchdog.enable(RESET_MODE, WDT_PRESCALER_1024);//режим сброса при зависании, таймаут 8 сек.
  //Либо размещается в любом месте сетапа, но с условием отключения WDT в начале сэтапа функцией watchdog.disable()
  //Это связано с тем, что контроллер автоматически ставит таймаут WDT на 16 мс и, если функция watchdog.enable() стоит не в начале, код до неё может
  //выполняться дольше 16 мс и контроллер уходит в bootloop, вернее WDT перезагружает контроллер каждые 16 мс. Может случиться, что даже если сбросили
  //таймер в начале сетапа, контроллер всё равно уходит в bootloop, тогда необходимо перепрошить загрузчик или убрать его совсем.

  lcd.init();//инициализация lcd1602
  //pinMode(turnpin1, INPUT); //для указателей поворота | при загрузке скетча через USBasp отсоединить
  //pinMode(turnpin2, INPUT); //для указателей поворота | провода от D11, D12, D13 (MOSI, MISO, SCK)
  pinMode(ledpin, OUTPUT);
  //digitalWrite(ledpin, LOW);//A3 светодиод

  analogPrescaler(128);//!!!ВНИМАНИЕ предделитель АЦП 128 - наивысшая точность
  analogReference(INTERNAL);//опорное напряжения для аналоговых измерений 1.1вольт
  attachInterrupt(0, sens, FALLING); // прерывание на 2 пин(2пин-0, 3пин-1)
  EEPROM.get(4, vals);//получаем весь массив из EEPROM
  disp.brightness(vals[0]);// Яркость индикатора (0-7)
  minV = float(vals[1]) / 10;
  maxV = float(vals[2]) / 10;

  lcd.createChar(1, degree);// Загружаем массив с символом градуса «°» в 1 ячейку ОЗУ дисплея
  lcd.createChar(2, rightcursor);
  lcd.createChar(3, leftcursor);
  lcd.createChar(4, battL);
  lcd.createChar(5, battR);

  lcd.backlight();//подсветка lcd1602
  lcd.setCursor(0, 0);
  printFromPGM(_t1);//t1 вывод строки осуществляется через функцию printFromPGM
  lcd.setCursor(0, 1);
  printFromPGM(_t2);//t2
  lcd.setCursor(10, 1);
  lcd.print(char(4)); //левая часть значка аккумулятора
  lcd.setCursor(11, 1);
  lcd.print(char(5));// правая часть
  lcd.setCursor(6, 0);
  printFromPGM(cels);//символ градуса
  lcd.setCursor(6, 1);
  printFromPGM(cels);

  //Serial.begin(9600);
  byte ON[4] = {0, 0, 0, 0};
  disp.twist(ON, 30);//анимация при включении
  delay(200);//стабилизация max6675 (время конверсии чипа - 170-220 мс)
  disp.clear();
  //float(+4) необходимо очистить значения в каждом адресе памяти, затем убрать или закомментировать
  //  for (int i = 0; i < sizeof(vals) + 4 ; i++) {
  //    EEPROM.put (i, 0);
  //  }
  myTimer2 = myTimer3 = myTimer4 = myTimer7 = myTimer9 = myTimer10 = millis();
  myTimer1 = millis() - 1000;
}


void sens() {
  tacho.tick(); //в прерывании вазываем tick для обработки RPM
}


void loop() {
  /*--обработка энкодера с кнопкой--*/
  enc.tick();// обработчик энкодера с кнопкой

  if (enc.held()) {
    lcd.clear();
    P = true;//меняем флаг для однократного чтения из EEPROM при запуске настроек
    Hold = !Hold; //переключаем в режим настройки
    digitalWrite (ledpin, LOW);
    if (!Hold) lcdUpdate();//Hold == 0, очищаем дисплей
    else printGUI();   // выводим интерфейс
  }


  //Hold == false
  if ((!Hold) && (enc.hasClicks(1))) isButtonSingle();
  if ((!Hold) && (enc.hasClicks(2))) {
    L = true;//для обновления моточасов при двойном нажатии
    isButtonDouble();
  }

  if (Hold) { //Hold == true
    if (P) {//получили переменные 1 раз
      EEPROM.get(4, vals);
      P = false;//получили.
    }
    disp.displayByte(_t, _u, _n, _e);
    if (enc.click()) {
      controlState = !controlState; // изменяем состояние контроля из 0 в 1
      if (!controlState) {//если controlState не в режиме настроек (на выходе из него изменился на режим прокрутки в прошлой строчке)
        EEPROM.put(4, vals);//при клике и переходе из режима настроек
      }
      printGUI(); //печатаем на дисплее (названия настроек)
    }
    /*или если не перешёл в режим прокрутки меню (и при выключении), а просто вышел, тоже сохраняем*/
    if ((enc.press() && controlState) || input_volt < minV) EEPROM.put(4, vals);

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
      if (vals[5] == 1) {//если равно единице обнуляем счётчик моточасов (покрутить энкодер)
        vals[5] = 0;
        e_hours = 0;
        EEPROM.put(0, e_hours);
      }
      printGUI();
    }
    if (vals[7] == 1) {//Тест пищалки
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


  /* --вывод значения тахометра-- */
  if (millis() - myTimer2 >= 50) {
    if (!Hold) disp.displayInt(R = tacho.getRPM());

    /* --обработка вольтметра-- */
    input_volt = firstbatt.getVolt(analogRead(analogpin1));
#ifdef bufferBatt
    buff_input_volt = firstbatt.getVolt(analogRead(analogpin2));
#endif
    do {
      myTimer2 += 50;
      if (myTimer2 < 50)break;
    } while (myTimer2 < millis() - 50);
  }

  /* --счетчик с момента включения микроконтроллера-- */
  if (millis() - myTimer4 >= 60000) {
    ++m;
    do {
      myTimer4 += 60000;
      if (myTimer4 < 60000)break;
    } while (myTimer4 < millis() - 60000);
  }

  if (m >= 60) {
    m = 0;
    ++h;
  }


  /*записываеем значание моточасов в память каждые 10мин, при выключении м-к или при нажатии на кнопку*/
  if (((input_volt < minV) && (input_volt != 0.0)) || (millis() - myTimer9 > 600000) || L) {
    float e_h;
    static uint32_t sec1;
    uint32_t sec2;// sec2 делаем просто локальной, а sec1 - static, чтобы сохраняла значение между вызовами функции
    myTimer9 = millis();
    L = false;
    sec2 = millis();
    e_h = (float(sec2 - sec1) / 3600); //читаем разницу настоящего значения и предыдущего в миллисекундах и преобразуем в десятичные часы
    EEPROM.get(0, e_hours);//читаем из памяти и прибавляем
    e_hours += e_h / 1000; //делаем вычисления и значение e_hours получается десятичное, т.е. float
    EEPROM.put(0, e_hours);
    sec1 = sec2;
  }


  /* --обработка указателей поворота-- */
  if (!Hold) {
    bool buzzState;
    if (digitalRead(turnpin1) == HIGH) {
      myTimer10 = millis(); //запоминаем время для избежания наложения включений светодиода
      ledState = HIGH;
      buzzState = HIGH;
      digitalWrite(ledpin, ledState);
      lcd.setCursor(9, 1);
      lcd.print(char(2));
    }
    else {
      if (millis() - myTimer10 < 1200) {//чтобы постоянно не выключался светодиод
        ledState = LOW;//выключаем по таймеру
        buzzState = LOW;
        digitalWrite(ledpin, ledState);
      }
      lcd.setCursor(9, 1);
      lcd.print(F(" "));
    }

    if (digitalRead(turnpin2) == HIGH) {
      myTimer10 = millis();
      ledState = HIGH;
      buzzState = HIGH;
      digitalWrite(ledpin, ledState);
      lcd.setCursor(8, 1);
      lcd.print(char(3));
    }
    else {
      if (millis() - myTimer10 < 1200) {
        ledState = LOW;
        buzzState = LOW;
        digitalWrite(ledpin, ledState);
      }
      lcd.setCursor(8, 1);
      lcd.print(F(" "));
    }
    if (vals[3]) {
#ifdef buzzActive
      digitalWrite (buzz, buzzState);//пищим если разрешено в настройках
#endif
#ifdef buzzPassive
      if (buzzState) tone (buzz, 2000);
      else noTone(buzz);
#endif
    }
  }


  /* --вывод ошибок и неисправностей-- */
  if (millis() - myTimer3 >= 1500) { //для мигания текстом LCD 1602
    z = !z;
#ifdef bufferBatt
    if ((buff_input_volt < minV) && (++bv > 2)) bv = 0; //переключаем индекс для предупреждений буферного акб: 0, 1, 2, 0
#endif
    do {
      myTimer3 += 1500;
      if (myTimer3 < 1500)break;
    } while (myTimer3 < millis() - 1500);
  }


  if (!Hold) {
    lcd.setCursor(8, 0);
    if (t1 > vals[6] || t2 > vals[6]) {
      switch (z) {
        case 1:
          lcd.print(F("OVERheat"));
          break;
        case 0:
          if (input_volt < minV) lcd.print(F("LOWvolt "));
          else if (input_volt > maxV) lcd.print(F("OVERvolt"));
          else lcd.print(F("        "));
          break;
      }
    }
    else if (input_volt < minV) {
      switch (z) {
        case 1:
          lcd.print(F("LOWvolt"));
          break;
        case 0:
          lcd.print(F("        "));
          break;
      }
    }
    else if (input_volt > maxV) {
      switch (z) {
        case 1:
          lcd.print(F("OVERvolt"));
          break;
        case 0:
          lcd.print(F("        "));
          break;
      }
    }
    else if (R < 500) lcd.print(F("  Hello "));
    else lcd.print(F("        "));
  }

  /*--управление светодиодом--*/
  if ((!Hold) && (millis() - myTimer10 > 1200) && ((t1 > vals[6]) ||  (t2 > vals[6]) || (input_volt < minV) || (input_volt > maxV))) {
    //управляем светодиодом если прошло больше секунды с момента включения указателей поворота
    switch (z) {
      case 1:
        ledState = HIGH;
        break;
      case 0:
        if ((digitalRead(turnpin1) == LOW ) || (digitalRead(turnpin2) == LOW)) ledState = LOW;
        break;
    }
    digitalWrite(ledpin, ledState);
  }
#ifdef bufferBatt
  if ((!Hold) && (buff_input_volt < minV)) {
    switch (bv) {
      case 0:
        lcd.setCursor(8, 1);
        lcd.print(F("LOWBvolt"));
        break;
      case 1:
        lcd.setCursor(9, 1);
        lcd.print(F("BV="));
        lcd.setCursor(12, 1);
        lcd.print(buff_input_volt);
        break;
      case 2:
        lcd.setCursor(10, 1);
        lcd.print(F("V="));
        lcd.setCursor(12, 1);
        lcd.print(input_volt);
        break;
    }
  }
#endif

#ifdef RPMwarning
  if (R >= 5800) { //если кол-во оборотов больше 5800, то включать, выключать светодиод
    if (millis() - myTimer7 >= 400) {
      myTimer10 = millis();
      myTimer7 = millis();
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

  /*--вывод информации на дисплей--*/
  if (millis() - myTimer1 >= 1000) {
    Watchdog.reset();//защита от зависания - сбрасываем таймер Watchdog раз в секунду
    thermocouple();
    if (t1 < 99) {
      p1 = 5;//динамическая размерность температуры
      if (!Hold) {
        lcd.setCursor(7, 0);
        lcd.print(F(" "));
      }
    }
    else p1 = 6;
    if (t2 < 99) {
      p2 = 5;
      if (!Hold) {
        lcd.setCursor(7, 1);
        lcd.print(F(" "));
      }
    }
    else p2 = 6;
    if (!Hold) {
      lcd.setCursor(3, 0);
      lcd.print(t1);
      lcd.setCursor(p1, 0);
      printFromPGM(cels);//символ градуса
      lcd.setCursor(3, 1);
      lcd.print(t2);
      lcd.setCursor(p2, 1);
      printFromPGM(cels);//символ градуса
#ifdef bufferBatt
      if (buff_input_volt >= minV) {
#endif
        lcd.setCursor(12, 1);
        lcd.print(input_volt);
      }
#ifdef bufferBatt
    }
#endif
    do {
      myTimer1 += 1000;
      if (myTimer1 < 1000)break;
    } while (myTimer1 < millis() - 1000);
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
  uint32_t myTimer5;
  myTimer5 = millis();
  disp.displayByte(0, _U);//выводим версию программы на дисплей
  disp.display(1, 2);
  disp.display(2, 4);
  disp.display(3, 3);
  lcd.clear();//очищаем дисплей для показа параметров
  while (millis() - myTimer5 < 3000) {
    lcd.setCursor(0, 0);
    lcd.print(F("trip time:"));
    lcd.setCursor(11, 0);
    lcd.print(h);
    if (h < 10) {
      lcd.setCursor(12, 0);
    } else {
      lcd.setCursor(13, 0);
    }
    lcd.print(F(":"));
    if (h < 10) {
      lcd.setCursor(13, 0);
    } else {
      lcd.setCursor(14, 0);
    }
    lcd.print(m);
#ifdef bufferBatt
    lcd.setCursor(0, 1);
    lcd.print(F("Buff Voltage:"));
    lcd.setCursor(13, 1);
    lcd.print(buff_input_volt);
#endif
  }
  lcdUpdate();
  disp.clear();
}

/*--выводим температуру процессора и моточасы--*/
void isButtonDouble() { // действия после двойного нажатия кнопки
  uint32_t myTimer8;
  myTimer8 = millis();
  disp.displayInt(memoryFree());
  lcd.clear();
  float CPUt = temperature.getCPUTemp();
  EEPROM.get(0, e_hours); //читаем значение моточасов из памяти
  while (millis() - myTimer8 < 5000) {
    lcd.setCursor(0, 0);
    lcd.print(F("motor hours:"));
    lcd.setCursor(12, 0);
    lcd.print(e_hours);
    lcd.setCursor(0, 1);
    lcd.print(F("CPU temp:")); //выводим температуру процессора
    lcd.setCursor(9, 1);
    lcd.print(CPUt);
  }
  lcdUpdate();
  disp.clear();
}


void lcdUpdate() { //обновляем экран на LCD
  lcd.clear();//очистка экрана и возврат курсора в (0, 0)
  printFromPGM(_t1);//t1
  lcd.setCursor(0, 1);
  printFromPGM(_t2);//t2
  lcd.setCursor(10, 1);
  lcd.print(char(4));//левая половина значка аккумулятора
  lcd.setCursor(11, 1);
  lcd.print(char(5));// правая половина
  lcd.setCursor(p1, 0);
  printFromPGM(cels);//символ градуса
  lcd.setCursor(p2, 1);
  printFromPGM(cels);//символ градуса
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
  uint16_t ptr = pgm_read_word(charMap);    // получаем адрес из таблицы ссылок
  while (pgm_read_byte(ptr) != NULL) {      // всю строку до нулевого символа
    lcd.print(char(pgm_read_byte(ptr)));    // выводим названия пунктов меню
    ptr++;                                  // следующий символ
  }
}


void smartArrow(bool state1) {  // рисует стрелку, галку или пробел
  lcd.write(state1 ? (controlState ? 62 : 126) : 32);
}


int memoryFree() {//функция вывода свободной оперативки
  int freeValue;
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
  return freeValue;
}

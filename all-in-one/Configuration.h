/*-----------------------Предустановки-----------------------*/
//#define bufferBatt  // включение обработки буферного аккумулятора
//#define RPMwarning  // включение предупреждения о высоких оборотах
#define ecoRPM        // включение предупреждения об экономичных оборотах
//#define buzzPassive // дефайнить, если пищалка пассивная
#define buzzActive   // дефайнить, если пищалка активная
//#define OneCylinder // Настройка количества цилиндров двигателя
#define TwoCylinders
#define switchonanimation // Анимация при включении
/*-----------------------------------------------------------*/

#define ecoInterval 2200 ... 3000 // интервал эко режима RPM
// LCD1602
#define lcdAddr 0x27 // i2c адрес
// MAX6675
#define thermoSO 9  // DO  ( SO, MISO )
#define thermoCS 8  // CS  ( SS )
#define thermoSCK 7 // CLK ( SCK )

#ifdef TwoCylinders // термопара второго цилиндра
# define thermoSO2 6
# define thermoCS2 5
# define thermoSCK2 4
#endif

// пины указателей поворота
#define turnpin1 12
#define turnpin2 13

// tm1637
#define CLK 1
#define DIO 0
// светодиод
#define ledpin 3
#define ecoledpin 11
// пищалка
#define buzz 10
//#define buzzFrq 2500 // частота пассивной пищалки
// пины энкодера
#define A A2
#define B A3
// кнопка
#define keypin A1

// измерение напряжения
#define analogpin1 A0
#define r1 12120.0    // сопротивление резистора r1
#define r2 1498.0     // сопротивление резистора r2 (нижнее плечо)
#define calibration1 1.103

#define analogpin2 A6
#define r3 46450.0
#define r4 5580.0
#define calibration2 1.1

// калибровочные значения для измерения температуры процессора
#define tempsizing 297.6
#define tempGain 1.2

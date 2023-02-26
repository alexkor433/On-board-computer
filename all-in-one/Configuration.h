/*-----------------------Предустановки-----------------------*/
//#define bufferBatt  // включение обработки буферного аккумулятора
//#define RPMwarning  // включение предупреждения о высоких оборотах
//#define buzzPassive // дефайнить, если пищалка пассивная
#define buzzActive   // дефайнить, если пищалка активная
//#define OneCylinder // Настройка количества цилиндров двигателя
#define TwoCylinders
//#define switchonanimation // Анимация при включении
/*-----------------------------------------------------------*/
// LCD1602
#define lcdAddr 0x27 // i2c адрес 
// MAX6675
#define thermoSO 6  // DO  ( SO, MISO )
#define thermoCS 5  // CS  ( SS )
#define thermoSCK 4 // CLK ( SCK )

#ifdef TwoCylinders // термопара второго цилиндра
# define thermoSO2 7
# define thermoCS2 8
# define thermoSCK2 9
#endif

// пины указателей поворота
#define turnpin1 12
#define turnpin2 11

// tm1637
#define CLK 0
#define DIO 1
// светодиод
#define ledpin 3
// пин пищалки
#define buzz 10
// пины энкодера
#define A 16
#define B 17
// кнопка
#define keypin 13

// измерение напряжения
#define analogpin1 14
#define r1 22700.0    // сопротивление резистора r1
#define r2 2710.0     // сопротивление резистора r2
#define calibration1 1.126

#define analogpin2 15
#define r3 46450.0
#define r4 5580.0
#define calibration2 1.12

// калибровочные значения для измерения температуры процессора
#define tempsizing 296.89
#define tempGain 0.94

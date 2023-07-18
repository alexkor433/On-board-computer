#pragma once
/*-----------------------Предустановки-----------------------*/
//#define BUFFER_BATTERY    // включение обработки буферного аккумулятора
//#define RPM_WARNING       // включение предупреждения о высоких оборотах
#define ECO_RPM           // включение предупреждения об экономичных оборотах
//#define BUZZER_PASSIVE    // дефайнить, если пищалка пассивная
#define BUZZER_ACTIVE     // дефайнить, если пищалка активная
//#define ONE_CYLINDER      // Настройка количества цилиндров двигателя
#define TWO_CYLINDERS
#define SWITCH_ON_ANIMATION // Анимация при включении
/*-----------------------------------------------------------*/

#define ECO_INTERVAL 2200 ... 3000 // интервал эко режима RPM
// LCD1602
#define LCD_ADDR 0x27 // i2c адрес (изменена скорость тактирования в файле cpp, low speed mode)
// MAX6675
#define SO_PIN 9  // DO  ( SO, MISO )
#define CS_PIN 8  // CS  ( SS )
#define SCK_PIN 7 // CLK ( SCK )

#ifdef TWO_CYLINDERS // термопара второго цилиндра
# define SO_PIN_2 6
# define CS_PIN_2 5
# define SCK_PIN_2 4
#endif

// пины указателей поворота
#define TURN_PIN_1 12
#define TURN_PIN_2 13

// tm1637
#define CLK 1
#define DIO 0
// светодиод
#define LED_PIN 3
#define ECO_LED_PIN 11
// пищалка
#define BUZZER_PIN 10
//#define BUZZER_FREQUENCY 2500 // частота пассивной пищалки
// пины энкодера
#define A A2
#define B A3
// кнопка
#define KEY_PIN A1

// измерение напряжения
#define ANALOG_PIN_1 A0
#define R1 12120.0    // сопротивление резистора r1
#define R2 1498.0     // сопротивление резистора r2 (нижнее плечо)
#define CALIBRATION_1 1.103

#define ANALOG_PIN_2 A6
#define R3 46450.0
#define R4 5580.0
#define CALIBRATION_2 1.1

// калибровочные значения для измерения температуры процессора
#define TEMP_OFFSET 297.6
#define TEMP_GAIN 1.2

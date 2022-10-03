// https://alexgyver.ru/servosmooth/
// https://alexgyver.ru/lessons/filters/
// https://alexgyver.ru/gyverfilters/

#include <SoftwareSerial.h>
#include <ServoSmooth.h>
#include <GyverFilters.h>

#define SERVO_AMOUNT 4 // Количество серво

#define POT1_PIN A0 // Пины потенциометров
#define POT2_PIN A1
#define POT3_PIN A2
#define POT4_PIN A3

#define SERVO1_PIN 2 // Пины серво
#define SERVO2_PIN 4
#define SERVO3_PIN 7
#define SERVO4_PIN 8

#define POT1_MAX_ANALOG_VAL 1022 // Максимальные аналоговые значения на потенцометрах
#define POT2_MAX_ANALOG_VAL 1019
#define POT3_MAX_ANALOG_VAL 1020
#define POT4_MAX_ANALOG_VAL 1020

#define SERVOS_SPEED 130 // Скорость для серво
#define SERVOS_ACCEL 0.2 // Ускорение для серво

#define RA_FILTER_COEF 0.1 // Коэффицент фильтра
#define RA_FILTER_SET_STEP 8 // Установка шага фильтрации

ServoSmooth servos[SERVO_AMOUNT];
GFilterRA potsAnalogGFilterRA[SERVO_AMOUNT];

int potsValues[4], servosPos[4]; // Массив для хранения значений с потенцометра
byte potsPins[4] = {POT1_PIN, POT2_PIN, POT3_PIN, POT4_PIN};
byte servosPins[4] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
int servosMaxAnalogValues[4] = {POT1_MAX_ANALOG_VAL, POT2_MAX_ANALOG_VAL, POT3_MAX_ANALOG_VAL, POT4_MAX_ANALOG_VAL};

void setup() {
  Serial.begin(9600);
  // Настройка выходов/выходов
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    pinMode(potsPins[i], INPUT); // Установить режим пина потенциометра
    servos[i].attach(SERVO1_PIN, 600, 2400); // Подключаем серво к пину
    servos[i].setSpeed(SERVOS_SPEED); // Ограничить скорость серво
    servos[i].setAccel(SERVOS_ACCEL); // Установить ускорение (разгон и торможение) серво
    servos[i].setTargetDeg(analogRead(potsValues[i])); // Установить стартовую позицию серво по потенциометра
    potsAnalogGFilterRA[i].setCoef(RA_FILTER_COEF); // Установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
    potsAnalogGFilterRA[i].setStep(RA_FILTER_SET_STEP); // Установка шага фильтрации (мс). Чем меньше, тем резче фильтр
  }
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick();
  }
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    potsValues[i] = potsAnalogGFilterRA[i].filteredTime(analogRead(potsPins[i]));
    servosPos[i] = map(potsValues[i], 0, servosMaxAnalogValues[i], 0, 180);
    Serial.print(potsValues[i]);
    if (i < SERVO_AMOUNT - 1) Serial.print(", ");
    servos[i].setTargetDeg(servosPos[i]);
  }
  Serial.println();
  while (servos[0].getCurrent() == 0 && servos[1].getCurrent() == 0 && servos[2].getCurrent() == 0 && servos[3].getCurrent() == 0); // Пока сервоприводы двигаются мы ждём
}

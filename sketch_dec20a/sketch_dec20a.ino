// https://alexgyver.ru/servosmooth/
// https://alexgyver.ru/lessons/filters/
// https://alexgyver.ru/gyverfilters/

#include <SoftwareSerial.h>
#include <ServoSmooth.h>
#include "GyverFilters.h"

#define SERVO_AMOUNT 2 // Количество серво
#define POT1_PIN A0
#define POT2_PIN A1
#define SERVO1_PIN 9
#define SERVO2_PIN 10

ServoSmooth servos[SERVO_AMOUNT];
GFilterRA analogGFilterRA;

void setup() {
  Serial.begin(9600);
  // Настройка выходов/выходов
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  servos[0].attach(POT1_PIN, 600, 2400);
  servos[1].attach(POT2_PIN, 600, 2400);
  servos[0].setSpeed(130); // Ограничить скорость
  servos[1].setSpeed(130);
  servos[0].setAccel(0.1); // Установить ускорение (разгон и торможение)
  servos[1].setAccel(0.1);
  analogGFilterRA.setCoef(0.01); // Установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
  analogGFilterRA.setStep(10); // Установка шага фильтрации (мс). Чем меньше, тем резче фильтр
}

void loop() {
  Serial.println(analogGFilterRA.filteredTime(analogRead(POT1_PIN)));
  /*if (millis() - servoTimer >= 3000) { // Каждые 3 сек
    servoTimer = millis();
    servos[0].setTargetDeg(flag ? 0 : 180);  // Двигаем на углы 50 и 120
  }*/
  servos[0].setTargetDeg(120);  // Двигаем на углы 50 и 120
  while (servos[0].getCurrent() == 0); // Пока сервопривод двигается
  servos[0].setTargetDeg(0);
  /*
  int pot1Value = analogRead(POT1_PIN);
  int servo1Pos = map(pot1Value, 0, 1023, 0, 180);
  int pot2Value = analogRead(POT2_PIN);
  int servo2Pos = map(pot2Value, 0, 1023, 0, 180);
  Serial.print(servo1Pos);
  Serial.print(", ");
  Serial.print(servo2Pos);
  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  Serial.println();
  delay(10);
  */
}

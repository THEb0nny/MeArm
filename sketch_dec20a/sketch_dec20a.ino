// https://alexgyver.ru/servosmooth/

//#include <Servo.h>
#include <ServoSmooth.h>
#include <SoftwareSerial.h>

#define AMOUNT_SERVO 2 // Количество серво

#define POT1_PIN A0
#define POT2_PIN A1
#define SERVO1_PIN 9
#define SERVO2_PIN 10

//Servo servo1, servo2;
ServoSmooth servos[AMOUNT_SERVO];

uint32_t timer;
boolean flag;

void setup() {
  Serial.begin(9600);
  // Настройка выходов/выходов
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  //servo1.attach(SERVO1_PIN);
  //servo2.attach(SERVO2_PIN);
  servos[0].attach(POT1_PIN);
  servos[1].attach(POT2_PIN);
  servos[0].setSpeed(130); // ограничить скорость
  servos[1].setSpeed(130);
  servos[0].setAccel(0.1);       // установить ускорение (разгон и торможение)
  servos[1].setAccel(0.1);
  //servo1.write(90);
  //servo2.write(90);
}

void loop() {
  servo.tick();

  if (millis() - timer >= 3000) {   // каждые 3 сек
    timer = millis();
    flag = !flag;
    servo[0].setTargetDeg(flag ? 50 : 120);  // двигаем на углы 50 и 120
  }
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

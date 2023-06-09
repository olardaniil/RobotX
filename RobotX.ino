// Библиотека для работы с серво-машинкой
#include <Servo.h>
// Библиотека для работы с датчиком расстояния
#include <NewPing.h>
// Пины датчиков линии
#define lineSensorLeft_PIN A5
#define lineSensorLeftCenter_PIN A4
#define lineSensorCenter_PIN A3
#define lineSensorRightCenter_PIN A2
#define lineSensorRight_PIN A1
// Пины моторов
#define motorLeft_PWM_PIN 4
#define motorLeft_DIR_PIN 5
#define motorRight_PWM_PIN 2
#define motorRight_DIR_PIN 3
// Пины датчика расстояния
#define distanceSensor_TRIG_PIN A6
#define distanceSensor_ECHO_PIN A7
// Устанавливаем режим работы датчика расстояния
// NewPing distanceSensor(distanceSensor_TRIG_PIN, distanceSensor_ECHO_PIN, 20);
// Сервоприводы
Servo servoLift;
Servo servoHand;

int SpeedMax = 80;
int SpeedMidle = 60;
int SpeedMin = 40;

int lineSensorL = 1;
int lineSensorLC = 1;
int lineSensorC = 1;
int lineSensorRC = 1;
int lineSensorR = 1;

bool _startZone = false;
bool _start2Rom_1 = false;
bool _startZoneRom_1 = false;
bool _room_1 = false;

// Функция для инициализации робота 
void setup() {
  // Устанавливаем режим работы датчиков линии
  pinMode(lineSensorLeft_PIN, INPUT);
  pinMode(lineSensorLeftCenter_PIN, INPUT);
  pinMode(lineSensorCenter_PIN, INPUT);
  pinMode(lineSensorRightCenter_PIN, INPUT);
  pinMode(lineSensorRight_PIN, INPUT);
  // Устанавливаем режим работы моторов
  pinMode(motorLeft_PWM_PIN, OUTPUT);
  pinMode(motorLeft_DIR_PIN, OUTPUT); 
  pinMode(motorRight_PWM_PIN, OUTPUT);
  pinMode(motorRight_DIR_PIN, OUTPUT); 
  servoHand.attach(9);
  // Инициируем передачу данных по последовательному порту (на скорости 9600 бит/сек)
  Serial.begin(9600); while(!Serial){}
  _startZone = true;
}
// Функция для работы с моторами
void setMotorSpeed(int pwmPin, int dirPin, int motorSpeed) {
  if (motorSpeed == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
  }
  else if (motorSpeed > 0) {
    analogWrite(pwmPin, (motorSpeed * 2.55));
    digitalWrite(dirPin, LOW);
  }
  else if (motorSpeed < 0) {
    analogWrite(pwmPin, 255 + (motorSpeed * 2.55));
    digitalWrite(dirPin, HIGH);
  }
}
// Читаем значения сенсоров
void readSensor() {
  lineSensorL = digitalRead(lineSensorLeft_PIN);
  lineSensorLC = digitalRead(lineSensorLeftCenter_PIN);
  lineSensorC = digitalRead(lineSensorCenter_PIN);
  lineSensorRC = digitalRead(lineSensorRightCenter_PIN);
  lineSensorR = digitalRead(lineSensorRight_PIN);
}
// Выводим значения сенсоров
void printSensorValue() {
  Serial.println("---");
  Serial.println(lineSensorL);
  Serial.println(lineSensorLC);
  Serial.println(lineSensorC);
  Serial.println(lineSensorRC);
  Serial.println(lineSensorR);
  // delay(500);
}
// Поднять предмет
bool pickUpAnObject = false;
void liftAnObject() {
  servoLift.attach(8);
  servoLift.write(0);
  delay(500);
  servoLift.write(180);
  delay(660);
  servoLift.detach();
  pickUpAnObject = false;
}
// Опустить предмет
bool putDownAnObject = false;
void lowerAnObject() {
  servoLift.attach(8);
  servoLift.write(0);
  delay(660);
  putDownAnObject = false;

}
// Выход из зоны старта
void startZone() {
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  // delay(2000);
  _startZone = false;
  _start2Rom_1 = true;
}
// Перемещение | Зона старта - Зона старта Комнаты 1
void start2Rom_1() {
    if (lineSensorLC == 1 && lineSensorC == 0 && lineSensorRC == 0) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
    }
    if (lineSensorLC == 1  && lineSensorC == 1 && lineSensorRC == 0) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMidle);
     }
    if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 1) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
    }
    if (lineSensorLC == 0 && lineSensorC == 1 && lineSensorRC == 1) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMidle);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
    }
    if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 0) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
    }
    if (lineSensorLC == 1 && lineSensorC == 1 && lineSensorRC == 1) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -SpeedMin);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -SpeedMin);
    }
    if (lineSensorR == 0 && lineSensorC == 0) {
        setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
        setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
        delay(1350);
        // setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -40);
        // setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -40);
        // delay(1500);
        _start2Rom_1 = false;
        _startZoneRom_1 = true;
    }
    
}
// 
void startZoneRom_1() {
  if (lineSensorLC == 1 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  }
  if (lineSensorLC == 1  && lineSensorC == 1 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMidle);
   }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMidle);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  }
  if (lineSensorLC == 1 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -SpeedMin);
    delay(500);
    _startZoneRom_1 = false;
    _room_1 = true;
  }
}
// Перемещение | Перемещение по комнате 1
void Rom_1() {
  if (lineSensorLC == 1 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  }
  if (lineSensorLC == 1  && lineSensorC == 1 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMidle);
   }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMidle);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  }
  if (lineSensorLC == 1 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -SpeedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -SpeedMin);
  }
  if (lineSensorL == 1 && lineSensorC == 0 && lineSensorR == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    delay(1350);
  }
}

// Обработчик событий
void loop() {
  readSensor();

  if (_startZone == true) {
    startZone();
  }
  if (_start2Rom_1 == true) {
    start2Rom_1();
  }
  if (_startZoneRom_1 == true) {
    startZoneRom_1();
  }
  if (_room_1 == true) {
    Rom_1();
  }

}

// void yield() {

// }

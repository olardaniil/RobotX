// Библиотека для работы с серво-машинкой
#include <Servo.h>
// Библиотека для работы с датчиком расстояния
#include <NewPing.h> //ultrasinic.h
// Пины датчиков линии
#define lineSensorLeft_PIN A15
#define lineSensorLeftCenter_PIN A14
#define lineSensorCenter_PIN A13
#define lineSensorRightCenter_PIN A12
#define lineSensorRight_PIN A11
// Пины моторов
#define motorLeft_PWM_PIN 4
#define motorLeft_DIR_PIN 5
#define motorRight_PWM_PIN 2
#define motorRight_DIR_PIN 3
// Пины датчика расстояния
#define distanceSensor_TRIG_PIN A6
#define distanceSensor_ECHO_PIN A7
// Устанавливаем режим работы датчика расстояния
NewPing distanceSensor(distanceSensor_TRIG_PIN, distanceSensor_ECHO_PIN, 30);
// Сервоприводы
Servo servo1; // Нижний серво
Servo servo2; // Верхний серво

int speedMax = 80;
int speedMidle = 60;
int speedMin = 40;

int timeTurn_180 = 3000;
int timeTurn_90 = 1700;

int lineSensorL = 1;
int lineSensorLC = 1;
int lineSensorC = 1;
int lineSensorRC = 1;
int lineSensorR = 1;

int distance = 0;

bool _start = false;
bool _roadStartToRoom_1 = false;
bool _room_1_step_1 = false;
bool _room_1_step_1_exit = false;
bool _room_1_step_2_exit = false;
bool _toStart = false;

bool _toRoom_4 = false;

bool _room_1_step_2 = false;
bool _road_Room_1_To_Road_4_step_1 = false;

bool _isGrabBank = false;

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
  servo1.attach(A2);
  servo2.attach(A1);
  // Инициируем передачу данных по последовательному порту (на скорости 9600 бит/сек)
  Serial.begin(9600); while(!Serial){}
  _start = true;
  putBank();
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
  Serial.print(lineSensorL);
  Serial.print(lineSensorLC);
  Serial.print(lineSensorC);
  Serial.print(lineSensorRC);
  Serial.print(lineSensorR);
  // delay(1000);
}
// Езда по линии по трём датчикам
void drivingAlongTheLine() {
  if (lineSensorLC == 1 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMin);
  }
  if (lineSensorLC == 1  && lineSensorC == 1 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMidle);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMin);
  }
  if (lineSensorLC == 1 && lineSensorC == 1 && lineSensorRC == 1) {
    stop();
  }
}
// Обратная езда по линии по трём датчикам
void reverseDrivingAlongTheLine() {
  if (lineSensorLC == 1 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -speedMax);
  }
  if (lineSensorLC == 1  && lineSensorC == 1 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMidle);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -speedMax);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -speedMin);
  }
  if (lineSensorLC == 0 && lineSensorC == 1 && lineSensorRC == 1) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMax);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -speedMin);
  }
  if (lineSensorLC == 0 && lineSensorC == 0 && lineSensorRC == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMin);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -speedMin);
  }
  if (lineSensorLC == 1 && lineSensorC == 1 && lineSensorRC == 1) {
    stop();
  }
}
void stop() {
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, 0);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
}
// Взять банку
void grabBank() {
  // Опускаем манипулятор
  servo2.write(0);
  delay(1000);
  // Разжимаем манипулятор
  servo1.write(0);
  delay(1000);
  // Cжимаем манипулятор
  servo1.write(180);
  delay(1000);
  // Поднимаем манипулятор
  servo2.write(180);
  delay(1300);
  // Останавливаем манипулятор
  servo2.write(90);
  delay(1000);
}
// Опускаем банку
void putBank() {
  // Опускаем манипулятор
  servo2.write(0);
  delay(1000);
  // Останавливаем манипулятор
  servo2.write(90);
  delay(1000);
  // Разжимаем манипулятор
  servo1.write(0);
  delay(1000);
}
// Старт
void start() {
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMin);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMin);
  delay(3200);
  _start = false;
  _roadStartToRoom_1 = true;
}
// Перемещение | Старт >>> Комната 1
void roadStartToRoom_1() {
  readSensor();
  drivingAlongTheLine();
  if (lineSensorL == 1 && lineSensorC == 0 && lineSensorR == 0) {
    stop();
    delay(200);
    while(lineSensorL != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    while(lineSensorL != 1) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    while(lineSensorLC != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    _roadStartToRoom_1 = false;
    _room_1_step_1 = true;
  }
}
// Перемещение | Комната 1 | Участок 1
void room_1_step_1() {
  readSensor();
  if (10 < distance && distance <= 15) {
    stop();
    grabBank();
    _isGrabBank = true;
  }
  if (_room_1_step_1 == true) {
    if (lineSensorLC == 1 && lineSensorRC == 1) {
      stop();
      if (_isGrabBank == true) {
        while (lineSensorL != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorL != 1) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorLC != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        drivingAlongTheLine();
        _room_1_step_1 = false;
        _room_1_step_1_exit = true;
      }
      else {
        while (lineSensorL != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorL != 1) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorLC != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        drivingAlongTheLine();
        _room_1_step_1 = false;
        _room_1_step_1_exit = true;
      }
    }
    else {
      drivingAlongTheLine();
    }
  }
}
void room_1_step_1_exit() {
  if (lineSensorLC == 1 && lineSensorRC == 1) {
    while(lineSensorR != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -speedMidle);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMidle);
    }
    stop();
    while(lineSensorLC != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMidle);
    }
    stop();
    _room_1_step_1_exit = false;
    _toStart = true;
  }
  else {
    drivingAlongTheLine();
  }
}
// Перемещение | Комната 1 | Участок 2
void room_1_step_2() {
  if (10 < distance && distance <= 14) {
    stop();
    grabBank();
    _isGrabBank = true;
  }
  if (_isGrabBank == true) {
    while (lineSensorL != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorL != 1) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        while (lineSensorLC != 0) {
          readSensor();
          setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
          setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -(speedMidle+5));
        }
        drivingAlongTheLine();
    _room_1_step_2 = false;
    // _room_1_step_2_exit = true;
  }
  else {
    drivingAlongTheLine();
  }
}
void room_1_step_2_exit() {
  readSensor();
  drivingAlongTheLine();
  if (lineSensorL == 0 && lineSensorC == 0 && lineSensorR == 1) {
    stop();
    delay(200);
    while(lineSensorR != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    while(lineSensorR != 1) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    while(lineSensorRC != 0) {
      readSensor();
      setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMax);
      setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    }
    stop();
    _room_1_step_2_exit = false;
  }
}
// Перемещение | Финиш
}
void toStart() {
  if (lineSensorL == 0 && lineSensorR == 0) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, speedMidle);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, speedMidle);
    delay(800);
    stop();
    // putBank();
    _toStart = false;
  }
  else {
    drivingAlongTheLine();
  }
}
// Обработчик событий
void loop() {
  readSensor();
  printSensorValue();
  distance = distanceSensor.ping_cm();
  
  if (_start == true) {
    start();
  }
  if (_roadStartToRoom_1 == true) {
    roadStartToRoom_1();
  }
  if (_room_1_step_1 == true) {
    room_1_step_1();
  }
  if (_room_1_step_1_exit == true) {
    room_1_step_1_exit();
  }
  if (_room_1_step_2 == true) {
    room_1_step_2();
  }
  if (_room_1_step_2_exit == true) {
    room_1_step_2_exit();
  }
  if (_toStart == true) {
    toStart();
  }

}

// void yield() {

// }
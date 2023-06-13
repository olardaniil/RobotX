// Библиотека для работы с серво-машинкой
#include <Servo.h>
// Библиотека для работы с датчиком расстояния
#include <NewPing.h>
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
Servo servo1;
Servo servo2;

int SpeedMax = 80;
int SpeedMidle = 60;
int SpeedMin = 40;

int TimeTurn_180 = 3000;
int TimeTurn_90 = 1700;

int lineSensorL = 1;
int lineSensorLC = 1;
int lineSensorC = 1;
int lineSensorRC = 1;
int lineSensorR = 1;

int distance = 0;

bool _startZone = false;
bool _start2Rom_1 = false;
bool _startZoneRom_1 = false;
bool _room_1 = false;
bool _leaveRoom_1_Type_1 = false;
bool _leaveRoom_1_Type_2 = false;

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
  _startZone = true;
  PutBank();
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
  delay(1000);
}
// Взять банку
void GrabBank() {
  // Опускаем манипулятор
  servo2.write(85);
  delay(1000);
  // Разжимаем манипулятор
  servo1.write(0);
  delay(1000);
  // Останавливаем манипулятор
  servo2.write(90);
  delay(1000);
  // Cжимаем манипулятор
  servo1.write(180);
  delay(1000);
  // Поднимаем манипулятор
  servo2.write(185);
  delay(1000);
  // Останавливаем манипулятор
  // servo2.write(90);
  // delay(1000);
}
// Опускаем банку
void PutBank() {
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
// Выход из зоны старта
void startZone() {
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMin);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, SpeedMin);
  // delay(2000);
  _startZone = false;
  _start2Rom_1 = true;
}
// Перемещение | Зона старта - Зона старта Комнаты 1
void start2Room_1() {
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
        delay(TimeTurn_90);
        // setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -40);
        // setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -40);
        // delay(1500);
        _start2Rom_1 = false;
        _startZoneRom_1 = true;
    }
    
}
// Перемещение | Зона старта Комнаты 1 - Поворот направо
void startZoneRoom_1() {
  if (10 < distance && distance <= 14) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, 0);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    GrabBank();
    Turn_180();
    _startZoneRom_1 = false;
    _leaveRoom_1_Type_1 = true;
  } 
  else {
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
  
}
// Перемещение | Перемещение по комнате 1 после поворота
void Rom_1() {
  if (10 < distance && distance <= 14) {
    setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, 0);
    setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
    GrabBank();
    _room_1 = false;
    _leaveRoom_1_Type_2 = true;
  } 
  else {
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
}
void Turn_180() {
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
  delay(TimeTurn_90-100);
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -SpeedMax);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -SpeedMax);
  delay(1000);
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, SpeedMax);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
  delay(TimeTurn_90-500);
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, -SpeedMax);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, -SpeedMax);
  delay(1300);
  setMotorSpeed(motorLeft_PWM_PIN, motorLeft_DIR_PIN, 0);
  setMotorSpeed(motorRight_PWM_PIN, motorRight_DIR_PIN, 0);
  delay(10000);
}
// Выход из комнаты 1, если робот нашел банку до поворота
void LeaveRoom_1_Type_1() {
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

}
// Выход из комнаты 1, если робот нашел банку после поворота
void LeaveRoom_1_Type_2() {
  Turn_180();

}
// Обработчик событий
void loop() {
  readSensor();
  // printSensorValue();
  distance = distanceSensor.ping_cm();
  Serial.println(distance);
  // delay(1000);
 
  if (_startZone == true) {
    startZone();
  }
  if (_start2Rom_1 == true) {
    start2Room_1();
  }
  if (_startZoneRom_1 == true) {
    startZoneRoom_1();
  }
  if (_room_1 == true) {
    Rom_1();
  }
  // _leaveRoom_1_Type_1 = true;
  if (_leaveRoom_1_Type_1 == true) {
    LeaveRoom_1_Type_1();
  }
  if (_leaveRoom_1_Type_2 == true) {
    LeaveRoom_1_Type_2();
  }

}

// void yield() {

// }

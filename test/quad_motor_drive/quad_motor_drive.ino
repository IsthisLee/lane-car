// quad_motor_drive.ino
// 4개 모터를 좌/우 그룹으로 제어 (L298N Motor Driver)

int IN1 = 5;    // 왼쪽 방향1
int IN2 = 6;    // 왼쪽 방향2
int ENA = 9;    // 왼쪽 PWM

int IN3 = 3;    // 오른쪽 방향1
int IN4 = 4;    // 오른쪽 방향2
int ENB = 10;   // 오른쪽 PWM

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
}

void loop() {
  // 1) 1초 전진
  forward(150);    // 0~255
  delay(1000);

  // 2) 1초 후진
  backward(150);
  delay(1000);

  // 3) 1초 제자리 왼쪽 회전
  turnLeft(150);
  delay(1000);

  // 4) 1초 제자리 오른쪽 회전
  turnRight(150);
  delay(1000);
}

void forward(int speed) {
  // 왼쪽 모터 그룹
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // 오른쪽 모터 그룹
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void backward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // 왼쪽 뒤 / 오른쪽 앞
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// ROS2로부터 "steeringAngle" 문자열을 받아서 4모터를 제어하는 코드

int IN1 = 5;
int IN2 = 6;
int ENA = 9;

int IN3 = 3;
int IN4 = 4;
int ENB = 10;

const int LEFT_THRESHOLD  = 70;   // 이 값보다 작으면 좌회전
const int RIGHT_THRESHOLD = 110;  // 이 값보다 크면 우회전

int steeringAngle = 90;           // 초기값: 직진

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
}

void loop() {
  // 1) 시리얼에서 "각도" 한 줄 읽음
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int angle = line.toInt();
      if (angle >= 0 && angle <= 180) {
        steeringAngle = angle;
      }
    }
  }

  // 2) 각도 조건에 따라 동작 결정
  if (steeringAngle < LEFT_THRESHOLD) {
    turnLeft(150);
  } else if (steeringAngle > RIGHT_THRESHOLD) {
    turnRight(150);
  } else {
    goForward(150);
  }

  delay(20); // 너무 자주 바뀌지 않도록

  stopMotors();
  while(1);
}

void goForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // 차등 회전: 안쪽 바퀴를 느리게, 바깥쪽을 빠르게
  int inner = speed / 3;
  int outer = speed;

  // 왼쪽이 안쪽, 오른쪽이 바깥쪽이라고 가정
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, inner);
  analogWrite(ENB, outer);
}

void turnRight(int speed) {
  int inner = speed / 3;
  int outer = speed;

  // 오른쪽이 안쪽, 왼쪽이 바깥쪽
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, outer);
  analogWrite(ENB, inner);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
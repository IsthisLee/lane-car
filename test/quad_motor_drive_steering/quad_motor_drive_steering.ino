// Arduino 쪽: quad_motor_drive/quad_motor_drive.ino

int IN1 = 5;
int IN2 = 6;
int ENA = 9;

int IN3 = 3;
int IN4 = 4;
int ENB = 10;

const int MOTOR_SPEED = 255;
const int LEFT_THRESHOLD  = 80;   // 이 값보다 작으면 왼쪽
const int RIGHT_THRESHOLD = 100;  // 이 값보다 크면 오른쪽
int steeringAngle = 90;           // 기본 직진

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
  // Serial로 "각도\n" 수신
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

  if (steeringAngle < LEFT_THRESHOLD) {
    turnLeft(255);
  } else if (steeringAngle > RIGHT_THRESHOLD) {
    turnRight(255);
  } else {
    goForward(MOTOR_SPEED);
  }

  delay(20);
  
  stopMotors();
  while(1);
}

void goForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENB, speed);
  analogWrite(ENA, speed*0.4);
}

void turnLeft(int speed) {
  int innerSpeed = speed / 5;
  int outerSpeed = speed;

  // 왼쪽(안쪽) 느리게, 오른쪽(바깥쪽) 빠르게
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENB, innerSpeed);
  analogWrite(ENA, outerSpeed);
}

void turnRight(int speed) {
  int innerSpeed = speed / 10;
  int outerSpeed = speed;

  // 오른쪽(안쪽) 느리게, 왼쪽(바깥쪽) 빠르게
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENB, outerSpeed);
  analogWrite(ENA, innerSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

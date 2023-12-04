#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <math.h>
MPU6050 mpu1; // 첫 번째 IMU
MPU6050 mpu2; // 두 번째 IMU
int ENA = 6;
int IN1 = 7;
int IN2 = 8;

// 로우패스 필터 매개변수
float alpha = 0.1; // 로우패스 필터 계수 (0.0에서 1.0 사이의 값)

double setpoint = 0; // 목표 가속도 (0으로 수렴)
double input, output;
double Kp = 0.1; // P (비례) 제어게수
double Ki = 0; // I (적분) 제어게수
double Kd = 0; // D (미분) 제어게수

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  mpu1.initialize();
  mpu2.initialize();
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // PWM 출력 제한 (리니어 모터를 제어하는 경우 PWM 값을 사용할 수 있음)
}

void loop() {

  int16_t ax2, ay2, az2;
  mpu2.getAcceleration(&ay2, &ax2, &az2);

  double accelY2 = ay2 / 16384.0;

  Serial.print("IMU2 ay: ");
  Serial.println(accelY2);


  int16_t ax1, ay1, az1;
  mpu1.getAcceleration(&ay1, &ax1, &az1);


  // 로우패스 필터를 사용하여 가속도 데이터를 평활화
  static float filtered_ax1 = 0.0;
  static float filtered_ax2 = 0.0;
  filtered_ax1 = alpha * filtered_ax1 + (1 - alpha) * ax1;
  filtered_ax2 = alpha * filtered_ax2 + (1 - alpha) * ax2;

  input = filtered_ax1; // 첫 번째 IMU의 로우패스 필터를 통과한 데이터를 PID 입력으로 설정

  pid.Compute(); // PID 연산 수행


  if (output <= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(output));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(output));
  }

  // 두 번째 IMU의 ay 방향 속도 출력

  delay(10); // 100 밀리초마다 측정과 제어 수행
}

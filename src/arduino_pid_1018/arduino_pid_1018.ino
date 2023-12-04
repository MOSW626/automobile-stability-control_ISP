#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;
int  ENA = 6;
int IN1 = 7;
int IN2 = 8;
double setpoint = 0; // 목표 가속도 (0으로 수렴)
double input, output;
double Kp = 0.5; // P (비례) 제어게수
double Ki = 0.0; // I (적분) 제어게수
double Kd = 0.0; // D (미분) 제어게수

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);
  pinMode(ENA,OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // PWM 출력 제한 (리니어 모터를 제어하는 경우 PWM 값을 사용할 수 있음)
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  input = ax; // 현재 가속도 값을 PID 입력으로 설정
  pid.Compute(); // PID 연산 수행

  // PID 제어 결과를 사용하여 동작 수행 (여기에서는 시리얼 출력)
  Serial.print("Input: ");
  Serial.print(input);
  Serial.print(" Output: ");
  Serial.println(output);

  if(output<=0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA,abs(output));
  }else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,abs(output));
  }
  

  delay(100); // 100 밀리초마다 측정과 제어 수행
}


#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <math.h>

const int MPU2 = 0x69, MPU1=0x68;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ, gyroX, gyroY, gyroZ, rotX, rotY, rotZ;
long accelX2, accelY2, accelZ2;
float gForceX2, gForceY2, gForceZ2, gyroX2, gyroY2, gyroZ2, rotX2, rotY2, rotZ2;

int ENA = 6;
int IN1 = 9;
int IN2 = 10;

// 로우패스 필터 매개변수
float alpha = 0.0001; // 로우패스 필터 계수 (0.0에서 1.0 사이의 값)

double setpoint = 0; // 목표 가속도 (0으로 수렴)
double input, output;
double Kp = 0.4; // P (비례) 제어게수
double Ki = 0.001; // I (적분) 제어게수
double Kd = 0.01; // D (미분) 제어게수
double output2;
double Kp2 = 0.4; // P (비례) 제어게수
double Ki2 = 0.01; // I (적분) 제어게수
double Kd2 = 0.01; // D (미분) 제어게수

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup_mpu()
{
// ========  MPU1  ==========
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(MPU1);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(MPU1);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();

// ========  MPU2  ==========
  Wire.begin();
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(MPU2);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(MPU2);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void GetMpuValue_MPU1(){
  Wire.beginTransmission(MPU1);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU1,6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();

  Wire.beginTransmission(MPU1);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU1,6);
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();


  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
  rotX = gyroX / 131.0 + 4;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
/*
  Serial.print("Acc\t");
  Serial.print(accelX);
  Serial.print("\t");
  Serial.print(accelY);
  Serial.print("\t");
  Serial.print(accelZ);
  Serial.print("\t");
  Serial.print("gyro\t");
  Serial.print(rotX);
  Serial.print("\t");
  Serial.print(rotY);
  Serial.print("\t");
  Serial.print(rotZ);
  Serial.print("\tAcc\t");
  Serial.print(gForceX);
  Serial.print("\t");
  Serial.print(gForceY);
  Serial.print("\t");
  Serial.print(gForceZ);
  */
  delay(10);
}

void GetMpuValue_MPU2(){
  Wire.beginTransmission(MPU2);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU2,6);
  while(Wire.available() < 6);
  accelX2 = Wire.read()<<8|Wire.read();
  accelY2 = Wire.read()<<8|Wire.read();
  accelZ2 = Wire.read()<<8|Wire.read();
  
  accelX2 = accelX2 - 3080;
  accelY2 = accelY2 - 726;
  accelZ2 = accelZ2 + 1300;

  Wire.beginTransmission(MPU2);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU2,6);
  while(Wire.available() < 6);
  gyroX2 = Wire.read()<<8|Wire.read();
  gyroY2 = Wire.read()<<8|Wire.read();
  gyroZ2 = Wire.read()<<8|Wire.read();

  gyroX2 = gyroX2 + 6;
  gyroY2 = gyroY2 - 60;
  gyroZ2 = gyroZ2 + 93;


  gForceX2 = accelX2 / 16384.0;
  gForceY2 = accelY2 / 16384.0;
  gForceZ2 = accelZ2 / 16384.0;
  rotX2 = gyroX2 / 131.0;
  rotY2 = gyroY2 / 131.0;
  rotZ2 = gyroZ2 / 131.0;
  /*
  Serial.print("Acc\t");
  Serial.print(accelX2);
  Serial.print("\t");
  Serial.print(accelY2);
  Serial.print("\t");
  Serial.print(accelZ2);
  Serial.print("\t");
  Serial.print("gyro\t");
  Serial.print(rotX2);
  Serial.print("\t");
  Serial.print(rotY2);
  Serial.print("\t");
  Serial.print(rotZ2);
  Serial.print("\tAcc\t");
  Serial.print(gForceX2);
  Serial.print("\t");
  Serial.print(gForceY2);
  Serial.print("\t");
  Serial.print(gForceZ2);
  */
  delay(10);
}

void read_imu()
{
  GetMpuValue_MPU1();
 // Serial.print("\t ||| \t");

  GetMpuValue_MPU2();
 // Serial.println("");
}

void start_imu()
{
  Serial.println("AcX\tAcY\tAcZ\tGyX\tGyY\tGyZ\tAcX2\tAcY2\tAcZ2\tGyX2\tGyY2\tGyZ2\tinput\toutput");
}

void setup() {
  Wire.begin();
  setup_mpu();
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // PWM 출력 제한 (리니어 모터를 제어하는 경우 PWM 값을 사용할 수 있음)
  start_imu();
}

void loop() {
  read_imu();
  // imu 값 read

  Serial.print(accelX);
  Serial.print("\t");
  Serial.print(accelY);
  Serial.print("\t");
  Serial.print(accelZ);
  Serial.print("\t");
  Serial.print(gyroX);
  Serial.print("\t");
  Serial.print(gyroY);
  Serial.print("\t");
  Serial.print(gyroZ);
  Serial.print("\t");
  Serial.print(accelX2);
  Serial.print("\t");
  Serial.print(accelY2);
  Serial.print("\t");
  Serial.print(accelZ2);
  Serial.print("\t");
  Serial.print(gyroX2);
  Serial.print("\t");
  Serial.print(gyroY2);
  Serial.print("\t");
  Serial.print(gyroZ2);
  Serial.print("\t");
  


  // 로우패스 필터를 사용하여 가속도 데이터를 평활화
  static float filtered_ay = 0.0;
  filtered_ay = alpha * filtered_ay + (1 - alpha) * accelY;


  input = filtered_ay/20;
  Serial.print(input);
  Serial.print("\t");
  pid.Compute();
  if(abs(input) < 50)
  {
    //pid.SetTunings(Kp, Ki, Kd);
    //pid.Compute();
    output = output * 1.3;
  }
  else
  {
    //pid.SetTunings(Kp2, Ki2, Kd2);
    //pid.Compute();
    output = output * 0.87;
    //0.87
  }
  //pid.Compute();
  //output = output * 1;
  Serial.println(output);
  if (output <= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(output));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(output));
  }
  delay(10);
}

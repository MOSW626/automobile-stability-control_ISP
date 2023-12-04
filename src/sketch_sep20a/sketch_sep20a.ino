#include <Wire.h>

// MOTOR 변수 선언
int ENA = 8;
int ENB = 9;
int ENC = 10;
int END = 11;
// HIGH, LOW : UP & LOW, HIGH : DOWN
int INA1 = 30;
int INA2 = 31;
int INB1 = 32;
int INB2 = 33;
int INC1 = 34;
int INC2 = 35;
int IND1 = 36;
int IND2 = 37;

// case variable
int cnt=2;

void setup_motor()
{
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(ENC, OUTPUT);
  pinMode(INC1, OUTPUT);
  pinMode(INC2, OUTPUT);

  pinMode(END, OUTPUT);
  pinMode(IND1, OUTPUT);
  pinMode(IND2, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(ENC, HIGH);
  digitalWrite(END, HIGH);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  analogWrite(ENC, 255);
  analogWrite(END, 255);
}

void setup()
{
  setup_motor();
}

void loop()
{
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
  digitalWrite(INC1, HIGH);
  digitalWrite(INC2, LOW);
  digitalWrite(IND1, HIGH);
  digitalWrite(IND2, LOW);
  delay(5000);
  
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
  digitalWrite(INC1, LOW);
  digitalWrite(INC2, HIGH);
  digitalWrite(IND1, LOW);
  digitalWrite(IND2, HIGH);
  delay(5000);
}
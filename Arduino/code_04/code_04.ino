#include <QTRSensors.h>
QTRSensors qtr;
const int IN1 = 5;
const int IN2 = 3;
const int IN3 = 6;
const int IN4 = 9;
const int ENA = 10;
const int ENB = 11;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
float Kp = 0.45;
float Ki = 0;
float Kd = 0.3;
int P;
int I;
int D;
const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 200;
const uint8_t basespeedb = 200;
int lastError = 0;
boolean onoff = false;
int buttoncalibrate = 13; //or pin A3
int buttonstart = 2;
void setup()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, SensorCount);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false) { // the main function won't start until the robot is calibrated
    if (digitalRead(buttoncalibrate) == HIGH) {
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  forward_brake(0, 0);
}


void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{

  if (digitalRead(buttonstart) == HIGH) {

    onoff = !onoff;
    if (onoff = true) {
      delay(5000);//a delay when the robot starts
      Serial.begin(9600);
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
      }
      Serial.println();

      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
      }
      Serial.println();
      Serial.println();
      delay(1000);
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) {
    PID_control();
  }
  else {
    forward_brake(0, 0); //stop the motors
  }

  uint16_t position = qtr.readLineBlack(sensorValues);


  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  PID_control();

}
void forward_brake(int posa, int posb) {
  analogWrite(ENA, posa);
  analogWrite(ENB, posb);

  //control direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}


void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }
  forward_brake(motorspeeda, motorspeedb);
}

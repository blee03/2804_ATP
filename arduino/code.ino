#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

#define TIMESTEP_MS 1
#define P_GAIN 0.001
#define D_GAIN 0.001

float th_initial_offset = 0;
float th_err = 0;
float th_err_prev = 0;
float th_desired = 90;

int motor_goal_l = 255;
int motor_goal_r = 255;
int motor_cmd_l = 0;
int motor_cmd_r = 0;

int r_motor_pwm = 11;
int r_motor_en_a = 10;
int r_motor_en_b = 9;
int l_motor_en_a = 8;
int l_motor_en_b = 7;
int l_motor_pwm = 6;

int buttonPin = 2;
int lastButtonState = 0;
int buttonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int turn_count = 0;

bool running = false;

MPU6050 mpu(Wire);
SoftwareSerial ble(3, 4);

void setup() {
  // put your setup code here, to run once:
  pinMode(r_motor_pwm, OUTPUT);
  pinMode(r_motor_en_a, OUTPUT);
  pinMode(r_motor_en_b, OUTPUT);
  pinMode(r_motor_pwm, OUTPUT);
  pinMode(r_motor_en_a, OUTPUT);
  pinMode(r_motor_en_b, OUTPUT);

  pinMode(buttonPin, INPUT);
  
  digitalWrite(r_motor_en_b, LOW);
  digitalWrite(r_motor_pwm, LOW);
  digitalWrite(r_motor_en_a, LOW);
  digitalWrite(r_motor_en_b, LOW);
  
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status != 0){};

  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");

  ble.begin(9600);
  Serial.println("Established connection with HM-10 at 9600");
  Serial.println("Awaiting start...");
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  th_initial_offset = mpu.getAngleZ();

  if (ble.available())
  {
    Serial.print("Got the character: "); Serial.println(ble.read());
    running = true;
  }

  pollForButton();

  // Main control loop
  while (running)
  {
    unsigned long tStart = millis();
    mpu.update();
    double current_heading = mpu.getAngleZ();

    th_err = (th_desired - th_initial_offset) - current_heading;
    float C = (P_GAIN * th_err) + (D_GAIN * (th_err - th_err_prev));
    motor_cmd_r = motor_goal_r + C;
    motor_cmd_l = motor_goal_l - C;

    th_err_prev = th_err;

    Serial.print("Heading Z: "); Serial.print((current_heading - th_initial_offset)); Serial.print("\t");
    Serial.print("Right Motor: "); Serial.print(motor_cmd_r); Serial.print("\t");
    Serial.print("Left Motor Z: "); Serial.println(motor_cmd_l);

    if (motor_cmd_r < 0)
    {
      digitalWrite(r_motor_en_a, HIGH);
      digitalWrite(r_motor_en_b, LOW);
      analogWrite(r_motor_pwm, motor_cmd_r);
    }
    else
    {
      digitalWrite(r_motor_en_a, LOW);
      digitalWrite(r_motor_en_b, HIGH);
      analogWrite(r_motor_pwm, motor_cmd_r);
    }

    if (motor_cmd_l < 0)
    {
      digitalWrite(l_motor_en_a, HIGH);
      digitalWrite(l_motor_en_b, LOW);
      analogWrite(l_motor_pwm, motor_cmd_r);
    }
    else
    {
      digitalWrite(l_motor_en_a, LOW);
      digitalWrite(l_motor_en_b, HIGH);
      analogWrite(l_motor_pwm, motor_cmd_r);
    }
    ble.print(mpu.getTemp()); ble.print(", ");
    
    ble.print(mpu.getAccX()); ble.print(", ");
    ble.print(mpu.getAccY()); ble.print(", ");
    ble.print(mpu.getAccZ()); ble.print(", ");

    ble.print(mpu.getGyroX()); ble.print(", ");
    ble.print(mpu.getGyroY()); ble.print(", ");
    ble.print(mpu.getGyroZ()); ble.print(", ");

    ble.print(mpu.getAccAngleX()); ble.print(", ");
    ble.print(mpu.getAccAngleY()); ble.print(", ");

    ble.print(mpu.getAngleX()); ble.print(", ");
    ble.print(mpu.getAngleY()); ble.print(", ");
    ble.println(mpu.getAngleZ());

    pollForButton();

    while (millis() - tStart < TIMESTEP_MS) {}
  }
}

void pollForButton()
{
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;

      if (buttonState == HIGH)
      {
        running = !running;
      }
    }
  }

  lastButtonState = reading;
}

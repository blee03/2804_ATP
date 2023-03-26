#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

#define TIMESTEP_MS 1
#define K_P 40
#define K_D 0

typedef enum {OFF, CW, CCW} _motor_dir;
typedef enum {START, DRIVE, TURN, STOP} _tractorFSM_state;
typedef enum {StableP, PR, RP, StableR} _button_state;
typedef enum {START_MSG, STOP_MSG, NONE} _ble_msg;

typedef struct
{
  int en_a;
  int en_b;
  int pwm_pin;
  _motor_dir dir;
  int pwm;
} _Motor;

typedef struct
{
  int pin;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  _button_state state;
  _button_state lastState;
  bool isTapped;
} _Button;

typedef struct
{
  SoftwareSerial ss;
  _ble_msg msg;
} _BLE;

typedef struct
{
  int trig;
  int echo;
  long duration;
  int distance;
} _Ultrasonic;

typedef struct
{
  int turn_count;
  double application_delay_ms;
  _tractorFSM_state state;
  int intended_angle;
} _Application;

_Motor r_motor = {9, 10, 11, OFF, 0};
_Motor l_motor = {8,7, 6, OFF, 0};

_Button _b = {2, 0, 30, StableR};

SoftwareSerial ss(3, 4);
_BLE _ble = {ss, NONE};

_Ultrasonic _us = {13, 12, 0, 0};

_Application app;

float th_initial_offset = 0;

MPU6050 mpu(Wire);


void setup() {
  Serial.begin(9600);
  
  init_motor(&r_motor);
  init_motor(&l_motor);
  
  init_mpu();

  pinMode(_b.pin, INPUT);

  _ble.ss.begin(9600);
  Serial.println("Established connection with HM-10 at 9600");

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(_us.trig, OUTPUT);
  pinMode(_us.echo, INPUT);

  app = {0, 25, START, 1};
  
  Serial.println("Awaiting start...");
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tStart = millis();
  
  Application_loop();

  pollForButton(&_b);
  pollForBLE(&_ble);
  // ultrasonicPing(&_us);
  ble_data_tx(&_ble);

  while (millis() - tStart < app.application_delay_ms);
}

void Application_loop()
{
  mpu.update();
  switch (app.state)
  {
    case START:
      if (_b.isTapped || (_ble.msg == START_MSG))
      {
        Serial.println("\tSTART -> DRIVE");
        // Perform other startup tasks
        
        th_initial_offset = mpu.getAngleZ();
        app.state = DRIVE;
      }
      break;
    case DRIVE:
      // if ultrasonic sensor distance too close
      // transition to stop

      if(turn() == true) {
        app.state = TURN;
        Serial.println("turn");
      }
      else {
        // Serial.println("continue driving");
      }

      if (_b.isTapped || (_ble.msg == STOP_MSG))
      {
        app.state = STOP;
        Serial.println("\tDRIVE -> STOP");
      }
      else
      {
        motor_pd(app.intended_angle);
      }
      break;
    case TURN:
      if(app.turn_count==(1 || 2 || 5 || 6)){
        motor_pd(app.intended_angle + 90);
        app.intended_angle = app.intended_angle + 90;
        app.turn_count = app.turn_count + 1;
        Serial.println("right turn");
        app.state = DRIVE;
      }
      else if(app.turn_count = (3 || 4)) {
        motor_pd(app.intended_angle-90);
        app.intended_angle = app.intended_angle - 90;
        app.turn_count = app.turn_count + 1;
        Serial.println("left turn");
        app.state = DRIVE;
      }
      else {
        app.state = STOP;
      }
      break;
    case STOP:
      l_motor.dir = OFF;
      r_motor.dir = OFF;
      drive_motor(&l_motor);
      drive_motor(&r_motor);

      Serial.println("Terminating...");
      Serial.println("\tSTOP -> START");

      // Perform other stop tasks

      app.state = START;
      break;
  }
}

void init_motor(_Motor* m)
{
  pinMode(m->en_a, OUTPUT);
  pinMode(m->en_b, OUTPUT);
  pinMode(m->pwm_pin, OUTPUT);

  digitalWrite(m->en_a, LOW);
  digitalWrite(m->en_b, LOW);
}

void init_mpu()
{
  Wire.begin();

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status != 0){};

  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");
}

void pollForButton(_Button* b)
{
  int reading = digitalRead(b->pin);

  bool released = true;
  b->lastState = b->state;

  switch (b->state)
  {
    case StableR:
      if (reading)
      {
        b->lastDebounceTime = millis();
        b->state = RP;
      }
      break;
    case StableP:
      if (!reading)
      {
        b->lastDebounceTime = millis();
        b->state = PR;
      }
      break;
    case RP:
      if (!reading)
      {
        b->state = StableR;
      }
      else if ((millis() - b->lastDebounceTime) > b->debounceDelay)
      {
        b->state = StableP;
      }
      break;
    case PR:
      if (reading)
      {
        b->state = StableP;
      }
      else if ((millis() - b->lastDebounceTime) > b->debounceDelay)
      {
        b->state = StableR;
      }
      break;
  }
  
  if ((b->lastState != b->state) && b->state == StableP)
  {
    b->isTapped = true;
  }
  else
    b->isTapped = false;
}

void pollForBLE(_BLE* ble)
{
  if (ble->ss.available())
  {
    byte c = ble->ss.read() - 48;
    Serial.print("Received: "); Serial.println(c);
    switch(c)
    {
      case 1:
        Serial.println("Starting Loop!");
        ble->msg = START_MSG;
        break;
      case 2:
        Serial.println("Ending Loop!");
        ble->msg = STOP_MSG;
        break;
      default:
        ble->msg = NONE;
    }
  }
}

void ultrasonicPing(_Ultrasonic* u)
{
  digitalWrite(u->trig, LOW);
  delayMicroseconds(2);
  digitalWrite(u->trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(u->trig, LOW);
  u->duration = pulseIn(u->echo, HIGH);

  u->distance = u->duration * 0.034 / 2;
}

void motor_pd(float th_goal)
{
  static float th_err_prev = 0;
  
  float current_heading = mpu.getAngleZ();

  float th_err = (th_goal) - (current_heading - th_initial_offset);
  float PD = (K_P * th_err) + (K_D * (th_err - th_err_prev));
  int motor_cmd_r = 220 - PD;
  int motor_cmd_l = 220 + PD;

  Serial.print(th_err); Serial.print("\t");
  Serial.print(motor_cmd_r); Serial.print("\t");
  Serial.println(motor_cmd_l);

  th_err_prev = th_err;

  if (motor_cmd_r < 0)
  {
    r_motor.dir = CW;
  }
  else if (motor_cmd_r > 0)
  {
    r_motor.dir = CCW;
  }
  else
    r_motor.dir = OFF;

  if (motor_cmd_l < 0)
  {
    l_motor.dir = CW;
  }
  else if (motor_cmd_l > 0)
  {
    l_motor.dir = CCW;
  }
  else
    l_motor.dir = OFF;
  
  r_motor.pwm = abs(motor_cmd_r);
  l_motor.pwm = abs(motor_cmd_l);

  drive_motor(&l_motor);
  drive_motor(&r_motor);
}

void drive_motor(_Motor* m)
{
  switch (m->dir)
  {
    case CW:
      digitalWrite(m->en_a, LOW);
      digitalWrite(m->en_b, HIGH);
      analogWrite(m->pwm_pin, min(255, m->pwm));
      break;
    case CCW:
      digitalWrite(m->en_a, HIGH);
      digitalWrite(m->en_b, LOW);
      analogWrite(m->pwm_pin, min(255, m->pwm));
      break;
    case OFF:
      digitalWrite(m->en_a, LOW);
      digitalWrite(m->en_b, LOW);
      break;
  }
}

void ble_data_tx(_BLE* ble)
{
  ble->ss.print("WHAT IS UP MY NEIGHBOR");
  ble->ss.print("MSG 2");
  ble->ss.print("MSG 3");
}

bool turn() {
  static float lastReading = analogRead(0);
  
  float reading = analogRead(0);
  if(reading>1000) {
    return true; // on dark surface
  }
  else {
    return false; // on light surface
  }
}
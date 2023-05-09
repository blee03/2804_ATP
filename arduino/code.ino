#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

#define TIMESTEP_MS 1
#define K_P 40
#define K_D 20

typedef enum {OFF, CW, CCW} _motor_dir;
typedef enum {START, DRIVE, TURN, STOP} _tractorFSM_state;
typedef enum {StableP, PR, RP, StableR} _button_state;
typedef enum {NONE, START_MSG, STOP_MSG, PING_MSG} _ble_msg;

typedef struct {
  int en_a;
  int en_b;
  int pwm_pin;
  _motor_dir dir;
  int pwm;
} _Motor;

typedef struct {
  int pin;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  _button_state state;
  _button_state lastState;
  bool isTapped;
} _Button;

typedef struct {
  SoftwareSerial ss;
  _ble_msg msg;
} _BLE;

typedef struct {
  int trig;
  int echo;
  long duration;
  int distance;
} _Ultrasonic;

typedef struct {
  int turn_count;
  int old_turn_count;
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

long int runtime_start = 0;
long int runtime_stop = 0;
float th_initial_offset = 0;

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  
  init_motor(&r_motor);
  init_motor(&l_motor);
  
  init_mpu();

  pinMode(_b.pin, INPUT);

  pinMode(13, INPUT);

  _ble.ss.begin(9600);
  Serial.println("Established connection with HM-10 at 9600");

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(_us.trig, OUTPUT);
  pinMode(_us.echo, INPUT);

  app = {1, 0, 25, START, 0};
  
  Serial.println("Awaiting start...");
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long tStart = millis();
  Application_loop();

  pollForButton(&_b);
  pollForBLE(&_ble);
  // ultrasonicPing(&_us);

  while (millis() - tStart < app.application_delay_ms);
}

void Application_loop()
{
  mpu.update();
  switch (app.state) {
    case START:
      if (_b.isTapped || (_ble.msg == START_MSG)) {
        Serial.println("\tSTART -> DRIVE");
        runtime_start = millis();
        th_initial_offset = mpu.getAngleZ();
        app.state = DRIVE;
      }
      break;
    case DRIVE:
      // if ultrasonic sensor distance too close
      // transition to stop

      if (_b.isTapped || (_ble.msg == STOP_MSG) || digitalRead(13)) {
        app.state = STOP;
        Serial.println("\tDRIVE -> STOP");
      } 
      else if(turn()) {
        app.state = TURN;
        Serial.println("\tDRIVE -> TURN");

        if (app.turn_count== 1 || app.turn_count == 2 || app.turn_count == 5 || app.turn_count == 6){
          app.intended_angle += 90;
          app.turn_count++;
          Serial.println("Turning left...");
        } else if (app.turn_count == 3 || app.turn_count == 4) {
          app.intended_angle -= 90;
          app.turn_count++;
          Serial.println("Turning right...");
        } else {
          app.state = STOP;
          Serial.println("\tTURN -> STOP");
        }
      }
      else {
        motor_pd(app.intended_angle);
      }
      break;
    case TURN:
      if (_b.isTapped || (_ble.msg == STOP_MSG) || digitalRead(13)) {
        app.state = STOP;
        Serial.println("\tTURN -> STOP");
      } else if (turnTofloor()) {
        app.state = DRIVE;
        Serial.println("\tTURN -> DRIVE");
      } else {
        motor_pd(app.intended_angle);
      }
      break;
    case STOP:
      l_motor.dir = OFF;
      r_motor.dir = OFF;
      drive_motor(&l_motor);
      drive_motor(&r_motor);

      app.intended_angle = 0;
      app.old_turn_count = app.turn_count;
      app.turn_count = 1;
      runtime_stop = millis();
      Serial.println("Terminating...");
      Serial.println("\tSTOP -> START");

      // Perform other stop tasks

      app.state = START;
      break;
  }
}

void init_motor(_Motor* m) {
  pinMode(m->en_a, OUTPUT);
  pinMode(m->en_b, OUTPUT);
  pinMode(m->pwm_pin, OUTPUT);

  digitalWrite(m->en_a, LOW);
  digitalWrite(m->en_b, LOW);
}

void init_mpu() {
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

void pollForButton(_Button* b) {
  int reading = digitalRead(b->pin);

  bool released = true;
  b->lastState = b->state;

  switch (b->state)
  {
    case StableR:
      if (reading) {
        b->lastDebounceTime = millis();
        b->state = RP;
      }
      break;
    case StableP:
      if (!reading) {
        b->lastDebounceTime = millis();
        b->state = PR;
      }
      break;
    case RP:
      if (!reading) {
        b->state = StableR;
      } else if ((millis() - b->lastDebounceTime) > b->debounceDelay) {
        b->state = StableP;
      }
      break;
    case PR:
      if (reading) {
        b->state = StableP;
      } else if ((millis() - b->lastDebounceTime) > b->debounceDelay) {
        b->state = StableR;
      }
      break;
  }
  
  if ((b->lastState != b->state) && b->state == StableP) {
    b->isTapped = true;
  } else
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
      case 3:
        ble->msg = PING_MSG;
        delay(50);
        if(app.state==DRIVE)
        {
          ble->ss.print("tractor in motion");
        }
        else
        {
          ble->ss.print((float) (runtime_stop-runtime_start) / 1000);
          ble->ss.print("\t");
          ble->ss.print(app.old_turn_count);
        }
        
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

  // Serial.print(th_err); Serial.print("\t");
  // Serial.print(motor_cmd_r); Serial.print("\t");
  // Serial.println(motor_cmd_l);

  th_err_prev = th_err;

  if (motor_cmd_r < 0) {
    r_motor.dir = CW;
  } else if (motor_cmd_r > 0) {
    r_motor.dir = CCW;
  } else
    r_motor.dir = OFF;

  if (motor_cmd_l < 0) {
    l_motor.dir = CW;
  } else if (motor_cmd_l > 0) {
    l_motor.dir = CCW;
  } else
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
  // Print motors and direction
  // print LR pwm
  // print LR motor direction
  // ble->ss.print(r_motor.dir);ble->ss.print("\t");
  // ble->ss.print(r_motor.pwm);ble->ss.print("\t");
  // ble->ss.print(l_motor.dir);ble->ss.print("\t");
  // ble->ss.print(l_motor.pwm);ble->ss.print("\t");

  // // Print imu acc_x,acc_y,acc_z, w_x,w_y,w_z, orientation
  // ble->ss.print(mpu.getAccX());ble->ss.print("\t");
  // ble->ss.print(mpu.getAccY());ble->ss.print("\t");
  // ble->ss.print(mpu.getAccZ());ble->ss.print("\t");
  // ble->ss.print(mpu.getGyroX());ble->ss.print("\t");
  // ble->ss.print(mpu.getGyroY());ble->ss.print("\t");
  // ble->ss.print(mpu.getGyroZ());ble->ss.print("\t");

  // ble->ss.print(mpu.getAngleZ());ble->ss.print("\t");
  
  // // Print object detection status 
  // // not implemented yet

  // // Print FSM state
  // // print turn count and intended angle
  ble->ss.print(app.turn_count);ble->ss.print("\t");
  // ble->ss.print(app.intended_angle);ble->ss.print("\t");
  // ble->ss.print(app.state);ble->ss.print("\t");
}

bool turn() {
  static float lastReading = 0;

  bool retVal = false;

  float reading = analogRead(0);
  if((reading - lastReading) > 100) {
    retVal = true; // on dark surface
    Serial.println("Detected turn!");
  } else {
    retVal = false; // on light surface
  }

  lastReading = reading;
  return retVal;
}

bool turnTofloor() {
  static float lastReading = analogRead(0);

  bool retVal = false;

  float reading = analogRead(0);
  if ((lastReading - reading) > 100) {
    Serial.println("Moving from tape to floor!");
    retVal = true;
  } else
    retVal = false;

  lastReading = reading;
  return retVal;
}

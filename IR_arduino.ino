#include "pins_arduino.h"
#include "AngularSpeedSupervisor.h"
#include "SharpIR.h"

#define GET_SENSOR_5 0xff
#define GET_SENSOR_4 0xfe
#define GET_SENSOR_3 0xfd
#define GET_SENSOR_2 0xfc
#define GET_SENSOR_1 0xfb

#define GET_TICKS_L 0xfa
#define GET_TICKS_R 0xf9

#define SET_ANGULAR_VELOCITY_L 0xf8
#define SET_ANGULAR_VELOCITY_R 0xf7

#define SIZE 4
#define SENSOR_MODEL GP2Y0A21YK0F
#define RANGE_SENSOR_PIN_0  0
#define RANGE_SENSOR_PIN_1  1
#define RANGE_SENSOR_PIN_2  2
#define RANGE_SENSOR_PIN_3  3
#define RANGE_SENSOR_PIN_4  4

long buf[SIZE];
volatile short pos = 0;
volatile byte command;
volatile bool processSpiData;

double k_p_angular_speed = 2.1;
double k_i_angular_speed = 120.0;
double k_d_angular_speed = 0.5;

/*
double k_p_angular_speed = 2.0;
double k_i_angular_speed = 120.0;
double k_d_angular_speed = 0.01;
*/

unsigned int cpr = 930;
int lower_pwm_left = 80;
int lower_pwm_right = 110;
int max_rpm = 130;

Encoder* enc_left = new Encoder(2, 3, cpr);
Encoder* enc_right = new Encoder(19, 18, cpr);
Motor* motor_left = new Motor(10, 11, lower_pwm_left, max_rpm);
Motor* motor_right = new Motor(9, 8, lower_pwm_right, max_rpm);
PIDController* pid_angular_speed_left = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);
PIDController* pid_angular_speed_right = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);

SharpIR* sensor_0 = new SharpIR(SENSOR_MODEL, RANGE_SENSOR_PIN_0);
SharpIR* sensor_1 = new SharpIR(SENSOR_MODEL, RANGE_SENSOR_PIN_1);
SharpIR* sensor_2 = new SharpIR(SENSOR_MODEL, RANGE_SENSOR_PIN_2);
SharpIR* sensor_3 = new SharpIR(SENSOR_MODEL, RANGE_SENSOR_PIN_3);
SharpIR* sensor_4 = new SharpIR(SENSOR_MODEL, RANGE_SENSOR_PIN_4);

AngularSpeedSupervisor* angular_speed_supervisor_left = new AngularSpeedSupervisor(enc_left, motor_left, pid_angular_speed_left);
AngularSpeedSupervisor* angular_speed_supervisor_right = new AngularSpeedSupervisor(enc_right, motor_right, pid_angular_speed_right);

void enc_left_A()
{
  enc_left->A_interrupt();
}

void enc_left_B()
{
  enc_left->B_interrupt();
}

void enc_right_A()
{
  enc_right->A_interrupt();
}

void enc_right_B()
{
  enc_right->B_interrupt();
}

long tmp_left = 0;
long tmp_right = 0;

unsigned long last_time = 0L;
unsigned long time = 0L;
double dt = 0.0;

void watch_time()
{
  last_time = time;
  time = millis();
  dt = (time - last_time) / 1000.0;
  if(dt == 0.0)
  {
    dt = 0.012;
    delay(1);
  }
}

ISR(SPI_STC_vect)
{
  byte c = SPDR;
  if(command == GET_TICKS_L || command == GET_TICKS_R || command == GET_SENSOR_5 || command == GET_SENSOR_4 || command == GET_SENSOR_3 || command == GET_SENSOR_2 || command == GET_SENSOR_1)
  {
    SPDR = buf[pos++];
    if(pos == SIZE)
    {
      pos = 0;
      command = 0;
    }
  }
  else if(command == SET_ANGULAR_VELOCITY_L || command == SET_ANGULAR_VELOCITY_R)
  {
      buf[pos++] = c;
      if(pos == SIZE)
      {
        long value = buf[3] | (buf[2] | (buf[1] | (buf[0]) << 8) << 8) << 8;
        set_angular_velocity(command, value);
        pos = 0;
        command = 0;
      }
  }
  else if(c >= SET_ANGULAR_VELOCITY_R)
  {
    command = c;
    if(c == GET_TICKS_L)
    {
      long ticks_l = enc_left->count;
      value_to_bytes(ticks_l);
      SPDR = buf[pos++];
    }
    else if(c == GET_TICKS_R)
    {
      long ticks_r = enc_right->count;
      value_to_bytes(ticks_r);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_1)
    {
      long distance = sensor_0->getDistance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_2)
    {
      long distance = sensor_1->getDistance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_3)
    {
      long distance = sensor_2->getDistance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_4)
    {
      long distance = sensor_3->getDistance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_5)
    {
      long distance = sensor_4->getDistance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
    }
  }
}

void spi_init()
{
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  pos = 0;
  processSpiData = false;
}

void motor_init()
{
  attachInterrupt(digitalPinToInterrupt(enc_left->pinA), enc_left_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_left->pinB), enc_left_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right->pinA), enc_right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right->pinB), enc_right_B, CHANGE);
}

void set_angular_velocity(byte command, long value)
{
  //Serial.print(value);
  //Serial.print(" ");
  if(command == SET_ANGULAR_VELOCITY_L)
  {
    angular_speed_supervisor_left->set_ref_by_speed(value, false);
  }
  else if(command == SET_ANGULAR_VELOCITY_R)
  {
    angular_speed_supervisor_right->set_ref_by_speed(value, false);
  }
}

void value_to_bytes(long value)
{
  buf[0] = value & 0xff;
  buf[1] = value >> 8 & 0xff;
  buf[2] = value >> 16 & 0xff;
  buf[3] = value >> 24 & 0xff;
}

void setup() {
  Serial.begin(115200);

  motor_init();
  spi_init();
}

void loop() {
  if(tmp_left != enc_left->count)
  {
    tmp_left = enc_left->count;
  }
  if(tmp_right != enc_right->count)
  {
    tmp_right = enc_right->count;
  }
  watch_time();
  angular_speed_supervisor_left->execute(dt + 0.02);
  angular_speed_supervisor_right->execute(dt);
  Serial.println();
  delay(10);
}

#include "pwm_lib.h"
#include "tc_lib.h"
#include <AccelStepper.h>
using namespace arduino_due::pwm_lib;

#define ANGLES 8
#define PWM_PERIOD 2000000 // hundredth of usecs (1e-8 secs)
#define CAPTURE_TIME_WINDOW 40000 // usecs
#define ANGLE_CHANGE_TIME 500 // msecs
#define pull_der 5
#define dir_der 37
#define pull_izq 6
#define dir_izq 39
#define pull_aux 7
#define dir_aux 35

AccelStepper motor_derecho(1,pull_der,dir_der);
AccelStepper motor_izquierdo(1,pull_izq,dir_izq);
AccelStepper motor_aux(1,pull_aux,dir_aux);


uint32_t angles[ANGLES]= // degrees
{
  0,
  45, 
  90,
  135,
  180,
  135,
  90,
  45
};
uint32_t angle=0;
uint32_t last_angle;

servo<pwm_pin::PWML0_PB16> servo_pwm_pinDAC1; // PB16 is DUE's pin DAC1

capture_tc0_declaration();
capture_tc8_declaration();
capture_tc7_declaration();

auto& capture_pin2=capture_tc0;
auto& capture_pin3=capture_tc7;
auto& capture_pin11=capture_tc8;

uint32_t status2,duty2,period2,int1;
uint32_t status3,duty3,period3,int2;
uint32_t status4,duty4,period4,int3;

float der,izq,aux;
long sup_izq, inf_izq, sup_der, inf_der, sup_aux, inf_aux;

void setup() {
  Serial.begin(250000);

  pinMode(50,INPUT_PULLUP);
  capture_pin2.config(CAPTURE_TIME_WINDOW);
  capture_pin3.config(CAPTURE_TIME_WINDOW);
  capture_pin11.config(CAPTURE_TIME_WINDOW);

  motor_derecho.setMaxSpeed(3000);
  motor_derecho.setSpeed(0);
  motor_derecho.setAcceleration(1);

  motor_izquierdo.setMaxSpeed(3000);
  motor_izquierdo.setSpeed(0);
  motor_izquierdo.setAcceleration(1);
  
  motor_aux.setMaxSpeed(3000);
  motor_aux.setSpeed(0);
  motor_aux.setAcceleration(1000);

  
  servo_pwm_pinDAC1.start(
    PWM_PERIOD, // pwm servo period
    100000, // 1e-8 secs. (1 msecs.), minimum duty value
    200000, // 1e-8 secs. (2 msecs.), maximum duty value
    0, // minimum angle, corresponding minimum servo angle
    180, // maximum angle, corresponding minimum servo angle
    angles[angle] // initial servo angle 
  );
  last_angle=millis();
  Serial.println("********************************************************");
  Serial.print("angle "); Serial.print(angle); 
  Serial.print(": "); Serial.print(angles[angle]); 
  Serial.println(" dgs.");
  Serial.println("********************************************************");
  
}

void loop() {

  status2=capture_pin2.get_duty_and_period(duty2,period2);
  int1=duty2/capture_pin2.ticks_per_usec();
  status3=capture_pin3.get_duty_and_period(duty3,period3);
  int2=duty3/capture_pin3.ticks_per_usec();
  status4=capture_pin11.get_duty_and_period(duty4,period4);
  int3=duty4/capture_pin11.ticks_per_usec();

  
  if(1470>int1 && int1>950){
    der=-mapf(int1, 1470, 1050, 0, 2000);
    motor_derecho.setSpeed(der);
    motor_derecho.runSpeed();
  }else if(1580<int1 && int1<2100){
    der=mapf(int1, 1580, 1955, 0, 2000);
    motor_derecho.setSpeed(der);
    motor_derecho.runSpeed();
  }else{
    motor_derecho.stop();
    der=0;
  }

  if(1450>int2 && int2>950){
    izq=-mapf(int2, 1470, 1050, 0, 2400);
    motor_izquierdo.setSpeed(izq);
    motor_izquierdo.runSpeed();
  }else if(1550<int2 && int1<2100){
    izq=mapf(int2, 1580, 1955, 0, 2400);
    motor_izquierdo.setSpeed(izq);
    motor_izquierdo.runSpeed();
  }else{
    motor_izquierdo.stop();
    izq=0;
  }
  
  if(1450>int3 && int3>950){
    aux=-mapf(int3, 1470, 1000, 0, 760);
    motor_aux.setSpeed(aux);
    motor_aux.runSpeed();
  }else if(1550<int3 && int1<2100){
    aux=mapf(int3, 1580, 2000, 0, 760);
    motor_aux.setSpeed(aux);
    motor_aux.runSpeed();
  }else{
    motor_aux.stop();
    aux=0;
  }
  

  if(millis()-last_angle>ANGLE_CHANGE_TIME)
  {
    angle=(angle+1)&0x07;
    //servo_pwm_pinDAC1.set_angle(angles[angle]);
    last_angle=millis();
/*    
    Serial.print("entrada 1  "); 
    Serial.println(int1);
  
    Serial.print("pulsos/s  "); 
    Serial.println(int2);
  
    Serial.print("entrada 3  "); 
    Serial.println(aux);
  
    Serial.print("angle ");
    Serial.print(angles[angle]); */
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define SERVO_RIGHT_LIMIT 1030
#define SERVO_LEFT_LIMIT  1830
#define SERVO_ZERO        1480
#define MOTOR_BACK_LIMIT  0
#define MOTOR_FWD_LIMIT   0
#define MOTOR_ZERO        0

#include "PinChangeInterrupt.h"
#include <Servo.h>

// These come from the radio receiver via three black-red-white ribbons.
#define PIN_SERVO_IN 11
#define PIN_MOTOR_IN 10

// These go out to ESC (electronic speed controller) and steer servo via black-red-white ribbons.
#define PIN_SERVO_OUT 9
#define PIN_MOTOR_OUT 8

Servo servo;
Servo motor;

int RC_servo_pwm;
int RC_motor_pwm;
int servo_pwm;
int motor_pwm;
int servos_attached = 0;

int servo_delay;
int motor_delay;
int max_communication_delay = 100;

volatile long unsigned int servo_prev_interrupt_time  = 0;
volatile long unsigned int motor_prev_interrupt_time  = 0;
long unsigned int servo_command_time;
long unsigned int motor_command_time;

volatile float encoder_value_1 = 0.0;
volatile float encoder_value_2 = 0.0;

 

void setup() {
  // put your setup code here, to run once:

  motor_servo_setup();
  encoder_setup();
  Serial.begin(115200);
  Serial.setTimeout(5);

  //empty the input buffer
  while(Serial.available() > 0) {
    Serial.read();
  }

}




void loop() {

  int pwm_input;
  int num_serial_reads = 2;

  // takes in steering pwm and then throttle pwm, where 10,000 is temporarily 
  // added to the throttle pwm to know it refers to throttle and not steering
  for( int i = 0; i < num_serial_reads; i = i + 1 ) {
    pwm_input = Serial.parseInt();
    Serial.println(pwm_input);

    // steering pwm
    if (pwm_input > 0 && pwm_input < 10000) {
      servo_pwm = pwm_input;
      servo_command_time = millis();
    } 

    // throttle pwm + 10000
    else if (pwm_input >= 10000 && pwm_input < 100000) {
      motor_pwm = pwm_input - 10000;
      motor_command_time = millis();
    }
  }

  // time since last command
  servo_delay = millis() - servo_command_time;
  motor_delay = millis() - motor_command_time;

  // if too much time passes, stop all motors. Eventually just detach them...why?
  if(0){
  //if(servo_delay > max_communication_delay || motor_delay > max_communication_delay) {
    servo_pwm = SERVO_ZERO;
    motor_pwm = MOTOR_ZERO;
    
    if(servo_delay > 4*max_communication_delay || motor_delay > 4*max_communication_delay) {
      servo.detach(); 
      motor.detach(); 
      servos_attached = 0;      
    }
  } 
  
  else {
    if(servos_attached==0) {
      servo.attach(PIN_SERVO_OUT); 
      motor.attach(PIN_MOTOR_OUT); 
      servos_attached = 1;
    }
  }
  
  encoder_loop(); 
  print_stats(); 
  
  delay(500);
}




void print_stats() {
  Serial.print("('mse',");
  Serial.print(RC_servo_pwm);
  Serial.print(",");
  Serial.print(RC_motor_pwm);
  Serial.print(",");
  Serial.print(encoder_value_1);
  Serial.println(")");
}




void motor_servo_setup() { 
  // not sure why needs to be PULLUP
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN, INPUT_PULLUP);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_MOTOR_IN),
    motor_interrupt_service_routine, CHANGE);
  
  while(servo_pwm==0 || motor_pwm==0) {
    delay(200);
    Serial.println("waiting for remote controller...");
  }
}




// this func gets invoked always (even if RC is off) for some reason. 
// Maybe needs a pullup resistor? Or leave it be
// not sure why this even works because it would also count for how long
// the signal is low, rather than just how long it's on
void servo_interrupt_service_routine(void) {
  
  // will use controller pwm later for having human take control of car
  RC_servo_pwm = micros() - servo_prev_interrupt_time;
  servo_prev_interrupt_time = micros(); 
  if (servo_pwm > SERVO_RIGHT_LIMIT && servo_pwm < SERVO_LEFT_LIMIT) {
    servo.writeMicroseconds(servo_pwm);
  } 
}




void motor_interrupt_service_routine(void) {
  
  RC_motor_pwm = micros() - motor_prev_interrupt_time;
  motor_prev_interrupt_time = micros(); 
  if (motor_pwm > MOTOR_BACK_LIMIT && motor_pwm < MOTOR_FWD_LIMIT) {
    servo.writeMicroseconds(motor_pwm);
  }
}













////////////// ENCODER //////////////////
//PIN's definition
#include "RunningAverage.h"
#define encoder0PinA  2
#define encoder0PinB  3

RunningAverage enc_avg(10);

volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile unsigned long int a = 0;
volatile unsigned long int b = 0;
volatile unsigned long int t1 = micros();
volatile unsigned long int t2 = 0;
volatile unsigned long int last_t2 = 0;
volatile unsigned long int dt = 0;


void encoder_setup() 
{
  //Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 

  enc_avg.clear();
}

volatile unsigned long int doEncoderAdtSum = 1;

void encoder_loop()
{  
  dt = micros()-t1;
  if (doEncoderAdtSum > 0) {
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderAdtSum); //6 magnets
    encoder_value_1 = enc_avg.getAverage();
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    encoder_value_1 = 0;
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  }
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
volatile float doEncoderAdt = 0.;
void doEncoderA()
{
  t2 = micros();
  a = a + 1;
  doEncoderAdtSum += t2 - last_t2; 
  //doEncoderAdt = float(t2 - last_t2);
  //enc_avg.addValue(62500. / doEncoderAdt);
  //encoder_value_1 = enc_avg.getAverage();
  last_t2 = t2;
}

void doEncoderB()
{
     b += 1;
}
//
///////////////////

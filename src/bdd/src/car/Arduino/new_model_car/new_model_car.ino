#define SERVO_RIGHT_LIMIT 1100
#define SERVO_ZERO        1440
#define SERVO_LEFT_LIMIT  1780

#define MOTOR_BACK_LIMIT  1000 // Unkown, but shouldn't be lower to not appear as servo pwm in code below
#define MOTOR_FWD_LIMIT   2000 // Unknown, could be higher, but as 2000 it makes MOTOR_ZERO be halfway to BACK_LIMIT
#define MOTOR_ZERO        1500

#define STATE_ONE         1908 // NN control
#define STATE_TWO         1720 // Human annotation
#define STATE_THREE       1432 // Human correction
#define STATE_FOUR        872  // Calibration


#include "PinChangeInterrupt.h"
#include <Servo.h>

// These come from the radio receiver via three black-red-white ribbons.
#define PIN_MOTOR_IN 10
#define PIN_SERVO_IN 11
#define PIN_BUTTON_IN 12


// These go out to ESC (electronic speed controller) and steer servo via black-red-white ribbons.
#define PIN_MOTOR_OUT 8
#define PIN_SERVO_OUT 9

Servo servo;
Servo motor;

volatile int RC_servo_pwm = 0;
volatile int RC_motor_pwm = 0;
volatile int RC_button_pwm = 1210;
int button_pwm_epsilon = 50;
int servo_pwm = SERVO_ZERO;
int motor_pwm = MOTOR_ZERO;
int servos_attached = 0;

int servo_delay = 0;
int motor_delay = 0;
int max_communication_delay = 100;

volatile unsigned long int servo_prev_interrupt_time  = 0;
volatile unsigned long int motor_prev_interrupt_time  = 0;
volatile unsigned long int button_prev_interrupt_time = 0;
unsigned long int servo_command_time = 0;
unsigned long int motor_command_time = 0;

volatile float encoder_value_1 = 0.0;
volatile float encoder_value_2 = 0.0;

 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(5);

  //empty the input buffer
  while(Serial.available() > 0) {
    Serial.read();
  }
  
  motor_servo_setup();
  encoder_setup();
}




void motor_servo_setup() { 
  pinMode(PIN_BUTTON_IN, INPUT_PULLUP);
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN, INPUT_PULLUP);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_MOTOR_IN),
    motor_interrupt_service_routine, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BUTTON_IN),
    button_interrupt_service_routine, CHANGE);


  while(RC_servo_pwm == 0 || RC_motor_pwm == 0) {
    delay(200);
    Serial.println("waiting for remote controller to be on...");
  }
}




void loop() {

  int serial_pwm_input;
  int num_serial_reads = 2;

  /* 
   *  takes in steering pwm and then throttle pwm. 
   *  10,000 is temporarily added to throttle pwm to differentiate
   *  it from steeting pwm
  */
  for( int i = 0; i < num_serial_reads; i++ ) {
    serial_pwm_input = Serial.parseInt();

    // steering pwm
    if (serial_pwm_input >= SERVO_RIGHT_LIMIT && serial_pwm_input <= SERVO_LEFT_LIMIT) {
      servo_pwm = serial_pwm_input;
      servo_command_time = millis();
    } else if (serial_pwm_input <= SERVO_RIGHT_LIMIT) {
      servo_pwm = SERVO_RIGHT_LIMIT;
      servo_command_time = millis();
    } else if (serial_pwm_input >= SERVO_LEFT_LIMIT && serial_pwm_input < 10000) {
      servo_pwm = SERVO_LEFT_LIMIT;
      servo_command_time = millis();
    }

    // throttle pwm + 10,000. To differentiate it from serial input servo PWM
    else if (serial_pwm_input >= 10000 && serial_pwm_input < 100000) {
      serial_pwm_input -= 10000;
      if (serial_pwm_input >= MOTOR_BACK_LIMIT && serial_pwm_input <= MOTOR_FWD_LIMIT) {
        motor_pwm = serial_pwm_input;
        motor_command_time = millis();
      } else if (serial_pwm_input < MOTOR_BACK_LIMIT) {
        motor_pwm = MOTOR_BACK_LIMIT;
        motor_command_time = millis();
      } else if (serial_pwm_input > MOTOR_FWD_LIMIT) {
        motor_pwm = MOTOR_FWD_LIMIT;
        motor_command_time = millis();
      }
    }
  }

  servo.writeMicroseconds(servo_pwm);
  motor.writeMicroseconds(motor_pwm);

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
    if(servos_attached == 0) {
      servo.attach(PIN_SERVO_OUT); 
      motor.attach(PIN_MOTOR_OUT); 
      servos_attached = 1;
    }
  }
  
  encoder_loop(); 
  print_stats(); 
  
  delay(30);
}




void print_stats() {
  Serial.print("mse, ");
  Serial.print(RC_button_pwm);
  Serial.print(", ");
  Serial.print(RC_servo_pwm);
  Serial.print(", ");
  Serial.println(RC_motor_pwm);
  //Serial.print(", ");
  //Serial.print(encoder_value_1);
  //Serial.println(")");
}




// this func gets invoked always (even if RC is off) for some reason. 
// Maybe needs a pullup resistor? Or leave it be
// not sure why this even works because it would also count for how long
// the signal is low, rather than just how long it's on, since it detects changes
void servo_interrupt_service_routine(void) {
  /*volatile*/ int received_pwm = micros() - servo_prev_interrupt_time;
  servo_prev_interrupt_time = micros();
  // padded limits by 1000 to gaurd against rouge values but still allow legroom for 
  // trim knob to not be centered.
  if (received_pwm >= (SERVO_RIGHT_LIMIT - 1000) && received_pwm <= (SERVO_LEFT_LIMIT + 1000)) {
    RC_servo_pwm = received_pwm;
  }
}




void motor_interrupt_service_routine(void) {
  /*volatile*/ int received_pwm = micros() - motor_prev_interrupt_time;
  motor_prev_interrupt_time = micros(); 
  // padded limits by 1000 to gaurd against rouge values but still allow legroom for 
  // trim knob to not be centered.
  if (received_pwm >= (MOTOR_BACK_LIMIT - 1000) && received_pwm <= (MOTOR_FWD_LIMIT + 1000)) {
    RC_motor_pwm = received_pwm;
  }
}




void button_interrupt_service_routine(void) {
  /*volatile*/ int received_pwm = micros() - button_prev_interrupt_time;
  button_prev_interrupt_time = micros();
  if (received_pwm >= 500 && received_pwm <= 2000) {
    RC_button_pwm = received_pwm;
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

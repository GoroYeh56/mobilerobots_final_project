// ============ Motion Control of Robot ============ //
/*

Subscribe Topics:
  /cmd_vel


Publish Topics: (maybe)
  /odometry


Description:
- The arduino_node is responsible for the motion control of the robot.
  It subscribes to /cmd_vel topic and drives the robot actuators(motor)
  using PID feedback control approach using L298N H-bridge motor control
  circuit board.

*/



/* ==== Include Libraries ===== */
//#if (ARDUINO >= 100)
//#include <Arduino.h>
//#else
//#include <WProgram.h>
//#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>


/* ==== Initialization & Global Variables ===== */

//#include <ros.h>
//#include <std_msgs/Empty.h>
//#include <std_msgs/Int64.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Float32.h>

// Pin definition

#define enA 5
#define in1 8   // left wheel
#define in2 9
#define enB 6
#define in3 10  // right wheel
#define in4 11

#define LtouchPin 12
#define RtouchPin A2
#define GoalTouchPin A1 // analog pin but input is okay!
#define IRPin 13

#define REDUCTION_RATIO 120
#define ENCODER_CPR 16

#define max_setpoint 255
#define min_setpoint 0

//ros::NodeHandle nh;         // Input value from RPi.
double Left_PWM, Right_PWM; // PWM signals for Motor.
double Left_setpoint, Right_setpoint;
double Lcontrol_signal;
double Rcontrol_signal;



/* --- Original Github ----- */

// Pin variables for motors.
const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = true;
const bool right_fwd = false;

// Default_speed.
const int default_vel = 100;
int state_vel = default_vel;
const int max_vel = 255;

/* ----------------------- */


/* ==== PID Control ===== */


#include <PID_v1.h>
const byte Lencoder0pinA = 2; //A pin -> the interrupt pin 0
const byte Lencoder0pinB = 7; //B pin -> the digital pin 4
const byte Rencoder0pinA = 3; //A pin -> the interrupt pin 1
const byte Rencoder0pinB = 4; //B pin -> the digital pin 5

byte Lencoder0PinALast;
byte Rencoder0PinALast;

double Lduration, Rduration; //the number of the pulses
double Lduration_last, Rduration_last;
double Labs_duration, Rabs_duration;

boolean LDirection; //the rotation direction
boolean Lresult;

boolean RDirection;
boolean Rresult;

/* ==== PID Configuration ===== */
double LKp = 1;
double LKi = 5;
double LKd = 0;
double RKp = 1;
double RKi = 5;
double RKd = 0;

        //Feedback from encoder  //.Compute() resulte  // Command setpoint
PID LeftPID(&Labs_duration, &Lcontrol_signal, &Left_setpoint, LKp, LKi, LKd, DIRECT);
PID RightPID(&Rabs_duration, &Rcontrol_signal, &Right_setpoint, RKp, RKi, RKd, DIRECT);
const int T = 100; // Sampling time for PID



/* ==== Motion Control Functions ===== */
void MoveStop() {
    Left_setpoint = abs(0);
    Right_setpoint = abs(0);
    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);
    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}


void Left_Forward(int cmd){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Left_setpoint = cmd;
    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);
}

void Left_Backward(int cmd){

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Left_setpoint = cmd;
    Labs_duration = abs(Lduration);
    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
    if (Lresult)
        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, Lcontrol_signal);
}


void Right_Forward(int cmd){
 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Right_setpoint = cmd;
    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}

void Right_Backward(int cmd){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    Right_setpoint = cmd;
    Rabs_duration = abs(Rduration);
    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
    if (Rresult)
        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, Rcontrol_signal);
}






/* ============= OPEN LOOP MOTOR FUNCTIONS ============ */
void Left_Forward_Open(int cmd){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
//    Left_setpoint = cmd;
//    Labs_duration = abs(Lduration);
//    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
//    if (Lresult)
//        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, cmd);
}

void Left_Backward_Open(int cmd){

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
//    Left_setpoint = cmd;
//    Labs_duration = abs(Lduration);
//    Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
//    if (Lresult)
//        Lduration = 0; //Count clear, wait for the next count
    analogWrite(enA, cmd);
}


void Right_Forward_Open(int cmd){
 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
//    Right_setpoint = cmd;
//    Rabs_duration = abs(Rduration);
//    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
//    if (Rresult)
//        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, cmd);
}

void Right_Backward_Open(int cmd){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
//    Right_setpoint = cmd;
//    Rabs_duration = abs(Rduration);
//    Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
//    if (Rresult)
//        Rduration = 0; //Count clear, wait for the next count
    analogWrite(enB, cmd);
}





/* ==== ROS Subscribers & Publishers ===== */
void cmd_vel_cb(const geometry_msgs::Twist & msg);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);


/* ==== Callback Function ===== */


  ////////// Differential Kinematics Model ///////////
  // ccw: angular.z
  // Separation: 13cm = 0.13 m
  // encoder_cpr = 1920 
  // circumference = PI() * diameter(6cm =  0.06m) = 0.3768
  // m_s_to_cps = encoder_cpr / circumference;

  const float Separation = 0.13;
  const float circumference = 0.3768;
  const int encoder_cpr =  1920; // 1920 const per motor revolution.
  float m_s_to_cps = encoder_cpr/circumference;
  float left_wheel_value, right_wheel_value; // r.p.s
  float angular_to_linear; 
  int  left_rpm, right_rpm;     
  int left_cmd, right_cmd;


/*
 /cmd_vel ranges from : to 
    linear.x : -0.16 ~ +0.16
    angular.z : -1.0 ~ +1.0
 */

void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
//  const float x = msg.linear.x;
//  const float z_rotation = msg.angular.z;

  float forward = msg.linear.x;
  float ccw = msg.angular.z; 

  // angular.z : 1.08 ~ -1.08
  /* ------------- Differential Drive ------------------ */
//  int right_cmd = x*default_vel * min(1, max(z_rotation*1.5 + 1, -1)) ;   // z rotation.
//  int left_cmd =  x*default_vel * min(1, max(-z_rotation*1.5 + 1 , -1));  // z rotation.
//  left_cmd = -left_cmd;

  /* ------------- Ours -------------------- */
  angular_to_linear = ccw * (Separation/2.0) ;        
  left_wheel_value  = int((forward - angular_to_linear) * m_s_to_cps);
  right_wheel_value = int((forward + angular_to_linear) * m_s_to_cps);

  left_rpm = 60 * left_wheel_value/encoder_cpr ;
  right_rpm = 60 * right_wheel_value/encoder_cpr;
  left_rpm = -left_rpm;

  // Determine directions of left/ right wheels.
  bool right_dir = (right_rpm>0)? right_fwd : !right_fwd;
  bool left_dir = (left_rpm>0)? left_fwd : !left_fwd;

  //   map cmd from rpm(0-60) to (30 - 255)
  left_cmd = map(abs(left_rpm), 0, 60, 0, 180);
  right_cmd = map(abs(right_rpm), 0, 60, 0, 180);


//  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
//  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;


  #ifndef OPEN_LOOP
  // Send cmd to motors.
  if(right_dir && left_dir){
    Left_Forward(left_cmd);
    Right_Forward(right_cmd);
  }
  else if(right_dir && !left_dir){
    Left_Backward(left_cmd);
    Right_Forward(right_cmd); 
  }
  else if(!right_dir && left_dir){
    Left_Forward(left_cmd);
    Right_Backward(right_cmd);
  }
  else{
    Left_Backward(left_cmd);
    Right_Backward(right_cmd);
  }
  #endif


  #ifdef OPEN_LOOP
    // Send cmd to motors.
    if(right_dir && left_dir){
      Left_Forward_Open(left_cmd);
      Right_Forward_Open(right_cmd);
    }
    else if(right_dir && !left_dir){
      Left_Backward_Open(left_cmd);
      Right_Forward_Open(right_cmd); 
    }
    else if(!right_dir && left_dir){
      Left_Forward_Open(left_cmd);
      Right_Backward_Open(right_cmd);
    }
    else{
      Left_Backward_Open(left_cmd);
      Right_Backward_Open(right_cmd);
    }
  #endif


/*

    
    digitalWrite(right_dir_pin, right_dir);
    digitalWrite(left_dir_pin, left_dir);
    analogWrite(right_pwm_pin, abs(right_cmd));
    analogWrite(left_pwm_pin, abs(left_cmd));
  
    digitalWrite(right_dir_pin, right_dir);
    digitalWrite(left_dir_pin, left_dir);
    analogWrite(6, abs(right_cmd));
    analogWrite(5 abs(left_cmd));

 */

  
//  digitalWrite(right_dir_pin, right_dir);
//  digitalWrite(left_dir_pin, left_dir);
//  analogWrite(right_pwm_pin, abs(right_cmd));
//  analogWrite(left_pwm_pin, abs(left_cmd));




//  digitalWrite(right_dir_pin, right_dir);
//  digitalWrite(left_dir_pin, left_dir);
//  analogWrite(6, abs(right_cmd));
//  analogWrite(5 abs(left_cmd));
  





/*
  Labs_duration = abs(Lduration);
  Lresult = LeftPID.Compute(); //PID conversion is complete and returns 1
  if (Lresult)
     Lduration = 0; //Count clear, wait for the next count
  analogWrite(enA, Lcontrol_signal);

  Rabs_duration = abs(Rduration);
  Rresult = RightPID.Compute(); //PID conversion is complete and returns 1
  if (Rresult)
      Rduration = 0; //Count clear, wait for the next count
  analogWrite(enB, Rcontrol_signal);

*/

//  if (forward == 0){
//      MoveStop();
//  }


}



/* ==== Encoders ===== */
void EncoderInit()
{
    LDirection = true; //default -> Forward
    RDirection = true;

    // Initialize
    Lduration_last = Rduration_last = 0;

    pinMode(Lencoder0pinB, INPUT);
    pinMode(Rencoder0pinB, INPUT);           // Interrupt pin 0 : = LencoderpinA = digital 2
                                             // Interrupt pin 1 : = RencoderpinA = digital 3
    attachInterrupt(0, LwheelSpeed, CHANGE); // Trigger Interrupt on Change
    attachInterrupt(1, RwheelSpeed, CHANGE);
}

void LwheelSpeed()
{
    int Lstate = digitalRead(Lencoder0pinA);
    if ((Lencoder0PinALast == LOW) && Lstate == HIGH)
    {
        int val = digitalRead(Lencoder0pinB);
        if (val == LOW && LDirection)
        {
            LDirection = false; //Reverse
        }
        else if (val == HIGH && !LDirection)
        {
            LDirection = true; //Forward
        }
    }
    Lencoder0PinALast = Lstate;

    if (!LDirection)
        Lduration++;
    else
        Lduration--;
}

void RwheelSpeed()
{
    int Rstate = digitalRead(Rencoder0pinA);
    if ((Rencoder0PinALast == LOW) && Rstate == HIGH)
    {
        int val = digitalRead(Rencoder0pinB);
        if (val == LOW && RDirection)
        {
            RDirection = false; //Reverse
        }
        else if (val == HIGH && !RDirection)
        {
            RDirection = true; //Forward
        }
    }
    Rencoder0PinALast = Rstate;

    if (!RDirection)
        Rduration++;
    else
        Rduration--;
}


/* ==== Setup() ===== */

void setup() {

  /*
  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
//  pinMode(13, OUTPUT);
  // Send forward command.
  analogWrite(right_pwm_pin, default_vel);
  analogWrite(left_pwm_pin, default_vel);
  */
  
  delay(500);
  MoveStop();


  nh.initNode();
  nh.subscribe(sub);
  nh.spinOnce();

//  nh.advertise(IR_ratio_pub);



    pinMode(enA, OUTPUT); //we have to set PWM pin as output
    pinMode(in1, OUTPUT); //Logic pins are also set as output
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); //we have to set PWM pin as output
    pinMode(in3, OUTPUT); //Logic pins are also set as output
    pinMode(in4, OUTPUT);

    // Touch Sensor (if TOUCHED -> input == HIGH )
//    pinMode(LtouchPin, INPUT);
//    pinMode(RtouchPin, INPUT);
//    pinMode(GoalTouchPin, INPUT);
//    pinMode(IRPin, INPUT);

    //Initialize left/right wheel pwm values.
//    Left_PWM = 0;
//    Right_PWM = 0;
    Left_setpoint = 0;
    Right_setpoint = 0;

    LeftPID.SetMode(AUTOMATIC);  //PID is set to automatic mode
    LeftPID.SetSampleTime(T);    //Set PID sampling frequency is 100ms
    RightPID.SetMode(AUTOMATIC); //PID is set to automatic mode
    RightPID.SetSampleTime(T);   //Set PID sampling frequency is 100ms


    EncoderInit();


}

/* ==== Main Loop() ===== */
void loop() {
  nh.spinOnce();
}

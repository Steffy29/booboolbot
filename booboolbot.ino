//***********************************************
//
//      SPHERICAL QUADRUPED ARDUINO ROBOT
//
//***********************************************

//************************************ LIBRAIRIES
#include "global.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Constants
#define nbPCAServo 16

//Parameters
int MIN_IMP [nbPCAServo] ={500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500};
int MAX_IMP [nbPCAServo] ={2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
int MIN_ANG [nbPCAServo] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int MAX_ANG [nbPCAServo] ={180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180};

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//***************************************** SETUP
void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.print("Initialize System");
  pca.begin();
  pca.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // US
  pinMode (Ultrasonic_Sensor_Trigger_Pin, OUTPUT);
  digitalWrite(Ultrasonic_Sensor_Trigger_Pin, LOW);
  pinMode (Ultrasonic_Sensor_Echo_Pin, INPUT);

  // SERVO
  pwm.begin();
  pwm.setPWMFreq(60);
  Servo_Start_Position();
}

//****************************************** LOOP
void loop() {
  // test servo
  pcaScenario();
 
  switch (Sequence) {
    case Stand_By:
      if (Ultrasonic_Sensor() < Distance_To_Activate_Switch) Sequence = Legs_Deployment;
      break;

    case Legs_Deployment:
      Door(Opening);
      Legs_Partially_Unfolded(); // DEPLOY THE LEGS UP TO TOUCH THE GROUND
      Legs_Unfolded(); // DEPLOY THE LEGS TO LIFT THE ROBOT

    case Moving_Forward:
      while (Ultrasonic_Sensor() > Distance_To_Activate_Switch) { // CONTINUE TO WALK UNTIL SWITCH ACTIVATION
        if (Ultrasonic_Sensor() > Distance_Obstacle) walk();
        else if (Head_Rotation() == Left) Rotation(Left);
        else Rotation(Right);
      }

    case Folding_Legs:
      Legs_Partially_Folded(); // FOLD THE LEGS UP TO PUT DOWN THE ROBOT
      Legs_Folded(); // FOLD THE LEGS UP TO THE RETRACTED POSITION
      Door(Closing);

    case Restart:
      Sequence = Stand_By;
  }
}

void pcaScenario(){/* function pcaScenario */ 
  ////Scenario to test servomotors controlled by PCA9685 I2C Module
  for (int i=0; i<nbPCAServo; i++) {
    Serial.print("Servo");
    Serial.println(i);
    //int middleVal=((MAX_IMP[i]+MIN_IMP[i])/2)/20000*4096; // conversion µs to pwmval
    //pca.setPWM(i,0,middleVal);
    for(int pos=(MAX_IMP[i]+MIN_IMP[i])/2;pos<MAX_IMP[i];pos+=10){
      pca.writeMicroseconds(i,pos);delay(10);
    }
    
    for(int pos=MAX_IMP[i];pos>MIN_IMP[i];pos-=10){
      pca.writeMicroseconds(i,pos);delay(10);
    }
    
    for(int pos=MIN_IMP[i];pos<(MAX_IMP[i]+MIN_IMP[i])/2;pos+=10){
      pca.writeMicroseconds(i,pos);delay(10);
    }
    pca.setPin(i,0,true); // deactivate pin i
  }
}
  
int jointToImp(double x,int i){/* function jointToImp */ 
  ////Convert joint angle into pwm command value
       
  int imp=(x - MIN_ANG[i]) * (MAX_IMP[i]-MIN_IMP[i]) / (MAX_ANG[i]-MIN_ANG[i]) + MIN_IMP[i];
  imp=max(imp,MIN_IMP[i]);
  imp=min(imp,MAX_IMP[i]);
      
  return imp;
}

//******************************** START POSITION
void Servo_Start_Position() {
  Serial.println("Servo start position");
  Legs_Folded();
  pwm.setPWM(Head_Servo_Pin, 0, Head_Servo_Start_Position);
  Hip_Centered();
  pwm.setPWM(Door_Servo_Pin, 0, Door_Servo_Close);
  delay(1000);
}

//****************************************** HEAD
int Head_Rotation() {

  int Head_Angle = Head_Servo_Start_Position;
  long Distance_Left, Distance_Right;

  for (byte rotation_step = 0; rotation_step < 4; rotation_step++) {

    // MEASURMENT
    if (rotation_step == 1 ) {
      Distance_Left = Ultrasonic_Sensor();
      delay(1000);
    }
    if (rotation_step == 3) {
      Distance_Right = Ultrasonic_Sensor();
      delay(1000);
    }

    // HEAD ROTATION
    for (byte pitch = 0; pitch < 20; pitch++) {
      pwm.setPWM(Head_Servo_Pin, 0, Head_Angle);
      if (rotation_step == 0 || rotation_step == 3 ) Head_Angle += 2;
      else Head_Angle -= 2;
      delay(Head_Servo_Speed);
    }
  }

  // SELECT THE FREE SIDE
  if (Distance_Left > Distance_Right) return Left;
  else return Right;
}

//****************************************** KNEE
// FOLDED
void Legs_Folded() {
  Serial.println("legs folded");
  Knee_Servo_Front_Right (Knee_Servo_Front_Right_Bent);
  Knee_Servo_Front_Left (Knee_Servo_Front_Left_Bent);
  Knee_Servo_Rear_Right (Knee_Servo_Rear_Right_Bent);
  Knee_Servo_Rear_Left (Knee_Servo_Rear_Left_Bent);
  delay(2000);
}

// PARTIALLY UNFOLDED
void Legs_Partially_Unfolded() {
  Serial.println("legs partially unfolded");
  Knee_Servo_Front_Right (Knee_Servo_Front_Right_Partially_Unfolded);
  Knee_Servo_Front_Left (Knee_Servo_Front_Left_Partially_Unfolded);
  Knee_Servo_Rear_Right (Knee_Servo_Rear_Right_Partially_Unfolded);
  Knee_Servo_Rear_Left (Knee_Servo_Rear_Left_Partially_Unfolded);
  delay(2000);
}

// KNEE UNFOLDED LOW SPEED
void Legs_Unfolded() {
  Serial.println("legs unfolded");
  for (byte a = 0; a < 20; a++) { // LOOP TO REDUCE THE SERVOMOTOR SPEED
    Knee_Servo_Front_Right(map(a, 0, 20, Knee_Servo_Front_Right_Partially_Unfolded, Knee_Servo_Front_Right_Unfolded));
    Knee_Servo_Front_Left(map(a, 0, 20, Knee_Servo_Front_Left_Partially_Unfolded, Knee_Servo_Front_Left_Unfolded));
    Knee_Servo_Rear_Right(map(a, 0, 20, Knee_Servo_Rear_Right_Partially_Unfolded, Knee_Servo_Rear_Right_Unfolded));
    Knee_Servo_Rear_Left(map(a, 0, 20, Knee_Servo_Rear_Left_Partially_Unfolded, Knee_Servo_Rear_Left_Unfolded));
    delay(Knee_Servo_Speed);
  }
  delay(2000);
}

// KNEE BENT LOW SPEED
void Legs_Partially_Folded() {
  Serial.println("legs partially folded");
  for (byte a = 0; a < 20; a++) { // LOOP TO REDUCE THE SERVOMOTOR SPEED
    Knee_Servo_Front_Right(map(a, 0, 20, Knee_Servo_Front_Right_Unfolded, Knee_Servo_Front_Right_Partially_Unfolded));
    Knee_Servo_Front_Left(map(a, 0, 20, Knee_Servo_Front_Left_Unfolded, Knee_Servo_Front_Left_Partially_Unfolded));
    Knee_Servo_Rear_Right(map(a, 0, 20, Knee_Servo_Rear_Right_Unfolded, Knee_Servo_Rear_Right_Partially_Unfolded));
    Knee_Servo_Rear_Left(map(a, 0, 20, Knee_Servo_Rear_Left_Unfolded, Knee_Servo_Rear_Left_Partially_Unfolded));
    delay(Knee_Servo_Speed);
  }
  delay(2000);
}

// COMMAND
void Knee_Servo_Front_Right (int pulse) {
  pwm.setPWM(Knee_Servo_Front_Right_Pin, 0, pulse);
}

void Knee_Servo_Front_Left (int pulse) {
  pwm.setPWM(Knee_Servo_Front_Left_Pin, 0, pulse);
}

void Knee_Servo_Rear_Right (int pulse) {
  pwm.setPWM(Knee_Servo_Rear_Right_Pin, 0, pulse);
}

void Knee_Servo_Rear_Left (int pulse) {
  pwm.setPWM(Knee_Servo_Rear_Left_Pin, 0, pulse);
}

//******************************************* HIP
// CENTERED
void Hip_Centered() {
  Serial.println("hip centered");
  Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered);
  Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered);
  Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered);
  Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered);
}

// COMMAND
void Hip_Servo_Front_Right (int pulse) {
  pwm.setPWM(Hip_Servo_Front_Right_Pin, 0, pulse);
}

void Hip_Servo_Front_Left (int pulse) {
  pwm.setPWM(Hip_Servo_Front_Left_Pin, 0, pulse);
}

void Hip_Servo_Rear_Right (int pulse) {
  pwm.setPWM(Hip_Servo_Rear_Right_Pin, 0, pulse);
}

void Hip_Servo_Rear_Left (int pulse) {
  pwm.setPWM(Hip_Servo_Rear_Left_Pin, 0, pulse);
}

//****************************************** WALK
void walk() {
  Serial.println("walk");
  // SEE INSTRUCTABLES DETAIL FOR THE WALKING STRATEGY
  Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded - Knee_Vertical_Amplitude);
  Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded - Knee_Vertical_Amplitude);
  delay(delay_knee);

  Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered - Hip_Servo_Front_Right_Move);
  Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered + Hip_Servo_Rear_Left_Move);
  delay(delay_hip);

  Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded + Knee_Vertical_Amplitude);
  Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded + Knee_Vertical_Amplitude);
  delay(delay_knee);

  Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered);
  Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered);
  delay(delay_hip);

  Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded);
  Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded);
  delay(delay_knee);

  Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded - Knee_Vertical_Amplitude);
  Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded - Knee_Vertical_Amplitude);
  delay(delay_knee);

  Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered + Hip_Servo_Front_Left_Move);
  Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered - Hip_Servo_Rear_Right_Move);
  delay(delay_hip);

  Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded + Knee_Vertical_Amplitude);
  Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded + Knee_Vertical_Amplitude);
  delay(delay_knee);

  Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered);
  Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered);
  delay(delay_hip);

  Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded);
  Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded);
  delay(delay_knee);
}

//************************************** ROTATION
void Rotation (byte Direction) {
  Serial.println("rotation");
  // SEE INSTRUCTABLES DETAIL FOR THE ROTATION STRATEGY
  for (byte a = 0; a < 4; a++) {
    Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded - Knee_Vertical_Amplitude);
    Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded - Knee_Vertical_Amplitude);
    delay(delay_knee);

    if (Direction == Right) {
      Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered - Hip_Servo_Front_Right_Move);
      Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered - Hip_Servo_Rear_Left_Move);
    }
    else {
      Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered + Hip_Servo_Front_Right_Move);
      Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered + Hip_Servo_Rear_Left_Move);
    }
    delay(delay_hip);

    Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded + Knee_Vertical_Amplitude);
    Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded + Knee_Vertical_Amplitude);
    delay(delay_knee);

    Hip_Servo_Front_Right (Hip_Servo_Front_Right_Centered);
    Hip_Servo_Rear_Left (Hip_Servo_Rear_Left_Centered);
    delay(delay_hip);

    Knee_Servo_Front_Right(Knee_Servo_Front_Right_Unfolded);
    Knee_Servo_Rear_Left(Knee_Servo_Rear_Left_Unfolded);
    delay(delay_knee);

    Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded - Knee_Vertical_Amplitude);
    Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded - Knee_Vertical_Amplitude);
    delay(delay_knee);

    if (Direction == Right) {
      Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered - Hip_Servo_Front_Left_Move);
      Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered - Hip_Servo_Rear_Right_Move);
    }
    else {
      Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered + Hip_Servo_Front_Left_Move);
      Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered + Hip_Servo_Rear_Right_Move);
    }
    delay(delay_hip);

    Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded + Knee_Vertical_Amplitude);
    Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded + Knee_Vertical_Amplitude);
    delay(delay_knee);

    Hip_Servo_Front_Left (Hip_Servo_Front_Left_Centered);
    Hip_Servo_Rear_Right (Hip_Servo_Rear_Right_Centered);
    delay(delay_hip);

    Knee_Servo_Front_Left(Knee_Servo_Front_Left_Unfolded);
    Knee_Servo_Rear_Right(Knee_Servo_Rear_Right_Unfolded);
    delay(delay_knee);
  }
}

//***************************** ULTRASONIC SENSOR
long Ultrasonic_Sensor() {
  Serial.println("read ultrasonic sensor");
  digitalWrite(Ultrasonic_Sensor_Trigger_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultrasonic_Sensor_Trigger_Pin, LOW);
  Measure = pulseIn(Ultrasonic_Sensor_Echo_Pin, HIGH);
  Distance_mm = Measure / 58;
  delay(100);
  return Distance_mm;
}

//****************************************** DOOR
void Door (int State) {
  Serial.println("door");
  if (State == Closing) {
    for (byte a = 0; a < 20; a++) { // LOOP TO REDUCE THE SERVOMOTOR SPEED
      pwm.setPWM(Door_Servo_Pin, 0, map(a, 0, 20, Door_Servo_Open, Door_Servo_Close));
      delay(Door_Servo_Speed);
    }
  }
  else {
    for (byte a = 0; a < 20; a++) { // LOOP TO REDUCE THE SERVOMOTOR SPEED
      pwm.setPWM(Door_Servo_Pin, 0, map(a, 0, 20, Door_Servo_Close, Door_Servo_Open));
      delay(Door_Servo_Speed);
    }
  }
  delay(1000);
}

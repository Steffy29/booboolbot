#include "Arduino.h"

// ROBOT
enum {Stand_By, Legs_Deployment, Moving_Forward, Folding_Legs, Restart};
byte Sequence = Stand_By;
enum {Rotation_Left, Rotation_Right, Opening, Closing, Right, Left};
const byte Distance_To_Activate_Switch (4);
const byte Distance_Obstacle (20);

// ULTRASONIC SENSOR
const byte Ultrasonic_Sensor_Trigger_Pin (6);
const byte Ultrasonic_Sensor_Echo_Pin (8);
long Measure = 0;
long Distance_mm = 0;

// DOOR
const byte Door_Servo_Pin (1);
const int Door_Servo_Open (430);
const int Door_Servo_Close (205);
const int Door_Servo_Speed (150);

// HIP
const byte Hip_Servo_Front_Right_Pin (2);
const int Hip_Servo_Front_Right_Centered (560);
const int Hip_Servo_Front_Right_Move (90);

const byte Hip_Servo_Front_Left_Pin (4);
const int Hip_Servo_Front_Left_Centered (321);
const int Hip_Servo_Front_Left_Move (66);

const byte Hip_Servo_Rear_Right_Pin (9);
const int Hip_Servo_Rear_Right_Centered (347);
const int Hip_Servo_Rear_Right_Move (62);

const byte Hip_Servo_Rear_Left_Pin (13);
const int Hip_Servo_Rear_Left_Centered (268);
const int Hip_Servo_Rear_Left_Move (64);

const int delay_hip (400);

// KNEE
const byte Knee_Servo_Front_Right_Pin (3);
const int Knee_Servo_Front_Right_Bent (220);
const int Knee_Servo_Front_Right_Partially_Unfolded (343);
const int Knee_Servo_Front_Right_Unfolded (429);

const byte Knee_Servo_Front_Left_Pin (5);
const int Knee_Servo_Front_Left_Bent (216);
const int Knee_Servo_Front_Left_Partially_Unfolded (335);
const int Knee_Servo_Front_Left_Unfolded (423);

const byte Knee_Servo_Rear_Right_Pin (10);
const int Knee_Servo_Rear_Right_Bent (211);
const int Knee_Servo_Rear_Right_Partially_Unfolded (340);
const int Knee_Servo_Rear_Right_Unfolded (408);

const byte Knee_Servo_Rear_Left_Pin (14);
const int Knee_Servo_Rear_Left_Bent (200);
const int Knee_Servo_Rear_Left_Partially_Unfolded (349);
const int Knee_Servo_Rear_Left_Unfolded (436);

const byte Knee_Servo_Speed (130);
const byte delay_knee (150);
const byte Knee_Vertical_Amplitude (8);

// HEAD
const byte Head_Servo_Pin (0);
const int Head_Servo_Start_Position (339);
const int Head_Servo_Speed (100);

// Functions
void testLeg();
void pcaScenario();
int jointToImp(double x,int i);
void Servo_Start_Position();
int Head_Rotation();
void Legs_Folded();
void Legs_Partially_Unfolded();
void Legs_Unfolded();
void Legs_Partially_Folded();
void Knee_Servo_Front_Right (int pulse);
void Knee_Servo_Front_Left (int pulse);
void Knee_Servo_Rear_Right (int pulse);
void Knee_Servo_Rear_Left (int pulse);
void Hip_Centered();
void Hip_Servo_Front_Right (int pulse);
void Hip_Servo_Front_Left (int pulse);
void Hip_Servo_Rear_Right (int pulse);
void Hip_Servo_Rear_Left (int pulse);
void walk();
void Rotation (byte Direction);
long Ultrasonic_Sensor();
void Door (int State);

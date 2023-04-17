#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below

int convertPower(double power_input);

// void DriveStraight(double voltage);
// void driveStraight(double Kp, double Ki, double power);

void drivePcont (double target, double Kp, double Ki);

int encTodistance (int L_encoder,int R_incoder);
void drive_to_can(int stopping_distance);

//void turnAngle (double Kp,double Ki,double degree);
   void TurnAngle(double Kp, double Kpdiff, double wantedAngle);

void rotateAngle(double Kp, double targetAngle);
void driveUntilBlack(double precent_power);
 void moveArmAngle(double Angle, double kp);
void turnRobot(double angle, double voltage, double Kp, double Ki) ;

void driveStriaght( double voltage);
void TurnRight (double voltage);
void TurnLeft(double voltage);
void stop();
void linefollowing();
// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H
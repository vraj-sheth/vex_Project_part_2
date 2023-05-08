#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below

int convertPower(double power_input);



void drivePIcont (double target, double Kp, double Ki);
double encTodistance (int enc_count);
void drive_to_can(int stopping_distance);
void rotateAngle(double Kp, double targetAngle);
void driveUntilBlack(double precent_power);
void driveStriaght( double voltage);
void TurnRight (double voltage);
void TurnLeft(double voltage);
void stop();
void linefollowing();
double arm_enc_2_angle(int encoderCounts);
void armangle(double degrees, double Kp, int tolerance) ;
// most of the turn angle functions dont work, the newest one is the turRobot function
// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H
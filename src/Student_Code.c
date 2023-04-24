/**
 * @file Student_Code.c
 * @author your name (you@domain.com)
 * @brief description of this file
 * @version 0.1
 * @date yyyy-mm-dd
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Student_Code.h"

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 103;	    // diameter of the driving wheels [mm]
int robotWidth = 250;					// width of the robot including the wheel thickness [mm]
int wheelWidth = 22;					// width of the driving wheel [mm]
double drivingWheelRatio = 0.0;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 0.0;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 900.0;
double pivotWheelRatio = 114.0;    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main(){
{
    //  armUp(5000);
    // resetEncoder(ArmEncoder);
// drivePcont ( 300.0,2.6,0.08);
// delay(3000);
// drivePcont ( -300.0,1.3,0.1);


//driveUntilBlack(40.0);

//rotateAngle(0.8,-90.0);
// //armUp(4000);
// lcd_print(1,"hfdbjfdbfj");
// moveArmAngle(0.0, 2.0);
// delay(3000);
// lcd_print(4,"hfdbjfdbfj");
// moveArmAngle(30.0,2.0);
//turnRobot(90.0,2,0.8);
//turnPcont(90.0,2000.0);

// linefollowing();
drive_to_can(300);

// rotateAngle(20.0, 90.0);
// delay(3000);
// rotateAngle(20.0, -180);



   
}

}
// Task 1 //
int convertPower(double power_input){
  int voltage, saturated_power;
// Limit power percentage to be between -100% and 100%
  saturated_power = saturate(power_input,-100,100); 
  //Converts power percentage to voltage by using the saturate function
  voltage = saturated_power * 50; 
  
  // return the voltage value.
  return voltage;



}

//drive given distance function//
void drivePcont (double target,double Kp,double Ki){
    double currentPos,u=0;
    int Pwr=0;
    int i=0;
    int error=0;


    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    while(currentPos<=abs(target)){
    int L_encoder=readSensor(LeftEncoder);
    int R_encoder=readSensor(RightEncoder);

    double avg_encoder_count= abs(L_encoder+R_encoder)/2;
    
    currentPos=0.3593*avg_encoder_count;    

    error=target-currentPos;

    i=i+error;
    u=(Kp*error + Ki*i);
    double differance=(L_encoder-R_encoder)*7; 
    Pwr=(int) saturate(u,-3000,3000);

    motorPower(LeftMotor, Pwr - differance);
    motorPower(RightMotor, Pwr + differance);
   // motorPower(LeftMotor,Pwr);
   // motorPower(RightMotor,Pwr);
    delay(50);
    }

    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);
}

/*void drivePcont(float target, float Kp){

float currentPos, error, u = 0;
int Pwr = 0;

resetEncoder(LeftEncoder);
resetEncoder(RightEncoder);

do{
    
    int L_encoder=readSensor(LeftEncoder);
    int R_encoder=readSensor(RightEncoder);

    double avg_encoder_count= (L_encoder+R_encoder)/2;
    
    currentPos=0.3593*avg_encoder_count;




}



}*/



/*int encTodistance (int L_encoder,int R_incoder){

    double avg_count= (L_encoder+R_incoder)/2;

    double distance_traveled= (avg_count/900)*pi*0.103;

    int distance=(int)distance_traveled;

    return distance;
}*/


// converting the enc to distance // 
int encTodistance (int L_encoder,int R_incoder){

    double avg_count= (L_encoder+R_incoder)/2;

    double distance_traveled= (avg_count/encCountPerRev)*3.14169265358979323846*(drivingWheelDiameter*10^-3);

    int distance=(int)distance_traveled;

    return distance;
}


/// drive to can function //
void drive_to_can(int stopping_distance){
    
    double d2can = readSensor(SonarSensor);

    double end_distance = (d2can+20)-stopping_distance;

    drivePcont(end_distance,1.0,1.0);
    
}    



void rotateAngle(double Kp, double targetAngle){
    int leftencCount, rightencCount;
    double i=0.0;
    double left_dis, right_dis;
    double error;
    double pwr;
    double average_distance_traveled;
    double pivotDiameter=robotWidth-22.0;
    double pivotCircle= pivotDiameter*3.14169265358979323846;
    double distance_to_travel=(targetAngle)/360.0*pivotCircle;
    double v1,v2;
    double diff;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    do{
        leftencCount=readSensor(LeftEncoder);
        rightencCount=readSensor(RightEncoder);
        left_dis=leftencCount*0.3593;
        right_dis=rightencCount*0.3593;

        //Averages the left and right encoder count
        average_distance_traveled=(fabs(left_dis)+fabs(right_dis))/2.0; 
        
        // Calculates the difference between the left and right wheel encoders
        diff=fabs(left_dis)-fabs(right_dis); 

        if(distance_to_travel <= 0){

            error = (-distance_to_travel) - fabs(average_distance_traveled);
            pwr=Kp*-error;
        }
        else{ 

            error= distance_to_travel - fabs(average_distance_traveled);
            pwr=Kp*error;

        }


        //Need to change voltage to power percentage
        v1=convertPower(pwr);
        v2=convertPower(diff);
       
        motorPower(LeftMotor,v1*-1); //Anticlockwise is positive 
        motorPower(RightMotor,(v1+v2));
        delay(50);
        if (fabs(average_distance_traveled)>=distance_to_travel){
             break;
         }
        
    }while(1);

    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);

}


void driveUntilBlack(double precent_power){
    int s1,s2,s3;
    int upper_lim, lower_lim;

    int Pwr=convertPower(precent_power);
    
    upper_lim=2400;
    lower_lim=2000;
    
    // puts the code into an infinate while loop as i always = 1.
    while (1){

        s1=readSensor(LeftLight);
        s2=readSensor(MidLight);
        s3=readSensor(RightLight);
        // if any of the three sensors find a reading that falls in the "black range" then the if statement will stop
        // sending power to the motors and break out of the while loop.
        if((s1<=upper_lim && s1>= lower_lim) || (s2<=upper_lim && s2>= lower_lim) || (s3<=upper_lim && s3>= lower_lim)){
            motorPower(LeftMotor,0);
            motorPower(RightMotor,0);
            break;
        }

        // if the reading from the sensors if outside the "black range" than the code will continue to send power to the motors.
        else {  
            int L_encoder=readSensor(LeftEncoder);
            int R_encoder=readSensor(RightEncoder);  
            double differance=(L_encoder-R_encoder)*7;
   

            motorPower(LeftMotor, Pwr - differance);
            motorPower(RightMotor, Pwr + differance);
        // this delay will makes it so that the that code takes samples in Nms intivals,
        // after which it should restart the while loop placing new reading into the sensor variables,
        // thus starting the whole process again.
        

    }
}
}
 void moveArmAngle(double Angle, double kp){
   // double upperEncLim;
   // double lowerEncLim;
    double u=0.0;
    double error=0.0;
    double currentAngle=0.0;
    double Pwr=0.0;

    // armUp(5000);
    // resetEncoder(ArmEncoder);
    
    while(abs(error)<5){
        int currnetArmEnc=readSensor(ArmEncoder);
        currentAngle=((currnetArmEnc*360)/(7*900))+41; // 56 is the angle of the arm at top position
        error=Angle-currentAngle;
        u=kp*error;

        Pwr=convertPower(u);
        motorPower(ArmMotor,Pwr);
        delay(50);

    }
    motorPower(ArmMotor,0);

 }




void turnRobot(double angle,  double Kp, double Ki) {

  double angleInRadians = fabs(angle) * 3.14159265359 / 180.0;

  double Arcdistance = angleInRadians * 125.0;

  double error = angle;
  double i = 0;             //consider making the power = to the kp* the error, and define the error earlier on.


  motorPower(LeftMotor, (Kp*error));
  motorPower(RightMotor, -(Kp*error));

  while (fabs(error) < 0.1) { // the 0.1 is the tolerance
    int leftEncoderValue = readSensor(LeftEncoder);
    int rightEncoderValue = readSensor(RightEncoder);

    double leftDistance = leftEncoderValue * 0.103 * PI; //the 0.103 is the wheel diameter in m
    double rightDistance = rightEncoderValue * 0.103 * PI;

    error = (leftDistance - rightDistance) - Arcdistance;

    i =i + error;

    double u = Kp*error+Ki*i;

    motorPower(LeftMotor,u);
    motorPower(RightMotor,-u);
  }

  motorPower(LeftMotor, 0);
  motorPower(RightMotor, 0);

}


void turnPcont(double targetAngle, double Kp){
  // Intialise variables
  double error, U = 0;
  double arc_length;
  int LeftWheelPower, RightWheelPower, L_Encoder_Count, R_Encoder_Count;
  double required_encoder_count;

  // Zero encoders
  resetEncoder(LeftEncoder);
  resetEncoder(RightEncoder);
    double angleInRadians = fabs(targetAngle) * 3.14159265359 / 180.0;

  //Arc length calculation
  //Formula: Arc length = theta * (pi/180) * r - where theta is in degrees

  arc_length = angleInRadians * 125.0; //pivotWheelRatio * (3.14159265359/180) * targetAngle;   //This converts the desired angle of rotation into mm
  
  required_encoder_count = (arc_length / 0.3593) ; // Converts the desired arc length into encoder counts

  lcd_print(2,"%d", arc_length);

  while(1){
    L_Encoder_Count = readSensor(LeftEncoder);
    R_Encoder_Count = readSensor(RightEncoder);

    int Encoder_Average = (abs(R_Encoder_Count) + abs(L_Encoder_Count))/2; //The average encoder count of the two encoders
    
    lcd_print(3,"%d", L_Encoder_Count);

    // If the encoder average is less than the required encoder count send power to the motors
    if(Encoder_Average <= required_encoder_count){
      
      //P loop control

    error = R_Encoder_Count - L_Encoder_Count; 
     U = (Kp * error);

      //Clockwise turn requires left wheel to move forward -> positive power | Right wheel to reverse -> negative power
      LeftWheelPower += (U);
      RightWheelPower -= (-U);
      motorPower(RightMotor, convertPower(RightWheelPower));
      motorPower(LeftMotor, convertPower(LeftWheelPower));

    }
    else{
      motorPower(RightMotor,0);
      motorPower(LeftMotor,0);
    }

    delay(50); //The required 50ms delay

  }
}


///////////////////

// defining functions for line following
void driveStriaght( double voltage){
    motorPower(RightMotor,3500);
    motorPower(LeftMotor,3500);


}


void TurnRight (double voltage){
    motorPower(RightMotor,-voltage);
    motorPower(LeftMotor,voltage);
}


void TurnLeft(double voltage){
    motorPower(RightMotor,voltage);
    motorPower(LeftMotor,-voltage);
}


void stop(){
    motorPower(RightMotor,0);
    motorPower(LeftMotor,0);
}


void linefollowing(){

    double PWR=2000.0;
    double brownUpperIm= 2200.0;
    double brownLowerLim= 2000.0;


    double blackUpperLim= 2700.0;
    double blackLowerLim=2450.0;



    while(1){
        double s1=readSensor(LeftLight);
        double s2=readSensor(MidLight);
        double s3=readSensor(RightLight);


        int leftSeesBrown= false;
        int middleSeesBrown= false;
        int rightSeesBrown= false;


        int leftSeesBlack= false;
        int middleSeesBlack= false;
        int rightSeesBlack= false;


// Brown TF readings
        if (s1>=brownLowerLim && s1<=brownUpperIm){
            leftSeesBrown=true;
        }


        if (s2>=brownLowerLim && s2<=brownUpperIm){
            middleSeesBrown=true;
        }
        
        if (s3>=brownLowerLim && s3<=brownUpperIm){
            rightSeesBrown=true;
        }
 // blacks TF readings
        if (s1>=blackLowerLim && s1<=blackUpperLim){
            leftSeesBlack=true;
        }


        if (s2>=blackLowerLim && s2<=blackUpperLim){
            middleSeesBlack=true;
        }
        
        if (s3>=blackLowerLim && s3<=blackUpperLim){
            rightSeesBlack=true;
        }


        //sensor conditions
        if (leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown== false){
                motorPower(RightMotor,PWR);                                                // middle sees brown
                motorPower(LeftMotor,PWR);
        }


        if(leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown==true){
            TurnRight(PWR);
                motorPower(RightMotor,PWR);                                                //90 to the right
                motorPower(LeftMotor,PWR);
        }


        if(leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown==false){
            TurnLeft(PWR);
            //delay(50);                                                                      //90 to the left
                motorPower(RightMotor,PWR);
                motorPower(LeftMotor,PWR);
        }


        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==false){                   //not on the line
            TurnRight(PWR);
            delay(50); // hoping turning right will point the robot in the correct direction
        }


        // if (leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown== true){
        //         motorPower(RightMotor,PWR);
        //         motorPower(LeftMotor,PWR);                                                     // robot perpendicular to line
        //     //delay(50);
        //     TurnRight(PWR);
        // }


        // if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==true){
        //     TurnRight(PWR); //robot is wedged in corner, hoping turning right will fix it        // stuck in corner
        // }


        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==true){
            TurnRight(PWR);
            // delay(50);
                motorPower(RightMotor,PWR);                                                // compensateing for over turn to the left
                motorPower(LeftMotor,PWR);
        }


        if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==false){
            TurnLeft(PWR);
            // delay(50);                                                                      // compensating for over turn to the right
                motorPower(RightMotor,PWR);
                motorPower(LeftMotor,PWR);
        }


        if ( leftSeesBlack==true && middleSeesBlack== true && rightSeesBlack== true){
            stop();
            break;                                                                          // sees balck
        }


    }
    stop();

}

// things to do:
// rewrite a encoder to milimeter fucntion
// test line following
// test arm angle function
// test turning angle fucntion

// i really hope these functions work 







// ----------------------------------------------- Function defintions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

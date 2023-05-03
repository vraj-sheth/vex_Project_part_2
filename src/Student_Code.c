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
    armUp(4000);
double dis2can = readSensor(SonarSensor);
delay(50);
drive_to_can(320);
armangle(-15.5,5.0,3.0);
delay(50);
drivePcont(60.0,0.8,0.1);
delay(50);
armUp(4000);
double drive_back_dis= dis2can-320-20-673-140+100;
drivePcont(-drive_back_dis,0.8,0.1);
delay(50);
rotateAngle(7.0,90.0);
delay(50);
driveUntilBlack(40.0);
delay(1000);
drivePcont(60.0,0.8,0.1);
delay(50);
linefollowing();
delay(50);
drivePcont(254.0,0.8,0.1);// might need to edit
delay(50);
armangle(-10.5,5.0,3.0);
delay(50);
drivePcont(-150.0,0.8,0.1);
delay(50);
armUp(4000);
delay(50);
rotateAngle(7.0,-90.0);
delay(50);
drivePcont(1308.0,0.8,0.1);
delay(50);
rotateAngle(7.0,90.0);
delay(50);
drivePcont(100.0,0.8,0.1);

// /////// testing for function in lab:

// //moveArmAngle(0.0,20.0);
// // delay(50);
// // moveArmAngle(30.0,20.0);
// armangle(0.0,5.0,3.0);
// delay(50);
// // armangle(30.0,5.0,2.0);
// rotateAngle(7.0,90.0);
// delay(500);
// linefollowing();
// rotateAngle(7.0,-90.0);
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
// converting the enc to distance // 
double encTodistance (int enc_count){
    double distance;
    double Circumfrance= PI*drivingWheelDiameter;

    distance=enc_count*(Circumfrance/900);

    return distance;
}


//drive given distance function//
void drivePcont (double target,double Kp,double Ki){// 0.35kp 0.01ki
    double currentPos,u=0;
    int Pwr=0;
    int i=0;
    int error=0;


    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    do{
        int L_encoder=readSensor(LeftEncoder);
        int R_encoder=readSensor(RightEncoder);

        double avg_encoder_count= abs(L_encoder+R_encoder)/2;
        
        currentPos=encTodistance(avg_encoder_count); //0.3593*avg_encoder_count;    

        error=target-currentPos;

        //Stop integrator wind up
        if( abs(u) <= 100){
            i=i+error;
        }
 
        u=(Kp*error + Ki*i); // Divide Kp by 50 when called - allows u to be a percentage equivalent of mv


        double differance=(L_encoder-R_encoder)*10; //Why is there a 7
        u = saturate(u,-60.0,60.0);
        Pwr = convertPower(u);


        motorPower(LeftMotor, Pwr - differance);
        motorPower(RightMotor, Pwr + differance);
        // motorPower(LeftMotor,Pwr);
        // motorPower(RightMotor,Pwr);
        delay(50);
    }while((currentPos+5)<=(abs(target)));

    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);
}








/// drive to can function //
void drive_to_can(int stopping_distance){
    
    double d2can = readSensor(SonarSensor);

    double end_distance = (d2can)-stopping_distance;

    drivePcont(end_distance,0.35,0.1);
    
}    



void rotateAngle(double Kp, double targetAngle){
    int leftencCount, rightencCount;
    double i=0.0;
    double left_dis, right_dis;
    double error;
    double pwr;
    double average_distance_traveled;
    double pivotDiameter=robotWidth-22.0;
    double pivotCircle= pivotDiameter*PI;
    double distance_to_travel= (targetAngle)/360.0*pivotCircle;
    double v1,v2;
    double diff;

    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    do{
        leftencCount=readSensor(LeftEncoder);
        rightencCount=readSensor(RightEncoder);
        left_dis=encTodistance(leftencCount); //leftencCount*0.3593;
        right_dis= encTodistance(rightencCount);//rightencCount*0.3593;

        //Averages the left and right encoder count
        average_distance_traveled=(fabs(left_dis)+fabs(right_dis))/2.0; 

        if (targetAngle<0){
            
            error = distance_to_travel + (average_distance_traveled);
        }
        else if ( targetAngle>0){

        error = distance_to_travel - (average_distance_traveled);
        }
        
        // Calculates the difference between the left and right wheel encoders
        diff=((left_dis)-(right_dis))/100.0; //Implement second controller


        // if (targetAngle < 0){
        //     pwr = -error * Kp;
        // }
        // else{
        //     pwr = error * Kp;

        // }
        //Need to change voltage to power percentage
        pwr = error * Kp;

        v1=convertPower(pwr/5);
        v2=convertPower(diff/5);
        // add saturation
       
        motorPower(LeftMotor,v1); //Anticlockwise is positive 
        motorPower(RightMotor,-(v1+v2));
        
        if (fabs(error)<10.0){
            break;
        }
        lcd_print(2,"%f",error);
        delay(50);
        
    }while(1);

    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);

}
// James
// void rotateAngle(double Kp, double targetAngle){
//     int leftencCount, rightencCount;
//     double i=0.0;
//     double left_dis, right_dis;
//     double error;
//     double pwr;
//     double average_distance_traveled;
//     double pivotDiameter=robotWidth-22.0;
//     double pivotCircle= pivotDiameter*PI;
//     double distance_to_travel= fabs(targetAngle)/360.0*pivotCircle;
//     double v1,v2;
//     double diff;

//     resetEncoder(LeftEncoder);
//     resetEncoder(RightEncoder);

//     do{
//         leftencCount=readSensor(LeftEncoder);
//         rightencCount=readSensor(RightEncoder);
//         left_dis=leftencCount*0.3593;
//         right_dis=rightencCount*0.3593;

//         //Averages the left and right encoder count
//         average_distance_traveled=(fabs(left_dis)+fabs(right_dis))/2.0; //Try using just one encoder
        
//         // Calculates the difference between the left and right wheel encoders
//         diff=fabs(left_dis)-fabs(right_dis); //Implement second controller

//         error = distance_to_travel - fabs(average_distance_traveled);

//         if (targetAngle < 0){
//             pwr = -error * Kp;
//             lcd_print(2,"%f",pwr);
//         }
//         else{
//             pwr = error * Kp;
//             lcd_print(3,"%f",pwr);

//         }
//         //Need to change voltage to power percentage

//         //Lower the amount of power first - reduce twitching
//         pwr = saturate(pwr, -60, 60); 


//         v1=convertPower(pwr);
//         v2=convertPower(diff);
       
//         motorPower(LeftMotor,v1); //Clockwise  is positive - CHANGE BEFORE SUBMITION
//         motorPower(RightMotor,(v1+v2*-1));
        
//         if (average_distance_traveled >= distance_to_travel){
//             break;
//         }

//         lcd_print(2,"%f",average_distance_traveled);
//         lcd_print(3,"%f",distance_to_travel);
//         delay(50);
        
//     }while(1);

//     motorPower(LeftMotor,0);
//     motorPower(RightMotor,0);

// }

// void rotateAngle(double Kp, double targetAngle) {
//     int leftencCount, rightencCount;
//     double left_dis, right_dis;
//     double error;
//     double power, diff;
//     double average_distance_traveled;
//     double pivotDiameter = robotWidth - 22.0;
//     double pivotCircle = pivotDiameter * PI;
//     double distance_to_travel = (targetAngle / 360.0) * pivotCircle;
//     double v1, v2;

//     resetEncoder(LeftEncoder);
//     resetEncoder(RightEncoder);

//     do {
//         leftencCount = readSensor(LeftEncoder);
//         rightencCount = readSensor(RightEncoder);
//         left_dis = encTodistance(leftencCount); //leftencCount*0.3593;
//         right_dis = encTodistance(rightencCount);//rightencCount*0.3593;

//         // Averages the left and right encoder count
//         average_distance_traveled = (fabs(left_dis) + fabs(right_dis)) / 2.0;

//         error = distance_to_travel - average_distance_traveled;

//         // Calculates the difference between the left and right wheel encoders
//         diff = (left_dis - right_dis) / 100.0;

//         power = Kp * error;
//         v1 = convertPower(power);
//         v2 = convertPower(Kp * diff);


//         motorPower(LeftMotor, v1); // Anticlockwise is positive
//         motorPower(RightMotor, -(v1 + v2));

//         if (fabs(error) < 0.1) {
//             break;
//         }

//         delay(50);

//     } while (1);

//     motorPower(LeftMotor, 0);
//     motorPower(RightMotor, 0);
// }

void armangle(double targetAngle, double Kp, int tolerance)    // possible if to make sure we dont put a value outside of our range (wont exit loop)
{

double currentAngle;
double error;
double PWR;
int enc_counts;

armUp(4000);

resetEncoder(ArmEncoder);

do{

 enc_counts=readSensor(ArmEncoder);

currentAngle = (360.0/6300.0)*(enc_counts) + 41.0;
error= targetAngle-currentAngle;
double u=Kp*error;
PWR=convertPower(u);
motorPower(ArmMotor,PWR);
delay(50);
}
while((currentAngle-tolerance) > targetAngle);

motorPower(ArmMotor,0);

}

double arm_enc_2_angle(int encoderCounts){

    double currentAngle=((encoderCounts*360.0)/(7.0*900.0))+ 41.0;
    return currentAngle;
}

void driveUntilBlack(double precent_power){
    int s1,s2,s3;
    int upper_lim, lower_lim;
    resetEncoder(RightEncoder);
    resetEncoder(LeftEncoder);

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

    armUp(5000);
    resetEncoder(ArmEncoder);
    

    // Test with a different error threshold
    while((currentAngle-5.0)>Angle){
        int currnetArmEnc=readSensor(ArmEncoder);

        //Why are we using 41??
        currentAngle=((currnetArmEnc*360.0)/(7.0*900.0))+ 41.0; 
        error=Angle-currentAngle;
        u=kp*error;

        Pwr=convertPower(u);
        motorPower(ArmMotor,Pwr);
        delay(50);

    }
    motorPower(ArmMotor,0);

 }

// void moveArmAngle(double Angle, double kp){
//     double u=0.0;
//     double error=0.0;
//     double currentAngle=0.0;
//     double Pwr=0.0;
//     double lastAngle = 0.0;
//     resetTimer(T_1); // Reset timer T_1

//     while(abs(error) <= 2 && readTimer(T_1) < 2000){ // Exit loop if angle doesn't change for 2 seconds
//         int currnetArmEnc = readSensor(ArmEncoder);

//         currentAngle = ((currnetArmEnc*360.0)/(7.0*900.0))+ 41.0;
//         error = Angle - currentAngle;
//         u = kp * error;

//         Pwr = convertPower(u);
//         motorPower(ArmMotor,Pwr);
//         delay(50);

//         // Check if angle has changed and reset timer
//         if (abs(currentAngle - lastAngle) > 1.0) {
//             lastAngle = currentAngle;
//             resetTimer(T_1);

//         }
//     }
//     motorPower(ArmMotor,0);
// }



void turnPcont(double targetAngle, double Kp){
  // Intialise variables
  double error, U = 0;
  double arc_length;
  int LeftWheelPower, RightWheelPower, L_Encoder_Count, R_Encoder_Count;
  double required_encoder_count;

  // Zero encoders
  resetEncoder(LeftEncoder);
  resetEncoder(RightEncoder);
    double angleInRadians = fabs(targetAngle) * (PI / 180.0);

  //Arc length calculation
  //Formula: Arc length = theta * (pi/180) * r - where theta is in degrees

  arc_length = (double) angleInRadians * 125.0; //pivotWheelRatio * (3.14159265359/180) * targetAngle;   //This converts the desired angle of rotation into mm
  
  required_encoder_count = (arc_length / 0.3595) ; // Converts the desired arc length into encoder counts

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
      break;
    }
    motorPower(RightMotor,0);
    motorPower(LeftMotor,0);

    delay(50); //The required 50ms delay

  }
}


///////////////////

// defining functions for line following
void driveStriaght( double voltage){
    motorPower(RightMotor,voltage*0.945);
    motorPower(LeftMotor,voltage);


}


void TurnRight (double voltage){
    motorPower(RightMotor,-voltage*0.945);
    motorPower(LeftMotor,voltage);
}


void TurnLeft(double voltage){
    motorPower(RightMotor,voltage*0.945);
    motorPower(LeftMotor,-voltage);
}


void stop(){
    motorPower(RightMotor,0);
    motorPower(LeftMotor,0);
}


void linefollowing(){

    double PWR=2000.0;
    double brownUpperIm= 2200.0;
    double brownLowerLim= 1800.0;


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

        int last_turn;
        int sin_case; 


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
        if (leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown== false){// middle sees brown 
                driveStriaght(PWR);
                sin_case=1;
        }

        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==true){// right sees brown
            motorPower(RightMotor,(-PWR*0.945));
            motorPower(LeftMotor,(PWR));
            last_turn=2;
            // delay(50);
              sin_case=2;
        }


        if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==false){// left sees brown
            motorPower(RightMotor,(PWR*0.945));
            motorPower(LeftMotor,(-PWR));
            last_turn=1;
            sin_case=3;
            // delay(50);                                                                     
             
        }

        if(leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown==true){// middle and right see brown
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR);
            sin_case=4;
               
        }


        if(leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown==false){// left nad middle see brown
           motorPower(RightMotor,PWR*0.945);
           motorPower(LeftMotor,-PWR);
           sin_case=5;
            //delay(50);                                                                      //90 to the left
               
        }


        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==false && last_turn==1){//none see brown
            TurnLeft(PWR);
        sin_case=6;
        }

        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==false && last_turn==2){//none see brown
            TurnRight(PWR);
        sin_case=87;
        }



        if (leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown== true){// all see brown
                                                               
            //delay(50);
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR);
            sin_case=7;
        }


        if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==true){// left and right see brown
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR); // add previous sensor reading and depending on that turn right or left      
            sin_case=8;
        }


       


        if ( leftSeesBlack==true || middleSeesBlack== true || rightSeesBlack== true){// any see black
            stop();
            break; 
            sin_case=9;                                                                      
        }
        lcd_print(3,"%d",sin_case);
        delay(50);

    }
    stop();

}

// things to do:
// rewrite a encoder to milimeter fucntion
// test line following
// test arm angle function
// test turning angle fucntion

// i really hope these functions work 

void turnAngle1(float angle, float Kp){


	float error, circumference, Distance, AngleTurn_ratio, ArcLength_ratio, u = 0;
	int r = 103; //radius of turning circle of the robot in mm
	int power = 0;  //wheel power variable
    int L_Encoder_Count,R_Encoder_Count;


	circumference = 2*PI*r; //circumference of turning circle in mm
	AngleTurn_ratio = abs(angle/360);

	resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

	if (angle > 0){

// If the angle > 0, then we have to turn anti-clockwise.

		do{

			Distance = (((readSensor(RightEncoder)) + abs(readSensor(LeftEncoder)))/2)*0.3595;

			ArcLength_ratio = Distance/circumference;
            L_Encoder_Count = readSensor(LeftEncoder);
            R_Encoder_Count = readSensor(RightEncoder);
            error = R_Encoder_Count - L_Encoder_Count; 
			//error = AngleTurn_ratio - ArcLength_ratio;
            lcd_print(2,"%d",error);
			u = Kp*error;
			power = (int)saturate(u, 3000, -3000);  
			motorPower(LeftMotor, power);
			motorPower(RightMotor, -power);
			delay(100);
		}while (u > 0);
	}
	else{


		do{
			Distance = (((abs(readSensor(RightEncoder)) + readSensor(LeftEncoder))/2))*0.3595;
			ArcLength_ratio = Distance/circumference;
			L_Encoder_Count = readSensor(LeftEncoder);
            R_Encoder_Count = readSensor(RightEncoder);
            error = R_Encoder_Count - L_Encoder_Count; 
			u = Kp*error;
            lcd_print(2,"%d",error);
			power = (int)saturate(u, 3000, -3000);  
			motorPower(LeftMotor, -power);
			motorPower(RightMotor, power);
			delay(100);
		}while (u > 0);
	}
}





// ----------------------------------------------- Function defintions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

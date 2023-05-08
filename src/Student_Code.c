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

// Lifts the arm up and prevent it from interfering with the sonar
armUp(4000);

// Read the distance between the robot at the starting position and the can
double dis2can = readSensor(SonarSensor);
delay(50);

// Driving to the can 
drive_to_can(400.0);

// Lowering arm for picking up the payload
armangle(-15.5,5.0,3.0);
delay(50);

// Going forward after the arm has been lowered 
drivePIcont(60.0,0.8,0.1);
delay(50);

// Picks up the payload
armUp(4000);

// Calculating the driving back distance
double drive_back_dis= (dis2can-(1130.5));

// Drive back the calculated distance between the ultrasonic shield and the centre of the black line
drivePIcont(-drive_back_dis,0.8,0.1);
delay(50);

// Rotate so that the robot is aligned with the centre of the black line
rotateAngle(7.0,-90.0);
delay(50);

// Robot will drive until the black line, move forward a bit to prevent early termination of line following and line following will engage
driveUntilBlack(40.0);
delay(1000);
drivePIcont(60.0,0.8,0.1);
delay(50);
linefollowing();
delay(50);

// Drives the forward to the drop off point
drivePIcont(342.0,0.8,0.1);// might need to edit
delay(50);

// Drops off the payload and drives backwards and lifts the arm up again
armangle(-15.5,5.0,3.0);
delay(50);
drivePIcont(-150.0,0.8,0.1);
delay(50);
armUp(4000);
delay(50);


// Rotates the robot 90 degrees, drives the specified distance to the finish zone 
rotateAngle(7.0,-90.0);
delay(50);
drivePIcont(1292.0,0.8,0.1);
delay(50);

// Robot rotates so that the front is facing the finish zone and drives the distance so its inside the finish zone
rotateAngle(7.0,110.0);
delay(50);
drivePIcont(400.0,0.8,0.1);


}

}
// Convert power function //
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
    // this fnction takes one input, the encoder count and converts it into a distacne in mm.

    // input: enc_count - encouder count

    double distance;
    // finding the circumfrance for the turing circle 
    double Circumfrance= PI*drivingWheelDiameter;
    // using the encoder counts of a wheel to convert the counts to a distance in mm
    distance=enc_count*(Circumfrance/900);
    // returns the converted distance in mm
    return distance;
}


void drivePIcont (double target,double Kp,double Ki){
    // this function takes three inputs, the target distance, the KP, and the Ki to drive the robot a given distance.

    // inputs:
        // target- the target distance in mm
        // Kp - the proportional gain
        // Ki - the integral gain


    // initalising the varibles used in the code
    double currentPos,u=0;
    int Pwr=0;
    int i=0;
    int error=0;

    // reseting encoders so previous counts dont interfair with current code
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    // the code will run at least one time before reaching a while statement
    do{
        // saving the encouder counts into theor respective variables
        int L_encoder=readSensor(LeftEncoder);
        int R_encoder=readSensor(RightEncoder);
        // finding the mean encoder count
        double avg_encoder_count= abs(L_encoder+R_encoder)/2;
        // finding thee current position in mm by converting the encoder counts to a distance
        currentPos=encTodistance(avg_encoder_count);   
        // defining the error for the PI controller as being the distanced we want to traven the distance the robot has currently travlelled
        error=target-currentPos;

        //Stop integrator wind up
        if( abs(u) <= 100){
            i=i+error;
        }
        // defining the control effort using the formula for a PI controlller
        u=(Kp*error + Ki*i);                                                                                                                            

        // creating a variable called differnece that records the differnece in thee wheels encoder counts as to send more or less power to the wheel later
        double differance=(L_encoder-R_encoder)*10; // the "10" is a second Kp 
        // saturating the poweras to not damage the wheel motors this is done using convertPower, however we also saturate to limit the speed of thee wheels
        u = saturate(u,-60.0,60.0);
        Pwr = convertPower(u);

        // sending power to the motors based on the saturated control effort and the diffenence in wheel encoder counts
        motorPower(LeftMotor, Pwr - differance);
        motorPower(RightMotor, Pwr + differance);
        // delaying so that the motors have time to react
        delay(50);
        // the code will run while the current position of the robot is 5mm from the target distance, the 5 is a tolerance to account for inertia
    }while((currentPos+5)<=(abs(target)));
    // telling the wheel motors to stop once the wile statement has been met
    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);
}








/// drive to can function //
void drive_to_can(int stopping_distance){
    // this funciton drive the robot straight a given distance from the obstical

        // input: stopping_distance - the distacne to drive away formt he obsticel

    // finding the distance to the can using the ultrasonic sensor and sorting it into a variable
    double d2can = readSensor(SonarSensor);
    // defining the stopping distance as being the distance to the can minus the given stoppping distance
    double end_distance = (d2can)-stopping_distance;
    // calling the drive forwards function to drive the wanted distance
    drivePIcont(end_distance,0.35,0.1);
    
}    



void rotateAngle(double Kp, double targetAngle){
    // this fucntion piviots the robot a wanted angle.
        
        // inputs:
            //Kp - the proportional gain
            //targetAngle - the angle we want the robot to rotate to

    // initalising the variables used in the code
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

        // setting target angle to be negative as our code uses clockwiew turns as being positive. this makes it that the target angle is now respective of anticlockwise being positive
        targetAngle=targetAngle*-1.0;
    
    // reseting the wheels encoders so previous reading don't interfere with the current code
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    // the do makes sure the following code will run atleast once
    do{
        // storing the encoder counts for the wheels into theor respective variables
        leftencCount=readSensor(LeftEncoder);
        rightencCount=readSensor(RightEncoder);
        // using the encTodistacne function to conver the encoder coutns into a distance
        left_dis=encTodistance(leftencCount);
        right_dis= encTodistance(rightencCount);
        // finding the mean encoder count
        average_distance_traveled=(fabs(left_dis)+fabs(right_dis))/2.0; 
        // the following if statements make sure that the robot turns both clockwise and counter clockwise, this is done by changing the error which is then later fed into the contorle effort
        if (targetAngle<0){                                                                                                                                                                          
            
            error = distance_to_travel + (average_distance_traveled);
        }
        else if ( targetAngle>0){

        error = distance_to_travel - (average_distance_traveled);
        }
        // diffing the differnece to be the left distance minus the right difference and multiplying it by 0.01 so that the difference doestn have too large a impact on the adjustments made to the right motor
        diff=((left_dis)-(right_dis))*0.01; 


        // defining the control effort using the equation for a P controller
        pwr = error * Kp;
        // setting the input voltage as being the power we calculated in the previous step, and deviding it by 5 to reduce the power, also saturating it using convertPower to protect the motors form recivign too much power
        v1=convertPower(pwr/5);
        // defining v2 to be the power used for adjusting anc compensating for inaccuracies in the wheels turning, again using converPower to protect the wheel motors.
        v2=convertPower(diff/5);
     
       // sending power to theri respective wheels
        motorPower(LeftMotor,v1); 
        motorPower(RightMotor,-(v1+v2));
        // this breakis the code out of the while loop if the error is less than 10
        if (fabs(error)<10.0){
            break;
        }
        lcd_print(2,"%f",error);
        // adding a delay so that motors have time to react to the code
        delay(50);
        // the infinate loop makes it that the code only break out if the error is less than 10
    }while(1);
    // stopping the wheel motors 
    motorPower(LeftMotor,0);
    motorPower(RightMotor,0);

}

void armangle(double targetAngle, double Kp, int tolerance)   
{
    // this function takes three inputs and uses them to rotate the arm to a given angle
    // inputs:
        // targerAngle - the wanted angle in degrees
        // Kp -  the potential gain
        // tolerance - the degree of inaccuracy we are wiling to accept so the code can break out of the loop

// defining variables:

double currentAngle;
double error;
double PWR;
int enc_counts;
// reseting the arm to its topmost position 
armUp(4000);
// reseting the arm encoder as to stop previous encoder counts from interfering with current code
resetEncoder(ArmEncoder);
// the do endures the code runs atleast once =
do{
// reaing the encoder counts into a variable
enc_counts=readSensor(ArmEncoder);
// this line converts the encoder counts into a angle from the horizontal opositon by using the angle the arm is at its top position
currentAngle = (360.0/(7.0*900.0))*(enc_counts) + 41.0;
// definning the error as the wanted angle minus the angle the arem is currently at
error= targetAngle-currentAngle;
// defining the contorl effort using the equation for a P controller
double u=Kp*error;
// saturating  the power using the converPower finction so safegarf the motor
PWR=convertPower(u);
// sending the saturated power to the arm motor
motorPower(ArmMotor,PWR);
// delaying the code by 50ms to allow the arm motor ample time to react to the code
delay(50);
}
// this while statement will stop the arm motor once the arm angle minus a the input tolerance is greater than the target angle.
while((currentAngle-tolerance) > targetAngle);
// telling the motor the stop
motorPower(ArmMotor,0);

}

double arm_enc_2_angle(int encoderCounts){
    // this function take the encodercounts from the arm and converts them to a angle from the horizontal position

    double currentAngle=((encoderCounts*360.0)/(7.0*900.0))+ 41.0;
    // returning the current angle the arm is at
    return currentAngle;
}

void driveUntilBlack(double precent_power){
    // this function takes one input that being the precentage of power to send to the wheels, it uses this to drive straight until any of the light sensors see black

    // defining the variables used in the code
    int s1,s2,s3;
    int upper_lim, lower_lim;
    // reseting the wheels encoders so previous readings sont interfere with the current code 
    resetEncoder(RightEncoder);
    resetEncoder(LeftEncoder);
    // converting the inputed power to a precentage for the motors and also saturationg to to safegard the motors
    int Pwr=convertPower(precent_power);
    // defining th upper and lower limits for the values that correspond to black
    upper_lim=2400;
    lower_lim=2000;
    
    // puts the code into an infinate while loop as i always = 1.
    while (1){
        // reading the lightsensors reading into their respective variavles
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
 




///////////////////

// defining functions for line following
void driveStriaght( double voltage){
    // this function takes the voltage as an input and tells the wheel motors to dieve forwards
    motorPower(RightMotor,voltage*0.945);
    motorPower(LeftMotor,voltage);


}


void TurnRight (double voltage){
    // this fuicntion take the voltage as an input and tells the robots wheels motors to turn right
    motorPower(RightMotor,-voltage*0.945);
    motorPower(LeftMotor,voltage);
}


void TurnLeft(double voltage){
        // this fuicntion take the voltage as an input and tells the robots wheels motors to turn left

    motorPower(RightMotor,voltage*0.945);
    motorPower(LeftMotor,-voltage);
}


void stop(){
    // this function tells the wheel motors to stop rotating
    motorPower(RightMotor,0);
    motorPower(LeftMotor,0);
}


void linefollowing(){
    // this code takes no inpts, it simple folows the brown line while the light sensors dont see black
    // definiog the power to 2000.0mv
    double PWR=2000.0;
    // setting the limits for the brown reading
    double brownUpperIm= 2200.0;
    double brownLowerLim= 1800.0;

    // setting the limits for the black reading
    double blackUpperLim= 2700.0;
    double blackLowerLim=2450.0;


    // putting the code into an infinate loop
    while(1){
        // reading te light sensors reading into its respective variables
        double s1=readSensor(LeftLight);
        double s2=readSensor(MidLight);
        double s3=readSensor(RightLight);

        // defaulting all variavles for if a sensor sees the wnated colour to false so that it can be changed when certain conditions are met later
        int leftSeesBrown= false;
        int middleSeesBrown= false;
        int rightSeesBrown= false;


        int leftSeesBlack= false;
        int middleSeesBlack= false;
        int rightSeesBlack= false;
        // this variable allos us to store what the last turn the robot made was
        int last_turn;
        // this variabvle is used so we can display to the robots screen what senario is running
        int sin_case; 


// Brown TF readings

        // if the left sensor's readig are within the brown limits it means that the left sensor sees brown
        if (s1>=brownLowerLim && s1<=brownUpperIm){
            leftSeesBrown=true;
        }

        // if the middle sensor's readig are within the brown limits it means that the middle sensor sees brown
        if (s2>=brownLowerLim && s2<=brownUpperIm){
            middleSeesBrown=true;
        }
        // if the right sensor's readig are within the brown limits it means that the right sensor sees brown
        if (s3>=brownLowerLim && s3<=brownUpperIm){
            rightSeesBrown=true;
        }
 // blacks TF readings
        // if the left sensor's readig are within the black limits it means that the left sensor sees black
        if (s1>=blackLowerLim && s1<=blackUpperLim){
            leftSeesBlack=true;
        }

        // if the middle sensor's readig are within the black limits it means that the middle sensor sees black
        if (s2>=blackLowerLim && s2<=blackUpperLim){
            middleSeesBlack=true;
        }
        // if the right sensor's readig are within the black limits it means that the right sensor sees black
        if (s3>=blackLowerLim && s3<=blackUpperLim){
            rightSeesBlack=true;
        }


        //sensor conditions
        // iff only the middle sesor sees brown drive straight
        if (leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown== false){// middle sees brown 
                driveStriaght(PWR);
                sin_case=1;
        }
        // if only the right sensor sees brown turn right and set the last turn variable to 2(meaning right trurn)
        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==true){// right sees brown
        // the 0.945 is to keep the wheels turning at the ame rate
            motorPower(RightMotor,(-PWR*0.945));
            motorPower(LeftMotor,(PWR));
            last_turn=2;
            // delay(50);
              sin_case=2;
        }

        // if only the left sensor sees brown turn left and set last turn varlble to 1 indicating a left turn
        if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==false){// left sees brown
            motorPower(RightMotor,(PWR*0.945));
            motorPower(LeftMotor,(-PWR));
            last_turn=1;
            sin_case=3;
            // delay(50);                                                                     
             
        }
        // if the middle and right sensors see brown then turn right 
        if(leftSeesBrown==false && middleSeesBrown== true && rightSeesBrown==true){// middle and right see brown
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR);
            sin_case=4;
               
        }

        // if the left an middle sensors see brown than turn left
        if(leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown==false){// left nad middle see brown
           motorPower(RightMotor,PWR*0.945);
           motorPower(LeftMotor,-PWR);
           sin_case=5;
            //delay(50);                                                                      //90 to the left
               
        }

        // if no sensor sees brown and also doesnt see black and the last turn was to the left, turn left
        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==false && last_turn==1){//none see brown
            TurnLeft(PWR);
        sin_case=6;
        }
        // if no sensor sees brown and also doesnt see black and the last turn was to the right, turn right
        if (leftSeesBrown==false && middleSeesBrown==false && rightSeesBrown==false && last_turn==2){//none see brown
            TurnRight(PWR);
        sin_case=87;
        }


        // if all the sensors see brown hope turning right fixes the problem, (turn right)
        if (leftSeesBrown==true && middleSeesBrown==true && rightSeesBrown== true){// all see brown
                                                               
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR);
            sin_case=7;
        }

        // if the left and right sensor sees brown youre stuck in a 90 degree bend ,, turn right and hope that puts you back on course
        if (leftSeesBrown==true && middleSeesBrown==false && rightSeesBrown==true){// left and right see brown
            motorPower(RightMotor,-PWR*0.945);
            motorPower(LeftMotor,PWR);     
            sin_case=8;
        }


       

        // if any sensor sees black break out of the loop and stop the motors.
        if ( leftSeesBlack==true || middleSeesBlack== true || rightSeesBlack== true){// any see black
            stop();
            break; 
            sin_case=9;                                                                      
        }
        // prints the senirio number to the display 
        lcd_print(3,"%d",sin_case);
        // the delay allows for the motors to catch up to the code
        delay(50);

    }
    stop();

}







// ----------------------------------------------- Function defintions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

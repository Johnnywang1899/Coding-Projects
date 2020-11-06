/*
 Robotics with the BOE Shield – LeftServoClockwise
 Generate a servo full speed clockwise signal on digital pin 13.
 */

#include <Servo.h>                           // Include servo library

int fwdtime=3000;
int left_trntime=2300; 
int right_trntime=1100;
int stay=2000;
int minor_change=400; 
int minor_change_fwd=200; 
int minor_left=383;
int mionr_right=183;

Servo servoLeft;                             // Declare left servo
Servo servoRight;                            // Declare right servo
void setup()                                 // Built in initialization block
{ Serial.begin(9600);
  servoLeft.attach(7);                      // Attach left signal to pin 8
  servoRight.attach(12);                    // Attach right signal to pin 12                   
}  
 
void loop()                                  // Main loop auto-repeats
{ 
  //where call functions


}

  void Right_Turn(){
  servoLeft.writeMicroseconds(1530);         // right turn
  servoRight.writeMicroseconds(1550);        
  delay(right_trntime);  
  }

  void Left_Turn(){
  servoLeft.writeMicroseconds(1455);         
  servoRight.writeMicroseconds(1450);        // left turn
  delay(left_trntime);  
  }

  void Move_Fwd(){
  servoLeft.writeMicroseconds(1545);         
  servoRight.writeMicroseconds(1420);        // fwd
  delay(fwdtime);
  }
  
  void minor_right_shift(){
  servoLeft.writeMicroseconds(1530);         // minor shift to right step 1-small right turn
  servoRight.writeMicroseconds(1550);        
  delay(minor_change); 

  servoLeft.writeMicroseconds(1545);         // fwd
  servoRight.writeMicroseconds(1420);        
  delay(minor_change_fwd);

  servoLeft.writeMicroseconds(1455);         // minor shift to right step 2-small left turn
  servoRight.writeMicroseconds(1450);        
  delay(minor_change*23/11);
  }  

  void minor_left_shift(){
  servoLeft.writeMicroseconds(1455);         // minor shift to left step 1-small right turn
  servoRight.writeMicroseconds(1450);        
  delay(minor_change*23/11);
 

  servoLeft.writeMicroseconds(1545);         // fwd
  servoRight.writeMicroseconds(1420);        
  delay(minor_change_fwd);

  servoLeft.writeMicroseconds(1530);         // minor shift to left step 2-small left turn
  servoRight.writeMicroseconds(1550);        
  delay(minor_change); 
  }

  void left_minor(){ 
  servoLeft.writeMicroseconds(1455);         
  servoRight.writeMicroseconds(1450);        // minor left turn
  delay(minor_left);
  }
  
  void right_minor(){
  servoLeft.writeMicroseconds(1530);         // minor right turn
  servoRight.writeMicroseconds(1550);        
  delay(mionr_right);    
  }
  void parking(){
  servoLeft.detach();                        // stay
  servoRight.detach();       
  delay(stay);  
  }

  void reactive(){
  servoLeft.attach(7);                        //  Re-sending servo signals
  servoRight.attach(12);
  }

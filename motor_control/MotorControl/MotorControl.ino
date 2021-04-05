
#include <Servo.h>
Servo MCL;  //define motor controller object in this case MCL - motor control Left


void setup() {
  MCL.attach(30); //set pin to use with specified MC object
}

void loop() {

  //choose one of these
  //MCL.writeMicroseconds(1500); // use miliseconds to control motor can bee seen in documentation more precise but less intuitive
  MCL.write(90); //use 0-180 for writing the speed
  
}

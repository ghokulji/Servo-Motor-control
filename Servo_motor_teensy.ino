/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of servos
 * using ROS Serial and Teensy 3.2
 * 
 * http://www.arduino.cc/en/Reference/Servo
 * 
 * 
 */

#define USE_USBCON
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
int opens = 180; // opens to an angle of 180
int closed = 100; // closes to an angle of 100

Servo myservo;
Servo myservo1;

//Callback function, if the topic data is 1(unlatch) then the servo motor opens.
void servo_cb( const std_msgs::UInt16& cmd_msg){

  if ( cmd_msg.data == 1) //openning
  {
    for(int pos = closed; pos <= opens; pos += 1)// loop for speed controlfrom 180 to 100, or else the gears don't mesh well due to high speed.
  {                                   
    myservo.write(pos); 
    myservo1.write(pos);             
    delay(15);              
    if(pos == opens) // if the opening position is reached,break from the loop
    {
      break;
    }
  } 
  
  }

  else if (cmd_msg.data == 0)//closing
  {
    for(int pos = opens; pos >= closed; pos -= 1) //loop for closing from 180 to 100
  {                                  
    myservo.write(pos); 
    myservo1.write(pos);             
    delay(15); 
    if(pos == closed) // if the closing position is reached, break from the loop
    {
      break;
    }
  } 
  }
  
                                                 }


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb); // subscribe to the topic servo which publlishes 1(open) or 0(closed).

void setup(){

  Serial.begin(115200);
  nh.initNode();
  
  //pin layout for Teensy 3.2 (https://www.pjrc.com/teensy/pinout.html)
  myservo.attach(20); //attach it to pin 20 of Teesny
  myservo1.attach(22); //attach it to pin 22 of Teensy 
  
  
}

void loop(){

  nh.subscribe(sub);
  nh.spinOnce();
  delay(1);    //changing the delay value gives error " Unable to sync with device; possible link problem or 
                   //  link software version mismatch such as hydro rosserial_python with groovy Arduino"
           }


/* Instructions for running                     
 *  Publish a topic 'servo' with data 1(open) or 0(close)
 *  RUN:  rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200 ( make sure the port is correct). 
 */ 

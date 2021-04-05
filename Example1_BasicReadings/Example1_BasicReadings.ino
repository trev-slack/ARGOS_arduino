/******************************************************************************
  Example1_BasicReadings.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch configures the TMP117 temperature sensor and prints the
  temperature in degrees celsius and fahrenheit with a 500ms delay for
  easier readings. 

  Resources:
  Wire.h (included with Arduino IDE)
  SparkFunTMP117.h (included in the src folder) http://librarymanager/All#SparkFun_TMP117

  Development environment specifics:
  Arduino 1.8.9+
  Hardware Version 1.0.0

  This code is beerware; if you see me (or any other SparkFun employee) at
  the local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

/*
  NOTE: For the most accurate readings:
  - Avoid heavy bypass traffic on the I2C bus
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor,
  reference the "Precise Temperature Measurements with TMP116" datasheet that is
  linked on Page 35 of the TMP117's datasheet
*/

#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor

// The default address of the device is 0x48 = (GND)
TMP117 sensor; // Initalize sensor

#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle  arduino_node;
std_msgs::Float32 temp_msg;
ros::Publisher temp_pub("/argos/temperature",&temp_msg);

void setup()
{
  Wire.begin();
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  //arduino_node.getHardware()->setBaud(57600);
  arduino_node.initNode();
  arduino_node.advertise(temp_pub);
  
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    1;
  }
  else
  {
    
    while (1); // Runs forever
  }
}

void loop()
{
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    temp_msg.data = sensor.readTempC()+273.15;
    temp_pub.publish(&temp_msg);
    arduino_node.spinOnce();
  } else {
    arduino_node.spinOnce();
  }
}

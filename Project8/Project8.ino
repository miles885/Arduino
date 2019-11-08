/*
  Project 8 - Quadcopter downloading IMU to RPi
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program reads orientation data from the BNO055 IMU, calculates roll
  pitch, and yaw from the Quaternion data, and transmits the data over serial
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Constants */
const int SENSOR_SAMPLE_DELAY = 100;

/* Global variables */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**
 * Initialization that gets run when you press reset or power the board
 * 
 * @params None
 * 
 * @return None
 */
void setup()
{
    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);

    // Initialise the BNO055 sensor
    if(!bno.begin())
    {
        Serial.print("No BNO055 detected... check your wiring or I2C address!");
        while(1);
    }

    delay(1000);

    bno.setExtCrystalUse(true);
}

/**
 * Runs indefinitely
 * 
 * @param None
 * 
 * @return None
 */
void loop()
{
    //TODO: Check calibration here? 0=uncalibrated, 3=fully calibrated
    
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);

    // Display orientation data
    //TODO: Retrieve quaternion and calculate RPY
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");

    //TODO: Transmit RPY over serial (USB)

    delay(SENSOR_SAMPLE_DELAY);
}

/*
  Project 8 - Quadcopter downloading IMU to RPi
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program reads orientation data from the BNO055 IMU, calculates roll
  pitch, and yaw from the Quaternion data, and transmits the data over serial
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino_FreeRTOS.h>
#include <utility/imumaths.h>
#include <Wire.h>

/* Constants */
const bool CALIBRATING_BNO = false;

const int BNO_SAMPLE_RATE = 100;  // ms

const adafruit_bno055_offsets_t SENSOR_OFFSETS = {-19, -32, -35,  // Acceleration
                                                  -10, 171, -70,  // Magnetometer
                                                  -1, 0, 0,       // Gyroscope
                                                  1000,           // Acceleration radius
                                                  805};           // Magnetometer radius

/* Global variables */

/**
 * Initialization that gets run when you press reset or power the board
 * 
 * @params None
 * 
 * @return None
 */
void setup()
{
    // Setup up the serial connection at 115200 bits per second
    Serial.begin(115200);
    delay(1000);

    // Create tasks
    xTaskCreate(TaskReadIMU, "ReadIMU", 256, NULL, 2, NULL);
}

/**
 * Runs indefinitely (not used by FreeRTOS)
 * 
 * @param None
 * 
 * @return None
 */
void loop()
{
    
}

/**
 * Task entry point for configuring the BNO055 sensor, reading
 * data off the sensor, and transmitting roll, pitch, yaw over serial
 * 
 * @param pvParameters Task parameters
 * 
 * @return None
 */
void TaskReadIMU(void * pvParameters)
{
    //TODO: BNO055_ID, BNO055_ADDRESS_A
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    
    initBNOSensor(bno);

    while(true)
    {
        // Retrieve quaternion
        imu::Quaternion quat = bno.getQuat();
    
        float w = quat.w();
        float x = quat.x();
        float y = quat.y();
        float z = quat.z();
    
        // Calculate RPY
        float roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        float pitch = asin(2.0 * w * y - x * z);
        float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    
        float rollDeg = roll * RAD_TO_DEG;
        float pitchDeg = pitch * RAD_TO_DEG;
        float yawDeg = yaw * RAD_TO_DEG;
    
        // Transmit RPY over serial (USB)
        //TODO: Replace Serial with Wire library when using I2C (probably need separate task to handle I2C requests too)
        Serial.print(rollDeg);
        Serial.print(",");
        Serial.print(pitchDeg);
        Serial.print(",");
        Serial.print(yawDeg);
        Serial.println("");
        
        delay(BNO_SAMPLE_RATE);
    }
}

/**
 * Initialize the BNO055 sensor
 * 
 * @param bno The BNO055 sensor object
 * 
 * @return None
 */
void initBNOSensor(Adafruit_BNO055 & bno)
{
    // Initialize the BNO055 sensor
    if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
    {
        Serial.print("No BNO055 detected... check your wiring or I2C address!");
        while(1);
    }

    // Set calibration data
    if(!CALIBRATING_BNO)
    {
        bno.setSensorOffsets(SENSOR_OFFSETS);
    }

    delay(1000);

    bno.setExtCrystalUse(true);

    // Check calibration status
    sensors_event_t event;

    if(!CALIBRATING_BNO)
    {
        Serial.println("Wave sensor around to calibrate magnetometers");

        while(!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            
            delay(BNO_SAMPLE_RATE);
        }
    }
    else
    {
        Serial.println("Please calibrate the sensor:");
        Serial.println("- Gyroscope: Sensor must start at rest");
        Serial.println("- Magnetometer: Wave sensor around");
        Serial.println("- Accelerometer: Place sensor in +X,-X,+Y,-Y,+Z,-Z orientations");

        while(!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            
            getCalData(bno);

            delay(BNO_SAMPLE_RATE);
        }
    }
}

/**
 * Prints calibration status and sensor offsets
 * when sensor is fully calibrated
 * 
 * @param bno The BNO055 sensor object
 * 
 * @return None
 */
void getCalData(Adafruit_BNO055 & bno)
{
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.print("System: ");
    Serial.print(system);
    Serial.print("\tGyro: ");
    Serial.print(gyro);
    Serial.print("\tAccel: ");
    Serial.print(accel);
    Serial.print("\tMag: ");
    Serial.print(mag);
    Serial.println("");

    adafruit_bno055_offsets_t sensorOffsets;
    bool calStatus = bno.getSensorOffsets(sensorOffsets);

    if(calStatus)
    {
        Serial.print("Accel offset: ");
        Serial.print(sensorOffsets.accel_offset_x);
        Serial.print(" | ");
        Serial.print(sensorOffsets.accel_offset_y);
        Serial.print(" | ");
        Serial.print(sensorOffsets.accel_offset_z);
        Serial.println("");
    
        Serial.print("Mag offset: ");
        Serial.print(sensorOffsets.mag_offset_x);
        Serial.print(" | ");
        Serial.print(sensorOffsets.mag_offset_y);
        Serial.print(" | ");
        Serial.print(sensorOffsets.mag_offset_z);
        Serial.println("");
    
        Serial.print("Gyro offset: ");
        Serial.print(sensorOffsets.gyro_offset_x);
        Serial.print(" | ");
        Serial.print(sensorOffsets.gyro_offset_y);
        Serial.print(" | ");
        Serial.print(sensorOffsets.gyro_offset_z);
        Serial.println("");
    
        Serial.print("Accel radius: ");
        Serial.print(sensorOffsets.accel_radius);
        Serial.println("");
    
        Serial.print("Mag radius: ");
        Serial.print(sensorOffsets.mag_radius);
        Serial.println("");
    }
}

/*
  Project 10 - Quadcopter downloading IMU to RPi
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program reads orientation data from the BNO055 IMU, calculates roll
  pitch, and yaw from the Quaternion data, and transmits the data over I2C
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino_FreeRTOS.h>
#include <utility/imumaths.h>
#include <Wire.h>

union FloatToBytes
{
    float value;
    byte bytes[4];
};

/* Constants */
const int I2C_SLAVE_ADDRESS = 0x05;
const int I2C_DATA_SIZE = 12;

const bool CALIBRATING_BNO = false;
const int BNO_TICK_DELAY = 7;  // 7 * 15 ms per tick = 105 ms

const adafruit_bno055_offsets_t SENSOR_OFFSETS = {-2, -41, -27,    // Acceleration
                                                  -28, 162, -106,  // Magnetometer
                                                  0, 0, 0,         // Gyroscope
                                                  1000,            // Acceleration radius
                                                  853};            // Magnetometer radius

/* Global variables */
volatile bool readIMU = false;
byte rpyBuffer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * Initialization that gets run when you press reset or power the board
 * 
 * @params None
 * 
 * @return None
 */
void setup()
{
    // Setup up serial and I2C
    Serial.begin(115200);

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveCommand);
    Wire.onRequest(sendRPY);
    
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
    delay(100);
}

/**
 * I2C receive callback that reads a command from the I2C master
 * 
 * @param numBytes The number of bytes available to read
 * 
 * @return None
 */
void receiveCommand(int numBytes)
{
    if(numBytes > 0)
    {
        readIMU = true;
    }

    for(int i = 0; i < numBytes; i++)
    {
        char value = Wire.read();
    }
}

/**
 * I2C request callback that sends roll, pitch, and yaw data
 * to the I2C master as a byte array
 * 
 * @param None
 * 
 * @return None
 */
void sendRPY()
{
    Wire.write(rpyBuffer, I2C_DATA_SIZE);
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
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    
    initBNOSensor(bno);

    while(true)
    {
        if(readIMU)
        {
            // Retrieve RPY
            sensors_event_t event;
            bno.getEvent(&event);

            FloatToBytes roll;
            FloatToBytes pitch;
            FloatToBytes yaw;

            roll.value = -event.orientation.z;
            pitch.value = event.orientation.y;
            yaw.value = map(event.orientation.x, 0.0, 360.0, -180.0, 180.0);

            // Store RPY in a byte buffer
            rpyBuffer[0] = roll.bytes[0];
            rpyBuffer[1] = roll.bytes[1];
            rpyBuffer[2] = roll.bytes[2];
            rpyBuffer[3] = roll.bytes[3];
            rpyBuffer[4] = pitch.bytes[0];
            rpyBuffer[5] = pitch.bytes[1];
            rpyBuffer[6] = pitch.bytes[2];
            rpyBuffer[7] = pitch.bytes[3];
            rpyBuffer[8] = yaw.bytes[0];
            rpyBuffer[9] = yaw.bytes[1];
            rpyBuffer[10] = yaw.bytes[2];
            rpyBuffer[11] = yaw.bytes[3];

            readIMU = false;
        }

        vTaskDelay(BNO_TICK_DELAY);
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

    vTaskDelay(67);  // 67 ticks * 15 ms/tick = 1005 ms

    bno.setExtCrystalUse(true);

    // Check calibration status
    sensors_event_t event;

    if(!CALIBRATING_BNO)
    {
        Serial.println("Wave sensor around to calibrate magnetometers");

        while(!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            
            vTaskDelay(BNO_TICK_DELAY);
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
            
            getCalStatus(bno);

            vTaskDelay(BNO_TICK_DELAY);
        }

        getSensorOffsets(bno);
    }

    Serial.println("Calibration complete!");
}

/**
 * Retrieves and prints the calibration status
 * 
 * @param bno The BNO055 sensor object
 * 
 * @return None
 */
void getCalStatus(Adafruit_BNO055 & bno)
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
}

/**
 * Retrieves and prints the sensor offsets for calibration purposes
 * 
 * @param bno The BNO055 sensor object
 * 
 * @return None
 */
void getSensorOffsets(Adafruit_BNO055 & bno)
{
    adafruit_bno055_offsets_t sensorOffsets;
    bno.getSensorOffsets(sensorOffsets);

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

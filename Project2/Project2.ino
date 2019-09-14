/*
  Project 2 - Serial Transmit of Temperature
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program reads temperature data from a sensor at a periodic rate
  of 10 seconds. The temperature is converted to fahrenheit and stored
  on the Arduino so that it can be transmitted over Serial (USB) to a
  host machine at a later time.
*/

/* Constants */
const int TEMPERATURE_SENSOR_PIN = A0;
const int TEMPERATURE_READ_RATE = 10000;  // Milliseconds

/**
 * Initialization that gets run when you press reset or power the board
 * 
 * @params None
 * 
 * @return None
 */
void setup()
{
    // Initialize pins

    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);
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
    // Read the ADC value
    int adcValue = analogRead(TEMPERATURE_SENSOR_PIN);

    Serial.print("ADC value: ");
    Serial.print(adcValue);

    // Convert temperature value to voltage
    // NOTE: The Arduino UNO ADC pins are 10 bits in size, thus the ADC value range is 0-1023
    float temperatureV = (adcValue / 1024.0) * 5.0;

    Serial.print(" | Temperature voltage: ");
    Serial.print(temperatureV);

    // Convert temperature voltage to Celcius
    // NOTE: The TMP36 sensor has a resolution of 10 mV per degree centigrade with a temperature range
    //       of -50C to 125C. The forumla for converting from mV to centigrade is: C = (mV - 500) / 10,
    //       where a 500 mV offset is used to allow for negative temperatures
    //       (source: https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor)
    float temperatureC = (temperatureV - 0.5) * 100;

    Serial.print(" | Temperature Celcius: ");
    Serial.print(temperatureC);

    // Convert Celcius to Fahrenheit
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    Serial.print(" | Temperature Felcius: ");
    Serial.println(temperatureF);
}

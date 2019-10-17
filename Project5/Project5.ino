/*
  Project 5 - Weather Station
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program measures the tempature and humidity and displays it on an OLED screen.
*/

#include <DHTesp.h>       // Temperature and humidity sensor
#include <SSD1306Wire.h>  // OLED display

/* Constants */
const int DISPLAY_ADDRESS = 0x3C;
const int DISPLAY_SDA_PIN = D3;
const int DISPLAY_SCL_PIN = D4;
const int DHT_DAT_PIN = D6;

/* Global variables */
DHTesp dht;
SSD1306Wire ledDisp(DISPLAY_ADDRESS, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

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

    // Setup the HDT sensor
    dht.setup(DHT_DAT_PIN, DHTesp::DHT11);

    // Setup the OLED display
    ledDisp.init();
    ledDisp.clear();
    ledDisp.display();

    ledDisp.setFont(ArialMT_Plain_10);
    ledDisp.setTextAlignment(TEXT_ALIGN_CENTER);
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
    // Retrieve the humidity and temperature from the DHT sensor
    float humidity = dht.getHumidity();
    float temperatureC = dht.getTemperature();
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    // Check to make sure the sensor was able to read the humidity and temperature
    if(!isnan(humidity) && !isnan(temperatureF))
    {
        ledDisp.clear();
        ledDisp.drawString(64, 0, "Humidity (%)");
        ledDisp.drawString(64, 16, String(humidity));
        ledDisp.drawString(64, 32, "Temperature (F)");
        ledDisp.drawString(64, 48, String(temperatureF));
        ledDisp.display();
    }

    delay(500);
}

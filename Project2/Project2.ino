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

/* Timer variables */
static unsigned int timerCount = 0;

/* Temperature variables */
static bool receivedADCValue = false;
static bool usingADCValueA = false;
static int adcValueA = 0;
static int adcValueB = 0;

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

    // --------------------------------------------------------------------------------------------
    // Timer Terminology
    // --------------------------------------------------------------------------------------------
    // TCCRx  - Timer/Counter Control Register used to configure prescaler
    // TCNTx  - Timer/Counter Register used to store timer value
    // OCRx   - Output Compare Register
    // ICRx   - Input Capture Register (only for Timer1)
    // TIMSKx - Timer/Counter Interrupt Mask Register used to enable/disable timer interrupts
    // TIFRx  - Timer/Counter Interrupt Flag Register used to indicate a pending timer interrupt
    //
    // WGM    - Waveform Generator Mode
    // CS     - Clock Select
    // OCIE1A - Timer/Counter1, Output Compare A Match Interrupt Enable
    // 
    // CTC Mode - Interrupts triggered when counter reaches a specified value.
    // NOTE: Set TCCRxA for Timer0 and Timer2, TCCR1B for Timer1
    //
    // Source: http://www.electronoobs.com/eng_arduino_tut12.php and
    //         http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf

    // Create a timer that interrupts at 1 Hz
    // NOTE: Calculate the compare match register using the formula:
    //       cmp = (16,000,000 / (preScaler * interruptFreq)) - 1
    //       Source: https://www.instructables.com/id/Arduino-Timer-Interrupts/
    //
    //       As such, Arduino Timer1 cannot count at 0.1 Hz (10 seconds) because the
    //       max prescale value is 1024, resulting in a compare value of ‭156,249‬, which
    //       exceeds the max value of 65,536 that the 16 bit Timer1 register can hold
    noInterrupts();
    
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 15624;  // Compare register value: (16 MHz / (1024 * 1)) - 1
    TCCR1B |= (1 << WGM12);  // Turn on CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);  // 0x5 = 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);  // Enable compare interrupt

    interrupts();
}

/**
 * Interrupt service routine that reads the ADC value from the sensor pin
 * NOTE: This ISR is called at 1 Hz by a timer
 * 
 * @param None
 * 
 * @return None
 */
ISR(TIMER1_COMPA_vect)
{
    // Check to see if it has been 10 seconds
    if(timerCount == 9)
    {
        // Reset the timer count
        timerCount = 0;

        // Read the sensor ADC value
        if(!usingADCValueA)
        {
            adcValueA = analogRead(TEMPERATURE_SENSOR_PIN);
        }
        else
        {
            adcValueB = analogRead(TEMPERATURE_SENSOR_PIN);
        }

        // Update flags
        usingADCValueA = !usingADCValueA;
        receivedADCValue = true;
    }
    // It has not been 10 seconds yet
    else
    {
        timerCount++;
    }
}

/**
 * Calculate the temperature (Fahrenheit) from the ADC value
 * 
 * @param None
 * 
 * @return temperatureF The temperature in Fahrenheit
 */
float calcTemperature()
{
    // Copy the most recent ADC value
    int adcValue = usingADCValueA ? adcValueA : adcValueB;

    //TODO: Remove
    Serial.print("ADC value: ");
    Serial.print(adcValue);

    // Convert temperature value to voltage
    // NOTE: The Arduino UNO ADC pins are 10 bits in size, thus the ADC value range is 0-1023
    float temperatureV = (adcValue / 1024.0) * 5.0;

    //TODO: Remove
    Serial.print(" | Temperature voltage: ");
    Serial.print(temperatureV);

    // Convert temperature voltage to Celcius
    // NOTE: The TMP36 sensor has a resolution of 10 mV per degree centigrade with a temperature range
    //       of -50C to 125C. The forumla for converting from mV to centigrade is: C = (mV - 500) / 10,
    //       where a 500 mV offset is used to allow for negative temperatures
    //       Source: https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
    float temperatureC = (temperatureV - 0.5) * 100;

    //TODO: Remove
    Serial.print(" | Temperature Celcius: ");
    Serial.print(temperatureC);

    // Convert Celcius to Fahrenheit
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    return temperatureF;
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
    //TODO: Don't read and store values by default? Need "mode" to read values from EEPROM as CSV. Need some kind of switch (digital input)?
    if(receivedADCValue)
    {
        // Reset flag denoting whether an ADC value was received or not
        receivedADCValue = false;

        // Calculate the temperature (Fahrenheit)
        float temperatureF = calcTemperature();

        //TODO: Remove
        Serial.print(" | Temperature Fahrenheit: ");
        Serial.println(temperatureF);

        //TODO: Store value in EEPROM
    }
}

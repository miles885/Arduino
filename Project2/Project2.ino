/*
  Project 2 - Serial Transmit of Temperature
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program reads temperature data from a sensor at a periodic rate
  of 10 seconds. The temperature is converted to fahrenheit and stored
  on the Arduino so that it can be transmitted over Serial (USB) to a
  host machine at a later time.
*/

#include <EEPROM.h>

/* Constants */
const int TEMPERATURE_SENSOR_PIN = A0;
const int WRITE_PIN = 3;
const int READ_PIN = 2;

const int EEPROM_ADDRESS_SIZE = sizeof(int);
const float EEPROM_SCALE = 100.0;

const long INTERRUPT_TIME_THRESHOLD = 250;  // Milliseconds

/* Temperature variables */
static bool receivedADCValue = false;
static bool usingADCValueA = false;

static int adcValueA = 0;
static int adcValueB = 0;

static int currEepromAddr = EEPROM_ADDRESS_SIZE;

/* Interrupt variables */
static unsigned int timerCount = 0;

static bool isWriting = false;
static bool isReading = false;

static bool resetProgramState = false;

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
    pinMode(WRITE_PIN, INPUT);
    pinMode(READ_PIN, INPUT);
    
    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);

    // -----------------------------------------------------------------------------------------
    // Timer Terminology
    // -----------------------------------------------------------------------------------------
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

    // Configure TIMER1 to interrupt at 1 Hz
    // NOTE: Calculate the compare match register using the formula:
    //       cmp = (16,000,000 / (preScaler * interruptFreq)) - 1
    //
    //       As such, Arduino Timer1 cannot count at 0.1 Hz (10 seconds) because the
    //       max prescale value is 1024, resulting in a compare value of ‭156,249‬, which
    //       exceeds the max value of 65,536 that the 16 bit Timer1 register can hold
    //
    //       Source: https://www.instructables.com/id/Arduino-Timer-Interrupts/
    noInterrupts();
    
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 15624;  // Compare register value: (16 MHz / (1024 * 1)) - 1
    TCCR1B |= (1 << WGM12);  // Turn on CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);  // 0x5 = 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);  // Enable compare interrupt

    // Attach read and write pins to interrupts triggered by a rising edge
    attachInterrupt(digitalPinToInterrupt(WRITE_PIN), writePinISR, RISING);
    attachInterrupt(digitalPinToInterrupt(READ_PIN), readPinISR, RISING);
    
    interrupts();
}

/**
 * Interrupt service routine that reads the ADC value from the sensor pin
 * NOTE: This ISR is called at 1 Hz by TIMER1
 * 
 * @param None
 * 
 * @return None
 */
ISR(TIMER1_COMPA_vect)
{
    // Check to see if 10 seconds have passed
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
 * Interrupt service routine that reads the input pin for the write switch
 * 
 * @param None
 * 
 * @return None
 */
 void writePinISR()
 {
    // Retrieve the current time in milliseconds
    static unsigned long lastInterruptTime = 0;
    unsigned long currTime = millis();

    // Check to see if enough time has passed since the last interrupt (debounce)
    if((currTime - lastInterruptTime) > INTERRUPT_TIME_THRESHOLD)
    { 
        isWriting = true;
        isReading = false;
        
        resetProgramState = true;

        lastInterruptTime = currTime;
    }
 }

/**
 * Interrupt service routine that reads the input pin for the read switch
 * 
 * @param None
 * 
 * @return None
 */
void readPinISR()
{
    // Retrieve the current time in milliseconds
    static unsigned long lastInterruptTime = 0;
    unsigned long currTime = millis();

    // Check to see if enough time has passed since the last interrupt (debounce)
    if((currTime - lastInterruptTime) > INTERRUPT_TIME_THRESHOLD)
    { 
        isWriting = false;
        isReading = true;
        
        resetProgramState = true;

        lastInterruptTime = currTime;
    }
}

/**
 * Resets the program state
 * 
 * @param None
 * 
 * @return None
 */
void resetProgram()
{
    receivedADCValue = false;
    usingADCValueA = false;
    adcValueA = 0;
    adcValueB = 0;
    
    currEepromAddr = EEPROM_ADDRESS_SIZE;

    timerCount = 0;
}

/**
 * Calculates the temperature (Fahrenheit) from the ADC value, and then
 * stores the temperature in EEPROM so it can be read at a later time.
 * 
 * @param None
 * 
 * @return None
 */
float calcAndStoreTemperature()
{
    /******************************************************************************************
     * Calculate temperature
     ******************************************************************************************/
    // Copy the most recent ADC value
    int adcValue = usingADCValueA ? adcValueA : adcValueB;

    // Convert temperature value to voltage
    // NOTE: The Arduino UNO ADC pins are 10 bits in size, thus the ADC value range is 0-1023
    float temperatureV = (adcValue / 1024.0) * 5.0;

    // Convert temperature voltage to Celcius
    // NOTE: The TMP36 sensor has a resolution of 10 mV per degree centigrade with a temperature range
    //       of -50C to 125C. The forumla for converting from mV to centigrade is: C = (mV - 500) / 10,
    //       where a 500 mV offset is used to allow for negative temperatures
    //
    //       Source: https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
    float temperatureC = (temperatureV - 0.5) * 100;

    // Convert Celcius to Fahrenheit
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    /******************************************************************************************
     * Store temperature
     ******************************************************************************************/
    // Scale the temperature so that it fits inside an integer value (2 bytes instead of 4 bytes)
    int scaledTemperatureF = (int) (temperatureF * EEPROM_SCALE);

    // Store the scaled temperature in EEPROM
    EEPROM.put(currEepromAddr, scaledTemperatureF);
    EEPROM.put(0, currEepromAddr / EEPROM_ADDRESS_SIZE);

    currEepromAddr += EEPROM_ADDRESS_SIZE;
}

/**
 * Writes the temperature values stored in EEPROM to Serial
 * 
 * @param None
 * 
 * @return None
 */
void writeTemperaturesToSerial()
{
    Serial.println("Time,Temperature");
    
    int numTemperatureValues = 0;
    EEPROM.get(0, numTemperatureValues);

    for(int i = 0; i < numTemperatureValues; i++)
    {
        // Calculate the EEPROM address
        int eepromAddr = (i + 1) * EEPROM_ADDRESS_SIZE;
        
        // Retrieve the EEPROM value
        int eepromValue = 0;
        EEPROM.get(eepromAddr, eepromValue);

        // Scale the temperature so it's a floating point number again
        float temperatureF = ((float) eepromValue) / EEPROM_SCALE;

        // Write the time and temperature as CSV
        Serial.print(i * 10);
        Serial.print(",");
        Serial.println(temperatureF);
    }
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
    static int eepromSizeBytes = EEPROM.length();

    // Check to see if the program state needs to be reset
    if(resetProgramState)
    {
        resetProgramState = false;

        /* Begin Critical Section */
        noInterrupts();
        
        resetProgram();

        interrupts();
        /* End Critical Section */
    }
    // Check to see if in writing mode, have received a temperature value, and the EEPROM is not full
    else if(isWriting && receivedADCValue && (currEepromAddr < eepromSizeBytes))
    {
        // Reset flag denoting whether an ADC value was received or not
        receivedADCValue = false;

        // Calculate and store the temperature in EEPROM
        calcAndStoreTemperature();
    }
    // Check to see if in reading mode
    else if(isReading)
    {
        isReading = false;
        
        writeTemperaturesToSerial();
    }
}

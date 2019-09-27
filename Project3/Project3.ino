/*
  Project 3 - Measurement and transmission of propeller speed
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program measures the RPMs of a propeller over time and transmits
  the RPM data over Serial (USB).
*/

/* Constants */
const int POT_PIN = A0;

const int IR_RECEIVER_PIN = 2;
const int MOTOR_PIN = 5;
const int IR_TRANSMITTER_PIN = 7;
const int LED_STATUS_PIN = 12;

const unsigned int POT_READ_PERIOD = 100;  // ms
const unsigned int RPM_READ_PERIOD = 1000;  // ms

/* Global variables */
volatile int ledStatus;
unsigned int rpmCounter;

/**
 * Initialization that gets run when you press reset or power the board
 * 
 * @params None
 * 
 * @return None
 */
void setup()
{
    // Set global variables
    ledStatus = LOW;
    rpmCounter = 0;
    
    // Initialize pins
    pinMode(IR_RECEIVER_PIN, INPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(IR_TRANSMITTER_PIN, OUTPUT);
    pinMode(LED_STATUS_PIN, OUTPUT);

    digitalWrite(IR_TRANSMITTER_PIN, HIGH);
    
    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);

    // Setup interrupts
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), irRecvISR, FALLING);
}

/**
 * Interrupt service routine that is triggered when the IR
 * detector goes from HIGH to LOW
 * 
 * @param None
 * 
 * @return None
 */
void irRecvISR()
{
    // Swap the LED light
    ledStatus = (ledStatus == HIGH ? LOW : HIGH);
    digitalWrite(LED_STATUS_PIN, ledStatus);

    // Increment the RPM counter
    rpmCounter++;
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
    //TODO: Fix ISR triggering at startup (does VCC cause pin to go high at board startup, and then when IR Emitter starts transmitting it goes low?
    //TODO: Fix LED status light blinking when it shouldn't - need higher pull up resistor? Check school discussion thread

    // Source: http://arduinoprojects101.com/arduino-rpm-counter-tachometer/

    static unsigned long prevPotTime = 0;
    static unsigned long prevRPMTime = 0;

    noInterrupts();

    unsigned long currTime = millis();

    // Check to see if the potentiometer value should be checked
    if((currTime - prevPotTime) >= POT_READ_PERIOD)
    {
        // Read the potentiometer value (10 bits) and map it to the PWM range (8 bits)
        int pinValue = analogRead(POT_PIN);
        int motorDutyValue = map(pinValue, 0, 1023, 0, 255);

        // Write the PWM value to the motor pin (transistor that closes the motor circuit)
        analogWrite(MOTOR_PIN, motorDutyValue);

        // Update POT time
        prevPotTime = currTime;
    }

    // Check to see if the RPM value should be calculated and transmitted
    if(currTime - prevRPMTime >= RPM_READ_PERIOD)
    {
        // Multiply by 30 instead of 60 because the propeller will trigger the interrupt twice per revolution
        unsigned int currRPM = 30 * rpmCounter * (1000 / (currTime - prevRPMTime));

        Serial.println(currRPM);

        // Reset RPM counter and update RPM time
        rpmCounter = 0;
        prevRPMTime = currTime;
    }

    interrupts();
}

/*
  Project 3 - Measurement and transmission of propeller speed
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program measures the RPMs of a propeller over time and transmits
  the RPM data over Serial (USB).
*/

/* Constants */
const int IR_RECEIVER_PIN = 2;
const int MOTOR_PIN = 5;
const int IR_TRANSMITTER_PIN = 7;
const int LED_STATUS_PIN = 12;

/* LED status variables */
volatile int ledStatus;

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
    ledStatus = (ledStatus == HIGH ? LOW : HIGH);
    digitalWrite(LED_STATUS_PIN, ledStatus);

    //TODO: Increment RPM counter
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
    //TODO: Calculate RPM every second and send to serial (disable interrupts while doing so - shared data)
    //TODO: Fix ISR triggering at startup (does VCC cause pin to go high at board startup, and then when IR Emitter starts transmitting it goes low? 
    //      NOTE: When IR is blocked pin goes high
    //TODO: Configure some kind of switch (potentiometer?) to control DC motor speed with pulse width modulation
}

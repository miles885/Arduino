/*
  Project 1 - Morse Code LED or LCD System Development
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program displays a user typed string as morse code using an LED.
  The program will prompt the user for input indefinitely unless a
  sentinel signal is received (e.g. Ctrl+Z).
*/

/* Constants */
const int OUTPUT_PIN = 13;

const int ETX = 3;  // CTRL + C
const int LF = 10;
const int CR = 13;
const int SUB = 26;  // CTRL + Z

int SENTINELS[] = {ETX, SUB};

const int SERIAL_BUFFER_SIZE = 64;
const int INPUT_BUFFER_SIZE = SERIAL_BUFFER_SIZE * 10;

const int MORSE_CODE_TIME_UNIT = 1000;  // Milliseconds

char * MORSE_CODE_CHARS[] = {".-",     // A 
                             "-...",   // B
                             "-.-.",   // C
                             "-..",    // D
                             ".",      // E
                             "..-.",   // F
                             "--.",    // G
                             "....",   // H
                             "..",     // I
                             ".---",   // J
                             "-.-",    // K
                             ".-..",   // L
                             "--",     // M
                             "-.",     // N
                             "---",    // O
                             ".--.",   // P
                             "--.-",   // Q
                             ".-.",    // R
                             "...",    // S
                             "-",      // T
                             "..-",    // U
                             "...-",   // V
                             ".--",    // W
                             "-..-",   // X
                             "-.--",   // Y
                             "--.."};  // Z

char * MORSE_CODE_NUMS[] = {"-----",   // 0
                            ".----",   // 1
                            "..---",   // 2
                            "...--",   // 3
                            "....-",   // 4
                            ".....",   // 5
                            "-....",   // 6
                            "--...",   // 7
                            "---..",   // 8
                            "----."};  // 9

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
    pinMode(LED_PIN, OUTPUT);

    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);
}

/**
 * Retrieves user input from the serial port
 * 
 * @param userInput Character buffer to store the user input data
 * 
 * @return Flag denoting whether all user input has been retrieved or not
 */
bool getUserInput(char * userInput)
{
    static int charIndex = 0;
  
    while(Serial.available())
    {
        // Read a character from the serial buffer
        char serialInput = Serial.read();

        // Check to see if all user input has been retrieved or input buffer is full
        if(serialInput == LF || serialInput == CR || charIndex == INPUT_BUFFER_SIZE)
        {
            charIndex = 0;
      
            return true;
        }

        // Check to see if a sentinel was passed in
        int numSentinels = sizeof(SENTINELS) / sizeof(int);

        for(int i = 0; i < numSentinels; i++)
        {
            if(serialInput == SENTINELS[i])
            {
                Serial.println("Received sentinel - ending program!");

                //TODO: Need a system.exit(0) or something
                break;
            }
        }

        userInput[charIndex] = serialInput;
        charIndex++;
    }

    return false;
}

/**
 * Outputs the user input as morse code using the output pin
 * 
 * @param userInput Character buffer storing the user input data
 * 
 * @return None
 */
void outputMorseCode(char * userInput)
{
    for(int i = 0; i < INPUT_BUFFER_SIZE; i++)
    {
        // Check to see if end of string has been reached
        if(userInput[i] == NULL)
        {
            return;
        }

        // Check to see if the character is upper case
        if(userInput[i] >= 'A' && userInput[i] <= 'Z')
        {
            
        }
        // Check to see if the character is lower case
        else if(userInput[i] >= 'A' && userInput[i] <= 'Z')
        {
            
        }
        // Check to see if the character is a number
        else if(userInput[i] >= '0' && userInput[i] <= '9')
        {
            
        }
        // Check to see if the character is a space
        else if(userInput[i] == ' ')
        {
            
        }
    }
    //digitalWrite(OUTPUT_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //delay(1000);                       // wait for a second
    //digitalWrite(OUTPUT_PIN, LOW);    // turn the LED off by making the voltage LOW
    //delay(1000);                       // wait for a second
}

/**
 * 
 */
void outputSequence(char * codeSequence)
{
    
}

/**
 * Flushes the serial buffer
 * 
 * @param None
 * 
 * @return None
 */
void flushSerialBuffer()
{
    while(Serial.available())
    {
        Serial.read();
    }
}

/**
 * Runs indefinitely unless a sentinel signal is received
 * 
 * @param None
 * 
 * @return None
 */
void loop()
{
    static char userInput[INPUT_BUFFER_SIZE + 1];
    static bool needNewString = true;
  
    // Prompt the user to enter a string
    if(needNewString == true)
    {
        Serial.println("Please type something to output as morse code (max " + String(INPUT_BUFFER_SIZE) + " characters).");

        memset(userInput, 0, sizeof(userInput));
        needNewString = false;
    }

    // Retrieve the user input
    if(Serial.available())
    {
        bool hasAllUserInput = getUserInput(userInput);

        if(hasAllUserInput)
        {
            needNewString = true;
            
            outputMorseCode();
            flushSerialBuffer();
        }
    }
}

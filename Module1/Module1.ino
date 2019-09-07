/*
  Project 1 - Morse Code LED or LCD System Development
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program displays a user typed string as Morse code using an LED.
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

const int MORSE_CODE_TIME_UNIT = 500;  // Milliseconds

char * MORSE_CODE_CHAR_SEQUENCE[] = {".-",     // A 
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

char * MORSE_CODE_NUM_SEQUENCE[] = {"-----",   // 0
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
    pinMode(OUTPUT_PIN, OUTPUT);

    // Setup up the serial connection at 9600 bits per second
    Serial.begin(9600);
}

/**
 * Retrieves and buffers user input from the serial port
 * 
 * @param userInput Character buffer to store the user input data
 * 
 * @return Flag denoting whether all user input has been retrieved or not
 */
bool bufferUserInput(char * userInput)
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
                Serial.println("Received sentinel - ending program! Reset the device to run this program again.");
                delay(2000);

                exit(0);
            }
        }

        userInput[charIndex] = serialInput;
        charIndex++;
    }

    return false;
}

/**
 * Outputs the user input as Morse code
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

        // Check to see if the character is an upper case letter
        if(userInput[i] >= 'A' && userInput[i] <= 'Z')
        {
            outputMorseCodeSequence(MORSE_CODE_CHAR_SEQUENCE[userInput[i] - 'A']);
        }
        // Check to see if the character is a lower case letter
        else if(userInput[i] >= 'a' && userInput[i] <= 'z')
        {
            outputMorseCodeSequence(MORSE_CODE_CHAR_SEQUENCE[userInput[i] - 'a']);
        }
        // Check to see if the character is a number
        else if(userInput[i] >= '0' && userInput[i] <= '9')
        {
            outputMorseCodeSequence(MORSE_CODE_NUM_SEQUENCE[userInput[i] - '0']);
        }
        // Check to see if the character is a space
        else if(userInput[i] == ' ')
        {
            // NOTE: Delay an additional 4 time units because there is a time
            //       delay of 7 time units between words and there has already
            //       been a delay of 3 time units from outputCodeSequence
            delay(MORSE_CODE_TIME_UNIT * 4);
        }
    }
}

/**
 * Outputs the Morse code sequence using the output pin
 * 
 * @param codeSequence The Morse code sequence to be output
 * 
 * @return None
 */
void outputMorseCodeSequence(char * codeSequence)
{
    int i = 0;
    
    while(codeSequence[i] != NULL)
    {
        int signalDur = MORSE_CODE_TIME_UNIT;
        
        if(codeSequence[i] == '-')
        {
            signalDur = MORSE_CODE_TIME_UNIT * 3;
        }

        // Set output pin to high for signal duration
        digitalWrite(OUTPUT_PIN, HIGH);
        delay(signalDur);

        // Set output pin to low for time between sequence characters
        digitalWrite(OUTPUT_PIN, LOW);
        delay(MORSE_CODE_TIME_UNIT);
        
        i++;
    }

    // NOTE: Delay an additional 2 time units because there is a time
    //       delay of 3 time units between characters
    delay(MORSE_CODE_TIME_UNIT * 2);
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
        Serial.println("Please type something to output as Morse code (max " + String(INPUT_BUFFER_SIZE) + " characters).");

        memset(userInput, 0, sizeof(userInput));
        needNewString = false;
    }

    // Retrieve the user input
    if(Serial.available())
    {
        bool hasAllUserInput = bufferUserInput(userInput);

        if(hasAllUserInput)
        {
            needNewString = true;
            
            outputMorseCode(userInput);
            flushSerialBuffer();
        }
    }
}

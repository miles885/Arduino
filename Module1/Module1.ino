/*
  Project 1 - Morse Code LED or LCD System Development
  Author: Miles Gapcynski
  Class: EN.605.715.81 - Software Development for Real-Time Systems

  This program displays a user typed string as morse code using an LED.
  The program will prompt the user for input indefinitely unless a
  sentinel signal is received (e.g. Ctrl+Z).
*/

/* Constants */
const int LED_PIN = 13;

const int ETX = 3;  // CTRL + C
const int LF = 10;
const int CR = 13;
const int SUB = 26;  // CTRL + Z

int SENTINELS[] = {ETX, SUB};

const int MAX_SERIAL_BUFFER_SIZE = 64;

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
 * @param userInput The character buffer to store the user input into
 * 
 * @return Flag denoting whether the user string is complete or not
 */
bool getUserInput(String & userInput)
{
  static int charIndex = 0;
  
  while(Serial.available())
  {
    char serialInput = Serial.read();

    // Check to see if reached the end of the string
    if(serialInput == LF || serialInput == CR)
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

    userInput += serialInput;
    charIndex++;

    // Check to see if end of serial buffer has been reached
    if(charIndex == MAX_SERIAL_BUFFER_SIZE)
    {
      break;
    }
  }

  return false;
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
  //TODO: Use large char buffer to cap size and improve speed? See old commit
  static String userInput("");
  static bool needNewString = true;
  
  // Prompt the user to enter a string
  if(needNewString == true)
  {
    Serial.println("Please type something and press enter to output as morse code...");

    userInput = "";
    needNewString = false;
  }

  // Retrieve the user input
  if(Serial.available())
  {
    bool hasAllUserInput = getUserInput(userInput);

    if(hasAllUserInput)
    {
      Serial.println("Your user input:");
      Serial.println(userInput);
      Serial.println("Now has all user input data!");
    }

    //digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //delay(1000);                       // wait for a second
    //digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
    //delay(1000);                       // wait for a second
  
    if(hasAllUserInput)
    {
      needNewString = true;
    }
  }
}

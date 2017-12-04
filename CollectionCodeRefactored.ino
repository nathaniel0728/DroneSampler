// Pins

int 
         motorPin = 12,   // Controls motor
           brkPin = 9,    // Controls break
         powerPin = 3,    // Powers motor
           ledPin = 6,    // Controls LED
        switchPin = 7,    // Detects switch state
      pressurePin = 1,    // Pressure pin
       pixhawkPin = 2;        // Detects drone signal

// Times and Threshold

int
        fullPower = 255,  // Maximum power level
           tLower = 16000, // Lower container time
         tCollect = 4000, // Collection time
             tLed = 500,  // LED on time
        fsrBuffer = 250,  // Pressure Sensor Buffer
pressureThreshold = 30,   // Pressure sensor threshold upon press
        fsrReading;

// Initializations

boolean  
         previous = LOW, // Previous switch state
          current = LOW, // Current switch state
            motor = LOW, // Motor direction
              led = LOW; // LED state


void lowerContainer();
boolean signalCollect();
void raiseContainer();
void ledSignal();
void triggerBreak(String val);
void motorDirection(String dir);
boolean debounce(boolean prev);

void setup() 
{
  /**
   * Initial OUTPUT Setup: 
   *  Create connections to motor, break mechanism (to prevent roll),
   *  and LED to display status. 
   */
  pinMode( motorPin, OUTPUT);
  pinMode(   brkPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  /**
   * Initial INPUT Setup:
   *  Create connections to switch (to initiate collection process), 
   *  and pressure sensor.
   */
  pinMode(switchPin, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(powerPin, INPUT);
  pinMode(pixhawkPin, INPUT);

  /**
   * Connect to Serial port 9600 for debugging
   */
  Serial.begin(9600);
}



void loop() 
{
  fsrReading = analogRead(pressurePin);

  int testValuePixHawk = analogRead(pixhawkPin);
  delay(10);
  Serial.println("Value = " + testValuePixHawk);
  
  // Keep the break on
  triggerBreak("ON");
    
  // Read the state of the switch
  current = debounce(previous);


  // Perform the collection process if the switch is pressed
  boolean sC = signalCollect();
  if (sC)
  {
    // Lower the container
    lowerContainer();

    triggerBreak("ON");
    delay(tCollect);
    
    // Raise the container
    raiseContainer();
    
    // Break the motor
    triggerBreak("ON");
    
    // Light up the LED
    ledSignal();
  }
  
  previous = current;
}

// Helper Methods

/**
 * 
 * Raises Container for a certain time (tCollect).
 * Changes motor direction to FORWARD and turns off break.
 * 
 * @param none
 * @return void
 */
void lowerContainer()
{
  motorDirection("forward");
  triggerBreak("OFF");
  analogWrite(powerPin, fullPower);

  delay(tLower);
}

/**
 * 
 * Returns signal that indicates the start of collection.
 * Several options available:
 *  Option 1: 
 *    Returns true when switchPin is activated (not present in final prototype)
 *  Option 2:
 *    Established connection to Pixhawk microcontroller, returns signal
 *    from pilot
 *  
 */
boolean signalCollect()
{
  // Option 1
  // return current == HIGH && previous == LOW;

  // Option 2
  // return digitalRead(pixhawkPin) == HIGH);

  // Option 3
   return analogRead(pixhawkPin) > 500;
    
  
}

/**
 * 
 * Raises Container based on fsrReading, a value INPUT from pressurePin.
 * The pressureThreshold is determined experimentally, accounting for
 *  surrounding noise and resistivity value. 
 * In order to account for background noise (on pressure sensor), an 
 *  additional reading is taken (fsrCheck), and a buffer threshold is
 *  present (fsrBuffer).
 * Before the fsrReading < pressureThreshold (before pressure detected),
 *  the motor's direction is switched to REVERSE and break is turned off.
 * 
 * @param none
 * @return void
 * 
 */
void raiseContainer()
{
  int fsrCheck = 0;
  while(fsrReading < pressureThreshold && fsrCheck < pressureThreshold)
  {
    motorDirection("reverse");
    triggerBreak("OFF");  
    analogWrite(powerPin, fullPower);
    
    fsrReading = analogRead(pressurePin);
    delay(fsrBuffer);
    fsrCheck = analogRead(pressurePin);
  }
}

/**
 * 
 * Turns on LED upon finishing the water collection.
 * ledSignal is mainly used for debugging purposes, and is not
 *  present in the final prototype.
 *  
 *  @param none
 *  @return void
 * 
 */
void ledSignal()
{
  led = HIGH;
  digitalWrite(  ledPin, led);
  delay(tLed);  
  led = LOW;
  digitalWrite(  ledPin, led);
}

/**
 * 
 * Activates the break present in motor to prevent slip.
 * By default, the break is turned ON.
 * 
 * @param String
 *  val: "ON" or "OFF" to turn break ON and OFF resp.
 * @return void
 * 
 */
void triggerBreak(String val)
{
  if(val.equals("ON"))
    digitalWrite(brkPin, HIGH);
  else if(val.equals("OFF"))
    digitalWrite(brkPin, LOW);
  else
    digitalWrite(brkPin, HIGH);
}

/**
 * 
 * Changes motor direction based on provided value.
 * 
 * @param String
 *  dir: "reverse" or "forward" to turn clockwise or counterclockwise
 *    resp.
 * @return void
 * 
 */
void motorDirection(String dir)
{
  if(dir.equals("reverse"))
  {
    digitalWrite(motorPin, HIGH);
  }
  else if(dir.equals("forward"))
  {
    digitalWrite(motorPin, LOW);
  }
}

/** 
 *  Avoid the effect of switch bouncing by giving a second
 *    detection after a 10 ms delay.
 *  
 *  @param boolean
 *    prev: checks button's state
 *  @return boolean
 *    curr: current button's state
 */
boolean debounce(boolean prev)
{
  boolean curr = digitalRead(switchPin);
  
  if (prev != curr)
  {
    delay(10);
    
    // Read the switch state again
    curr = digitalRead(switchPin);
  }
  
  return curr;
}

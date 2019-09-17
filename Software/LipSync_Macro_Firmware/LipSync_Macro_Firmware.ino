/*
//                                                                                                  
//  +++         .+++:    /++++++++/:.     .:/+++++/: .+++/`     .+++/  ++++.      ++++.     `-/++++++/:
//  oooo         .ooo:    +ooo:--:+ooo/   :ooo/:::/+/  -ooo+`   .ooo+`  ooooo:     .o-o`   `/ooo+//://+:
//  oooo         .ooo:    +ooo`    :ooo-  oooo`     `   .ooo+` .ooo+`   oooooo/`   .o-o`  .oooo-`       
//  oooo         .ooo:    +ooo`    -ooo-  -ooo+:.`       .ooo+.ooo/`    ooo:/oo+.  .o-o`  +ooo.         
//  oooo         .ooo:    +ooo.`..:ooo+`   `:+oooo+:`     `+ooooo/      ooo: :ooo- .o-o`  oooo          
//  oooo         .ooo:    +ooooooooo+:`       `-:oooo-     `+ooo/       ooo/  .+oo/.o-o`  +ooo.         
//  oooo         .ooo:    +ooo-...``             `oooo      /ooo.       ooo/   `/oo-o-o`  .oooo-        
//  oooo::::::.  .ooo:    +ooo`           :o//:::+ooo:      /ooo.       ooo/     .o-o-o`   ./oooo/:::/+/
//  +ooooooooo:  .ooo:    /ooo`           -/++ooo+/:.       :ooo.       ooo:      `.o.+      `-/+oooo+/-
//
//  ++    ++       +        ++++    ++++++     +++
//  ob    do      db       dP""bo   oo""Yb    dP"oo  
//  oob  doo     dPoo     dP   `"   oo__dP   dP   oo 
//  oooodPoo    dP__oo    oo        oo"oo    oo   dP 
//  oo YY oo   dP""""oo    oooodP   oo  oo    ooodP  
//
//An assistive technology device which is developed to allow quadriplegics to use touchscreen mobile devices by manipulation of a mouth-operated joystick with integrated sip and puff controls.
*/

//Developed BY : MakersMakingChange
//Firmware : LipSync_Macro_Firmware
//VERSION : 1.0 (16 September 2019)

#include <EEPROM.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define BUTTON_UP_PIN 8                           // Switch Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define BUTTON_DOWN_PIN 7                         // Switch Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1_PIN 4                               // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL_PIN A3                      // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Bluetooth PIO4_PIN Command Pin - digital output pin A4

#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***CUSTOMIZE VALUES***//

#define PRESSURE_THRESHOLD 0.5                    //Pressure sip and puff threshold 
#define DEBUG_MODE true                           //Debug mode ( Enabled = true and Disabled = false )
#define BT_CONFIG_FLAG false                      //Configure bluetooth ( Configure = true and Not Configure = false ). This is used to reset bluetooth module
#define FIXED_DELAY 20                            //Increase this value to slow down the reaction time

//***VARIABLE DECLARATION***//

int xHigh, yHigh, xLow, yLow;  

//Declare variables for speed functionality 
int switchSpeedCounter = 5;  
int switchDelay;

int operationMode = 0;                            // Switch between 2 iOS modes
int bluetoothConfigDone;                          // Binary check of completed Bluetooth configuration
int pollCounter = 0;                              //Switch poll counter

float sipThreshold;                               //Declare sip and puff variables 
float puffThreshold;
float switchActivate;

unsigned int puffCount;
unsigned int sipCount;

bool settingsEnabled = false; 

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // Set baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // Set baud rate for Bluetooth module

  pinMode(LED_1_PIN, OUTPUT);                     //Set the LED pin 1 as output(GREEN LED)
  pinMode(LED_2_PIN, OUTPUT);                     //Set the LED pin 2 as output(RED LED)
  pinMode(TRANS_CONTROL_PIN, OUTPUT);             //Set the transistor pin as output
  pinMode(PIO4_PIN, OUTPUT);                      //Set the bluetooth command mode pin as output

  pinMode(PRESSURE_PIN, INPUT);                   //Set the pressure sensor pin input
  pinMode(X_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Right FSR )
  pinMode(X_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Left FSR )
  pinMode(Y_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Up FSR )
  pinMode(Y_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Down FSR )
  
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);           //Set the increase switch speed pin to input mode with pullup
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);         //Set the decrease switch speed pin to input mode with pullup

  pinMode(2, INPUT_PULLUP);                       //Set the unused pins to input mode with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);


  delay(10);
  
  while(!Serial1);
  
  pressureSensorInitialization();                 //Initialize the pressure sensor
  delay(10);
  readSwitchSpeed();                              // Reads saved switch speed parameter from EEPROM
  delay(10);
  operationModeValue();                           // Read saved operation mode parameter from EEPROM
  delay(10);
  
  int execTime = millis();
  Serial.print("Configuration time: ");
  Serial.println(execTime);

  ledBlink(4, 250, 3);                            // End the initialization visual feedback
  
  displayVersion();                               //Display firmware version number
  
  bluetoothConfigure(); 

  calculateSwitchDelay();                         //Calculate switch action delay

  //Functions below are for diagnostic feedback only
  if(DEBUG_MODE==true) {
    Serial.print("Speed level: ");
    Serial.println(switchSpeedCounter+1);
    delay(5);
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {

  settingsEnabled=serialSettings(settingsEnabled);    //Check to see if setting option is enabled in Lipsync
  
  xHigh = analogRead(X_DIR_HIGH_PIN);                 //Read analog values of FSR's : A0
  xLow = analogRead(X_DIR_LOW_PIN);                   //Read analog values of FSR's : A1
  yHigh = analogRead(Y_DIR_HIGH_PIN);                 //Read analog values of FSR's : A0
  yLow = analogRead(Y_DIR_LOW_PIN);                   //Read analog values of FSR's : A10

  xHigh = map(xHigh, 0, 1023, 0, 16);                 //Map x and y values from (0 to 1023) bound to (0 to 16) as target bound
  xLow = map(xLow, 0, 1023, 0, 16);                   //The actual input values are approximately in (0 to 800) bound range
  yHigh = map(yHigh, 0, 1023, 0, 16);
  yLow = map(yLow, 0, 1023, 0, 16);

  int xDelta = xHigh - xLow;                          //Calculate the x and y delta values   
  int yDelta = yHigh - yLow;
 
  int xx = (xDelta >= 0)? sq(xDelta):-sq(xDelta);     //Square the magnitude of x and y Delta values
  int yy = (yDelta >= 0)? sq(yDelta):-sq(yDelta);
  
  xx -= (xDelta >= 0)? int(sqrt(yDelta)):-int(sqrt(-yDelta));   //Subtract the square root of y Delta value from x Delta value to make movement smoother 
  yy -= (yDelta >= 0)? int(sqrt(xDelta)):-int(sqrt(-xDelta));   //Subtract the square root of x Delta value from y Delta value to make movement smoother 

  xx = constrain(xx, -128, 128);                      //Put constrain to set x and y range between -128 and 128 as lower and upper bounds 
  yy = constrain(yy, -128, 128);
  
  xx = map(xx, -128, 128, -10, 10);                   //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yy = map(yy, -128, 128, -10, 10);


  if (((abs(xx)) > 0) || ((abs(yy)) > 0)) {
   pollCounter++;
   delay(15);
      if (pollCounter >= 10) {
          if ((xx >= 5) && (-5 < yy < 5) && ((abs(xx)) > (abs(yy)))) {
         if (operationMode == 0) {
              //Right arrow key
              bluetoothKeyboardCommand((byte)0x00,byte(0x4F));
            }
            else {
              //Move keyboard cursor to the right direction (Alt/option Key + Right Arrow Key)
              bluetoothKeyboardCommand((1<<2),byte(0x4F));
            }
          } 
          else if ((xx < -5) && (-5 < yy < 5) && ((abs(xx)) > (abs(yy)))){
            //Serial.println("left"); 
            if (operationMode == 0) {
              //Left arrow key
              bluetoothKeyboardCommand((byte)0x00,byte(0x50)); 
            }
            else {
              //Move keyboard cursor to the left (Alt/option key + Left arrow key)
              bluetoothKeyboardCommand((1<<2),byte(0x50)); 
            }            
          }
          else if ((-5 < xx < 5) && (yy < -5) && ((abs(yy)) > (abs(xx)))){
            //Serial.println("Down"); 
         if (operationMode == 0) {
              //Down arrow key
              bluetoothKeyboardCommand((byte)0x00,byte(0x51));
            }
            else {
              //Begin text selection on the left side of keyboard cursor (Shift Key + Left Arrow key)
              bluetoothKeyboardCommand((1<<1),byte(0x50));  
            }            
          }
          else if ((-5 < xx < 5) && (yy > 5) && ((abs(yy)) > (abs(xx)))){
            //Serial.println("Up"); 
         if (operationMode == 0) {
              //Up arrow key
              bluetoothKeyboardCommand((byte)0x00,byte(0x52));
            }
            else {
              //Begin text selection on the right side of keyboard cursor (Shift Key + Right Arrow Key)
              bluetoothKeyboardCommand((1<<1),byte(0x4F));              
            }            
          }    
        delay(switchDelay);       
        pollCounter = 0;
        }

  }
 
  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    } else {
      increaseSwitchSpeed();      // Increase switch speed with push button up
    }
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    delay(250);
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
    } else {
      decreaseSwitchSpeed();      // Decrease switch speed with push button down
    }
  }

   //Pressure sensor sip and puff functions

  switchActivate = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (switchActivate < puffThreshold) {
    while (switchActivate < puffThreshold) {
      switchActivate = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      puffCount++;         //Threshold counter
      delay(5);
    }
      if (puffCount < 150) {
        //Enter or select
        bluetoothKeyboardCommand(byte(0x00),byte(0x28));
      } else if (puffCount > 150 && puffCount < 750) {
        //Dot
        bluetoothKeyboardCommand(byte(0x00),byte(0x37));    
      } 
      delay(switchDelay);
      puffCount = 0;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds
  if (switchActivate > sipThreshold) {
    while (switchActivate > sipThreshold) {
      switchActivate = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      sipCount++;         ///Threshold counter
      delay(5);
    }
      if (sipCount < 150) {
         //Space
        bluetoothKeyboardCommand(byte(0x00),byte(0x2C));  
      } else if (sipCount > 150 && sipCount < 750) {
        //Dash
        bluetoothKeyboardCommand(byte(0x00),byte(0x2D));    
      } else if (sipCount > 750) {
        changeMode();
      }
      delay(switchDelay); 
      sipCount = 0;
  }
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
    
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "settings" is in input string.
       if (settingsFlag==false && inString=="settings") {
       Serial.println("Actions:");                //Display list of possible actions 
       Serial.println("S,(+ or -)");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString.length()==((2*2)-1)){ //Check if the input parameter is true and the received string is 3 characters only
        inString.replace(",","");                 //Remove commas 
        if(inString.length()==2) {                //Perform settings actions if there are only two characters in the string.
          writeSettings(inString);
          Serial.println("Successfully changed.");
        }   
        Serial.println("Exiting the settings.");
        settingsFlag=false;   
       }
       else if (settingsFlag==true){
        Serial.println("Exiting the settings.");
        settingsFlag=false;         
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

void writeSettings(String changeString) {
    char changeChar[changeString.length()+1];
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Increase the cursor speed if received "S+" and decrease the cursor speed if received "S-"
    if(changeChar[0]=='S' && changeChar[1]=='+') {
      increaseSwitchSpeed();
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='-') {
      decreaseSwitchSpeed();
      delay(5);
    } 
}

//***CHANGE MODE FUNCTION***//

void changeMode(void) {
  if (operationMode == 0) {
    operationMode++;
  } else {
    operationMode=0;
  } 
  ledBlink(operationMode+1, 500, 1);
  EEPROM.put(30, operationMode);
  delay(25);
}

//***DISPLAY VERSION FUNCTION***//

void displayVersion(void) {

  Serial.println(" --- ");
  Serial.println("LipSync Macro Firmware Version 1.0 (13 June 2019)");
  Serial.println(" --- ");

}

//***LED ON FUNCTION***//

void ledOn(int ledNumber) {
  switch (ledNumber) {
    case 1: {
        digitalWrite(LED_1_PIN, HIGH);
        delay(5);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
    case 2: {
        digitalWrite(LED_2_PIN, HIGH);
        delay(5);
        digitalWrite(LED_1_PIN, LOW);
        break;
      }
  }
}

//***LED CLEAR FUNCTION***//

void ledClear(void) {
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
}

//***LED BLINK FUNCTIONS***//

void ledBlink(int numBlinks, int delayBlinks, int ledNumber ) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
  }
}

//***OPERATION MODE NUMBER FUNCTIONS***//

void operationModeValue(void) {
  int var;
  EEPROM.get(30, var);
  delay(5);
  operationMode = ((var == 0) || (var == 1)) ? var : 0;
  delay(5);
  EEPROM.put(30, operationMode);
  delay(5);
}

//***READ THE SWITCH SPEED LEVEL FUNCTION***//

void readSwitchSpeed(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  if(var>0){
    switchSpeedCounter = var;
  } 
  else {
    EEPROM.put(2, switchSpeedCounter);
    delay(5);
  }
}

//***CALCULATE JOYSTICK DELAY FUNCTION***//

void calculateSwitchDelay(void) {
  switchDelay = pow(1.6,(11-switchSpeedCounter))*FIXED_DELAY;
  delay(5);
}

//***INCREASE SWITCH SPEED FUNCTION***//

void increaseSwitchSpeed(void) {
  switchSpeedCounter++;

  if (switchSpeedCounter == 11) {
    ledBlink(6, 50, 3);
    switchSpeedCounter = 10;
  } else {
    ledBlink(switchSpeedCounter+1, 100, 1);
    switchDelay = pow(1.6,(11-switchSpeedCounter))*FIXED_DELAY;
    EEPROM.put(2, switchSpeedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println(switchSpeedCounter+1);
}

//***DECREASE JOYSTICK SPEED FUNCTION***//

void decreaseSwitchSpeed(void) {
  switchSpeedCounter--;

  if (switchSpeedCounter == -1) {
    ledBlink(6, 50, 3);     // twelve very fast blinks
    switchSpeedCounter = 0;
  } else if (switchSpeedCounter == 0) {
    ledBlink(1, 350, 1);
    switchDelay = pow(1.6,(11-switchSpeedCounter))*FIXED_DELAY;
    EEPROM.put(2, switchSpeedCounter);
    delay(25);
  } else {
    ledBlink(switchSpeedCounter, 100, 1);
    switchDelay = pow(1.6,(11-switchSpeedCounter))*FIXED_DELAY;
    EEPROM.put(2, switchSpeedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println(switchSpeedCounter+1);
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  float nominalPressure = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]

  sipThreshold = nominalPressure + PRESSURE_THRESHOLD;          //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation

  puffThreshold = nominalPressure - PRESSURE_THRESHOLD;         //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
}

//***BLUETOOTH HID KEYBOARD FUNCTION***//

void bluetoothKeyboardCommand(byte modifier,byte button) {

    byte modifierByte=(byte)0x00;
    byte buttonByte=(byte)0x00;
    byte bluetoothKeyboard[5];

    buttonByte=button;
    modifierByte=modifier;

    bluetoothKeyboard[0] = 0xFE;
    bluetoothKeyboard[1] = 0x3;
    bluetoothKeyboard[2] = modifierByte;
    bluetoothKeyboard[3] = buttonByte;
    bluetoothKeyboard[4] = 0x0;

    Serial1.write(bluetoothKeyboard,5);
    Serial1.flush();
    delay(10);
    bluetoothKeyboardClear();

}

//***BLUETOOTH HID KEYBOARD CLEAR FUNCTION***//

void bluetoothKeyboardClear(void) {

  byte bluetoothKeyboard[2];

  bluetoothKeyboard[0] = 0xFD;
  bluetoothKeyboard[1] = 0x00;
  Serial1.write(bluetoothKeyboard,2);
  Serial1.flush();
  delay(10); 
  
}

//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void bluetoothConfigureStatus(void) {
  int eepromVal;                                            // Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(0, eepromVal);                                // Assign value of EEPROM memory at index zero (0) to int variable var
  delay(10);
  bluetoothConfigDone = (eepromVal == 1) ? eepromVal : 0;  //Define the bluetoothConfigDone to 0 if the device is set for the first time
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void bluetoothConfigure(void) {
    bluetoothConfigureStatus();                             //Check if Bluetooth has previously been configured
    delay(10);
    if ((bluetoothConfigDone == 0) || BT_CONFIG_FLAG) {     //If Bluetooth has not been configured or Bluetooth config flag is true then execute configuration sequence
      bluetoothCommandMode();                               //Call Bluetooth command mode function to enter command mode
      bluetoothConfigSequence();                            //Send configuarion data to Bluetooth module
      delay(10);
    } else {
      Serial.println("Bluetooth configuration has previously been completed.");
      delay(10);
    }
}
//***BLUETOOTH CMD MODE FUNCTION***//

void bluetoothCommandMode(void) {                 
  digitalWrite(TRANS_CONTROL_PIN, HIGH);            //Set the transistor base pin to HIGH to ensure Bluetooth module is off
  digitalWrite(PIO4_PIN, HIGH);                     //Set the command pin to high
  delay(10);
  digitalWrite(TRANS_CONTROL_PIN, LOW);             //Set the transistor base pin to LOW to power on Bluetooth module
  delay(10);

  for (int i = 0; i < 3; i++) {                     //Cycle HIGH and LOW the PIO4_PIN pin 3 times with 1 sec delay between each level transition
    digitalWrite(PIO4_PIN, HIGH);
    delay(150);
    digitalWrite(PIO4_PIN, LOW);
    delay(150);
  }

  digitalWrite(PIO4_PIN, LOW);                      //Set the PIO4_PIN pin low as per command mode instructions
  delay(10);
  Serial1.print("$$$");                             //Enter Bluetooth command mode
  delay(50);                                        //Add time delay to visual inspect the red LED is flashing at 10Hz which indicates the Bluetooth module is in Command Mode
  Serial.println("Bluetooth CMD Mode Activated");
}

//***BLUETOOTH CONFIG FUNCTION***//

void bluetoothConfigSequence(void) {

  Serial1.println("ST,255");                        //Turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,2");                          //Set Authentication Value to 2
  delay(15);
  Serial1.println("SX,0");                          //Set Bonding to 0 or disabled
  delay(15);
  Serial1.println("SN,LipSyncMacro");               //Set the name of BT module
  delay(15);
  Serial1.println("SM,6");                          //Set the Pairing mode to auto-connect mode : "SM,6"
  delay(15);
  Serial1.println("SH,0000");                       //Configure device as HID keyboard
  delay(15);
  Serial1.println("S~,6");                          //Activate HID profile
  delay(15);
  Serial1.println("SQ,0");                          //Configure for latency NOT throughput : "SQ,0"
  delay(15);
  Serial1.println("S?,1");                          //Enable the role switch -for better performance of high speed data
  delay(15);
  Serial1.println("R,1");                           //Reboot BT module
  delay(15);

  int val0 = 1;
  int val1 = switchSpeedCounter;

  //Save the default cursor counter value and configuration completed value at EEPROM. This action will happen once when a new device is configured 

  EEPROM.put(0, val0);                              //Save the configuration completed value at EEPROM address location 0
  delay(15);
  EEPROM.put(2, val1);                              //Save the default cursor speed counter value at EEPROM address location 2
  delay(15);
}

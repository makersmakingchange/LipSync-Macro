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
//VERSION : 1.1 (3 July 2020)

#include <EEPROM.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define BUTTON_UP_PIN 8                           // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define BUTTON_DOWN_PIN 7                         // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1_PIN 4                               // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL_PIN A3                      // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Bluetooth PIO4_PIN Command Pin - digital output pin A4

#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***SERIAL SETTINGS VARIABLE***//

#define SERIAL_SETTINGS true

//***CUSTOMIZABLE VARIABLES***//

#define JS_FSR_DEADZONE 60                       //The deadzone for input FSR analog value
#define DEBUG_MODE false
#define RAW_MODE false
#define SENSITIVITY_COUNTER 5
#define PRESSURE_THRESHOLD 10                   //Pressure sip and puff threshold 
#define FIXED_SWITCH_DELAY 20                            //Increase this value to slow down the reaction time


#define ACTION_BUTTON_1 0                       //A1.Short Puff: Enter 
#define ACTION_BUTTON_2 1                       //A2.Short Sip: Space 
#define ACTION_BUTTON_3 2                       //A3.Long Puff: Dot
#define ACTION_BUTTON_4 3                       //A4.Long Sip: Dash
#define ACTION_BUTTON_5 5                       //A5.Very Long Puff: Joystick Switch Home Initialization
#define ACTION_BUTTON_6 4                       //A6.Very Long Sip: a

#define BT_CONFIG_FLAG false                      //Configure bluetooth ( Configure = true and Not Configure = false ). This is used to reset bluetooth module

//***DON'T CHANGE THESE VARIABLES***//

#define BT_CONFIG_NUMBER 1              //Bluetooth Config number for LipSync Macro
#define JS_DELAY 10                              //The fixed delay for each loop action 
#define LONG_PRESS_TIME 2
#define JS_MAPPED_IN_DEADZONE 0.50
#define JS_MAPPED_IN_NEUTRAL 12
#define JS_MAPPED_IN_MAX 16.00
#define JS_OUT_DEAD_ZONE 1
#define JS_OUT_MAX 127
#define JS_OUT_MIN 5

//***VARIABLE DECLARATION***//


//***Map Sip & Puff actions to cursor buttons for mode 1***//
int actionButton[6] = {ACTION_BUTTON_1, ACTION_BUTTON_2, ACTION_BUTTON_3, ACTION_BUTTON_4, ACTION_BUTTON_5, ACTION_BUTTON_6};

int lastButtonState[5];   

typedef struct {                                  //Structure for a degree five polynomial 
  float _equationACoef;
  float _equationBCoef;
  float _equationCCoef;
  float _equationDCoef;
  float _equationECoef;
  float _equationFCoef;
} _equationCoef;

//Initialize the equation coefficient structure for each FSR reading 
_equationCoef xHighEquation = {};
_equationCoef xLowEquation = {};
_equationCoef yHighEquation = {};
_equationCoef yLowEquation = {};

int xHigh, xLow, yHigh, yLow;                                                   //FSR raw values 

int xHighNeutral, xLowNeutral, yHighNeutral, yLowNeutral;                       //Neutral FSR values at the resting position 

int xHighMax, xLowMax, yHighMax, yLowMax;                                       //Max FSR values which are set to the values from EEPROM


//The input to output (x to y) curve equation using degree five polynomial equation for each sensitivity level
_equationCoef levelEquation1 = {0.0004,-0.0041,0.0000,-0.0185,1.8000,0.0000};
_equationCoef levelEquation2 = {0.0002,-0.0021,0.0201,-0.3704,4.3000,0.0000};
_equationCoef levelEquation3 = {-0.0008,0.0314,-0.3565,1.2731,3.6056,0.0000};
_equationCoef levelEquation4 = {0.0001,-0.0005,0.0309,-0.4954,7.2167,0.0000};
_equationCoef levelEquation5 = {-0.0004,0.0175,-0.2145,1.0093,5.1333,0.0000};
_equationCoef levelEquation6 = {0.0000,0.0000,0.0000,0.0000,8.4667,0.0000};
_equationCoef levelEquation7 = {-0.0001,0.0062,-0.125,0.7778,9.3000,0.0000};
_equationCoef levelEquation8 = {-0.0004,0.0195,-0.3133,1.6574,10.6889,0.0000};
_equationCoef levelEquation9 = {0.0001,-0.0010,0.0093,-0.9907,21.2444,0.0000};
_equationCoef levelEquation10 = {0.0008,-0.0303,0.5062,-5.1157,35.5500,0.0000};
_equationCoef levelEquation11 = {-0.0001,-0.0051,0.3441,-6.1204,45.4111,0.0000};


//All sensitivity levels
_equationCoef levelEquations[11] = {levelEquation1, levelEquation2, levelEquation3, levelEquation4, levelEquation5, levelEquation6, levelEquation7, levelEquation8, levelEquation9, levelEquation10, levelEquation11};


int bluetoothConfigDone;                          // Binary check of completed Bluetooth configuration

unsigned int puffCount, sipCount;                 //The puff and long sip incremental counter variables

int pollCounter = 0;                              //Cursor poll counter

int sensitivityCounter; 

bool debugModeEnabled;                                  //Declare raw and debug enable variable
bool rawModeEnabled;

float sipThreshold;                                     //Declare sip and puff variables 
float puffThreshold;
float switchPressure;

int joystickDeadzone;                                   //Declare joystick deadzone variable 

int modelNumber;                                        //Declare LipSync model number variable 

bool settingsEnabled = false;                           //Serial input settings command mode enabled or disabled 

//-----------------------------------------------------------------------------------//

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {
  
  Serial.begin(115200);                           //Setting baud rate for serial communication which is used for diagnostic data returned from Bluetooth and microcontroller
  Serial1.begin(115200);                          //Setting baud rate for Bluetooth AT command 

  pinMode(LED_1_PIN, OUTPUT);                     //Set the LED pin 1 as output(GREEN LED)
  pinMode(LED_2_PIN, OUTPUT);                     //Set the LED pin 2 as output(RED LED)
  pinMode(TRANS_CONTROL_PIN, OUTPUT);             //Set the transistor pin as output
  pinMode(PIO4_PIN, OUTPUT);                      //Set the bluetooth command mode pin as output

  pinMode(PRESSURE_PIN, INPUT);                   //Set the pressure sensor pin input
  pinMode(X_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Right FSR )
  pinMode(X_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Left FSR )
  pinMode(Y_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Up FSR )
  pinMode(Y_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Down FSR )

  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);           //Set increase cursor speed button pin as input
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);         //Set decrease cursor speed button pin as input

  pinMode(2, INPUT_PULLUP);                       //Set unused pins as inputs with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);


  delay(1000);
  getModelNumber(false);                          //Get LipSync model number 
  delay(10);
  sensitivityCounter = getJoystickSensitivity(false);   //Get saved joystick sensitivity parameter from EEPROM and sets the sensitivity counter
  delay(10);
  setSwitchJoystickInitialization(false);        //Set the Home joystick and generate movement threshold boundaries
  delay(10);
  getSwitchJoystickCalibration(false);            //Get FSR Max calibration values 
  delay(10);
  getPressureThreshold(false);                    //Set the pressure sensor threshold boundaries
  delay(10);
  debugModeEnabled = getDebugMode(false);         //Get the debug mode state
  delay(10);
  rawModeEnabled = getRawMode(false);             //Get the raw mode state
  delay(50); 
  joystickDeadzone = getDeadzone(false);         //Get the deadzone value 
  delay(10);
  getButtonMapping(false); 
  delay(10);


  ledBlink(4, 250, 3);                            //End initialization visual feedback
  
}

//-----------------------------------------------------------------------------------//

//***START OF MAIN LOOP***//

void loop() {
  
  settingsEnabled=serialSettings(settingsEnabled);       //Check to see if setting option is enabled in Lipsync

  xHigh = analogRead(X_DIR_HIGH_PIN);                 //Read analog values of FSR's : A0
  xLow = analogRead(X_DIR_LOW_PIN);                   //Read analog values of FSR's : A1
  yHigh = analogRead(Y_DIR_HIGH_PIN);                 //Read analog values of FSR's : A0
  yLow = analogRead(Y_DIR_LOW_PIN);                   //Read analog values of FSR's : A10


  //Debug information 
  
  if(debugModeEnabled) {
    
    Serial.print("LOG:3:");
    Serial.print(xHigh);
    Serial.print(",");
    Serial.print(xLow);
    Serial.print(",");
    Serial.print(yHigh);
    Serial.print(",");
    Serial.println(yLow); 
    delay(150);
  }

  //Map FSR values to (0 to 16 ) range 
  float xHighMapped=getMappedFSRValue(xHigh, joystickDeadzone, xHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xHighEquation);
  float xLowMapped=getMappedFSRValue(xLow, joystickDeadzone, xLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, xLowEquation);
  float yHighMapped=getMappedFSRValue(yHigh, joystickDeadzone, yHighNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yHighEquation);
  float yLowMapped=getMappedFSRValue(yLow, joystickDeadzone, yLowNeutral, JS_MAPPED_IN_DEADZONE, JS_MAPPED_IN_MAX, yLowEquation);
    
  //Calculate the x and y delta values 
  float xDelta = xHighMapped - xLowMapped;                            
  float yDelta = yHighMapped - yLowMapped;   
    
  //Get the final X and Y output values for Joystick set axis function
  int xOut = getXYValue(xDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);
  int yOut = getXYValue(yDelta, JS_OUT_DEAD_ZONE, JS_OUT_MAX, levelEquations[sensitivityCounter]);

  xOut = map(xOut, -128, 128, -10, 10);                   //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yOut = map(yOut, -128, 128, -10, 10);
  

  if (!rawModeEnabled && ((abs(xOut)) > 0) || ((abs(yOut)) > 0)) {
   pollCounter++;
   delay(15);
      if (pollCounter >= 5) {
          if ((xOut >= JS_OUT_MIN) && (-JS_OUT_MIN < yOut < JS_OUT_MIN) && ((abs(xOut)) > (abs(yOut)))) {
              //Right arrow key
              //Serial.println("Right");
              sendBluetoothCommand(byte(0x00),byte(0x4F));
          } 
          else if ((xOut < -JS_OUT_MIN) && (-JS_OUT_MIN < yOut < JS_OUT_MIN) && ((abs(xOut)) > (abs(yOut)))){
            //Serial.println("left"); 
            //Serial.println("Left");
            sendBluetoothCommand(byte(0x00),byte(0x50));           
          }
          else if ((-JS_OUT_MIN < xOut < JS_OUT_MIN) && (yOut < -JS_OUT_MIN) && ((abs(yOut)) > (abs(xOut)))){
            //Serial.println("Down");     
            //Serial.println("Down"); 
            sendBluetoothCommand(byte(0x00),byte(0x51));      
          }
          else if ((-JS_OUT_MIN < xOut < JS_OUT_MIN) && (yOut > JS_OUT_MIN) && ((abs(yOut)) > (abs(xOut)))){
            //Serial.println("Up");  
            //Serial.println("Up");
            sendBluetoothCommand(byte(0x00),byte(0x52));        
          }    
        delay(5);       
        pollCounter = 0;
        }

  }
  
  if(rawModeEnabled) {
   sendRawData(xOut,yOut,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow);
   delay(5);
  }
  //Perform sip and puff actions raw mode is disabled 
  else {
    sipAndPuffHandler();
    delay(5);
  }                                                       //Pressure sensor sip and puff functions                                                   //Pressure sensor sip and puff functions

  pushButtonHandler(BUTTON_UP_PIN,BUTTON_DOWN_PIN); 
  
  delay(JS_DELAY);                                      //The fixed delay for each action loop

}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------//


//***GET MODEL NUMBER FUNCTION***//

void getModelNumber(bool responseEnabled) {
  EEPROM.get(0, modelNumber);
  if (modelNumber != 4) {                                 //If the previous firmware was different model then factory reset the settings 
    modelNumber = 4;                                      //And store the model number in EEPROM 
    EEPROM.put(0, modelNumber);
    delay(10);
    factoryReset(false);
    delay(10);
  }  
  if(responseEnabled){
    Serial.println("SUCCESS:MN,0:4");
  }
}

//***GET VERSION FUNCTION***//

void getVersionNumber(void) {
  Serial.println("SUCCESS:VN,0:V1.1");
}

//***HID SWITCH SPEED FUNCTION***//

int getJoystickSensitivity(bool responseEnabled) {
  int sensitivity = SENSITIVITY_COUNTER;
  EEPROM.get(2, sensitivity);
  delay(5);
  if(sensitivity<0 || sensitivity >10){
    sensitivity = SENSITIVITY_COUNTER;
    EEPROM.put(2, sensitivity);
    delay(5);
  }
  if(responseEnabled){
    Serial.print("SUCCESS:SS,0:");
    Serial.println(sensitivity);      
  } 
  delay(5);
  return sensitivity;
}

//***INCREASE SENSITIVITY LEVEL FUNCTION***//

int increaseJoystickSensitivity (int sensitivity,bool responseEnabled) {
  sensitivity++;

  if (sensitivity == 11) {
    ledBlink(6, 50, 3);
    sensitivity = 10;
  } else {
    ledBlink(sensitivity+1, 100, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
  }
  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:"); 
  Serial.print("SS,1:");
  Serial.println(sensitivity); 
  delay(5);
  return sensitivity;
}

//***DECREASE SENSITIVITY LEVEL FUNCTION***//

int decreaseJoystickSensitivity(int sensitivity,bool responseEnabled) {
  sensitivity--;
  if (sensitivity == -1) {
    ledBlink(6, 50, 3);
    sensitivity = 0;
  } else if (sensitivity == 0) {
    ledBlink(1, 350, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
 
  } else {
    ledBlink(sensitivity+1, 100, 1);
    EEPROM.put(2, sensitivity);
    delay(25);
  }
  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:"); 
  Serial.print("SS,1:");
  Serial.println(sensitivity);  
  delay(5);
  return sensitivity;
}

//***GET PRESSURE THRESHOLD FUNCTION***//
void getPressureThreshold(bool responseEnabled) {
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  int pressureThreshold = PRESSURE_THRESHOLD;
  if(SERIAL_SETTINGS) {
    EEPROM.get(32, pressureThreshold);
    delay(5);
    if(pressureThreshold<=0 || pressureThreshold>50) {
      EEPROM.put(32, PRESSURE_THRESHOLD);
      delay(5);
      pressureThreshold = PRESSURE_THRESHOLD;
    }    
  } else {
    pressureThreshold = PRESSURE_THRESHOLD;
  }
  sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  if(responseEnabled) {
    Serial.print("SUCCESS:PT,0:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal);
    delay(5);
  }
}

//***SET PRESSURE THRESHOLD FUNCTION***//

void setPressureThreshold(int pressureThreshold, bool responseEnabled) {
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  if(SERIAL_SETTINGS && (pressureThreshold>0 && pressureThreshold<=50)) {
    EEPROM.put(32, pressureThreshold);
    delay(5); 
  } else {
    pressureThreshold = PRESSURE_THRESHOLD;
    delay(5); 
  }
  sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  if(responseEnabled) {
    Serial.print("SUCCESS:PT,1:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal); 
    delay(5);
  }
}

//***GET DEBUG MODE STATE FUNCTION***//

bool getDebugMode(bool responseEnabled) {
  bool debugState=DEBUG_MODE;
  if(SERIAL_SETTINGS) {
    EEPROM.get(34, debugState);
    delay(5);
    if(debugState!=0 && debugState!=1) {
      EEPROM.put(34, DEBUG_MODE);
      delay(5);
      debugState=DEBUG_MODE;
      }   
  } else {
    debugState=DEBUG_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS:DM,0:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SET DEBUG MODE STATE FUNCTION***//

bool setDebugMode(bool debugState,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    (debugState) ? EEPROM.put(34, 1) : EEPROM.put(34, 0);
    delay(5);    
  } else {
    debugState=DEBUG_MODE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DM,1:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SEND DEBUG DATA FUNCTION***//

void sendDebugData() {
  delay(100);
  Serial.print("LOG:1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(100);
  Serial.print("LOG:2:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(100);
}

//***SEND RAW DATA FUNCTION***//

void sendRawData(int x, int y, int action, int xUp, int xDown,int yUp,int yDown) {
  Serial.print("RAW:1:"); 
  Serial.print(x); 
  Serial.print(","); 
  Serial.print(y); 
  Serial.print(",");
  Serial.print(action); 
  Serial.print(":"); 
  Serial.print(xUp); 
  Serial.print(","); 
  Serial.print(xDown); 
  Serial.print(",");
  Serial.print(yUp); 
  Serial.print(",");
  Serial.println(yDown); 
}

//***GET RAW MODE STATE FUNCTION***//

bool getRawMode(bool responseEnabled) {
  bool rawState=RAW_MODE;
  if(SERIAL_SETTINGS) {
    EEPROM.get(36, rawState);
    delay(5);
    if(rawState!=0 && rawState!=1) {
      EEPROM.put(36, RAW_MODE);
      delay(5);
      rawState=RAW_MODE;
      }   
  } else {
    rawState=RAW_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS:RM,0:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***SET RAW MODE STATE FUNCTION***//

bool setRawMode(bool rawState,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    (rawState) ? EEPROM.put(36, 1) : EEPROM.put(36, 0);
    delay(5);    
  } else {
    rawState=RAW_MODE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:RM,1:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***GET DEADZONE VALUE FUNCTION***//

int getDeadzone(bool responseEnabled) {
  int deadzoneValue = JS_FSR_DEADZONE;
  if(SERIAL_SETTINGS) {
    EEPROM.get(38, deadzoneValue);
    delay(5);
    if(deadzoneValue<=0 || deadzoneValue>99) {
      EEPROM.put(38, JS_FSR_DEADZONE);
      delay(5);
      deadzoneValue=JS_FSR_DEADZONE;
      }    
  } else {
    deadzoneValue=JS_FSR_DEADZONE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DZ,0:");
    Serial.println(deadzoneValue); 
    delay(5);
   }
  return deadzoneValue;
}

//***SET DEADZONE VALUE FUNCTION***//

int setDeadzone(int deadzoneValue,bool responseEnabled) {
  if(SERIAL_SETTINGS) {
    if(deadzoneValue>0 || deadzoneValue<=99) {
      EEPROM.put(38, deadzoneValue);
      delay(5);
    } else {
      EEPROM.put(38, JS_FSR_DEADZONE);
      delay(5);
      deadzoneValue=JS_FSR_DEADZONE;
    }
  } else {
    deadzoneValue=JS_FSR_DEADZONE;
    delay(5);    
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:DZ,1:");
    Serial.println(deadzoneValue); 
    delay(5);
   }
  return deadzoneValue;
}


//***GET CURSOR INITIALIZATION FUNCTION***//

void getSwitchJoystickInitialization() {
  Serial.print("SUCCESS:IN,0:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(10);  
}

//***SET SWITCH JOYSTICK INITIALIZATION FUNCTION***//

void setSwitchJoystickInitialization(bool responseEnabled) {

  ledOn(1);
  
  xHigh = getAverage(X_DIR_HIGH_PIN,10);               //Set the initial neutral x-high value of joystick
  delay(10);

  xLow = getAverage(X_DIR_LOW_PIN,10);                 //Set the initial neutral x-low value of joystick
  delay(10);

  yHigh = getAverage(Y_DIR_HIGH_PIN,10);               //Set the initial neutral y-high value of joystick
  delay(10);

  yLow = getAverage(Y_DIR_LOW_PIN,10);                 //Set the initial Initial neutral y-low value of joystick
  delay(10);

  //Set the neutral values 
  xHighNeutral = xHigh;
  xLowNeutral = xLow;
  yHighNeutral = yHigh;
  yLowNeutral = yLow;

  //Get the max values from Memory 
  EEPROM.get(22, xHighMax);
  delay(10);
  EEPROM.get(24, xLowMax);
  delay(10);
  EEPROM.get(26, yHighMax);
  delay(10);
  EEPROM.get(28, yLowMax);
  delay(10);

  //Create equations to map FSR behavior 
  xHighEquation = getFSREquation(xHighNeutral,xHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  xLowEquation = getFSREquation(xLowNeutral,xLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yHighEquation = getFSREquation(yHighNeutral,yHighMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);
  yLowEquation = getFSREquation(yLowNeutral,yLowMax,JS_MAPPED_IN_NEUTRAL,JS_MAPPED_IN_MAX);
  delay(10);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.print("IN,1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  
  ledClear();
}

//*** GET SWITCH JOYSTICK CALIBRATION FUNCTION***//

void getSwitchJoystickCalibration(bool responseEnable) {
  
  Serial.print("SUCCESS:CA,0:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(10);
}

//*** SET SWITCH JOYSTICK CALIBRATION FUNCTION***//

void setSwitchJoystickCalibration(bool responseEnabled) {

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:0");                                                   //Start the joystick calibration sequence 
  ledBlink(4, 300, 3);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:1"); 
  ledBlink(6, 500, 1);
  //yHighMax = analogRead(Y_DIR_HIGH_PIN);
  yHighMax = getAverage(Y_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:2"); 
  ledBlink(6, 500, 1);
  //xHighMax = analogRead(X_DIR_HIGH_PIN);
  xHighMax = getAverage(X_DIR_HIGH_PIN,10);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:3"); 
  ledBlink(6, 500, 1);
  //yLowMax = analogRead(Y_DIR_LOW_PIN);
  yLowMax = getAverage(Y_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.println("CA,1:4"); 
  ledBlink(6, 500, 1);
  //xLowMax = analogRead(X_DIR_LOW_PIN);
  xLowMax = getAverage(X_DIR_LOW_PIN,10);
  ledBlink(1, 1000, 2);

  EEPROM.put(22, xHighMax);
  delay(10);
  EEPROM.put(24, xLowMax);
  delay(10);
  EEPROM.put(26, yHighMax);
  delay(10);
  EEPROM.put(28, yLowMax);
  delay(10);

  ledBlink(5, 250, 3);

  (responseEnabled) ? Serial.print("SUCCESS:") : Serial.print("MANUAL:");
  Serial.print("CA,1:5:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(10);
}

//***GET BUTTON MAPPING FUNCTION***//

void getButtonMapping(bool responseEnabled) {
  if (SERIAL_SETTINGS) {
    for (int i = 0; i < 6; i++) {
      int buttonMapping;
      EEPROM.get(42+i*2, buttonMapping);
      delay(5);
      if(buttonMapping<1 || buttonMapping >8) {
        EEPROM.put(42+i*2, actionButton[i]);
        delay(5);
      } else {
        actionButton[i]=buttonMapping;
        delay(5);
      }
    }
  }
  if(responseEnabled) {
    Serial.print("SUCCESS:MP,0:");
    Serial.print(actionButton[0]); 
    Serial.print(actionButton[1]); 
    Serial.print(actionButton[2]); 
    Serial.print(actionButton[3]); 
    Serial.print(actionButton[4]); 
    Serial.println(actionButton[5]); 
    delay(5);
   }
}

//***SET BUTTON MAPPING FUNCTION***//

void setButtonMapping(int buttonMapping[],bool responseEnabled) {
  if (SERIAL_SETTINGS) {
   for(int i = 0; i < 6; i++){
    EEPROM.put(42+i*2, buttonMapping[i]);
    delay(5);
    actionButton[i]=buttonMapping[i];
    delay(5);
   }     
  } 
  if(responseEnabled) {
    Serial.print("SUCCESS:MP,1:");
    Serial.print(actionButton[0]); 
    Serial.print(actionButton[1]); 
    Serial.print(actionButton[2]); 
    Serial.print(actionButton[3]); 
    Serial.print(actionButton[4]); 
    Serial.println(actionButton[5]); 
    delay(5);
   }
}

//***FACTORY RESET FUNCTION***//

void factoryReset(bool responseEnabled) {
  if (SERIAL_SETTINGS) {
    int defaultButtonMapping[6] = {ACTION_BUTTON_1, ACTION_BUTTON_2, ACTION_BUTTON_3, ACTION_BUTTON_4, ACTION_BUTTON_5, ACTION_BUTTON_6};
    EEPROM.put(2, SENSITIVITY_COUNTER);
    delay(10);
    setPressureThreshold(PRESSURE_THRESHOLD,false);
    delay(10);
    EEPROM.put(34, DEBUG_MODE);
    delay(10);  
    EEPROM.put(36, RAW_MODE);
    delay(10);  
    setButtonMapping(defaultButtonMapping,false);
    delay(10);
  
    setBluetoothConfig(false);                    //Reconfigure Bluetooth Module

    //Set the default values that are stored in EEPROM
    sensitivityCounter=SENSITIVITY_COUNTER;
    debugModeEnabled=DEBUG_MODE;  
    rawModeEnabled=RAW_MODE;
    joystickDeadzone=JS_FSR_DEADZONE;
                                       
    delay(10);
    }

  if(responseEnabled) {
    Serial.println("SUCCESS:FR,0:0");
    delay(5);
   }
   ledBlink(2, 250, 1);
}

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "SETTINGS" is in input string.
       if (settingsFlag==false && inString=="SETTINGS") {
        Serial.println("SUCCESS:SETTINGS");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString=="EXIT") {
        Serial.println("SUCCESS:EXIT");
       settingsFlag=false;                         //Set the return flag to false so settings actions can be exited
       }
       else if (settingsFlag==true && (inString.length()==(6) || inString.length()==(7) || inString.length()==(11)) && inString.charAt(2)==',' && inString.charAt(4)==':'){ //Check if the input parameter is true and the received string is 3 characters only
        inString.replace(",","");                 //Remove commas 
        inString.replace(":","");                 //Remove :
        writeSettings(inString); 
        settingsFlag=false;   
       }
       else {
        Serial.println("FAIL:SETTINGS");
        settingsFlag=false;      
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SPEED USING SOFTWARE***//

void writeSettings(String changeString) {
    char changeChar[changeString.length()+1];
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Get Model number : "MN,0:0"
    if(changeChar[0]=='M' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getModelNumber(true);
      delay(5);
    } 
    //Get version number : "VN,0:0"
    else if(changeChar[0]=='V' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getVersionNumber();
      delay(5);
    }   
    //Get joystick sensitivity value if received "SS,0:0", decrease the joystick sensitivity if received "SS,1:1" and increase the joystick sensitivity if received "SS,1:2"
    else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      sensitivityCounter = getJoystickSensitivity(true);
      delay(5);
    } else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      sensitivityCounter = decreaseJoystickSensitivity(sensitivityCounter,true);
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='1' && changeChar[3]=='2' && changeString.length()==4) {
      sensitivityCounter = increaseJoystickSensitivity(sensitivityCounter,true);
      delay(5);
    } 
     //Get pressure threshold values if received "PT,0:0" and pressure threshold values if received "PT,1:{threshold 1% to 50%}"
      else if(changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getPressureThreshold(true);
      delay(5);
    } else if (changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='1' && ( changeString.length()==4 || changeString.length()==5)) {
      String pressureThresholdString = changeString.substring(3);
      setPressureThreshold(pressureThresholdString.toInt(),true);
      delay(5);
    } 
     //Get debug mode value if received "DM,0:0" , set debug mode value to 0 if received "DM,1:0" and set debug mode value to 1 if received "DM,1:1"
     else if(changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = getDebugMode(true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(0,true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(1,true);
      delay(5);
    } 
    //Get raw mode value if received "RM,0:0" , set raw mode value to 0 if received "RM,1:0" and set raw mode value to 1 if received "RM,1:1"
     else if(changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = getRawMode(true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = setRawMode(0,true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      rawModeEnabled = setRawMode(1,true);
      delay(5);
    } 
     //Get deadzone value if received "DZ,0:0" , set deadzone value if received "DZ,1:{Value 1 to 99}" 
     else if(changeChar[0]=='D' && changeChar[1]=='Z' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      joystickDeadzone = getDeadzone(true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='Z' && changeChar[2]=='1' && (changeString.length()==4 || changeString.length()==5)) {
      String deadzoneString = changeString.substring(3);
      joystickDeadzone = setDeadzone(deadzoneString.toInt(),true);
      delay(5);
    }
     //Get cursor initialization values if received "IN,0:0" and perform cursor initialization if received "IN,1:1"
     else if(changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getSwitchJoystickInitialization();
      delay(5);
    } else if (changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setSwitchJoystickInitialization(true);
      delay(5);
    } 
     //Get cursor calibration values if received "CA,0:0" and perform cursor calibration if received "CA,1:1"
      else if(changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getSwitchJoystickCalibration(true);
      delay(5);
    } else if (changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setSwitchJoystickCalibration(true);
      delay(5);
    } 
    //Get Button mapping : "MP,0:0" , Set Button mapping : "MP,1:012345"
    else if (changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
    getButtonMapping(true);
    delay(5);
    } else if(changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='1' && changeString.length()==9) {
    int buttonTempMapping[6];
    for(int i = 0; i< 6; i++){
     buttonTempMapping[i]=changeChar[3+i] - '0';
    }
    setButtonMapping(buttonTempMapping,true);
    delay(5);
    }
  //Get bluetooth config value if received "BT,0:0" and set bluetooth config if received "BT,1:1"
     else if(changeChar[0]=='B' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
    getBluetoothConfig(true);
    delay(5);
    } else if (changeChar[0]=='B' && changeChar[1]=='T' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
    setBluetoothConfig(true);
    delay(5);
    } 
     //Perform factory reset if received "FR,0:0"
     else if(changeChar[0]=='F' && changeChar[1]=='R' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      factoryReset(true);
      delay(5);
    } else {
      Serial.println("FAIL:SETTINGS");
      delay(5);        
      }
}

//***GET AVERAGE ANALOG VALUE FUNCTION***//

int getAverage(int dataPin, int number) {
  long averageValue=0;
  for (int i=0; i<number; i++) {
     averageValue+=analogRead(dataPin);
     delay(2);
  }
  averageValue=averageValue/number;
  delay(10);
  return averageValue;
}

//***GET X AND Y VALUE IN (-maxOutputValue,maxOutputValue) RANGE FOR HOST DEVICE BASED ON MAPPED FSR VALUE AND THE DEGREE 5 POLYNOMIAL EQUATION COEFFICIENTS FUNCTION***//

int getXYValue(float rawValue, int deadzoneOutputValue , int maxOutputValue, _equationCoef equationCoef) {
  int xySign = sgn(rawValue);                                                 //Get the sign of input
  rawValue = abs(rawValue);                                                   //Solve for output regardless of the input sign and multiply the output by the sign ( the polynomial in quadrant 1 and 3 )
  int xyValue = (int)((equationCoef._equationACoef*pow(rawValue,5))+(equationCoef._equationBCoef*pow(rawValue,4))+(equationCoef._equationCCoef*pow(rawValue,3))+(equationCoef._equationDCoef*pow(rawValue,2))+(equationCoef._equationECoef*rawValue)+equationCoef._equationFCoef);
  delay(1);                                                                   //The points in quadrant 1 and 3 only (mirror (+,+) to (-,-) )
  xyValue = (xyValue >= maxOutputValue-deadzoneOutputValue) ? maxOutputValue: (xyValue<=deadzoneOutputValue) ? 0: xyValue;        //Set output value to maximum value if it's in maximum deadzone area and set output value to center value if it's in center deadzone area 
  delay(1);
  xyValue = xySign*xyValue;
  delay(1);
  return xyValue;
}

//***GET MAPPED FSR VALUE BASED ON THE EQUATION COEFFICIENTS FUNCTION***//

float getMappedFSRValue(int rawValue, int deadzoneInputValue, int neutralValue, float deadzoneOutputValue, float maxOutputValue, _equationCoef equationCoef) {
  float mappedValue;
  rawValue = (rawValue <= (neutralValue+deadzoneInputValue) && rawValue >=(neutralValue-deadzoneInputValue))? neutralValue:rawValue; //Set input value to neutral value if it's in neutral deadzone area 
  mappedValue = ((equationCoef._equationDCoef*pow(rawValue,2))+(equationCoef._equationECoef*rawValue));                    //Solve for mapped FSR Value using the coefficients of the equation ( result : value from 0 to 16 )
  mappedValue = (mappedValue>=maxOutputValue-deadzoneOutputValue) ? maxOutputValue: (mappedValue<=deadzoneOutputValue) ? 0.00: mappedValue;                     //Set output value to maximum value if it's in maximum deadzone area and set output value to center value if it's in center deadzone area 
  return mappedValue;
}

//***GET THE EQUATION COEFFICIENTS FOR MAPPING RAW FSR VALUES TO MAPPED VALUE FUNCTION***//

_equationCoef getFSREquation(int x1,int x2,int y1,int y2) {
  
  //Convert input values from int to float
  float x1Value = (float)x1;
  float x2Value = (float)x2;
  float y1Value = (float)y1;
  float y2Value = (float)y2;

  //Solve for coefficient d
  float dValue = (y2Value - ((y1Value*x2Value)/x1Value))/(x2Value*(x2Value-x1Value));
  
  //Solve for coefficient e
  float eValue = (y1Value/x1Value)-dValue*x1Value;

  //Output coefficients ( all others are zero )
  _equationCoef resultFactor = {0.00, 0.00, 0.00, dValue, eValue, 0.00};
  return resultFactor;
}

//***FIND SIGN OF VARIABLE FUNCTION***//

int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}


//***PUSH BUTTON SPEED HANDLER FUNCTION***//

void pushButtonHandler(int switchPin1, int switchPin2) {
    //Cursor speed control push button functions below
  if (digitalRead(switchPin1) == LOW) {
    delay(200);
    if (digitalRead(switchPin2) == LOW) {
      setSwitchJoystickCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      sensitivityCounter = increaseJoystickSensitivity(sensitivityCounter,false);
    }
  }

  if (digitalRead(switchPin2) == LOW) {
    delay(200);
    if (digitalRead(switchPin1) == LOW) {
      setSwitchJoystickCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      sensitivityCounter = decreaseJoystickSensitivity(sensitivityCounter,false);
    }
  }
}

//***SIP AND PUFF ACTION HANDLER FUNCTION***//

void sipAndPuffHandler() {
  //Perform pressure sensor sip and puff functions
  switchPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   //Read the pressure transducer analog value and convert it using ADC to a value between [0.0V - 5.0V]

  //Check if the pressure is under puff pressure threshold 
  if (switchPressure < puffThreshold) {             
    while (switchPressure < puffThreshold) {
      switchPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      puffCount++;                                //Count how long the pressure value has been under puff pressure threshold
      delay(5);
    }

    //Puff actions 
      if (puffCount < 150) {
        performButtonAction(actionButton[0]);
      } else if (puffCount > 150 && puffCount < 750) {
        performButtonAction(actionButton[2]);
      } else if (puffCount > 750) {
        performButtonAction(actionButton[4]);
      }
    puffCount = 0;                                //Reset puff counter
  }

  //Check if the pressure is above sip pressure threshold 
  if (switchPressure > sipThreshold) {
    while (switchPressure > sipThreshold) {
      switchPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      sipCount++;                                 //Count how long the pressure value has been above sip pressure threshold
      delay(5);
    }

    //Sip actions 
      if (sipCount < 150) {
        performButtonAction(actionButton[1]);
      } else if (sipCount > 150 && sipCount < 750) {
        performButtonAction(actionButton[3]);
      } else {
        //Perform seconday function if sip counter value is more than 750 ( 5 second Long Sip )
        performButtonAction(actionButton[5]);
      }
    sipCount = 0;                                 //Reset sip counter
  }
}

int sipAndPuffRawHandler() {
  int currentAction = 0;
  switchPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (switchPressure < puffThreshold) {
        delay(5);
        currentAction = 1;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (switchPressure > sipThreshold) {
        delay(5);
        currentAction = 2;
  }
  return currentAction;
}


void performButtonAction(int actionButtonNumber) {
    switch (actionButtonNumber) {
      case 0: {
        //Enter or select
        sendBluetoothCommand(byte(0x00),byte(0x28));
        break;
      }
      case 1: {
        //Space
        sendBluetoothCommand(byte(0x00),byte(0x2C));  
        break;
      }
      case 2: {
        //Dot
        sendBluetoothCommand(byte(0x00),byte(0x37)); 
        break;
      }
      case 3: {
        //Dash
        sendBluetoothCommand(byte(0x00),byte(0x2D)); 
        break;
      }
      case 4: {
        //a
        sendBluetoothCommand(byte(0x00),byte(0x61));    
        break;
      }
      case 5: {
        //Initialization: Perform joystick manual home initialization to reset default value of FSR's if puff counter value is more than 750 ( 5 second Long Puff )
        ledClear();
        ledBlink(4, 350, 3); 
        setSwitchJoystickInitialization(false);
        delay(5);
        break;
      }
      case 6: {
        //Calibration: Perform joystick Calibration to reset default value of FSR's if puff counter value is more than 750 ( 5 second Long Puff )
        ledClear();
        setSwitchJoystickCalibration(false);
        delay(5);
        break;
      }
    }
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

//***LED BLINK FUNCTION***//

void ledBlink(int numBlinks, int delayBlinks, int ledNumber) {
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
    case 6: {
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
  }
}

//***BLUETOOTH HID KEYBOARD COMMAND FUNCTION***//

void sendBluetoothCommand(byte modifier,byte button) {
  
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
    clearBluetoothCommand();

    delay(10);
}

//***BLUETOOTH HID MOUSE CLEAR FUNCTION***//

void clearBluetoothCommand(void) {

  byte bluetoothKeyboard[2];

  bluetoothKeyboard[0] = 0xFD;
  bluetoothKeyboard[1] = 0x00;
  Serial1.write(bluetoothKeyboard,2);
  Serial1.flush();
  delay(10); 
}


//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***GET BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void getBluetoothConfig(bool responseEnabled) {
  int configNumber= BT_CONFIG_NUMBER;
  EEPROM.get(54, configNumber);
  delay(5);
  if(configNumber<0 || configNumber>3) {
    setBluetoothConfig(false);
    delay(5);
    configNumber=BT_CONFIG_NUMBER;
   }   

  if(responseEnabled) {
    Serial.print("SUCCESS:BT,0:");
    Serial.println(configNumber); 
    delay(5);
  }
}

//***SET BLUETOOTH CONFIGURATION FUNCTION***//

void setBluetoothConfig(bool responseEnabled) {
    delay(10);
    setBluetoothCommandMode();                                //Call Bluetooth command mode function to enter command mode
    setBluetoothConfigSequence();                             //Send configuarion data to Bluetooth module
    delay(10);
  if(responseEnabled) {
    Serial.print("SUCCESS:BT,1:");
    Serial.println(BT_CONFIG_NUMBER); 
    delay(5);
  }
}

//***SET BLUETOOTH CMD MODE FUNCTION***//

void setBluetoothCommandMode(void) {
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
}

//***BLUETOOTH CONFIG FUNCTION***//

void setBluetoothConfigSequence(void) {
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
  Serial1.println("S?,1");                          //Enable the role switch for better performance of high speed data
  delay(15);
  Serial1.println("R,1");                           //Reboot BT module
  delay(15);
  
  EEPROM.put(54, BT_CONFIG_NUMBER);                  //Save the configuration nummber value at EEPROM address location 54
  delay(15);
}

#include <Arduino.h>

#include "common.h"

//***************************************
// VARIABLES

uint32_t programStartMs;
uint32_t lastTimeAddRecord=0;
uint32_t lastTimeLedBlink=0;
bool fCycleStart = false;

//***************************************
// INCLUDES
#include "myRtcLib.h"
#include "myUtilities.h"
#include "myImuLib.h"
#include "myDataStorage.h"
#include "powerMgt.h"

//*******************
void longClickEvent()
//*******************
{
if (fPrintDebug) Serial.println(">LONG CLICK");

storeDataRecords();
motorStart(1000,MOTOR_SPEED_HIGH);  delay(200);  motorStart(1000,MOTOR_SPEED_HIGH); delay(200);  motorStart(1000,MOTOR_SPEED_HIGH);
flashLed(3,100);

gotoSystemOff_WakeupOnSwitch();
}

//*******************
void shortClickEvent()
//*******************
{
if (fPrintDebug) Serial.println(">SHORT CLICK");
flashLed(2,200);

displayRecordsScreen();
motorStart(300,MOTOR_SPEED_HIGH); //delay(300);  motorStart(500,MOTOR_SPEED_LOW); delay(300); motorStart(500,MOTOR_SPEED_NORMAL); delay(300); motorStart(500,MOTOR_SPEED_HIGH);
}

//********************
void checkSerialChar() 
//********************
{
char c;
int value = 0; // default we read first page

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (fPrintDebug) 
     {
      if (c>='a'&& (c<='z')) {Serial.print ("CMD->"); Serial.println(c); }

     }
    switch (c) {

      case '?': // Help on commands
        Serial.println ("HELP: \n a: read Flash-dont load data \n c: display data in Csv\n d: Display Data\n e:erase flash (1st page) \n m: Memory dump \n r: Read All Memory\n s: Search Flash Records \n t: Test Flash \n w: write to flash\n x: reset records to zero \n\n");
      break;

      case 'a': // Read all memory
        readDataRecords(false);//does NOT overwrite tabRecords with flash data
      break;

      case 'r': // Read all memory and overwrite tabRecords with flash data
        readDataRecords(true);
      break;

      case 'd': // display data
        displayRecordsScreen();
      break;

      case 'c': // display data for CSV file
        displayRecordsForCSV();
      break;

      case 'e': // erase flash first page
        eraseFlashPagesBeforeWrite (FIRST_USER_FLASH_ADDR , 1); // to force delete of 1st Flash page only
      break;

      case 'w': // store function test
        storeDataRecords();
      break;

      case 's': 
        searchFlashRecords();
      break;

      case 't': 
        testFlash();
      break;

      case 'm': // expect the Flash page index number after 'm'. Eg: m3

        if (Serial.available()) {
           c = Serial.read();
            if (c >= '0' && c <= '9') {  value = c - '0';   }
        } 
        dump_memory(FIRST_USER_FLASH_ADDR + (value * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);
      break;

      case 'x': // resetRecords
        
        cRecords = 0;
        if (fPrintDebug) Serial.println("Records Reset");
      break;
    }
  }
}

//***************************************
void timerEvent() 
//***************************************
{
  uint32_t now;
  uint16_t v;// type of switch oressed
  
  enableAuxiliaries();

  if (fSwitchedPressed(v))
   {
    if (v==LONG_CLICK) longClickEvent();
    else if (v==SHORT_CLICK) shortClickEvent();
   }

  checkSerialChar();
  checkPosAndMovements();
  checkBreathAccelGyro();

  now = millis();

  if ( fPrintDebug && (TIMER_LED_BLINK && (now - lastTimeLedBlink > TIMER_LED_BLINK )) ) 
   {
    lastTimeLedBlink = now;
    flashLed(1,0);
   }

  if (now - lastTimeAddRecord > TIMER_ADD_RECORD) 
   {
    if (lastBreathFromPreviousPeriod > breathMax) 
      {
       breathMax = lastBreathFromPreviousPeriod;
       lastBreathFromPreviousPeriod =0;
      }

    addDataRecord();

    breathMax = 0;
    lastTimeAddRecord = now; 
    fCycleStart = true;
    breathCount = 0;
    cMoves = 0;

   }

  else fCycleStart = false;

  disableAuxiliaries();
}

//***************************************
void setup()
//***************************************
{
float v;
 unsigned long t0 = millis();

  motorStart(400,MOTOR_SPEED_HIGH);  delay(300);  motorStart(400,MOTOR_SPEED_HIGH);

  Serial.begin(115200);
 
  while (!Serial && millis() - t0 < 2000) {
    // waits max 2 seconds
  }

  if (fPrintDebug) Serial.println("BOARD STARTED");
  
  if ((bool) Serial) {
    if (fPrintDebug) Serial.println("Boot. USB/Serial port is active\n");
  }
 
  //if (fPrintDebug)  {  Serial.print("isSoftDeviceEnabled ->"); Serial.println(isSoftDeviceEnabled());  }
 
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  v = readVBAT();
  if (fPrintDebug)  {
      Serial.print ("Battery voltage = "); Serial.println (v,3);
    }

/*
  if ( v < VBAT_MINI)
    {
      if (fPrintDebug)
        {
          Serial.print ("WARNING: Battery voltage is too low. Switch systemOff. Voltage = "); Serial.println (v,3);
        }
      digitalWrite(LED_BUILTIN, LOW); motorStart(200,150);  digitalWrite(LED_BUILTIN, HIGH); delay(200); digitalWrite(LED_BUILTIN, LOW); motorStart(200,150);digitalWrite(LED_BUILTIN, HIGH);
      delay (5000); // we let time in case we want to update the software before is goes to systemoff (no way to update the bord when it is systemOff)
      gotoSystemOff_WakeupOnSwitch ();
    }
*/
  initIMU();
  delay(100);
  calibrateGyro();

  setTickPeriodMs(PROGRAM_WAKEUP_PERIOD); // ms
  rtc2_start();

  programStartMs = 0;

  flashLed(2,200);
}

//***************************************
void loop()
//***************************************
{
if (checkRtcEvent()) timerEvent();

sleep_until_irq();
}

//****************************
// consommation without call to disableAuxiliaries: 0.13 mA (130uA)
// avec en plus disableIMU: 0.12 mA

void disableAuxiliaries()
{
NRF_SAADC->ENABLE = 0;

//  disableIMU();

  //Serial.flush();
  //Serial.end();


  // IMU.sleep Inutile en SYSTEM OFF (power cut)
  //IMU.sleep();

  /*if (flash.begin()) {
    flash.deepPowerDown();  // ~1 mA économisé
  }*/

  /* GPIO flottants → entrée pull-down
  for (uint8_t pin = 0; pin < 32; pin++) {
    pinMode(pin, INPUT_PULLDOWN);
  }*/

//  __disable_irq(); => NE PAS UILISER: ca bloqie l usb pour mettre a jour la carte!!
}

//************************
void enableAuxiliaries()
{
NRF_SAADC->ENABLE = 1; // enable ADC

//enableIMU(); => Warning. apres disable/enableIMO le gyro retourne souvent n/importe quoi
//initIMU();
//delay(10);
// IMU.begin();
// flash.begin();

}

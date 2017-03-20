// 24/02/2017
// Aidan Bennett-Reilly
// This is the Arduino Code The BACON platform operates on
// There are Several Libraies that are modified from the original, thus 
// only the provided libraries should be used. 


#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>

#include "Adafruit_SharpMem.h"
#include "Adafruit_MCP23008.h"
#include "samd21_Sleep_function.h"
#include "I2C_FRAM.h"
#include "SparkFunDS3234RTC.h" //Real time clock library                       // no code to optimise
#include "SHT1X.h"             // SHT15 humidity sensor library                // no code to optimise
#include "ClosedCube_Si7051.h" // Si7051 digital temperature sensor library    // no code to optimize
#include "Adafruit_ADS1015.h"                                                  // no code reduction possible it seems

#include "K30_CO2_lib.h"
//#include "kSeries.h"


samd21_Sleep_function sleep;

#define DeviceVersionNumber 0.4

/* --- User configurable global Settings --- */
#define SPI_DIVISOR_GLOBAL 2
#define SERIAL_BAUD 115200
#define PWRLED 13
#define wakeButton A1
#define VBATPIN A7

#define debug false
bool keepUSBConnected = false;

bool runSampling = false; // variable controls wheather sampling is allowed to run. is set via GUI


/* ---------- Dsiplay variables ---------------- */
#define displayCS 10
Adafruit_SharpMem ePaper(displayCS); // display uses pin 10 as chip select
#define BLACK 0
#define WHITE 1

bool displayOn = true;


uint8_t timeOutLimitcCount = 104; // 22 for ~ 30 sec

volatile uint8_t timeOnCount = 0;
volatile uint8_t EXTCOM_PWM_OUTPUT = 0;

/* --------- GPIO variables ---------------- */
Adafruit_MCP23008 gpioExpander;


/*=============== SDcard variables ======================*/
//SD library may not pull chip select low automatically
//SPISettings SdSPISetting(4000000, MSBFIRST, SPI_MODE0); //not needed now
#define SD_CS_PIN 12
#define SD_card_detect A2
#define PWR_SW_SD A4
const char commar  = ','; // this is used so often that its better to have as global
const char zero = '0';
const char space = ' ';
volatile bool wakeFlag = false;
SdFile txtFile;
SdFat SD;

const int SD_POWER_LINE  = A3;        // input pin for reading SD card rail voltage
const int  SdADCThresh  = 50;         // threshold value the SD card rail must be below for proper operation


/* --- User configurable Settings --- */
const char fileName[]  = "DATA.CSV" ; //
const char tableName[] = "LOOKUP.CSV" ; //
const char CO2fileName [] = "CO2.CSV" ;

/* ---------------------------------- */


/*=============== ADC temp sensor variables ============*/
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
#define ADS_GAIN_TWOTHIRDS 0
#define ADS_GAIN_ONE 1
#define ADS_GAIN_TWO 2
#define ADS_GAIN_FOUR 3
#define ADS_GAIN_EIGHT 4
#define ADS_GAIN_SIXTEEN 5

/*  mV resolution for gains from two thirds to 16 times*/
const float GainResolution[] = {0.187503, 0.125002, 0.062501, 0.031250, 0.015625, 0.007813};


/* --- User configurable Settings --- */
// modifiable variables to set the mode and gain for each channel of the ADS1115 ADC

/* Sets the maximum number of inputs from ADS1115 ADC's, 4 for one ADS1115, 8 for two ADS1115*/
#define MAX_ADS1115_INPUTS 4

/* determines which inputs are enabled for sampling. Least bit significant.
   E.g. setting bit 0 to one enables measuring from the first ADS1115 A0 (first) input*/
byte ADS_BitMask_enable          = B00000000; //

/*  Determines which inputs are in single ennded mode, Least bit significant order, where A0 is
    bit 0 for the first ADS1115 input 1.
    If an input is enabled but not in single ended mode, a double ended reading will be performed  */
byte ADS_BitMask_single          = B00000011;



/* sets weather a lookup table should be applied to readings per port. In differential mode,
   the next channel after the first differential channel must be set to 0, otherwise a
   single ended reading will be done as well */
byte ADS_BITMASK_USE_LOOKUPTABLE = B00000000;

/* Sets the gain applied to each input on ADS1115 ADC. Is a user friendly mask for the
   use of the GainResolution array. Double ended readings also get this gain applied */
int8_t ADS1115Gain[] = { ADS_GAIN_ONE, ADS_GAIN_ONE, ADS_GAIN_SIXTEEN, ADS_GAIN_SIXTEEN};

// address of lookup table stored in eeprom, use integer values determining the index of a table,
// e.g 0 for first table, 3 for 4th table. up to 8 tables are possible
byte tableEEPROMAdrress[MAX_ADS1115_INPUTS] = {0, 0, 0, 0};



/*=============== K30 CO2 sensor variables ==============*/
K30_CO2_lib K30_CO2_SENS;

byte CO2buff[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
const int valMultiplier = 1;
uint8_t co2MinSampleinterval = 2;

bool enableCO2Sampling = false;
bool CO2SensorOn = false;
bool CO2Error = false;


/*=============== SHT15 sensor variables ================*/
#define SHT15_DATA 6
#define SHT15_SCK 5
SHT1x sht15(SHT15_DATA, SHT15_SCK);//Data, SCK


/*=============== Si7051 sensor variables ===============*/
ClosedCube_Si7051 si7051;

/*=============== RTC variables ========================*/

// RTC library handles the chip select pin automatically

// SPISettings In the SparkFunDS3234RTC by default use 4MHz speed, but here for stability, 2MHz is used,
// so the SparkFunDS3234RTC library has its #define DS3234_MAX_SCLK line set to 2000000 vs 4000000.
// If issues with RTC are observed, reduce down to 1MHz or lower if circuit wiring is poor
DS3234 DS3234;
#define DS3234_CS_PIN 11 //RTC chip select pin
#define RTC_alarm_pin A0
volatile bool rtcTimerInterrupt = false;

uint8_t sampleRateHour = 0;
uint8_t sampleRateMin = 1;

uint8_t nextAlarmTime_mainDevice_minutes = 0;
uint8_t nextAlarmTime_mainDevice_hours = 0;

uint8_t nextAlarmTime_CO2Device_minutes = 0;
uint8_t nextAlarmTime_CO2Device_hours = 0;


/*=============== misc variables =======================*/

//#define PWR_SW_SENS A2

const byte averageNum = 3;


/*=============== EEPROM variables =====================*/



/*===================== Sample data Struct =============*/

// each data struct holds ONE set of sample data and uses ~34 bytes if MAX_ADS1115_INPUTS = 4
struct DataSample {
  float Si7051_T;
  float SHT15_T;
  float SHT15_H;
  //uint16_t K30_CO2;
  int16_t adsSingle[MAX_ADS1115_INPUTS]; //variables for storing temporary sinlge ended analog readings
  int16_t adsDouble[MAX_ADS1115_INPUTS / 2]; //variables for storing temporary double ended analog readings
  uint8_t rtcHour;
  uint8_t rtcMinutes;
  uint8_t rtcSeconds;
  uint8_t rtcDate;
  uint8_t rtcMonths;
  uint8_t rtcyears;
};

struct Co2Sample {
  uint16_t K30_CO2;
  uint8_t rtcHour;
  uint8_t rtcMinutes;
  uint8_t rtcSeconds;
  uint8_t rtcDate;
  uint8_t rtcMonths;
  uint8_t rtcyears;
};

DataSample data; //temp struct to hold sampled data before saving to FRAM. also used to store data being
Co2Sample CO2Samp; //temp struct to hold C02 data before saving to FRAM. also used to store data being
// read from FRAM to write to SD card.


/*============Variables for FRAM =======================*/
I2C_FRAM fram;


const uint16_t maxSampleBufferSize = 512; // max number of samples to store in FRAM before writing out to SD card, allows some wiggle room for larger sample sizes
const uint16_t maxCO2SampleBufferSize = 512;

const uint8_t errCodeFramAddr  = 0; //FRAM address for stored error codes. (2 bytes)
const uint8_t SampleCountFramAddr  = 2; // FRAM address for number of stored samples. (2 bytes)

const uint8_t ADS1115_Enable_Addr = 8;   // FRAM addresses for saved ADS1115 variables
const uint8_t ADS1115_SingleCHN_Addr = 9;
const uint8_t ADS1115_Gain_Addr = 10; // 4 bytes long
const uint8_t ADS1115_UseLookup_Addr = 14; // 1 byte long
const uint8_t ADS1115_UseLookupTable_Addr = 15; // 4 bytes long

const uint8_t framSampleBufferCount_Addr = 19; // 2 bytes long, used for graphing
const uint8_t samplePeriod_Addr = 21; // 2 bytes long
const uint8_t sampleBufferSize = 23;  // 2 bytes long: the address for the current sample buffer size for the number of samples to buffer on FRAM before writing to SD

const uint8_t errCodeNumTotalAddr = 32; // address for total number of error codes stored
const uint8_t errCodeNumAddr = 33;  // current position counter for error code history
const uint8_t errCodeHistoryAddr = 34; // each error code is 2 bytes with time data of 6 bytes, thus 8 bytes long
const uint8_t errCodeMaxNum = 8;

const uint8_t SampleCount_CO2_FramAddr  = 110; // FRAM address for number of stored CO2 samples. (2 bytes)
const uint8_t samplePeriod_CO2_Addr = 112; // 2 bytes long
const uint8_t framSampleBufferCount_CO2_Addr = 114; // 2 bytes long, used for graphing
const uint8_t co2Enablesetting_FramAddr = 116; // byte for storing if co2 sensor sampling is enabled

const uint8_t lookupTablesStartAdd = 128;


const uint16_t framDataStartAddr  = 736; //start address for sample data. Must not save sample data before this. gives 32,000 bytes of space
const uint16_t CO2DataBufferStartAddr = 16384; // start address for CO2 sample buffer

uint16_t currAddrFRAM = 0;
uint16_t sampleCount = 0;
uint16_t co2SampleCount = 0;


/*======================================================*/

#define tableElementSize 16 //table size in values stored. 16 allows 4 differrent lookup tables
#define maxTableNum 4       // max number is 4 due to sample data start address

//define lookup table structure for using lookup tables
struct LookupTable {
  float inputRange[tableElementSize];
  float outputRange[tableElementSize];
};
LookupTable tempLookup;

/*============ Menu settings    ========================*/
const char **textList = ( const char **)malloc(32); //allocate memory for a pointer to pointers to char arrays, pointers are 4 bytes in size

uint8_t temp1 = 0;
uint8_t temp2 = 0;
uint8_t temp3 = 0;

uint16_t inMenuNum = 0;
uint16_t prevMenuNum = 0;
uint8_t cursorPos = 0;
uint8_t maxCursor = 0;
bool buttonPressedContinuous = false;
uint16_t lastButtonPress = 0;
bool loopButtonPress = false;

bool menuButtonFlag = false;
bool backButtonFlag = false;




/*======================================================*/

/* Interrupt Service Routine (ISR) for real time clock alarm */
void RTC_ISR() {
  sleep.interruptDisable();

  rtcTimerInterrupt = true;  // set real time clock interrupt detect flag

}


/* Interrupt Service Routine (ISR) for waking the device up*/
void wakeup_ISR() {

  sleep.interruptDisable();

  wakeFlag = true;  // set real time clock interrupt detect flag

}

void TC3_Handler()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt

    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    digitalWrite(A5, EXTCOM_PWM_OUTPUT);
    digitalWrite(13, HIGH);
    EXTCOM_PWM_OUTPUT = ~EXTCOM_PWM_OUTPUT;
    timeOnCount++;                 // for debug leds
  }
  if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
    //digitalWrite(A5, LOW);
    digitalWrite(13, LOW);
    TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
  }
}



void setup() {
  uint8_t beginAttempts = 0;

  //SYSCTRL->VREG.bit.RUNSTDBY = 1;
  /* Update display with logo to inform user device is starting

  */

  /* ======== Setup IO ========================== */

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  //pinMode(DS3234_CS_PIN, INPUT);

  pinMode(DS3234_CS_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(displayCS, OUTPUT);

  pinMode(SD_card_detect, INPUT);
  //pinMode(PWR_SW_SENS, OUTPUT);
  pinMode(PWR_SW_SD, OUTPUT);
  pinMode(PWRLED, OUTPUT);
  pinMode(RTC_alarm_pin, INPUT_PULLUP); // set alarm pin to input pullup mode
  pinMode(wakeButton, INPUT_PULLUP); // set wakeup button pin to input pullup mode
  //pinMode(SENSOR_POWER_LINE, INPUT);
  pinMode(SD_POWER_LINE, INPUT);

  // keep sensors & SD powered on
  //digitalWrite(PWR_SW_SENS, LOW);
  digitalWrite(PWR_SW_SD, HIGH);

  // define chip select lines
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(DS3234_CS_PIN, LOW);
  digitalWrite(PWRLED, LOW);

  if (debug) {
    Serial.begin(SERIAL_BAUD);                                  //start serial connection

    while (!Serial);
    Serial.println(F("I2C INIT"));
  }


  /* ====== initialise Devices  ==================*/

  beginI2cDevices();
  gpioExpander.begin();
  setupGPIOExpander();
  while (digitalRead(wakeButton) == 0) {
    gpioExpander.readINTF();
    gpioExpander.readGPIO();
  }

  gpioExpander.digitalWrite(0, HIGH); // switch on Display
  delay(1);

  // initialsie RTC before display is updated
  if (debug) {
    Serial.println(F("RTC INIT"));
  }
  DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);                             // initialise RTC connection
  DS3234.update();                                         //update rtc
  DS3234.set24Hour(true);                                  //Set RTC to operate in 24hour mode
  //clearRTCAlarm();// clear any alarms that may be pre-existing
  DS3234.enableAlarmInterrupt(false, false);  // enables interrupt from square wave pin on RTC to trigger on alarm event
  //DS3234.setAlarm1(0, DS3234.getMinute() + sampleRateMin, DS3234.getHour() + sampleRateHour); //set alarm to trigger after the sample rate period
  DS3234.alarm1(true);
  DS3234.alarm2(true);

  ePaper.begin();
  ePaper.clearDisplay();
  logo();
  ePaper.refresh();

  checkPower(SD_POWER_LINE, SdADCThresh, false); // wait till power stabilizes

  //  if (debug) {
  //    Serial.println(F("Alarm Set"));
  //  }





  /* ======== Configure Powered modules ========= */

  ADC->CTRLA.bit.ENABLE = 0;
  WDT->CTRL.bit.ALWAYSON = 0;
  WDT->CTRL.bit.ENABLE = 0;
  PM->APBAMASK.reg &= ~PM_APBAMASK_RTC;
  PM->APBAMASK.reg &= ~PM_APBAMASK_WDT;
  PM->APBCMASK.reg &= ~PM_APBCMASK_ADC; // disable clock to ADC
  PM->APBCMASK.reg &= ~PM_APBCMASK_DAC; // disable clock to DAC
  PM->APBCMASK.reg &= ~PM_APBCMASK_I2S; // disable clock to I2S
  PM->APBCMASK.reg &= ~PM_APBCMASK_AC; // disable clock to AC
  PM->APBCMASK.reg &= ~PM_APBCMASK_PTC; // disable clock to peripheral touch controller
  PM->APBCMASK.reg &= ~PM_APBCMASK_AC1; // disable clock to peripheral touch controller

  PM->APBCMASK.reg &= ~PM_APBCMASK_TC7; //
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC6;//
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;//
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC4;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;//
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;//
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;//

  //defineUnusedPins();

  /* ====== Check saved error codes in FRAM ===== */

  if ( fram.readUInt16(errCodeFramAddr) != 0 && debug) {
    printError();
  }
  fram.writeUInt16(errCodeFramAddr, 0); //clear errors stored in FRAM

  if (debug) {
    Serial.println(F("ERR cleared"));
  }

  /* ====== Setup external interrupts =========== */

  /*setup external interrupts for waking up device on real time clock alarms */


  attachInterrupt(RTC_alarm_pin, RTC_ISR, LOW ); // setup Interrupt
  attachInterrupt(wakeButton, wakeup_ISR, LOW ); // setup Interrupt
  if (debug) {
    Serial.println(F("Interrupts Attached"));
  }
  sleep.begin(RTC_alarm_pin);                // configure sleep library with RTC external interrupt pin
  sleep.addExternalInterrupt(wakeButton);    // add wake up button interrupt pin

  /* ============================================ */


  if (debug) {
    //Serial.println(F("SD card present checked"));
    Serial.println(F("initialising SD"));
  }

  //ensure SD card is present before proceeding
  //digitalWrite(PWR_SW_SD, HIGH);
  checkSDCard();


  while (beginAttempts < 5) {

    if (debug) {
      Serial.print(F("SD INIT FAIL: Try "));
      Serial.println(beginAttempts + 1);
    }
    if ( SD.begin(SD_CS_PIN, SPI_DIVISOR_GLOBAL)) {
      break;
    }

    beginAttempts++;
    delay(200);
  }


  if (beginAttempts >= 5 ) { //, SPI_HALF_SPEED
    error(0x8000);
    if (debug) {
      Serial.println(F("SD initialise failed"));
    }

  }
  else {

    //open data sample file on SD card to create it if it does not exist
    txtFile.open(fileName, (O_RDWR  | O_CREAT | O_AT_END));

    txtFile.close();

    txtFile.open(CO2fileName, (O_RDWR  | O_CREAT | O_AT_END));

    txtFile.close();

    if (debug) {
      Serial.println(F("Check files"));
    }
    //Check all files are found on SD card and save errors if none found
    if (!SD.exists(fileName)) {
      error(0x0080);
    }
    if (!SD.exists(tableName)) {
      error(0x0040);
    }
    else {
      // load lookup tables from sd card
      //if(debug){
      //Serial.println(F("initialise lookup"));
      //}
      readTableFromSD();
    }


    /* ===== Purge unsaved Data to SD card ======== */
    if (debug) {
      Serial.println(F("clear FRAM counter"));
    }

    if ( fram.readUInt16(SampleCountFramAddr) > 0) {
      saveToSD();
      fram.writeUInt16(SampleCountFramAddr, 0); // update sample counter on FRAM, clearing previous sample data is not needed

    }
    if ( fram.readUInt16(SampleCount_CO2_FramAddr) > 0) {
      saveCO2ToSD();
      fram.writeUInt16(SampleCount_CO2_FramAddr, 0); // update sample counter on FRAM, clearing previous sample data is not needed
    }
    if (debug) {
      Serial.println(F("Done initialising"));

    }
  }




  //====== disconnect USB ====================

  if (!debug && !keepUSBConnected ) {
    USBDevice.detach(); // Safely detach the USB
  }

  // finished with SD card so turn it off
  digitalWrite(PWR_SW_SD, LOW);


  //load config from FRAM
  ADS_BitMask_enable = fram.readByte(ADS1115_Enable_Addr);
  ADS_BitMask_single = fram.readByte(ADS1115_SingleCHN_Addr);
  ADS1115Gain[0] = fram.readByte(ADS1115_Gain_Addr);
  ADS1115Gain[1] = fram.readByte(ADS1115_Gain_Addr + 1);
  ADS1115Gain[2] = fram.readByte(ADS1115_Gain_Addr + 2);
  ADS1115Gain[3] = fram.readByte(ADS1115_Gain_Addr + 3);

  sampleRateHour = fram.readByte(samplePeriod_Addr);
  sampleRateMin = fram.readByte(samplePeriod_Addr + 1);

  // clear any erronious gain settings in FRAM
  for (int i = 0; i < 4; i++) {
    if ( ADS1115Gain[i] > 5) {
      ADS1115Gain[i] = 0;
    }
  }

  if (sampleRateHour > 24) {
    sampleRateHour = 24;
  }
  if (sampleRateHour > 59) {
    sampleRateMin = 59;
  }



  //    fram.writeByte(errCodeNumAddr, 0); // update error count number
  //    for (uint8_t i = 0; i < errCodeMaxNum; i++) {
  //
  //      fram.writeUInt16(errCodeHistoryAddr + i * 8 , 0); //store error in FRAM history for later checking
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 2, 0); //store timestamps for error in FRAM for later checking
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 3, 0);
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 4, 0);
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 5, 0);
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 6, 0);
  //      fram.writeByte(errCodeHistoryAddr + i * 8 + 7, 0);
  //
  //    }

  fram.writeByte(SampleCount_CO2_FramAddr, 0); // update error count number
  //displayOn = true;
  //pwrLED(5);
  updateStatusBar();
  ePaper.refresh();

}

//=====================================================================
//=====================================================================

void loop() {
  // put your main code here, to run repeatedly:
  //  while (true) {
  //    //printBits(gpioExpander.readINTCAP());
  //    printBits(gpioExpander.readINTF());
  //    Serial.print("  ");
  //    printBits(gpioExpander.readINTCAP());
  //    gpioExpander.readGPIO();
  //    Serial.println();
  //    delay(10);
  //  }


  beginI2cDevices(); //reinitialises I2C connections after power down.

  if (wakeFlag) {
    scanButtons();
    wakeFlag = false;
  }
  else {
    buttonPressedContinuous = false;
    loopButtonPress = false;
  }

  if (debug) {
    Serial.print(F("Alarm status: "));
    Serial.print(DS3234.alarm1(false));
    Serial.print(space);
    Serial.println(DS3234.alarm2(false));
  }

  // if the RTC alarm was triggered, sampling is enabled and the alarm time matches the expected time, take a sample of the main devices sensors

  if ( runSampling && rtcTimerInterrupt && DS3234.alarm1(true) ) {
    if ( compareAlarmTimes(nextAlarmTime_mainDevice_minutes, nextAlarmTime_mainDevice_hours) ) { //alarm matches Main device's next sample time, so take a data sample
      if (displayOn) {
        // enable display power
        drawSamplingInProgress();
      }
      takeSample();
      if (displayOn) {
        delay(1000);
        redrawMenus();
      }
    }

    // if the RTC alarm was triggered, sampling is enabled and the alarm time matches the expected time, take a sample of the CO2 sensor, if CO2 sampling is enabled
    if (enableCO2Sampling && compareAlarmTimes(nextAlarmTime_CO2Device_minutes, nextAlarmTime_CO2Device_hours)) { //alarm matches next CO2 alarm time, so take a CO2 sample

      if (displayOn) {
        // enable display power
        drawSamplingInProgress();
      }

      takeCO2Sample();

      if (displayOn) {
        delay(1000);
        redrawMenus();
      }

    }
    rtcTimerInterrupt = false;
  }

  if (!runSampling && rtcTimerInterrupt) {
    //rtcTimerInterrupt = false;
    //clearRTCAlarm();
    DS3234.alarm1(true);
    //DS3234.alarm1(true); //clear alarm
    //DS3234.alarm2(true); //clear alarm2 for future code use
    rtcTimerInterrupt = false;
  }

  DS3234.alarm2(true); // ensure alarm 2 is always disabled, as it can trigger from alarm 1 sometimes


  //sleep when finished sampling
  startSleep();

}

//=========================================

void startSleep() {
  //SPI.end();
  //pinMode(14, INPUT_PULLUP);
  //digitalWrite(

  //Serial1.flush();

  if (debug) {
    Serial.println("Standby");
    //    Serial.print(rtcTimerInterrupt);
    //    Serial.print(space);
    //    Serial.println(wakeFlag);
    //Serial.flush();

    //    Serial.println(F("FreeRam"));
    //    Serial.println(FreeRam());
  }

  if (debug && !keepUSBConnected) {
    //SPI.end();
    Serial.flush();

    Serial.end();
  }

  if (displayOn) {
    sleep.interruptDisable();
    setDisplayTimout();     // enable display timout counter
    //sleep.interruptEnable();

    SPI.end();
    pinMode(22, OUTPUT);
    pinMode(A5, OUTPUT);
    digitalWrite(A5, HIGH);

    digitalWrite(22, HIGH);
    //digitalWrite(24, LOW);
    digitalWrite(PWR_SW_SD, LOW); // ensure NPN is off - disconnect SD card from ground (is bad but still need SPI for RTC)

    while (timeOnCount < timeOutLimitcCount) {
      //digitalWrite(PWRLED, HIGH);

      //      if (debug) {
      //        Serial.println(timeOnCount);
      //        Serial.flush();
      //      }
      sleep.idle();   //sleep idle does not seem to work. need to check for interrupts that are not being cleared


      if (rtcTimerInterrupt || wakeFlag) {
        if (wakeFlag) {
          timeOnCount = 0;
          disableDisplayTimout();
        }
        digitalWrite(PWR_SW_SD, LOW); //keep SD card power low
        digitalWrite(22, LOW);
        digitalWrite(A5, LOW);
        pinMode(22, INPUT);
        SPI.begin();



        return;
      }
    }
    if (timeOnCount >= timeOutLimitcCount) {
      // turnoff display

      //delay(5);
      disableDisplayTimout();
      timeOnCount = 0;
      inMenuNum = 0;
      ePaper.clearDisplay();
      ePaper.refresh();
      delay(1);
      gpioExpander.digitalWrite(0, LOW); // switch off display
      //ePaper.refresh();
      pinMode(A5, INPUT);
      digitalWrite(PWRLED, LOW);
      displayOn = false;
      //disableDisplayTimout();

    }
  }

  else if (!keepUSBConnected) {
    //switchSensorPower(false);
    SPI.end();
    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);
    digitalWrite(PWR_SW_SD, LOW); // ensure NPN is off - disconnect SD card from ground (is bad but still need SPI for RTC)
    sleep.interruptDisable();
    sleep.standby();
    digitalWrite(22, LOW);
    pinMode(22, INPUT);
    SPI.begin();
  }

  // for debuging purpose
  else if ( !displayOn) {

    sleep.interruptDisable();
    sleep.interruptEnable();
    while (!rtcTimerInterrupt && !wakeFlag) {
      delay(1);
    }
  }

  //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);

  //  if (debug) {
  //    pwrLED(7);
  //  }

}

//-------------------------------------------------------------------------------------------



void takeSample() {
  pwrLED(1);

  if (debug) {
    //Serial.begin(SERIAL_BAUD);
    //while (! Serial);
    Serial.write('\n');
    Serial.println(F("Sampling:"));
    //      Serial.print(F("RTC Alarm Status: "));
    //      Serial.println(rtcTimerInterrupt);
    //      Serial.print(F("Wake Alarm Status: "));
    //      Serial.println(saveToSDFlag);

  }

  clearRTCAlarm();
  // save time from RTC
  saveTime();



  /* ====== initialise Sensor conections ============== */

  //beginI2cDevices(); //reinitialises sensor connections after power down.
  //Serial.println(F(" devices OK"));


  /* ===== Retrieve Sample data ================= */
  //Start saving Data
  if (debug) {
    printTime(); // Print the new time
  }

  readSi7051();
  readSHT15();
  readADS1115();



  /* ========= Save Sample data ================= */
  //save data struct into FRAM for writing to SD later
  //for (int i = 0; i < 128; i++) { //for FRAM testing
  saveStructFram( framDataStartAddr + currAddrFRAM );
  currAddrFRAM += sizeof(DataSample);
  sampleCount++;
  fram.writeUInt16(SampleCountFramAddr, sampleCount); // update sample counter on FRAM

  // data from FRAM is saved to SD card
  if (sampleCount  >= maxSampleBufferSize) {
    saveToSD();
  }
  //}

  /* ======== Clear alarm ======================= */
  //update new sample alarm time

  nextAlarmTime_mainDevice_minutes = data.rtcMinutes + sampleRateMin; // can use time from data sample since it has been recently updated
  nextAlarmTime_mainDevice_hours = data.rtcHour + sampleRateHour;

  // catch if time overflows, just in case
  if ( nextAlarmTime_mainDevice_minutes > 59) {
    nextAlarmTime_mainDevice_minutes = nextAlarmTime_mainDevice_minutes - 60;
    nextAlarmTime_mainDevice_hours += 1;
  }
  if ( nextAlarmTime_mainDevice_hours > 23) {
    nextAlarmTime_mainDevice_hours = nextAlarmTime_mainDevice_hours - 24;
  }

  incrementAlarmTime();

  if (debug) {

    //      Serial.print(F("RTC Alarm Status: "));
    //      Serial.println(rtcTimerInterrupt);
    //      Serial.print(F("Wake Alarm Status: "));
    //      Serial.println(saveToSDFlag);
    Serial.println(F("finished"));
    Serial.flush();
  }


  pwrLED(1);
}


//-------------------------------------------------------------------------------------------
void takeCO2Sample() {
  uint8_t co2Status;
  uint8_t CO2SampleMinute = fram.readByte(samplePeriod_CO2_Addr + 1); // read salmple interval from FRAM
  uint8_t CO2SampleHour = fram.readByte(samplePeriod_CO2_Addr );
  uint8_t currentTime_minute = 0;
  uint8_t currentTime_hour = 0;
  pwrLED(1);

  if (debug) {
    //Serial.begin(SERIAL_BAUD);
    //while (! Serial);
    Serial.write('\n');
    Serial.println(F("Sampling CO2:"));


  }

  /* ============= save timestamp ================== */

  while ( CO2Samp.rtcHour != DS3234.getHour()) {
    CO2Samp.rtcHour = DS3234.getHour();
    delay(1);
  }

  while ( CO2Samp.rtcMinutes != DS3234.getMinute()) {
    CO2Samp.rtcMinutes = DS3234.getMinute();
    delay(1);
  }

  while ( CO2Samp.rtcSeconds != DS3234.getSecond()) {
    CO2Samp.rtcSeconds = DS3234.getSecond();
    delay(1);
  }

  while ( CO2Samp.rtcDate != DS3234.getDate()) {
    CO2Samp.rtcDate = DS3234.getDate();
    delay(1);
  }

  while ( CO2Samp.rtcMonths != DS3234.getMonth()) {
    CO2Samp.rtcMonths = DS3234.getMonth();
    delay(1);
  }

  while ( CO2Samp.rtcyears != DS3234.getYear()) {
    CO2Samp.rtcyears = DS3234.getYear();
    delay(1);
  }

  /* ========= Read CO2 sensor ============ */

  co2Status = readCO2();
  if (co2Status == 2) { // CO2 sensor retunred with successful sample reading from sensor
    //for (int i = 0; i < 128; i++) {
    saveStructCO2Fram(CO2DataBufferStartAddr + (co2SampleCount * sizeof(CO2Samp)) );
    co2SampleCount++;
    fram.writeUInt16(SampleCount_CO2_FramAddr, co2SampleCount ); // update sample count
    if (debug) {
      Serial.print(F("CO2 sample count: "));
      Serial.println(co2SampleCount);
    }

    // update buffer count for use with graphing
    if ( fram.readUInt16(framSampleBufferCount_CO2_Addr) < maxCO2SampleBufferSize) {
      fram.writeUInt16(framSampleBufferCount_CO2_Addr, co2SampleCount);
    }
    //}
    nextAlarmTime_CO2Device_minutes = CO2Samp.rtcMinutes + CO2SampleMinute;
    nextAlarmTime_CO2Device_hours = CO2Samp.rtcHour + CO2SampleHour;

  }

  else if (co2Status == 1) { // CO2 sensor has been turned on, and now shall warm up before taking sample
    nextAlarmTime_CO2Device_minutes = CO2Samp.rtcMinutes + 2;
    nextAlarmTime_CO2Device_hours = CO2Samp.rtcHour;
  }

  // catch if time overflows, just in case
  if ( nextAlarmTime_CO2Device_minutes > 59) {
    nextAlarmTime_CO2Device_minutes = nextAlarmTime_CO2Device_minutes - 60;
    nextAlarmTime_CO2Device_hours += 1;
  }
  if ( nextAlarmTime_CO2Device_hours > 23) {
    nextAlarmTime_CO2Device_hours = nextAlarmTime_CO2Device_hours - 24;

  }

  incrementAlarmTime();



  /* ========= Save CO2 Sample data ================= */


  if ( co2SampleCount >= maxCO2SampleBufferSize) {
    if (debug) {
      Serial.print(F("CO2 sample count before saving: "));
      Serial.println(co2SampleCount);
    }
    saveCO2ToSD();
  }

  /* ======== Clear alarm ======================= */
  //clear RTC alarm
  //  while (DS3234.alarm2(false)) {
  //    DS3234.alarm2(true);
  //  }


  //  if (debug) {
  //    Serial.print(F("final CO2 ON status: "));
  //    Serial.println(CO2SensorOn);
  //    Serial.print(F("final Alarm 2 status: "));
  //    Serial.println(DS3234.alarm2(false));
  //    //    if (CO2SensorOn) {
  //    //      sendRequest();
  //    //      Serial.print(F("CO2 reading"));
  //    //      Serial.println(getValue(response));
  //    //    }
  //  }




  if (debug) {

    Serial.println(F("finished"));
    Serial.flush();
  }

  pwrLED(1);
}

//-------------------------------------------------------------------------------------------

void incrementAlarmTime() {
  uint8_t currentTime_hour = 0;
  uint8_t currentTime_minute = 0;

  uint8_t newMinute = 0;
  uint8_t newHour = 0;
  int32_t diffMain = 0;
  int32_t diffCO2 = 0;
  int32_t longtime = 0;
  const char c = ':';

  bool nextDeviceAlarmValid = false;
  bool nextCO2AlarmValid = false;

  //DS3234.update();   // ADD CATCH FOR OLD TIME STAMPS FOR CO2 DEVICE


  while ( currentTime_hour != DS3234.getHour()) {
    currentTime_hour = DS3234.getHour();
    delay(1);
  }
  while ( currentTime_minute != DS3234.getMinute()) {
    currentTime_minute = DS3234.getMinute();
    delay(1);
  }

  if (debug) {
    Serial.print(F("\nNEXT Device alarm time: "));
    Serial.print(nextAlarmTime_mainDevice_hours);
    Serial.print(c);
    Serial.println(nextAlarmTime_mainDevice_minutes);

    Serial.print(F("NEXT CO2 alarm time: "));
    Serial.print(nextAlarmTime_CO2Device_hours);
    Serial.print(c);
    Serial.println(nextAlarmTime_CO2Device_minutes);

    Serial.print(F("Current time: "));
    Serial.print(currentTime_hour);
    Serial.print(c);
    Serial.println(currentTime_minute);
  }

  longtime = (int32_t(currentTime_hour) * 60) + int32_t(currentTime_minute);
  if ( (longtime % 100) == 0) {
    longtime -= 60;
  }
  //  if (debug) {
  //    Serial.println(longtime);
  //  }


  // check if the next CO2 alarm time is valid. times greater than 24 hours and or 60 minutes are invalid
  if ( (nextAlarmTime_CO2Device_hours < 24) && (nextAlarmTime_CO2Device_minutes < 60)) {
    nextCO2AlarmValid = true;
    if (debug) {
      Serial.println(F("CO2 alarm time valid!"));
    }
  }


  // Now find time difference for next main device alarm
  diffMain = (int32_t(nextAlarmTime_mainDevice_hours) * 60);
  diffMain += int32_t(nextAlarmTime_mainDevice_minutes);
  //  if (debug) {
  //    Serial.println(diffMain);
  //  }
  diffMain -= longtime;

  // if the alarm times roll over and give negative differences, find the absolute time difference
  if (diffMain < 0) {
    diffMain = diffMain + (24 * 60);
  }


  if (debug) {
    Serial.print(F("Device alarm time difference: "));
    Serial.println(diffMain);

  }

  // find time difference for next valid CO2 alarm
  if (nextCO2AlarmValid && enableCO2Sampling) {
    diffCO2 = (int32_t(nextAlarmTime_CO2Device_hours) * 60 );
    diffCO2 += int32_t(nextAlarmTime_CO2Device_minutes);
    //    if (debug) {
    //      Serial.println(diffCO2);
    //    }

    diffCO2 -= longtime;

    if (debug) {
      Serial.print(F("CO2 alarm time difference: "));
      Serial.println(diffCO2);
    }
  }


  if ( nextCO2AlarmValid && enableCO2Sampling) {
    if (debug) {
      Serial.print(F("Comparing times.. "));
    }

    // compare the differences in time and chose the smaller of the two
    if ( diffMain < diffCO2) {
      // main device alarm is smaller, thus it is the next alarm
      newMinute = nextAlarmTime_mainDevice_minutes;
      newHour = nextAlarmTime_mainDevice_hours;
    }
    else {
      // CO2 alarm is smaller, thus it will be the next alarm
      newMinute = nextAlarmTime_CO2Device_minutes;
      newHour = nextAlarmTime_CO2Device_hours;
    }
  }

  else {  // otherwise if CO2 is disabled or invalid, use main devices next alarm time
    newMinute = nextAlarmTime_mainDevice_minutes;
    newHour = nextAlarmTime_mainDevice_hours;
  }


  // catch if time overflows, just in case
  if ( newMinute > 59) {
    newMinute = newMinute - 60;
    newHour += 1;
  }
  if ( newHour > 23) {
    newHour = newHour - 24;

  }

  // set alarm
  DS3234.enableAlarmInterrupt(true, false);
  DS3234.setAlarm1(0, newMinute , newHour );


  if (debug) {
    Serial.print(F("new alarm time: ") );
    Serial.print(newHour );
    Serial.print(c);
    Serial.println(newMinute);
  }
}


//-------------------------------------------------------------------------------------------

bool compareAlarmTimes(uint8_t minutes, uint8_t hours) {
  uint8_t currMin = 0;
  uint8_t currHour = 0;

  DS3234.update();

  while ( currHour != DS3234.getHour()) {
    currHour = DS3234.getHour();
    delay(1);
  }

  while ( currMin != DS3234.getMinute()) {
    currMin = DS3234.getMinute();
    delay(1);
  }

  if ( currMin >= minutes && currHour == hours) { // used a less than to catch alarm if alarm missed due to transferring data to pc
    return true;
  }

  return false;

}


//-------------------------------------------------------------------------------------------


//void incrementAlarm2Time(bool firstRun) {
//  uint8_t newMinute = 0;
//  uint8_t newHour = 0;
//  uint8_t CO2SampleMinute = fram.readByte(samplePeriod_CO2_Addr + 1); // read salmple interval from FRAM
//  uint8_t CO2SampleHour = fram.readByte(samplePeriod_CO2_Addr );
//
//  if (firstRun ) {
//    CO2SampleMinute = 1;
//    CO2SampleHour = 0;
//  }
//  // ensure correct time is read, just to check if spi transfer did not have errors
//  while ( newMinute != (DS3234.getMinute() + CO2SampleMinute) ) {
//    newMinute = DS3234.getMinute() + CO2SampleMinute;
//  }
//
//  while (newHour != (DS3234.getHour() + CO2SampleHour) ) {
//    newHour = DS3234.getHour() + CO2SampleHour;
//  }
//
//  //set alarm to trigger again after the sample interval
//
//
//  if ( newMinute > 59) {
//    newMinute = newMinute - uint8_t(60);
//    newHour += 1;
//  }
//  if ( newHour > 23) {
//    newHour = newHour - uint8_t(24);
//  }
//  DS3234.setAlarm2( newMinute , newHour );
//
//
//  if (debug) {
//    Serial.print(F("CO2 nextTime: "));
//    Serial.print(newHour );
//    Serial.print(':');
//    Serial.println(newMinute);
//  }
//}

//-------------------------------------------------------------------------------------------

void clearRTCAlarm() {
  byte count = 0;


  while ( DS3234.alarm1(false)) {
    //pwrLED(2);
    if (count >= 3) {
      error(0x2000); // on fourth attempt, save error as RTC allarm not clearing and fail.
      //pwrLED(20);
      delay(200);
      //fail();
    }

    DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
    DS3234.alarm1(true); //clear alarm
    //DS3234.alarm2(true); //clear alarm2 for future code use
    //DS3234.update();
    count++;

    if (debug) {
      Serial.println("RTC Alarm 1 CLEARED");
    }

  }


}


//-------------------------------------------------------------------------------------------

/*
    Highly device dependant Code. will only work with Atmel SAMD21 microcontrollers!!!!

    Code originally and Modified from https://github.com/maxbader/arduino_tools
*/
void setDisplayTimout() {
  PM->APBCMASK.reg |= PM_APBCMASK_TC3; // enable counter 3 Clock
  delay(1);

  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  // The type cast must fit with the selected timer mode
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCCx
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // TC->PER.reg = 0xFF;   // Set counter Top using the PER register but the 16/32 bit timer counts allway to max
  // while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CC[0].reg = 0x1F;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Interrupts
  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow
  TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync




}

/*
    Highly device dependant Code. will only work with Atmel SAMD21 microcontrollers!!!!
*/
void disableDisplayTimout() {
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; // disable timer counter
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
  TC->INTENSET.reg = 0;              // disable all interrupts

  PM->APBCMASK.reg &= ~PM_APBCMASK_TC3; // disable clock to timer counter 3
  timeOnCount = 0;

}


//-------------------------------------------------------------------------------------------

void readSHT15()
{
  float tempC = 0;
  float humidity = 0;
  //float humidityExternalCompensated = 0;

  // Read values from the sensor
  //for (int i = 0; i < averageNum; i++) {
  tempC =  sht15.readTemperatureC();
  humidity =  sht15.readHumidity();
  //humidityExternalCompensated = humidityExternalCompensated + sht15.readHumidityRaw(DataSamples[DataSampleNum].Si7051_T);
  //}

  if (tempC > 99.0) { // See datasheet: any readings above 99% RH means the air is fully saturated and must be processed as 100% RH
    tempC = 100.0;
  }
  data.SHT15_T = tempC ;
  data.SHT15_H = humidity ;
  //DataSamples[DataSampleNum].SHT15_H_Comp = humidityExternalCompensated / ((float)averageNum);
  if (debug) {
    printOutSHT15();
  }

}

//-------------------------------------------------------------------------------------------

void readSi7051() {

  data.Si7051_T = si7051.readTemperature();

  if (debug) {
    Serial.print(F(" (Si7050) Temp= "));
    Serial.print(data.Si7051_T);
    Serial.println(F("C"));
    ////Serial.print(si7051.readTemperature());
  }
}






//-------------------------------------------------------------------------------------------

void enableCO2() {
  gpioExpander.digitalWrite(6, LOW); // ensure pin is pulled down before switching to input
  gpioExpander.pinMode(6, INPUT);
  gpioExpander.digitalWrite(5, HIGH);

  CO2SensorOn = true;

}


//-------------------------------------------------------------------------------------------

void disableCO2() {
  //gpioExpander.digitalWrite(6, LOW); // ensure pin is pulled down before switching to input
  //gpioExpander.pinMode(6, OUTPUT);
  //gpioExpander.digitalWrite(6, LOW); // ensure pin is pulled down before switching to input
  gpioExpander.digitalWrite(5, LOW);

  CO2SensorOn = false;
  DS3234.alarm2(true);

}

uint8_t checkCO2ConnectionType() {
  bool UART = false;
  bool I2C = false;
  gpioExpander.pinMode(7, INPUT_PULLUP);

  if ( gpioExpander.digitalRead(7) == 0) { // connection detect pin is pulled down so CO2 board must be using UART
    UART = true;
  }
  gpioExpander.pinMode(7, INPUT);

  if ( gpioExpander.digitalRead(7) == 1) { // connection detect pin is pulled up so CO2 board must be using I2C
    I2C = true;
  }

  if ( UART && !I2C) {
    if (debug) {
      Serial.println(F("UART CO2 Connection found"));
    }
    return 1;
  }
  else if ( !UART && I2C) {
    if (debug) {
      Serial.println(F("I2C CO2 Connection found"));
    }
    return 2;
  }

  if (debug) {
    Serial.println(F("NO CO2 Connection found"));
  }
  return 0; // return 0 if connection could not be determined

}



//-------------------------------------------------------------------------------------------

/* NOTE: calling DS3234.update() seems to clear alarm flags

*/

uint8_t readCO2() {
  uint8_t connectionType;
  uint8_t hours = 0;
  uint8_t minutes = 0;
  bool success = false;

  CO2Samp.K30_CO2 = 0;


  //  if (debug) {
  //    Serial.print(F("CO2 ON status: "));
  //    Serial.println(CO2SensorOn);
  //    Serial.print(F("Alarm 2 status: "));
  //    Serial.println(DS3234.alarm2(false));
  //    //    if (CO2SensorOn) {
  //    //      sendRequest();
  //    //      Serial.print(F("CO2 reading"));
  //    //      Serial.println(getValue(response));
  //    //    }
  //  }

  if ( !CO2SensorOn) {
    enableCO2();
    //DS3234.enableAlarmInterrupt(true, true);  // enables interrupt from square wave pin on RTC to trigger on alarm event
    //DS3234.setAlarm2( minutes + 2, hours); //set alarm to trigger again after a 2 minute warm up period

    if (debug) {
      Serial.println(F("  Enabling CO2"));
    }

    return 1;
  }

  // if co2 sensor is on, get a sample
  else if ( CO2SensorOn ) { // now actually get the sample from the CO2 sensor
    //    if (debug) {
    //      //Serial.println(F("CO2 should sample now"));
    //      //sendRequest();
    //      Serial.print(F("CO2 status pin: "));
    //      Serial.println(gpioExpander.digitalRead(6));
    //
    //    }



    if (  gpioExpander.digitalRead(6) == 1) { // if status pin is Low there is an error with the CO2 sensor, othewise read CO2 sensor value
      if (debug) {
        Serial.println(F("CO2 Status OK!"));
      }
      connectionType = checkCO2ConnectionType(); //check CO2 sensor connection type for UART or I2C

      if ( connectionType == 1 ) { // connection is UART
        Serial1.begin(9600);
        sendRequest();
        CO2Samp.K30_CO2 = getValue(response);
        Serial1.end();

        success = true;
        if (debug) {
          Serial.println(F("CO2 sampled using UART"));
        }
      }


      else if (connectionType == 2) { // connection is I2C
        if (!K30_CO2_SENS.begin(0x68) ) {
          error(0x0400);
        }
        CO2Samp.K30_CO2 = K30_CO2_SENS.readCO2();

        success = true;
        if (debug) {
          Serial.println(F("CO2 sampled using I2C"));
        }
      }

      else if ( connectionType == 0) {
        disableCO2();

        success = false;
      }

    }
    else {
      if (debug) {
        Serial.println(F("CO2 sensor Error Pin LOW!"));
      }
      CO2Error = true;
      success = false;
    }
    disableCO2();


    if (debug) {

      Serial.print(F(" (K30) CO2= "));
      Serial.print(CO2Samp.K30_CO2);
      Serial.println(F(" PPM"));
    }
  }

  if (success) {
    return 2;
  }
  else {
    return 0;
  }

}


void sendRequest()
{
  //   Serial1.flush();
  while (!Serial1.available()) //keep sending request until we start to get a response
  {

    Serial1.write(CO2buff, 7);

    delay(50);
  }

  int timeout = 0; //set a timeoute counter
  while (Serial1.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10)   //if it takes to long there was probably an error
    {
      while (Serial1.available()) //flush whatever we have
        Serial1.read();

      break;                        //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response[i] = Serial1.read();

  }

}



unsigned long getValue(byte packet[])
{
  int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
  int low = packet[4];                         //low byte for value is 5th byte in the packet


  unsigned long val = high * 256 + low;              //Combine high byte and low byte with this formula to get value
  return val * valMultiplier;
}

//-------------------------------------------------------------------------------------------

void readADS1115() {
  int32_t temp;


  /*measure single ended conections*/
  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    temp = 0; //initialise temporary variable

    /* if channel is enabled in enable mask and single ended reading is enabled, read from the channel*/
    if (ADS_BitMask_enable & (1 << i)  && ADS_BitMask_single & (1 << i)) {

      setADS1115Gain( ADS1115Gain[i] ); //set gain defined in global field

      /*do some averaging*/
      for (int n = 0; n < averageNum ; n++) {
        temp = temp + ads.readADC_SingleEnded(i);
      }
      temp = temp / averageNum;
    }

    data.adsSingle[i] = (uint16_t)temp; // save averaged (or 0 to ensure random values are not saved) variable in DataSample struct

  }

  /*measure double ended conections*/
  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    temp = 0;
    if (i == 0 || i == 1) {
      data.adsDouble[i] = (uint16_t)temp;     // write 0 if double ended readings are disabled to ensure uninitialised values are overwritten in the data Struct
    }



    if ( (ADS_BitMask_enable & (1 << i)) && (ADS_BitMask_single & (0 << i)) ) { // if channel is enabled in enable mask and a single channel read is disabled
      setADS1115Gain( ADS1115Gain[i] );     //set ADS1115 gain defined in global field

      if ( i == 0 ) {
        //do some averaging
        for (int n = 0; n < averageNum ; n++) {
          temp = temp + ads.readADC_Differential_0_1();
        }
        temp = temp / averageNum;
        data.adsDouble[i] = (uint16_t)temp; // save averaged (or 0 to ensure random values are not saved) variable in DataSample struct
      }
      /* run double ended reading if it is channel 0 or 2 */
      else if ( i == 2 ) {
        /*do some averaging*/
        for (int n = 0; n < averageNum ; n++) {
          temp = temp + ads.readADC_Differential_2_3();
        }
        temp = temp / averageNum;
        data.adsDouble[i / 2] = (uint16_t)temp; // save averaged variable in DataSample struct
      }
    }
  }
}

//-------------------------------------------------------------------------------------------

/* Function which sets a gain on ADS1115 for a given value, sets globalTempInt to index position
   of reading resolution in GainResolution field
*/
void setADS1115Gain(int8_t gain) {
  switch (gain) {
    case ADS_GAIN_TWOTHIRDS :
      ads.setGain(GAIN_TWOTHIRDS);
      break;
    case ADS_GAIN_ONE :
      ads.setGain(GAIN_ONE);
      break;
    case ADS_GAIN_TWO :
      ads.setGain(GAIN_TWO);
      break;
    case ADS_GAIN_FOUR :
      ads.setGain(GAIN_FOUR);
      break;
    case ADS_GAIN_EIGHT :
      ads.setGain(GAIN_EIGHT);
      break;
    case ADS_GAIN_SIXTEEN :
      ads.setGain(GAIN_SIXTEEN);
      break;
    default:
      ads.setGain(GAIN_TWOTHIRDS);
      break;
  }
}

//-------------------------------------------------------------------------------------------
void printOutSHT15()
{
  if (debug) {
    Serial.print(F(" (SHT15) Temp = "));
    //  //Serial.print(tempF);
    //  //Serial.print("F, ");
    Serial.print(data.SHT15_T);
    Serial.println("C");
    Serial.print(F(" (SHT15) Humidity = "));
    Serial.print(data.SHT15_H);
    Serial.println('%');
    //  //Serial.print(F(" (SHT15) Humidity, external Compensation = "));
    //  //Serial.print(DataSamples[DataSampleNum].SHT15_H_Comp);
    //  //Serial.print(F("%  using: "));
    //  //Serial.print(DataSamples[DataSampleNum].Si7051_T);
    //  //Serial.println("C");
  }
}

//-------------------------------------------------------------------------------------------

void saveTime() {

  // use loops to help avoid SPI tranfer errors.

  while ( data.rtcHour != DS3234.getHour()) {
    data.rtcHour = DS3234.getHour();
    delay(1);
  }

  while ( data.rtcMinutes != DS3234.getMinute()) {
    data.rtcMinutes = DS3234.getMinute();
    delay(1);
  }

  while ( data.rtcSeconds != DS3234.getSecond()) {
    data.rtcSeconds = DS3234.getSecond();
    delay(1);
  }

  while ( data.rtcDate != DS3234.getDate()) {
    data.rtcDate = DS3234.getDate();
    delay(1);
  }

  while ( data.rtcMonths != DS3234.getMonth()) {
    data.rtcMonths = DS3234.getMonth();
    delay(1);
  }

  while ( data.rtcyears != DS3234.getYear()) {
    data.rtcyears = DS3234.getYear();
    delay(1);
  }

  //rtcDayString = DS3234.dayStr();

}
//-------------------------------------------------------------------------------------------

void printTime()
{
  Serial.print(data.rtcHour);
  Serial.print(':');
  if (data.rtcMinutes < 10) {
    //Serial.print('0'); // Print leading '0' for minute
  }
  Serial.print(data.rtcMinutes);
  Serial.print(':');
  if (data.rtcSeconds < 10) {
    //Serial.print('0'); // Print leading '0' for second
  }
  Serial.print(data.rtcSeconds); // Print second
  //  if (rtc.is12Hour()) // If we're in 12-hour mode
  //  {
  //    // Use rtc.pm() to read the AM/PM state of the hour
  //    if (rtc.pm()) //Serial.print(" PM"); // Returns true if PM
  //    else //Serial.print(" AM");
  //  }
  Serial.print(F(" | "));
  // Few options for printing the day, pick one:
  ////Serial.print(DataSamples[DataSampleNum].rtcDayString); // Print day string
  ////Serial.print(rtc.day()); // Print day character
  Serial.print(DS3234.day()); // Print day integer (1-7, Sun-Sat)
  Serial.print(F(" - "));
  Serial.print(data.rtcDate);
  Serial.print('/');
  Serial.print(data.rtcMonths);
  Serial.print('/');
  Serial.println(data.rtcyears);

}

//-------------------------------------------------------------------------------------------





/* note: the in array should have increasing values
   this method was sourced from the Arduino forums and is not my work
*/
float FmultiMap(float val, float * _in, float * _out, uint8_t size) {
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size - 1]) return _out[size - 1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while (val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}




//-------------------------------------------------------------------------------------------

//void switchSensorPower(bool turnOn) {
//  if (turnOn) {
//    digitalWrite(PWR_SW_SENS, LOW); // Switch on power to Sensors
//  }
//  else {
//    Wire.end();
//    //digitalWrite(PWR_SW_SENS, HIGH); // Switch off power to Sensors
//    //digitalWrite(SHT15_SCK, LOW);
//    //digitalWrite(SHT15_DATA, LOW);
//
//    //digitalWrite(A4, LOW);
//    //digitalWrite(A5, LOW);
//  }
//}

//-------------------------------------------------------------------------------------------





// used to re-initialise all sensors that rely on I2C or SPI modules
void beginI2cDevices() {


  fram.begin(0x50);
  si7051.begin(0x40); //Si7051 library does not check for proper connection
  ads.begin(); //Si7051 library does not check for proper connection
  //gpioExpander.begin();

  //if (!K30_CO2_SENS.begin(0x68) ) {
  //  error(0x0400);
  //}
  //if (debug) {
  //  Serial.println(F("I2C Initialised"));
  //}

}


void setupGPIOExpander() {
  gpioExpander.pinMode(0, OUTPUT);

  gpioExpander.pinMode(1, INPUT);
  gpioExpander.pinMode(2, INPUT);
  gpioExpander.pinMode(3, INPUT);
  gpioExpander.pinMode(4, INPUT);

  gpioExpander.pinMode(5, OUTPUT);
  gpioExpander.pinMode(6, OUTPUT);
  gpioExpander.pinMode(7, OUTPUT);

  gpioExpander.digitalWrite(0, LOW);
  gpioExpander.digitalWrite(5, LOW);
  gpioExpander.digitalWrite(6, LOW);
  gpioExpander.digitalWrite(7, LOW);


  //gpioExpander.pullUp(0, 1);
  gpioExpander.pullUp(1, 1);
  gpioExpander.pullUp(2, 1);
  gpioExpander.pullUp(3, 1);
  gpioExpander.pullUp(4, 1);
  //gpioExpander.pullUp(5, 1);
  //gpioExpander.pullUp(6, 1);
  //gpioExpander.pullUp(7, 1);

  gpioExpander.setGPINTEN(B00011110); // set interrupt on GPIO 2 to enabled
  gpioExpander.setINTCON(B00011110); // set pin to be compared against internal DEFVAL register
  gpioExpander.setDEFVAL(B00011110); // set normal pin state to be compared to 1
  uint8_t iocon = gpioExpander.readIOCON();
  iocon |= 0x04;  //set interrupt pin as open drain
  iocon &= 0xFD;  //set interrupt polarity to active low

  gpioExpander.setIOCON( iocon); // write changes to IOCON register
}

//-------------------------------------------------------------------------------------------

void printBits(byte myByte) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

//void defineUnusedPins() {
//  for (byte i = 0; i < 20; i++) { //make all pins inputs with pullups enabled
//    pinMode(i, INPUT_PULLUP);
//  }
//}


//-------------------------------------------------------------------------------------------


// dir defines if a upper or lower bounds check is performed
void checkPower(int pin, int bound, bool dir) {
  int temp;
  //saveADC = ADCSRA;

  PM->APBCMASK.reg |= 0x00010000; // enable the ADC Clock
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC

  pinMode(pin, INPUT);

  temp = analogRead(pin);
  if (dir) {
    while (temp <= bound) {
      //delay(1);
      temp = analogRead(pin);
      if (debug) {
        Serial.print(temp);
        Serial.print(F(".."));
      }
    }
  }
  else {
    while (temp >= bound) {
      //delay(1);
      temp = analogRead(pin);
      if (debug) {
        Serial.print(temp);
        Serial.print(F(".."));
      }
    }

  }

  ADC->CTRLA.bit.ENABLE = 0;       // disable ADC
  PM->APBCMASK.reg &= ~0x00010000; // disable the ADC Clock
}



//-------------------------------------------------------------------------------------------

/* This method chrashes if sd card is removed arfter recieving start command from pc to send data

*/

// Method that checks if SD card is present and prevents further execution of code until one is present
void checkSDCard() {
  uint8_t tryCount = 0;
  int sdCardPresent;

  pinMode(SD_card_detect, INPUT);
  sdCardPresent = digitalRead(SD_card_detect);

  while (sdCardPresent == 0) {
    if (tryCount >= 2) {
      if (debug) {
        Serial.println(F(" SD card not found, Device Halted"));
        Serial.flush();
      }
      error(0x0004);
      //pwrLED(4);
      startSleep(); // sleep until card is detected - woken from sleep by rtc

      clearRTCAlarm();
    }

    digitalWrite(PWR_SW_SD, HIGH); // Switch on power to Sensors
    sdCardPresent = digitalRead(SD_card_detect);
    if (sdCardPresent == 1) {
      delay(3000);
      if (debug) {
        Serial.println(F(" SD card found, Device Resuming"));
      }

      //digitalWrite(PWR_SW_SENS, HIGH); // Switch on power to Sensors

    }
    tryCount++;
  }
}


//-------------------------------------------------------------------------------------------



bool startSDCard(const char* file) {
  uint8_t beginAttempts = 0;
  bool initFlag = false;
  //startup SD card or ensure SD card is ready
  digitalWrite(PWR_SW_SD, HIGH); //Switch on power to SD card, NOTE NPN MOSFET
  checkPower(SD_POWER_LINE, SdADCThresh, false);
  //delay(30);

  checkSDCard();

  while (!initFlag && beginAttempts < 5) { //, SPI_HALF_SPEED

    if (debug) {
      Serial.print(F("SD INIT FAIL: Try "));
      Serial.println(beginAttempts + 1);
    }
    if ( SD.begin(SD_CS_PIN, SPI_DIVISOR_GLOBAL)) {
      initFlag = true;
    }
    beginAttempts++;
    delay(100);
  }
  if ( !initFlag) {
    error(0x4000);
  }
  //SPI.setClockDivider(SPI_CLOCK_DIV8);
  //digitalWrite(SD_CS_PIN, LOW); // not needed since handled by library
  //Open file
  //delay(1);
  if (!SD.exists(file)) {
    if (debug) {
      Serial.print(F("cannot find "));

      Serial.write(file);
      Serial.println(F(" file on SD card!"));
    }
    return false;
  }
  return true;
}



//-------------------------------------------------------------------------------------------

// Handles saving data to the SD card
void saveToSD() {
  //LookupTable loadLookup;
  float tempFloat;
  uint16_t maxSample = fram.readUInt16(SampleCountFramAddr);

  if ( !startSDCard(fileName)) {
    if (debug) {
      Serial.println(F("Error Opening SD Card"));
    }
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    error(0x0010);
    //return;
  }

  if ( maxSample == 0) {
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return;
  }
  if ( !txtFile.open(fileName, (O_RDWR   | O_CREAT | O_AT_END)) ) {
    error(0x0010);
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return;
  }


  for (uint16_t index = 0; index < maxSample && index <= maxSampleBufferSize; index++ ) {

    //load data sample Struct from FRAM at an address offset by the data start address defined globally

    loadStructFram(framDataStartAddr + index * sizeof(DataSample));
    if (debug) {
      Serial.println(F("Loaded Data Sample from Fram"));
    }

    // Save Time Stamp - Assumes 24 hour mode
    txtFile.print(data.rtcHour ); // Print hour + asciiTimeOffset
    txtFile.write(':');

    if ((data.rtcMinutes) < 10) {
      txtFile.write('0'); // Print leading '0' for minute
    }
    txtFile.print(data.rtcMinutes ); // Print minute
    txtFile.write(':');

    if ((data.rtcSeconds) < 10) {
      txtFile.write('0'); // Print leading '0' for second
    }
    txtFile.print((data.rtcSeconds )); // Print second

    txtFile.write(commar);

    //  print date
    txtFile.print((data.rtcDate));
    txtFile.write('/');
    //  print month
    txtFile.print( (data.rtcMonths ));
    txtFile.write('/');
    // Print year
    txtFile.print((data.rtcyears));
    txtFile.write(commar);

    ////Serial.println(F("Wrote Time"));
    // ===== Save sensor values ==============================================
    txtFile.print(data.Si7051_T);
    txtFile.write(commar);

    txtFile.print(data.SHT15_T);
    txtFile.write(commar);

    txtFile.print(data.SHT15_H);
    txtFile.write(commar);

    //txtFile.print(data.K30_CO2);
    //txtFile.write(commar);

    for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
      tempFloat = ((float)(data.adsSingle[i])) * GainResolution[ADS1115Gain[i]];
      if ( ADS_BITMASK_USE_LOOKUPTABLE & (1 << i)) {
        tempFloat = FmultiMap( tempFloat, tempLookup.inputRange, tempLookup.outputRange, tableElementSize);
      }
      txtFile.print( tempFloat );
      txtFile.write(commar);
    }



    for (int i = 0; i < MAX_ADS1115_INPUTS; i = i + 2) {
      // only print double ended variables
      tempFloat = 0;

      if (i == 0 && !(ADS_BitMask_single & (1 << i) )) {

        tempFloat = float(data.adsDouble[i]) * GainResolution[ADS1115Gain[i]];
        if (ADS_BITMASK_USE_LOOKUPTABLE & (1 << (i + 4) )) {
          tempFloat =  FmultiMap( tempFloat , tempLookup.inputRange, tempLookup.outputRange, tableElementSize);
        }
      }
      else if (!(ADS_BitMask_single & (1 << i)) ) {

        tempFloat = float(data.adsDouble[i / 2]) * GainResolution[ADS1115Gain[i]];
        if ( ADS_BITMASK_USE_LOOKUPTABLE & (1 << (i + 4))) {
          //EEPROM.get(tableEEPROMAdrress[i]*sizeof( loadLookup) , loadLookup);
          tempFloat =  FmultiMap( tempFloat , tempLookup.inputRange, tempLookup.outputRange, tableElementSize);
        }
      }
      txtFile.print( tempFloat);
      txtFile.write(commar);
    }
    txtFile.write('\n');

  }
  txtFile.sync();
  txtFile.close(); // close file to prevent issues when using sleep mode
  //===================================================================================================
  if (debug) {
    Serial.print(F("Current Errors: "));
    printError();
  }
  //fram.writeInt16(errCodeFramAddr, 0);

  //update FRAm
  currAddrFRAM = 0;
  sampleCount = 0;
  fram.writeUInt16(SampleCountFramAddr, sampleCount); // update sample counter on FRAM, clearing previous sample data is not needed



  // switch power off to sd card
  digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
  if (debug) {
    Serial.println(F("Done saving to SD card"));
  }

  pwrLED(3);

}


//==================================================================

// Handles saving data to the SD card
void saveCO2ToSD() {
  //LookupTable loadLookup;

  uint16_t maxSample = fram.readUInt16(SampleCount_CO2_FramAddr);
  //Serial.println(maxSample);

  if ( !startSDCard(CO2fileName)) {
    if (debug) {
      Serial.println(F("Error Opening SD Card"));
    }
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    error(0x0010);
    //return;
  }

  if ( maxSample == 0) {
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return;
  }
  if ( !txtFile.open(CO2fileName, (O_RDWR   | O_CREAT | O_AT_END)) ) {
    error(0x0010);
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return;
  }


  for (uint16_t index = 0; index < maxSample && index <= maxCO2SampleBufferSize; index++ ) {

    //load data sample Struct from FRAM at an address offset by the data start address defined globally

    loadStructCO2Fram(CO2DataBufferStartAddr + index * sizeof(CO2Samp));
    if (debug) {
      Serial.println(F("Loaded Data Sample from Fram"));
    }

    // Save Time Stamp - Assumes 24 hour mode
    txtFile.print(CO2Samp.rtcHour ); // Print hour + asciiTimeOffset
    txtFile.write(':');

    if ((CO2Samp.rtcMinutes) < 10) {
      txtFile.write('0'); // Print leading '0' for minute
    }
    txtFile.print(CO2Samp.rtcMinutes ); // Print minute
    txtFile.write(':');

    if ((CO2Samp.rtcSeconds) < 10) {
      txtFile.write('0'); // Print leading '0' for second
    }
    txtFile.print((CO2Samp.rtcSeconds )); // Print second

    txtFile.write(commar);

    //  print date
    txtFile.print((CO2Samp.rtcDate));
    txtFile.write('/');
    //  print month
    txtFile.print( (CO2Samp.rtcMonths ));
    txtFile.write('/');
    // Print year
    txtFile.print((CO2Samp.rtcyears));
    txtFile.write(commar);

    ////Serial.println(F("Wrote Time"));
    // ===== Save sensor values ==============================================


    txtFile.print(CO2Samp.K30_CO2);
    txtFile.write(commar);

    txtFile.write('\n');

  }
  txtFile.sync();
  txtFile.close(); // close file to prevent issues when using sleep mode
  //===================================================================================================
  if (debug) {
    Serial.print(F("Current Errors: "));
    printError();
  }

  //update FRAm
  co2SampleCount = 0;
  fram.writeUInt16(SampleCount_CO2_FramAddr, co2SampleCount); // update sample counter on FRAM, clearing previous sample data is not needed


  // switch power off to sd card
  digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
  if (debug) {
    Serial.println(F("Done saving to SD card"));
  }

  pwrLED(4);

}


//==================================================================


bool clearDataFromSd() {
  // clear Normal Data file
  if ( !startSDCard(fileName)) {
    if (debug) {
      Serial.println(F("Error Opening SD Card"));
    }
    error(0x0010);
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return false;
  }

  if (SD.remove(fileName)) {
    // if the file was successfully removed, create a new file and close it.
    if ( !txtFile.open(fileName, (O_RDWR   | O_CREAT | O_AT_END)) ) {
      error(0x0010);
      digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
      return false;
    }
    else {
      txtFile.close();

      //update FRAm
      currAddrFRAM = 0;
      sampleCount = 0;
      fram.writeUInt16(SampleCountFramAddr, sampleCount); // update sample counter on FRAM, clearing previous sample data is not needed



      // switch power off to sd card
      //digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
      if (debug) {
        Serial.println(F("Done saving to SD card"));
      }
      //return true;
    }
  }
  else {
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card since file could not be removed
    return false;
  }


  // clear CO2 data file

  if ( !startSDCard(CO2fileName)) {
    if (debug) {
      Serial.println(F("Error Opening SD Card"));
    }
    error(0x0010);
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return false;
  }

  if (SD.remove(CO2fileName)) {
    // if the file was successfully removed, create a new file and close it.
    if ( !txtFile.open(CO2fileName, (O_RDWR   | O_CREAT | O_AT_END)) ) {
      error(0x0010);
      digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
      return false;
    }
    else {
      txtFile.close();

      //update FRAm
      co2SampleCount = 0;
      fram.writeUInt16(SampleCount_CO2_FramAddr, co2SampleCount); // update sample counter on FRAM, clearing previous sample data is not needed




      if (debug) {
        Serial.println(F("Done saving to SD card"));
      }
      //return true;
    }
  }
  else {
    digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
    return false;
  }

  // switch power off to sd card
  digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
  return true;

}


//==================================================================



// this method loads in lookup tables from an SD card and stores the tables in the EEPROM
void readTableFromSD() {
  LookupTable test;
  float *ptr;
  char newFloat[8];
  byte newFloatIndex = 0;         // index of the next spot to put a new char into
  char tempChar;
  int numOfTable = 0;             // index of number of tables proccessed
  byte lookupArrayIndex = 0;      // variable for the index to place the next float into
  byte lookupStructArrayNum = 0;  // index for the float array the code is up to
  char newline = '\n';
  char space = ' ';

  int lookupTableSize = sizeof(test);
  int eepromAddress = 0;

  if (!startSDCard(tableName)) {
    //Serial.print(F("Error Opening SD Card: lookup"));
    error(0x0020);
    return;
  }

  txtFile.open(tableName, O_READ);

  ptr = test.inputRange; //set pointer to aray in struct


  while ( numOfTable < (1024 / lookupTableSize) && txtFile.available() ) {
    ptr = test.inputRange; //reset pointer
    lookupStructArrayNum = 0;
    lookupArrayIndex = 0;
    newFloatIndex = 0;
    //clear arrays to prevent possible contamination for the moment
    memset(newFloat, ' ', sizeof(newFloat));
    //memset(test.inputRange, 0, sizeof(test.inputRange));
    //memset(test.outputRange, 0, sizeof(test.outputRange));

    //work on one lookup array
    while (txtFile.available() && lookupStructArrayNum < 2 ) {
      tempChar = txtFile.read();

      // if a commar or endline is found, assume the whole number has been read and store value in array
      if (tempChar == commar || tempChar == newline) {
        *ptr = atof(newFloat);

        ptr = ptr + 1;
        newFloatIndex = 0;
        memset(newFloat, space, sizeof(newFloat));
        lookupArrayIndex++;

        //check for end of table list
        if (tempChar == newline ) {
          lookupStructArrayNum++;
          ptr = test.outputRange; //swap to next lookup array
        }
      }

      // if char is valid, add it to the char array
      if ( (tempChar != space) && (newFloatIndex < 8) && (tempChar != commar) && (tempChar != newline)) {
        //        //Serial.print("save");
        newFloat[newFloatIndex] = tempChar;
        newFloatIndex++;
      }
      //      //Serial.println();
    }
    //a catch to cleanup the last float variable and store in array, only if it has not already been done
    if (!txtFile.available() && lookupArrayIndex < tableElementSize) {
      *ptr = atof(newFloat);
    }


    // once a table is made, save it in eeprom
    //EEPROM.put(eepromAddress, test); //EEPROM.put prevents overwritting if data is already in eeprom
    //eepromAddress = eepromAddress + lookupTableSize;
    //numOfTable++;
  }
  txtFile.close();
  tempLookup = test;
}

//======================== Need To Test ==============================================


uint8_t writeDataToSerial() {
  uint8_t maxAttempts = 5;
  uint8_t buttonStatus = 0;

  uint32_t waitCounter = 0;

  uint8_t readStatus = 0;
  uint32_t checksum = 0;
  uint8_t attempts = 0;
  bool allDataSent  = false;
  bool sendData = false;
  bool badEnd = false;
  uint8_t bufferSize = 128;
  char charBuffer[bufferSize];

  uint8_t filenumber = 0;

  String serialBuffer;

  menuButtonFlag = false;
  backButtonFlag = false;

  wakeFlag = false;

  serialBuffer.reserve(10);
  
  //delay(500);
  

  if (!debug && !keepUSBConnected) {
    USBDevice.attach();
  }

  Serial.begin(SERIAL_BAUD);



  //Serial.begin(SERIAL_BAUD); // is this needed?
  Serial.setTimeout(5000);
  pwrLED(4);
  ePaper.clearDisplay();
  updateStatusBar();

  ePaper.setCursor(65, 30 );
  ePaper.setTextSize(3);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.print(F("Send Data To PC"));

  ePaper.setTextSize(2);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.setCursor(65, 100);
  ePaper.print(F("Waiting For PC.."));
  ePaper.refresh();
  ePaper.refresh();

  serialBuffer = ""; // clear serial buffer string

  // wait for start instruction from PC
  while (true) {                                                          // TRUE FOR NOW
    serialBuffer = Serial.readStringUntil('\n');
    if (serialBuffer == F("SndData")) {
      sendData = true;
      filenumber = 1;
      Serial.println("S");
      break;
    }
    else if (serialBuffer == F("SndDataCO2")) {
      sendData = true;
      filenumber = 2;
      Serial.println("S");
      break;
    }
    else if ( serialBuffer == F("END")) {
      return 2; // return 2 to say method exited safely
    }
    else if ( waitCounter >= 24) {
      sendData = false; // return 0 if it took longer than 2 minutes to wait for PC
      break;
    }
    delay(50);
    waitCounter++;
  }


  // double check it is ok to proceed
  if ( !sendData ) {

    if (!debug && !keepUSBConnected) {
      Serial.end();
      USBDevice.detach();
    }
    sleep.interruptDisable(); //clear interrupts, and re-enable so a button press can stop transfer
    sleep.interruptEnable();
    return 0; // return 0 to say method did not run properly
  }

  //Switch on power to SD card, NOTE NPN MOSFET
  digitalWrite(PWR_SW_SD, HIGH);
  checkPower(SD_POWER_LINE, SdADCThresh, false);


  sendData = true; // reuse variable to check if sd card starting was successfull
  // check SD card Is OK, throw error and turn off card if its not
  if (filenumber == 1) {
    if ( !startSDCard(fileName)) {
      sendData = false;
    }
  }
  else if (filenumber == 2) {
    if ( !startSDCard(CO2fileName)) {
      sendData = false;
    }
  }
  if (!sendData) {
    if (debug) {
      Serial.println(F("Error Opening SD Card"));
    }
    Serial.println(F("BADEND"));

    ePaper.fillRect(60, 95, 200, 30, WHITE);
    ePaper.setCursor(65, 100);
    ePaper.print(F("Error Reading From SD."));
    ePaper.setCursor(65, 150);
    ePaper.print(F("Transfer Cancelled"));
    error(0x0010);
    ePaper.refresh();
    ePaper.refresh();

    
    digitalWrite(PWR_SW_SD, LOW);
    delay(2000);
    return 8; // return with hardware error
  }

  //open file now we know it is needed, throw error and turn off card if open fails
  if (filenumber == 1) {
    if ( !txtFile.open(fileName, O_READ)) {
      sendData = false;
    }
  }
  else if(filenumber == 2){
    if ( !txtFile.open(CO2fileName, O_READ)) {
      sendData = false;
    }
  }
  
  if (!sendData) {
    Serial.println(F("BADEND"));

    ePaper.fillRect(60, 95, 200, 30, WHITE);
    ePaper.setCursor(65, 100);
    ePaper.print(F("Error Reading From SD."));
    ePaper.setCursor(65, 150);
    ePaper.print(F("Transfer Cancelled"));
    error(0x0010);
    ePaper.refresh();
    ePaper.refresh();

    
    digitalWrite(PWR_SW_SD, LOW);
    return 8;
  }

  ePaper.fillRect(60, 95, 200, 30, WHITE);
  ePaper.setCursor(65, 100);
  ePaper.print(F("Sending Data to PC."));

  ePaper.fillRect(150, 170, 85, 25, BLACK);
  ePaper.setCursor(160, 175 );
  ePaper.setTextColor(WHITE, BLACK);
  ePaper.print(F("cancel"));
  ePaper.refresh();
  ePaper.refresh();
  inMenuNum = 21;

  // Start reading from sd card and send to PC
  while ( txtFile.available() && !allDataSent) {
    attempts = 0;
    buttonStatus = scanButtons();

    if ( wakeFlag ) { // end gracefully if menu or back button was pressed
      scanButtons();
      if ( menuButtonFlag || backButtonFlag) {


        allDataSent = true;
      }

    }

    // read single sample from sd card
    memset(charBuffer, '\0', bufferSize );
    if ( readTillChar(&charBuffer[0], bufferSize, '\n', &checksum) ) {

      while ( attempts < maxAttempts) {
        /*send single sample send command ("SNGL") to PC and wait untill PC replies
          with "OK" before start sending sample */
        readStatus = serialWaitForReply( F("SNGL\n"), F("OK"), maxAttempts);

        if ( readStatus == 0) {
          if (debug) {
            Serial.println(F("Wait for PC timed out."));
          }
          return 3; // return with timeout
        }
        else if ( readStatus == 1) {
          break;
        }
        else if ( readStatus == 3) {
          badEnd = true;
          break;
        }
        attempts++;
      }

      if (badEnd) {
        break;
      }


      // write datasample to serial and wait for reply
      attempts = 0;
      while ( attempts < maxAttempts) {
        // print data to PC
        Serial.print(checksum);
        Serial.write(',');
        Serial.println(charBuffer);
        Serial.flush();

        // read reply status from PC
        readStatus = serialWaitForReply( "", F("CHKOK"), maxAttempts);
        if ( readStatus == 0 ) {
          Serial.println(F("END"));
          if (debug) {
            Serial.println(F("Wait for PC timed out."));
          }

          txtFile.close();
          digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
          return 3; // return with timeout
        }
        else if ( readStatus == 1) {
          break;
        }
        else if ( readStatus == 3) {
          badEnd = true;
          break;
        }

        attempts++;
      }
      if ( attempts >= maxAttempts || badEnd) {
        // PC did not recieve sample correctly
        Serial.println(F("END"));
        if (debug) {
          Serial.println(F("Too Many transmission errors."));
        }

        txtFile.close();
        digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
        return 4; // return with transmission error
      }
    }
    else {
      allDataSent = true;
    }

  }
  attempts = 0;
  while ( attempts < maxAttempts && !badEnd) {
    readStatus = serialWaitForReply(F("DATAEND\n"), F("OKEND"), maxAttempts);
    if ( readStatus == 1) {
      break;
    }

    else if ( attempts >= maxAttempts && readStatus == 0) {
      // PC did not recieve sample correctly
      Serial.println(F("END"));
      if (debug) {
        Serial.println(F("Wait for PC END timed out."));
      }

      txtFile.close();
      digitalWrite(PWR_SW_SD, LOW); //Switch off power to SD card
      return 5; // return with END transmission error
    }
  }

  txtFile.close();

  digitalWrite(PWR_SW_SD, LOW); //Switch on power to SD card, NOTE NPN MOSFET

  ePaper.setTextColor(BLACK, WHITE);
  ePaper.fillRect(60, 95, 200, 30, WHITE);
  ePaper.setCursor(65, 100);
  ePaper.print(F("Sending Data Finished!"));
  ePaper.fillRect(150, 170, 85, 25, WHITE);
  ePaper.refresh();
  ePaper.refresh();






  if (!debug && !keepUSBConnected) {

    Serial.end();

    USBDevice.detach();
  }
  if (badEnd) {
    pwrLED(9);
  }

  sleep.interruptDisable();
  sleep.interruptEnable();

  return 1;
}


uint8_t serialWaitForReply(String CMD, String reply, uint8_t maxAttempts) {
  uint8_t attempts = 0;
  String serialInBuffer;

  serialInBuffer.reserve(10);


  while ( attempts < maxAttempts) {
    Serial.print(CMD);
    Serial.flush();

    serialInBuffer = Serial.readStringUntil('\n');
    if (debug) {
      Serial.print(serialInBuffer);
      Serial.println(';');
      Serial.print(reply);
      Serial.println(';');
    }
    if (serialInBuffer == reply) {
      if (debug) {
        Serial.println(F("Command Recieved!"));
      }
      return 1;
    }
    else if (serialInBuffer == F("RETRY")) {
      return 2;
    }
    else if (serialInBuffer == F("END")) {
      return 3;
    }

    attempts++;
  }

  if (attempts >= maxAttempts) {
    return 0;
  }
  return 0;
}




bool readTillChar(char *charBuffer , uint8_t charBufferSize, char endchar, uint32_t *sum) {
  char temp = '\0';
  uint8_t charIndex = 0;
  bool stringRead = false;
  uint32_t checksum = 0;


  while ( !stringRead && txtFile.available() ) {

    temp = txtFile.read();
    //Serial.write(temp);
    if ( temp == endchar ) {
      stringRead = true;

    }
    else if (charIndex < charBufferSize) {
      charBuffer[charIndex] = temp;
      charIndex++;
      checksum += temp;
    }
  }
  *sum = checksum;
  //*sum = 0;
  return txtFile.available();
}




//===========================  =======================================



void saveStructFram(uint16_t address) {
  uint16_t addr = address;
  if (debug) {
    Serial.print(F("Saving to Fram :           "));
    printDataSample();
  }


  while ((fram.readFloat( addr) != data.Si7051_T) ) {
    fram.writeFloat( addr, data.Si7051_T);
  }
  addr += sizeof(float);

  while (fram.readFloat(addr) != data.SHT15_T) {
    fram.writeFloat( addr, data.SHT15_T );
  }
  addr += sizeof(float);

  while (fram.readFloat(addr) != data.SHT15_H ) {
    fram.writeFloat( addr, data.SHT15_H );
  }
  addr += sizeof(float);

  //  while (fram.readUInt16(addr) != data.K30_CO2) {
  //    fram.writeUInt16(addr, data.K30_CO2 );
  //  }
  //  addr += sizeof(uint16_t);

  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    while (fram.readUInt16(addr) != data.adsSingle[i]) {
      fram.writeUInt16(addr, data.adsSingle[i] );
    }
    addr += sizeof(uint16_t);
  }
  for (int i = 0; i < MAX_ADS1115_INPUTS / 2; i++) {
    while (fram.readUInt16(addr) != data.adsDouble[i]) {
      fram.writeUInt16(addr, data.adsDouble[i] );
    }
    addr += sizeof(uint16_t);
  }

  while (fram.readByte(addr) != data.rtcHour ) {
    fram.writeByte(addr, data.rtcHour );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != data.rtcMinutes ) {
    fram.writeByte(addr, data.rtcMinutes );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != data.rtcSeconds ) {
    fram.writeByte(addr, data.rtcSeconds );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != data.rtcDate) {
    fram.writeByte(addr, data.rtcDate);
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != data.rtcMonths) {
    fram.writeByte(addr, data.rtcMonths);
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != data.rtcyears) {
    fram.writeByte(addr, data.rtcyears);
  }
  if (debug) {
    Serial.print(F("\ncheck : "));
    loadStructFram(address);
  }
}

//======================================================================

void saveStructCO2Fram(uint16_t address) {
  uint16_t addr = address;

  if (debug) {
    Serial.print(F("Saving CO2 to Fram :           "));
    printCO2Sample();
  }


  while (fram.readUInt16(addr) != CO2Samp.K30_CO2) {
    fram.writeUInt16(addr, CO2Samp.K30_CO2 );
  }
  addr += sizeof(uint16_t);


  while (fram.readByte(addr) != CO2Samp.rtcHour ) {
    fram.writeByte(addr, CO2Samp.rtcHour );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != CO2Samp.rtcMinutes ) {
    fram.writeByte(addr, CO2Samp.rtcMinutes );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != CO2Samp.rtcSeconds ) {
    fram.writeByte(addr, CO2Samp.rtcSeconds );
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != CO2Samp.rtcDate) {
    fram.writeByte(addr, CO2Samp.rtcDate);
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != CO2Samp.rtcMonths) {
    fram.writeByte(addr, CO2Samp.rtcMonths);
  }
  addr += sizeof(byte);

  while (fram.readByte(addr) != CO2Samp.rtcyears) {
    fram.writeByte(addr, CO2Samp.rtcyears);
  }
  if (debug) {
    Serial.print(F("\ncheck : "));
    loadStructCO2Fram(address);
  }
}


//======================================================================

void loadStructFram(uint16_t address) {
  //DataSample temp;
  uint16_t addr = address;


  data.Si7051_T = fram.readFloat( addr);
  addr += sizeof(float);


  data.SHT15_T = fram.readFloat( addr);
  addr += sizeof(float);

  data.SHT15_H = fram.readFloat( addr);
  addr += sizeof(float);

  //data.K30_CO2 = fram.readUInt16(addr );
  //addr += sizeof(uint16_t);

  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    data.adsSingle[i]  = fram.readUInt16(addr );
    addr += sizeof(uint16_t);

  }
  for (int i = 0; i < MAX_ADS1115_INPUTS / 2; i++) {
    data.adsDouble[i] = fram.readUInt16(addr );
    addr += sizeof(uint16_t);

  }

  data.rtcHour  = fram.readByte(addr );
  addr += sizeof(byte);


  data.rtcMinutes  = fram.readByte(addr );
  addr += sizeof(byte);

  data.rtcSeconds = fram.readByte(addr  );
  addr += sizeof(byte);

  data.rtcDate = fram.readByte(addr);
  addr += sizeof(byte);

  data.rtcMonths = fram.readByte(addr);
  addr += sizeof(byte);

  data.rtcyears = fram.readByte(addr );

  if (debug) {
    Serial.print(F("Reading from FRAM : "));
    printDataSample();
    Serial.println();
  }
}

//======================================================================

void loadStructCO2Fram(uint16_t address) {
  //DataSample temp;
  uint16_t addr = address;




  CO2Samp.K30_CO2 = fram.readUInt16(addr );
  addr += sizeof(uint16_t);

  CO2Samp.rtcHour  = fram.readByte(addr );
  addr += sizeof(byte);


  CO2Samp.rtcMinutes  = fram.readByte(addr );
  addr += sizeof(byte);

  CO2Samp.rtcSeconds = fram.readByte(addr  );
  addr += sizeof(byte);

  CO2Samp.rtcDate = fram.readByte(addr);
  addr += sizeof(byte);

  CO2Samp.rtcMonths = fram.readByte(addr);
  addr += sizeof(byte);

  CO2Samp.rtcyears = fram.readByte(addr );

  if (debug) {
    Serial.print(F("Reading from FRAM : "));
    printCO2Sample();
    Serial.println();
  }
}

//======================================================================


void printDataSample() {

  Serial.print(data.Si7051_T);

  Serial.print(commar);
  Serial.print(data.SHT15_T);

  Serial.print(commar);
  Serial.print(data.SHT15_H );

  //Serial.print(commar);
  //Serial.print(data.K30_CO2);

  Serial.print(commar);
  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    Serial.print(data.adsSingle[i] );
    Serial.print(commar);
  }
  for (int i = 0; i < MAX_ADS1115_INPUTS / 2; i++) {
    Serial.print(data.adsDouble[i]);
    Serial.print(commar);
  }
  Serial.print(data.rtcHour );
  Serial.print(':');
  Serial.print(data.rtcMinutes);
  Serial.print(':');
  Serial.print(data.rtcSeconds);

  Serial.print(commar);
  Serial.print(data.rtcDate);
  Serial.print('/');
  Serial.print(data.rtcMonths);
  Serial.print('/');
  Serial.print(data.rtcyears);
  Serial.print(commar);
  //Serial.print(F(" Free RAM : "));
  //Serial.println(freeMemory());
  //Serial.println();
  Serial.flush();
}

//======================================================================

void printCO2Sample() {

  Serial.print(CO2Samp.K30_CO2);
  Serial.print(commar);

  Serial.print(CO2Samp.rtcHour );
  Serial.print(':');
  Serial.print(CO2Samp.rtcMinutes);
  Serial.print(':');
  Serial.print(CO2Samp.rtcSeconds);

  Serial.print(commar);
  Serial.print(CO2Samp.rtcDate);
  Serial.print('/');
  Serial.print(CO2Samp.rtcMonths);
  Serial.print('/');
  Serial.print(CO2Samp.rtcyears);
  Serial.print(commar);
  //Serial.print(F(" Free RAM : "));
  //Serial.println(freeMemory());
  //Serial.println();
  Serial.flush();
}

//======================================================================



void setADS1115_Enable() {

  // write ADS1115 config to FRAM
  while ( ADS_BitMask_enable != fram.readByte(ADS1115_Enable_Addr) ) {
    //Serial.println(ADS_BitMask_enable, BIN);
    fram.writeByte(ADS1115_Enable_Addr, ADS_BitMask_enable);
  }

}

void setADS1115_Single() {

  // write ADS1115 config to FRAM
  while ( fram.readByte(ADS1115_SingleCHN_Addr) != ADS_BitMask_single) {
    //Serial.println(ADS_BitMask_single, BIN);
    fram.writeByte(ADS1115_SingleCHN_Addr, ADS_BitMask_single) ;
  }
}

void setADS1115_Gain() {

  // write ADS1115 config to FRAM

  while ( fram.readByte(ADS1115_Gain_Addr) != (ADS1115Gain[0]) ) {
    fram.writeByte(ADS1115_Gain_Addr, ADS1115Gain[0]);
  }

  while ( fram.readByte(ADS1115_Gain_Addr + 1) != (ADS1115Gain[1]) ) {
    fram.writeByte(ADS1115_Gain_Addr + 1, ADS1115Gain[1]);
  }

  while ( fram.readByte(ADS1115_Gain_Addr + 2) != (ADS1115Gain[2]) ) {
    fram.writeByte(ADS1115_Gain_Addr + 2, ADS1115Gain[2]);
  }

  while ( fram.readByte(ADS1115_Gain_Addr + 3) != (ADS1115Gain[3]) ) {
    fram.writeByte(ADS1115_Gain_Addr + 3, ADS1115Gain[3]);
  }

}


/*
  Button Control methods - Warning! long amounts of code! ----------------------------------------
*/






uint8_t scanButtons() {
  //uint32_t startTime = millis();
  uint8_t gpio;
  //bool buttonPressed = false;
  uint8_t interruptReg = 0;

  //beginI2cDevices();



  interruptReg = gpioExpander.readINTF();
  //  gpio = gpioExpander.readGPIO(); // used to clear interrupt on MCP23008
  //  printBits(interruptReg);
  //  Serial.print(space);
  //  printBits(gpio);
  //  Serial.println();
  if ( interruptReg != 0) {

    if ( displayOn && interruptReg > 0 && (uint16_t(lastButtonPress + 400) > millis()) && buttonPressedContinuous ) {
      if (!loopButtonPress) {
        //Serial.println(F("returning for some reason"));
        return 0x00;
      }
    }
    if (buttonPressedContinuous) {
      loopButtonPress = true;
    }

    //Serial.println(interruptReg, BIN);


    if ( !displayOn) {
      gpioExpander.digitalWrite(0, HIGH);
      delay(5); // wait for displays boost converter to power up
      logo();

      ePaper.refresh();
      displayOn = true;
      return 0x00;
    }

    ePaper.begin();

    gpio = gpioExpander.readGPIO();
    //Serial.println(gpio, BIN);


    if ( ((gpio >> 1) & 0x1 ) == 0 ) {
      if (debug) {
        Serial.println(F("Menu button"));
      }
      menuButtonPressed();
      menuButtonFlag = true;
      displayOn = true;

    }
    else if (((gpio >> 2) & 0x1 ) == 0 ) {
      if (debug) {
        Serial.println(F("back button"));
      }

      displayOn = true;
      backButtonPressed();
      backButtonFlag = true;

    }

    else if (((gpio >> 3) & 0x1 ) == 0 ) {
      if (debug) {
        Serial.println(F("up button"));
      }
      displayOn = true;
      upButtonPressed();


    }

    else if (((gpio >> 4) & 0x1 ) == 0 ) {
      if (debug) {
        Serial.println(F("down button"));
      }
      displayOn = true;
      downButtonPressed();


    }
    lastButtonPress = millis();

    buttonPressedContinuous = true;
    //}

    //  if (debug) {
    //
    //    Serial.print("menu Level : ");
    //    Serial.println(inMenuNum);
    //    Serial.print("prev menu : ");
    //    Serial.println(prevMenuNum);
    //    Serial.print("cursor Pos : ");
    //    Serial.println(cursorPos);
    //    Serial.print("max cursor : ");
    //    Serial.println(maxCursor);
    //    Serial.print("Sampling : ");
    //    Serial.println(runSampling);
    //  }

    //  while (digitalRead(wakeButton) == 0) {
    //    gpioExpander.readGPIO();
    //  }
  }
  return interruptReg;
}

//-------------------------------------------------------------------------------
void menuButtonPressed() {

  switch (inMenuNum) {
    case 0:

      cursorPos = 1;
      drawBaseMenu(true);
      break;

    case 1:
      //base Menu screen options
      if ( cursorPos == 1) {
        cursorPos = 1;
        if (runSampling) {
          //runSampling = false;
          cursorPos = 2;
          drawSampleStopMenu(true);
        }
        else {
          cursorPos = 1;
          drawSampleMenu(true);
        }

      }
      else if (cursorPos == 2) {
        cursorPos = 2;
        buttonPressedContinuous = false;
        loopButtonPress = false;
        drawWriteToPC(true);
      }
      else if ( cursorPos == 3) {
        cursorPos = 1;
        prevMenuNum = 10;
        drawSettingsMenu(true);
      }
      break;

    //Sampling Menu screen options
    case 10:
      if ( cursorPos == 1) {
        runSampling = true;
        clearRTCAlarm();
        DS3234.update();
        if ( enableCO2Sampling && checkCO2ConnectionType() != 0) { // if CO2 Sensor is present and sampling is enabled
          nextAlarmTime_CO2Device_minutes = DS3234.minute() + 1;
          nextAlarmTime_CO2Device_hours = DS3234.hour();
        }
        else {
          nextAlarmTime_CO2Device_minutes = 99;   //set co2 alarm times to invalid times, just in case
          nextAlarmTime_CO2Device_hours = 99;
        }
        nextAlarmTime_mainDevice_minutes = DS3234.minute() + 1;
        nextAlarmTime_mainDevice_hours = DS3234.hour();
        DS3234.enableAlarmInterrupt(true, false);
        incrementAlarmTime();
        cursorPos = 1;
        drawSamplingStart();
        drawBaseMenu(true);
      }
      else if ( cursorPos == 2) {
        cursorPos = 1;
        // load in current values to temp variables
        temp1 = sampleRateHour;
        temp2 = sampleRateMin;

        drawChangeSampleRateMenu(true);
      }
      else if ( cursorPos == 3) {
        cursorPos = 1;
        drawCO2_SettingsMenu(true);
      }
      else if (cursorPos == 4) {
        cursorPos = 2;
        prevMenuNum = 10;
        drawClearSDCardMenu(true);
      }
      break;

    //stop sampling menu
    case 11:
      if ( cursorPos == 1) {
        runSampling = false;
        DS3234.enableAlarmInterrupt(false, false);  // enables interrupt from square wave pin on RTC to trigger on alarm event
        DS3234.alarm1(true);
        DS3234.alarm2(true);
        if ( checkCO2ConnectionType() != 0) { // if CO2 Sensor is present
          disableCO2();
        }
        //clearRTCAlarm();
        cursorPos = 1;
        drawBaseMenu(true);
      }
      else if ( cursorPos == 2) {
        cursorPos = 1;
        drawBaseMenu(true);
      }
      break;

    // set sample Rate menu
    case 12:
      if ( cursorPos == 1) {
        cursorPos = 2;
        drawChangeSampleRateMenu(false);
      }
      else if ( cursorPos == 2) {
        cursorPos = 2;
        sampleRateHour = temp1; // save variables entered
        sampleRateMin = temp2;
        fram.writeByte(samplePeriod_Addr, sampleRateHour);
        fram.writeByte(samplePeriod_Addr + 1 , sampleRateMin);
        temp1 = 0;
        temp2 = 0;
        drawSampleMenu(true);
      }
      break;


    case 13:
      if (cursorPos == 1) {
        cursorPos = 1;
        drawEnable_CO2_SamplingMenu(true);

      }
      else if (cursorPos == 2) {
        // load in current values to temp variables
        temp1 = fram.readByte(samplePeriod_CO2_Addr ); // read salmple interval from FRAM
        temp2 = fram.readByte(samplePeriod_CO2_Addr + 1);
        cursorPos = 1;
        drawChange_CO2_SampleRateMenu(true);
      }
      break;

    // enable CO2 setting menu
    case 14:
      if (cursorPos == 1) {
        if ( enableCO2Sampling) {
          enableCO2Sampling = false;
        }
        else {
          enableCO2Sampling = true;
        }

        cursorPos = 1;
        drawEnable_CO2_SamplingMenu(false);
      }

      break;


    // set CO2 sample Rate menu
    case 15:
      if ( cursorPos == 1) {
        cursorPos = 2;
        drawChange_CO2_SampleRateMenu(false);
      }
      else if ( cursorPos == 2) {
        cursorPos = 3;
        // update CO2 sample interval
        //        if (temp1 == 0 && temp2 < 5) {
        //          temp2 = 5;
        //        }
        fram.writeByte(samplePeriod_CO2_Addr, temp1); // hours
        fram.writeByte(samplePeriod_CO2_Addr + 1 , temp2); // minutes
        temp1 = 0;
        temp2 = 0;
        cursorPos = 2;
        drawCO2_SettingsMenu(true);
      }
      break;

    // send data to PC menu
    case 20:
      if (cursorPos == 1) {
        saveToSD();
        saveCO2ToSD();
        writeDataToSerial();
        delay(2000);
        cursorPos = 2;
        drawBaseMenu(true);
      }
      else {
        cursorPos = 2;
        drawBaseMenu(true);
      }
      break;

    case 21:
      cursorPos = 2;
      drawBaseMenu(true);
      break;


    // settings menu screen options
    case 30:
      if ( cursorPos == 1) {
        cursorPos = 1;
        drawTimeSettingMenu(true);
      }
      else if ( cursorPos == 2) {
        cursorPos = 1;
        drawADCSettingsMenu(true);
      }
      else if ( cursorPos == 3) {
        cursorPos = 1;
        drawDataSettingsMenu(true);
      }
      else if ( cursorPos == 4) {
        cursorPos = 1;
        drawErrorHistoryMenu(true);
      }

      break;

    case 110:
      if ( cursorPos == 1) {
        cursorPos = 1;
        // load in current values to temp variables
        //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
        DS3234.update();
        temp1 = DS3234.date();
        temp2 = DS3234.month();
        temp3 = DS3234.year();
        ePaper.begin();
        drawTimeSubSetting_Date_Menu(true);
      }
      if ( cursorPos == 2) {
        cursorPos = 1;
        // load in current values to temp variables
        //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
        DS3234.update();
        temp1 = DS3234.hour();
        temp2 = DS3234.minute();
        temp3 = DS3234.second();
        ePaper.begin();
        drawTimeSubSetting_Time_Menu(true);
      }

      break;
    // set date menu screen
    case 111:
      if ( cursorPos == 3) {
        cursorPos = 1;
        //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
        DS3234.setDate(temp1);
        DS3234.setMonth(temp2);
        DS3234.setYear(temp3);
        temp1 = 0;
        temp2 = 0;
        temp3 = 0;
        ePaper.begin();
        drawTimeSettingMenu(true);
      }
      else {
        cursorPos = cursorPos + 1;
        drawTimeSubSetting_Date_Menu(false);
      }
      break;

    //set time menu screen
    case 112:
      if ( cursorPos == 3) {
        cursorPos = 2;
        //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
        DS3234.setHour(temp1);
        DS3234.setMinute(temp2);
        DS3234.setSecond(temp3);
        temp1 = 0;
        temp2 = 0;
        temp3 = 0;
        ePaper.begin();
        drawTimeSettingMenu(true);
      }
      else {
        cursorPos = cursorPos + 1;
        drawTimeSubSetting_Time_Menu(false);
      }
      break;

    // ADC settings menu screen options
    case 120:
      if ( cursorPos == 1) {
        cursorPos = 1;
        drawADC_ChannelEnable_Menu(true);
      }
      else if ( cursorPos == 2) {
        cursorPos = 1;
        drawADC_Dual_Menu(true);
      }
      else if (cursorPos == 3) {
        cursorPos = 1;
        drawADC_PGAModes_Menu(true);
      }

      break;

    // ADC channel settings menu screen options
    case 121:

      if ( ((ADS_BitMask_enable >> (cursorPos - 1) ) & 0x01) == 1) {  // if the channel the cursor is on is enabled, toggle it to disabled
        ADS_BitMask_enable &= ~(1 << (cursorPos - 1));
      }
      else {                                                      // otherwise enable the channel
        ADS_BitMask_enable |= (1 << (cursorPos - 1));
      }
      drawADC_ChannelEnable_Menu(false);  // dont fully update screen as this menu only exits on back button press
      break;

    // ADC dual channel settings menu screen options
    case 122:
      if ( ((ADS_BitMask_enable >> ((cursorPos - 1) * 2) ) & 0x01) == 1) { // only change dual channel mode if the respective channel is enabled

        if ( ((ADS_BitMask_single >> (cursorPos - 1) ) & 0x01) == 1) {  // if the channel the cursor is on is enabled, toggle it to disabled
          ADS_BitMask_single &= ~(1 << (cursorPos - 1));
        }
        else {                                                      // otherwise enable the channel
          ADS_BitMask_single |= (1 << (cursorPos - 1));
        }
      }
      drawADC_Dual_Menu(false);// dont fully update screen as this menu only exits on back button press
      break;

    case 123:
      if ( ADS1115Gain[cursorPos - 1] < 5 ) {
        ADS1115Gain[cursorPos - 1] += 1;

      }
      else if ( ADS1115Gain[cursorPos - 1] == 5 ) {
        ADS1115Gain[cursorPos - 1] = 0;
      }

      drawADC_PGAModes_Menu(false);// dont fully update screen as this menu only exits on back button press
      break;

    // data settings menu
    case 130:
      if ( cursorPos == 1) {
        drawDataSetting_RemoveSD_Menu(true);
      }
      else if (cursorPos == 2) {
        cursorPos = 2;
        prevMenuNum = 130;
        drawClearSDCardMenu(true);
      }
      break;

    // remove SD card menu
    case 131:
      if ( cursorPos == 1) {
        drawDataSettingsMenu(true);
      }
      break;

    // clear SD card menu
    case 133:
      if (cursorPos == 1) {
        // run delete file method
        if (clearDataFromSd()) {
          drawDataClearedMessage();
        }
        else {
          drawDataClearFailedMessage();
        }

        if ( prevMenuNum == 10) {
          cursorPos = 3;
          drawSampleMenu(true);
        }
        else if (prevMenuNum == 130) {
          cursorPos = 2;
          drawDataSettingsMenu(true);
        }

      }
      else { // else do nothing

        if ( prevMenuNum == 10) {
          cursorPos = 3;
          drawSampleMenu(true);
        }
        else if (prevMenuNum == 130) {
          cursorPos = 2;
          drawDataSettingsMenu(true);
        }
      }

    // error history menu
    case 140:
      cursorPos = 4;
      drawSettingsMenu(true);
      break;


    default:
      break;

  }
  //  buttonPressedContinuous = false;
  //  loopButtonPress = false;
  updateStatusBar();
  ePaper.refresh();
}
//-------------------------------------------------------------------------------
void backButtonPressed() {

  //temp1, temp2, temp3 = 0;  //clear temp value holders as a catch all
  switch (inMenuNum) {
    case 0:
      //    inMenuNum = 0;
      //    cursorPos = 0;
      //    maxCursor = 0;
      //    Serial.println(inMenuNum);
      break;

    //logo();

    case 1:
      inMenuNum = 0;
      cursorPos = 0;
      maxCursor = 0;
      logo();
      break;

    // Sampling menu screen options
    case 10:
      cursorPos = 1;
      drawBaseMenu(true);
      break;

    // Stop Sampling menu screen options
    case 11:
      cursorPos = 1;
      drawBaseMenu(true);
      break;

    // set sampling rate menu screen options
    case 12:
      if (cursorPos == 1) {
        cursorPos = 2;
        drawSampleMenu(true);

      }
      else {
        cursorPos = cursorPos - 1;
        drawChangeSampleRateMenu(false); // full screen update not needed here
      }
      break;

    // CO2 Settings menu screen options
    case 13:
      cursorPos = 3;
      drawSampleMenu(true);
      break;

    // enable CO2 setting menu
    case 14:
      cursorPos = 1;
      drawCO2_SettingsMenu(true);

      break;

    // set CO2 sampling rate menu screen options
    case 15:
      if (cursorPos == 1) {
        cursorPos = 2;
        drawCO2_SettingsMenu(true);

      }
      else {
        cursorPos = cursorPos - 1;
        drawChange_CO2_SampleRateMenu(false); // full screen update not needed here
      }
      break;

    case 20:
      cursorPos = 2;
      drawBaseMenu(true);
      break;

    case 21:
      cursorPos = 2;
      drawBaseMenu(true);
      break;

    // settings menu screen options
    case 30:
      cursorPos = 3;
      drawBaseMenu(true);
      break;

    // Time settings menu screen options
    case 110:
      cursorPos = 1;
      drawSettingsMenu(true);
      break;

    //set date options
    case 111:
      if (cursorPos == 1) {
        cursorPos = 1;
        drawTimeSettingMenu(true);
      }
      else {
        cursorPos = cursorPos - 1;
        drawTimeSubSetting_Date_Menu(false);
      }
      break;

    // set time options
    case 112:
      if (cursorPos == 1) {
        cursorPos = 2;
        drawTimeSettingMenu(true);
      }
      else {
        cursorPos = cursorPos - 1;
        drawTimeSubSetting_Time_Menu(false);
      }
      break;

    case 120:
      cursorPos = 2;
      drawSettingsMenu(true);
      break;

    case 121:
      cursorPos = 1;
      drawADCSettingsMenu(true);
      setADS1115_Enable();
      break;

    case 122:
      cursorPos = 2;
      drawADCSettingsMenu(true);
      setADS1115_Single();
      break;

    case 123:
      cursorPos = 3;
      drawADCSettingsMenu(true);
      setADS1115_Gain();

      break;

    case 130:
      cursorPos = 3;
      drawSettingsMenu(true);
      break;

    case 133:
      if ( prevMenuNum == 10) {
        cursorPos = 3;
        drawSampleMenu(true);
      }
      else if (prevMenuNum == 130) {
        cursorPos = 2;
        drawDataSettingsMenu(true);
      }
      break;

    // error history menu
    case 140:
      cursorPos = 4;
      drawSettingsMenu(true);
      break;

    default:
      break;
  }

  //  buttonPressedContinuous = false;
  //  loopButtonPress = false;
  updateStatusBar();
  ePaper.refresh();
}



//-------------------------------------------------------------------------------
void upButtonPressed() {
  if ( (inMenuNum != 111) && (inMenuNum != 112) && inMenuNum != 12  && inMenuNum != 15) {

    if (cursorPos > 1) {
      cursorPos = cursorPos - 1;
    }
    else if ( cursorPos == 1 ) {
      cursorPos = maxCursor;
    }
  }
  //ePaper.clearDisplay();

  // base menu screen options
  switch ( inMenuNum ) {
    case 1:
      drawBaseMenu(false);
      break;

    // Sampling menu screen options
    case 10:
      drawSampleMenu(false);
      break;

    case 11:
      drawSampleStopMenu(false);
      break;

    //set sample rate menu
    case 12:
      if (cursorPos == 1) {
        if (temp1 < 24) { //sample rate hour
          temp1 = temp1 + 1;
        }
        else if (temp2 == 0 && temp1 == 24) {
          temp1 = 0;
          temp2 = 1;
        }
        else {
          temp1 = 0;
        }
      }
      else if (cursorPos == 2) {
        if (temp2 < 59) { //sample rate minute
          temp2 = temp2 + 1;
        }
        else if (temp1 > 0) {
          temp2 = 0;
        }
        else {
          temp2 = 1;
        }

      }
      drawChangeSampleRateMenu(false);
      break;

    // CO2 Sampling Settings screen options
    case 13:
      drawCO2_SettingsMenu(false);
      break;

    // Enable CO2 Sampling menu screen options
    case 14:
      drawEnable_CO2_SamplingMenu(false);
      break;

    //set CO2 sampling rate menu
    case 15:
      if (cursorPos == 1) {
        if (temp1 < 24) { //sample rate hour
          temp1 = temp1 + 1;
        }
        else if (temp2 == 0 && temp1 == 24) {
          temp1 = 0;
          temp2 = 5;
        }
        else {
          temp1 = 0;
        }
      }
      else if (cursorPos == 2) {
        if (temp2 < 59) { //sample rate minute
          temp2 = temp2 + 1;
        }
        else if (temp1 > 0) {
          temp2 = 0;
        }
        else {
          temp2 = 5;
        }

      }
      drawChange_CO2_SampleRateMenu(false);
      break;

    case 20:
      drawWriteToPC(false);
      break;


    // settings menu screen options
    case 30:
      drawSettingsMenu(false);
      break;

    // Time settings menu screen options
    case 110:
      drawTimeSettingMenu(false);
      break;

    // set date menu screen
    case 111:
      if (cursorPos == 1) {
        if (temp1 < 31) {
          temp1 = temp1 + 1;
        }
        else {
          temp1 = 1;
        }
      }
      else if (cursorPos == 2) {
        if (temp2 < 12) {
          temp2 = temp2 + 1;
        }
        else {
          temp2 = 1;
        }
      }
      else if (cursorPos == 3) {
        if ( temp3 < 99) {
          temp3 = temp3 + 1;
        }
        else {
          temp3 = 0;
        }
      }
      drawTimeSubSetting_Date_Menu(false);
      break;

    // set Time menu screen
    case 112:
      if (cursorPos == 1) {
        //hour
        if (temp1 < 23) {
          temp1 = temp1 + 1;
        }
        else {
          temp1 = 0;
        }
      }
      else if (cursorPos == 2) {
        //minutes
        if ( temp2 < 59) {
          temp2 = temp2 + 1;
        }
        else {
          temp2 = 0;
        }
      }
      else if (cursorPos == 3) {
        //sec
        if (temp3 < 59) {
          temp3 = temp3 + 1;
        }
        else {
          temp3 = 0;
        }
      }
      drawTimeSubSetting_Time_Menu(false);
      break;

    case 120:
      drawADCSettingsMenu(false);
      break;

    case 121:
      drawADC_ChannelEnable_Menu(false);
      break;

    case 122:
      drawADC_Dual_Menu(false);
      break;

    case 123:
      drawADC_PGAModes_Menu(false);
      break;

    // data settings menu
    case 130:
      drawDataSettingsMenu(false);
      break;

    case 131:
      drawDataSettingsMenu(false);
      break;

    case 133:
      drawClearSDCardMenu(false);
      break;

    default:
      break;
  }
  updateStatusBar();
  ePaper.refresh();
}

//-------------------------------------------------------------------------------
void downButtonPressed() {
  if ( (inMenuNum != 111) && (inMenuNum != 112) && inMenuNum != 12  && inMenuNum != 15) {

    if (cursorPos < maxCursor) {
      cursorPos = cursorPos + 1;
    }
    else if ( cursorPos == maxCursor ) {
      cursorPos = 1;
    }
  }


  if (inMenuNum == 0 ) {
    return;
  }

  switch (inMenuNum) {
    // Base menu screen options
    case 1:
      drawBaseMenu(false);
      break;

    // Sampling menu screen options
    case 10:
      drawSampleMenu(false);
      break;

    case 11:
      drawSampleStopMenu(false);
      break;

    //set sample rate menu
    case 12:
      if (cursorPos == 1) {
        if (temp1 > 0 && temp2 > 0) { //sample rate hour
          temp1 = temp1 - 1;
        }
        else if ( temp2 == 0 && temp1 == 1) {
          temp1 = 0;
          temp2 = 1;
        }
        else {
          temp1 = 24;
        }
      }

      else if (cursorPos == 2) {
        if (temp2 > 1 && temp1 != 0) {  //sample rate min
          temp2 = temp2 - 1;
        }
        else if (temp1 == 0 && temp2 > 1) {
          temp2 = temp2 - 1;
        }
        else if (temp1 > 0 && temp2 == 1) {
          temp2 = 0;
        }
        else if (temp1 > 0 && temp2 == 0) {
          temp2 = 59;
        }
        else if (temp2 == 1) {
          temp2 = 59;
        }
      }
      drawChangeSampleRateMenu(false);
      break;

    // CO2 Sampling Settings screen options
    case 13:
      drawCO2_SettingsMenu(false);
      break;

    // Enable CO2 Sampling menu screen options
    case 14:
      drawEnable_CO2_SamplingMenu(false);
      break;

    //set CO2 sample rate menu
    case 15:
      if (cursorPos == 1) {
        if (temp1 > 0 && temp2 > 0) { //sample rate hour
          temp1 = temp1 - 1;
        }
        else if ( temp2 == 0 && temp1 == 1) {
          temp1 = 0;
          temp2 = 5;
        }
        else {
          temp1 = 24;
        }
      }

      else if (cursorPos == 2) {
        if (temp2 > 1 && temp1 != 0) {  //sample rate min
          temp2 = temp2 - 1;
        }
        else if (temp1 == 0 && temp2 > 5) {
          temp2 = temp2 - 1;
        }
        else if (temp1 > 0 && temp2 == 1) {
          temp2 = 0;
        }
        else if (temp1 > 0 && temp2 == 0) {
          temp2 = 59;
        }
        else if (temp2 == 1) {
          temp2 = 59;
        }
      }
      drawChange_CO2_SampleRateMenu(false);
      break;



    case 20:
      drawWriteToPC(false);
      break;

    // settings menu screen options
    case 30:
      drawSettingsMenu(false);
      break;

    //    case 40:
    //      drawErrorHistoryMenu(false);
    //      break;

    // Time settings menu screen options
    case 110:
      drawTimeSettingMenu(false);
      break;

    // set date menu screen
    case 111:
      if (cursorPos == 1) {
        // day
        if (temp1 > 1) {
          temp1 = temp1 - 1;
        }
        else {
          temp1 = 31;
        }
      }
      else if (cursorPos == 2) {
        // Month
        if (temp2 > 1) {
          temp2 = temp2 - 1;
        }
        else {
          temp2 = 12;
        }
      }
      else if (cursorPos == 3) {
        //year
        if ( temp3 > 00) {
          temp3 = temp3 - 1;
        }
        else {
          temp3 = 99;
        }
      }
      drawTimeSubSetting_Date_Menu(false);
      break;

    // set Time menu screen
    case 112:
      if (cursorPos == 1) {
        //hour
        if (temp1 > 0) {
          temp1 = temp1 - 1;
        }
        else {
          temp1 = 23;
        }
      }
      else if (cursorPos == 2) {
        //minutes
        if ( temp2 > 0) {
          temp2 = temp2 - 1;
        }
        else {
          temp2 = 59;
        }
      }
      else if (cursorPos == 3) {
        //sec
        if (temp3 > 0) {
          temp3 = temp3 - 1;
        }
        else {
          temp3 = 59;
        }
      }
      drawTimeSubSetting_Time_Menu(false);
      break;

    //ADC settings menu
    case 120:
      drawADCSettingsMenu(false);
      break;

    //ADC channel settings menu
    case 121:
      drawADC_ChannelEnable_Menu(false);
      break;

    case 122:
      drawADC_Dual_Menu(false);
      break;

    case 123:
      drawADC_PGAModes_Menu(false);
      break;

    // data settings menu
    case 130:
      drawDataSettingsMenu(false);
      break;

    case 133:
      drawClearSDCardMenu(false);
      break;


    default:
      return;
  }

  updateStatusBar();
  ePaper.refresh();
}





/*
  Display Control methods - Warning! long amounts of code! =============================================
*/


void redrawMenus() {

  switch (inMenuNum) {

    case 1:
      drawBaseMenu(true);
      break;

    case 10:
      drawSampleMenu(true);
      break;
    case 11:
      drawSampleStopMenu(true);
      break;

    case 12:
      drawChangeSampleRateMenu(true);
      break;


    case 13:
      drawCO2_SettingsMenu(true);
      break;


    case 14:
      drawEnable_CO2_SamplingMenu(true);
      break;


    case 15:
      drawChange_CO2_SampleRateMenu(true);
      break;

    case 20:
      drawWriteToPC(true);
      break;

    case 30:
      drawSettingsMenu(true);
      break;

    case 110:
      drawTimeSettingMenu(true);
      break;

    case 111:
      drawTimeSubSetting_Date_Menu(true);
      break;

    case 112:
      drawTimeSubSetting_Time_Menu(true);
      break;

    case 120:
      drawADCSettingsMenu(true);
      break;

    case 121:
      drawADC_ChannelEnable_Menu(true);
      break;

    case 122:
      drawADC_Dual_Menu(true);
      break;

    case 123:
      drawADC_PGAModes_Menu(true);
      break;

    case 130:
      drawDataSettingsMenu(true);
      break;

    case 131:
      drawDataSetting_RemoveSD_Menu(true);
      break;

    case 133:
      drawClearSDCardMenu(true);
      break;

    // error history menu
    case 140:
      drawErrorHistoryMenu(true);
      break;


    default:
      logo();
      break;

  }
  updateStatusBar();
  ePaper.refresh();
}

//=========================================================

void logo() {

  ePaper.clearDisplay();

  ePaper.fillTriangle(135, 135, 135, 35, 265, 35, BLACK);
  ePaper.drawLine(265, 35, 265, 140, BLACK);
  ePaper.drawLine(135, 35, 265, 140, BLACK);


  ePaper.setCursor(142, 145);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.setTextSize(4);
  ePaper.print("BRANZ");
  ePaper.setCursor(147, 195);
  ePaper.setTextSize(2);
  ePaper.print("B.A.C.O.N");
  ePaper.setCursor(180, 215);
  ePaper.setTextSize(1);
  ePaper.print("ver ");
  ePaper.print(DeviceVersionNumber);


  updateStatusBar();

  //  if (!batteryGood()) { //!batteryGood()
  //    ePaper.setCursor(165, 5);
  //    ePaper.setTextColor( WHITE, BLACK);
  //    ePaper.fillRect(162, 2, 78, 13, BLACK);
  //    ePaper.print(F("LOW BATTERY!"));
  //
  //  }


  //ePaper.refresh();
}

//=========================================================

void logo2() {

  //ePaper.clearDisplay();

  ePaper.fillTriangle(135, 135, 135, 35, 265, 35, BLACK);
  ePaper.drawLine(265, 35, 265, 140, BLACK);
  ePaper.drawLine(135, 35, 265, 140, BLACK);


  ePaper.setCursor(142, 145);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.setTextSize(4);
  ePaper.print("BRANZ");
  ePaper.setCursor(147, 195);
  ePaper.setTextSize(2);
  ePaper.print("B.A.C.O.N");
  ePaper.setCursor(180, 215);
  ePaper.setTextSize(1);
  ePaper.print("ver ");
  ePaper.print(DeviceVersionNumber);

  updateStatusBar();

  //  if (!batteryGood()) { //!batteryGood()
  //    ePaper.setCursor(165, 5);
  //    ePaper.setTextColor( WHITE, BLACK);
  //    ePaper.fillRect(162, 2, 78, 13, BLACK);
  //    ePaper.print(F("LOW BATTERY!"));
  //
  //  }

  ePaper.refresh();
}

//=========================================================

void updateStatusBar() {
  uint8_t temp;
  ePaper.setTextSize(1);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.setCursor(10, 5);
  if (runSampling) {
    ePaper.print(F("Sampling Enabled"));
  }
  ePaper.drawLine(0, 16, 400, 16, BLACK);

  //DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
  DS3234.update();

  ePaper.setCursor(295, 5);

  //print time with zero padding
  temp = DS3234.hour();
  if (temp < 10) {
    ePaper.print(zero);
  }
  ePaper.print(temp);

  ePaper.print(':');
  temp = DS3234.minute();
  if (temp < 10) {
    ePaper.print(zero);
  }
  ePaper.print(temp);

  //print date with zero padding
  ePaper.print(space);
  ePaper.print(space);
  temp = DS3234.date();
  if (temp < 10) {
    ePaper.print(zero);
  }
  ePaper.print(temp);
  ePaper.print('/');
  temp = DS3234.month();
  if (temp < 10) {
    ePaper.print(zero);
  }
  ePaper.print(temp);
  ePaper.print('/');
  ePaper.print(20);
  temp = DS3234.year();
  if (temp < 10) {
    ePaper.print(zero);
  }
  ePaper.print(temp);


  if (!batteryGood()) { //!batteryGood()
    ePaper.setCursor(205, 5);
    ePaper.setTextColor( WHITE, BLACK);
    ePaper.fillRect(202, 2, 78, 13, BLACK);
    ePaper.print(F("LOW BATTERY!"));
    ePaper.setTextColor( BLACK, WHITE);

  }

}


//=========================================================

void drawMenuBox( const char * charList, uint8_t posVertIndex, int16_t boxWidths, int16_t posLeftAdjust, int16_t posYAdjust, bool invertColor) {
  uint8_t offsetY = uint8_t(80 + posYAdjust);
  uint8_t offsetX = uint8_t( 80 + posLeftAdjust );
  uint8_t yIncrement = 25;

  uint8_t charOffsetY = 5;
  uint8_t charOffsetX = 20;
  ePaper.setTextSize(2);


  if ( invertColor) {
    ePaper.fillRect(offsetX, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, BLACK);
    ePaper.setCursor(offsetX + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
    ePaper.setTextColor(WHITE, BLACK);

    ePaper.print(charList);
  }
  else {
    ePaper.fillRect(offsetX, offsetY + (yIncrement ) * posVertIndex - 1 , boxWidths, yIncrement, WHITE);
    ePaper.drawRect(offsetX, offsetY + (yIncrement - 1) * posVertIndex - 1, boxWidths, yIncrement, BLACK);
    ePaper.setCursor(offsetX + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(charList);
  }

}

//=========================================================

void drawList( const char ** charList, uint8_t num_list, uint8_t cursorPos, uint16_t textBoxWidth, int16_t xAdjust, int16_t yAdjust = 0) {

  for (int i = 0; i < num_list; i++) {
    if ( i == (cursorPos - 1)) {
      drawMenuBox(charList[i], i, textBoxWidth, xAdjust, yAdjust, true);
    }
    else {
      drawMenuBox(charList[i], i, textBoxWidth, xAdjust, yAdjust, false);
    }
  }

}

//=========================================================

void drawSettingChangeBox(const char * charList , uint8_t posHorizIndex, uint8_t posVertIndex,
                          int16_t boxWidths, int16_t posLeftAdjust, bool invertColor) {

  uint8_t offsetX = uint8_t( 80 + posLeftAdjust );
  uint8_t offsetY = 80;
  uint8_t yIncrement = 25;
  uint8_t xIncrement = 5;

  uint8_t charOffsetY = 5;
  uint8_t charOffsetX = 10;
  ePaper.setTextSize(2);


  if ( invertColor) {
    ePaper.fillRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, BLACK);
    ePaper.setCursor(offsetX + (xIncrement - 1)* posHorizIndex + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
    ePaper.setTextColor(WHITE, BLACK);

    ePaper.print(charList);
  }
  else {
    ePaper.fillRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, WHITE);
    //ePaper.fillRect(offsetX, offsetY + (yIncrement ) * posVertIndex - 1 , boxWidths, yIncrement, WHITE);
    ePaper.drawRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex - 1, boxWidths, yIncrement, BLACK);
    ePaper.setCursor(offsetX + (xIncrement - 1)* posHorizIndex + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(charList);
  }


}


//=========================================================


void drawBaseMenu(bool refresh) {
  int16_t boxwidth = 270;
  int16_t leftAdjust = -15;
  maxCursor = 3;


  const char menuStartText[] = "Start Sampling";
  const char menuStopText[] = "Stop Sampling";
  const char menuPCText[] = "Transfer Data to PC";
  const char menuSettingText[] = "Settings";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  if (!runSampling) {
    textList[0] = &menuStartText[0];
  }
  else {
    textList[0] = &menuStopText[0];
  }
  textList[1] = &menuPCText[0];
  textList[2] = &menuSettingText[0];

  // setup display if this menu is not currently being displayed
  if ( refresh ) {
    ePaper.clearDisplay();
    //ePaper.fillRect(1, 1, 400, 240, WHITE);

    ePaper.setCursor(162, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Menu"));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 1;    // update current menu value

}

//=========================================================

void drawSampleMenu(bool refresh) {

  int16_t boxwidth = 300;
  int16_t leftAdjust = -28;
  maxCursor = 4;

  const char menuStartText[] = "Start Sampling";
  const char menuSampling_SettingTime_Text[] = "Set Sampling Interval";
  const char menuSampling_CO2_SettingTime_Text[] = "CO2 Sample Settings";
  const char menuSampling_SettingClear_Text[] = "Clear Saved Data";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &menuStartText[0];
  textList[1] = &menuSampling_SettingTime_Text[0];
  textList[2] = &menuSampling_CO2_SettingTime_Text[0];
  textList[3] = &menuSampling_SettingClear_Text[0];

  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(72, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Start Sampling"));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 10;    // update current menu value
  prevMenuNum = 10;

}

//====================================================================================================================

void drawSampleStopMenu(bool refresh) {

  int16_t boxwidth = 85;
  int16_t leftAdjust = 70;
  maxCursor = 2;

  const char yes[] = "Yes";
  const char no[] = "No";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &yes[0];
  textList[1] = &no[0];



  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(85, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Stop Sampling"));
    ePaper.print('?');
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 11;    // update current menu value


}

//====================================================================================================================

void drawChangeSampleRateMenu(bool refresh) {

  int16_t boxwidth = 60;
  int16_t leftAdjust = 60;
  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 2;


  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(15, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);

    ePaper.print(F("Set Sampling Interval"));


    ePaper.setTextSize(2);
    ePaper.setCursor(140, 85 );
    ePaper.print(F("Hour"));
    ePaper.setCursor(215, 85 );
    ePaper.print(F("Min"));

    ePaper.setTextSize(1);
    ePaper.setCursor(120, 150 );
    ePaper.print(F("A sample will be taken after"));
    ePaper.setCursor(120, 160 );
    ePaper.print(F("this amount of time has passed"));

  }


  // draw menu boxes on display with box highlighted if cursor index matches
  if ( cursorPos == 1) {  // hours box
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 1, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 2, false);
  }
  else if ( cursorPos == 2) { // minutes box
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 2, true);
  }

  inMenuNum = 12;   // update current menu value

}

//====================================================================================================================

void drawCO2_SettingsMenu(bool refresh) {

  int16_t boxwidth = 300;
  int16_t leftAdjust = -25;

  const char menuEnableSampling_CO2_Text[] = "Enable CO2 Sampling";
  const char menuSampling_CO2_SettingTime_Text[] = "Set CO2 Sample Interval";

  maxCursor = 2;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &menuEnableSampling_CO2_Text[0];
  textList[1] = &menuSampling_CO2_SettingTime_Text[0];


  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(15, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);

    ePaper.print(F("CO2 Sampling Settings"));

  }


  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 13;   // update current menu value

}

//====================================================================================================================

void drawEnable_CO2_SamplingMenu(bool refresh) {
  int16_t boxwidth = 170;
  int16_t boxwidth2 = 120;
  int16_t leftAdjust = -35;
  int16_t leftAdjust2 = 140;

  const char CO2_Enabled[] = "CO2 Enabled:";
  const char en[] = "Enabled";
  const char dis[] = "Disabled";

  maxCursor = 1;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &CO2_Enabled[0];



  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(30, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);

    ePaper.print(F("Enable CO2 Sampling?"));


    ePaper.setTextSize(1);
    ePaper.setCursor(115, 150 );
    ePaper.print(F("Enables Sampling from CO2 sensor"));
    ePaper.setCursor(115, 160 );
    ePaper.print(F("if it is connected/present."));

  }

  drawList(textList, maxCursor, 0, boxwidth, leftAdjust);


  // draw enable boxes based on if they are enabled or not. display "enabled" if the are, "disabled" if they are not
  if ( enableCO2Sampling) {
    drawSettingChangeBox( &en[0] , 0, 0, boxwidth2, leftAdjust2, true);
  }
  else {
    drawSettingChangeBox( &dis[0] , 0, 0, boxwidth2, leftAdjust2, true);
  }

  inMenuNum = 14;   // update current menu value

}

//====================================================================================================================

void drawChange_CO2_SampleRateMenu(bool refresh) {

  int16_t boxwidth = 60;
  int16_t leftAdjust = 60;
  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 2;


  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(30, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);

    ePaper.print(F("Set CO2 Interval"));


    ePaper.setTextSize(2);
    ePaper.setCursor(140, 85 );
    ePaper.print(F("Hour"));
    ePaper.setCursor(215, 85 );
    ePaper.print(F("Min"));

    ePaper.setTextSize(1);
    ePaper.setCursor(120, 150 );
    ePaper.print(F("A sample will be taken after"));
    ePaper.setCursor(120, 160 );
    ePaper.print(F("this amount of time has passed"));

  }


  // draw menu boxes on display with box highlighted if cursor index matches
  if ( cursorPos == 1) {  // hours box
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 1, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 2, false);
  }
  else if ( cursorPos == 2) { // minutes box
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 2, true);
  }

  inMenuNum = 15;   // update current menu value

}



//====================================================================================================================


void drawWriteToPC(bool refresh) {

  int16_t boxwidth = 105;
  int16_t leftAdjust = 60;

  maxCursor = 2;

  const char start[] = "Start";
  const char cancel[] = "Cancel";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &start[0];
  textList[1] = &cancel[0];



  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(65, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Send Data To PC"));
    ePaper.print('?');

    //ePaper.setTextSize(2);

  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 20;    // update current menu value


}

//====================================================================================================================



void drawSettingsMenu(bool refresh) {

  int16_t boxwidth = 210;
  int16_t leftAdjust = 15;
  maxCursor = 4;

  const char subMenuTimeText[] = "Time Settings";
  const char subMenuADSText[] = "ADC Settings";
  const char subMenuDataText[] = "Data Settings";
  const char errHistText[] = "Error History";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &subMenuTimeText[0];
  textList[1] = &subMenuADSText[0];
  textList[2] = &subMenuDataText[0];
  textList[3] = &errHistText[0];

  // setup display if this menu is not currently being displayed
  if ( refresh ) {
    ePaper.clearDisplay();
    ePaper.setCursor(127, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Settings"));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 30;    // update current menu value


}



//====================================================================================================================

void drawTimeSettingMenu(bool refresh) {

  int16_t boxwidth = 90;
  int16_t leftAdjust = 70;
  maxCursor = 2;

  const char subMenuTime_SettingDate_Text[] = "Date";
  const char subMenuTime_SettingTime_Text[] = "Time";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &subMenuTime_SettingDate_Text[0];
  textList[1] = &subMenuTime_SettingTime_Text[0];


  if ( refresh ) {
    ePaper.clearDisplay();
    // draw title
    ePaper.setCursor(80, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Time Settings"));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 110;   // update current menu value


}


//====================================================================================================================

void drawTimeSubSetting_Date_Menu(bool refresh ) {

  int16_t boxwidth = 60;
  int16_t leftAdjust = 80;
  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 3;


  if ( refresh ) {
    ePaper.clearDisplay();
    // draw title
    ePaper.setCursor(120, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Set "));
    ePaper.print(F("Date"));
    // draw labels
    ePaper.setTextSize(2);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.setCursor(90, 85 );
    ePaper.print(F("Day"));
    ePaper.setCursor(165, 85 );
    ePaper.print(F("Month"));
    ePaper.setCursor(255, 85 );
    ePaper.print(F("Year"));
  }


  // draw menu boxes on display with box highlighted if cursor index matches
  if ( cursorPos == 1) {

    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2 , false);
  }
  else if ( cursorPos == 2) {

    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2, false);
  }
  else if ( cursorPos == 3) {
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2, true);
  }


  inMenuNum = 111;   // update current menu value


}

//====================================================================================================================

void drawTimeSubSetting_Time_Menu(bool refresh) {

  int16_t boxwidth = 60;
  int16_t leftAdjust = 80;
  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 3;


  if ( refresh ) {
    ePaper.clearDisplay();
    // draw title
    ePaper.setCursor(120, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Set "));
    ePaper.print(F("Time"));
    //draw labels
    ePaper.setTextSize(2);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.setCursor(90, 85 );
    ePaper.print(F("Hour"));
    ePaper.setCursor(165, 85 );
    ePaper.print(F("Min"));
    ePaper.setCursor(255, 85 );
    ePaper.print(F("Sec"));
  }




  // draw menu boxes on display with box highlighted if cursor index matches
  if ( cursorPos == 1) {

    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 , false);
  }
  else if ( cursorPos == 2) {

    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1, true);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 , false);
  }
  else if ( cursorPos == 3) {
    drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1, false);
    memset(numBuffer, '\0', sizeof(numBuffer));
    drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 , true);
  }

  inMenuNum = 112;   // update current menu value


}

//====================================================================================================================

void drawADCSettingsMenu(bool refresh) {

  int16_t boxwidth = 240;
  int16_t leftAdjust = 0;


  const char channelEnable[] = "Enable Channels";
  const char channelDual[] = "Dual Channel mode";
  const char channelGain[] = "ADC PGA modes";
  //const char channelTable[] = "Use lookup Tables";

  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 3;


  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &channelEnable[0];
  textList[1] = &channelDual[0];
  textList[2] = &channelGain[0];
  //textList[3] = &channelTable[0];

  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(95, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("ADC Settings"));
  }

  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 120;   // update current menu value

}

//====================================================================================================================

void drawADC_ChannelEnable_Menu(bool refresh) {

  int16_t boxwidth = 160;
  int16_t boxwidth2 = 120;
  int16_t leftAdjust = -30;
  int16_t leftAdjust2 = 140;

  const char channelA0[] = "Channel A0:";
  const char channelA1[] = "Channel A1:";
  const char channelA2[] = "Channel A2:";
  const char channelA3[] = "Channel A3:";
  const char en[] = "Enabled";
  const char dis[] = "Disabled";

  //char numBuffer[5];
  //memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 4;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &channelA0[0];
  textList[1] = &channelA1[0];
  textList[2] = &channelA2[0];
  textList[3] = &channelA3[0];

  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(75, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Enable Channels"));
  }

  drawList(textList, maxCursor, 0, boxwidth, leftAdjust);


  // draw enable boxes based on if they are enabled or not. display "enabled" if the are, "disabled" if they are not
  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    if ( ((ADS_BitMask_enable >> i) & 0x01) == 1) {
      if ( (cursorPos - 1) == i) {
        drawSettingChangeBox( &en[0] , 0, i, boxwidth2, leftAdjust2, true);
      }
      else {
        drawSettingChangeBox( &en[0] , 0, i, boxwidth2, leftAdjust2, false);
      }
    }
    else {
      if ( (cursorPos - 1) == i) {
        drawSettingChangeBox( &dis[0] , 0, i, boxwidth2, leftAdjust2, true);
      }
      else {
        drawSettingChangeBox( &dis[0] , 0, i, boxwidth2, leftAdjust2, false);
      }
    }

  }

  inMenuNum = 121;   // update current menu value

}


//====================================================================================================================

void drawADC_Dual_Menu(bool refresh) {

  int16_t boxwidth = 160;
  int16_t boxwidth2 = 210;
  int16_t leftAdjust = -70;
  int16_t leftAdjust2 = 100;

  const char channelA0[] = "Channel A0:";
  const char channelA2[] = "Channel A2:";
  const char en[] = "Enabled";
  const char dis[] = "Disabled";
  const char chanDis[] = "Channel Disabled";

  //char numBuffer[5];
  //memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 2;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &channelA0[0];
  textList[1] = &channelA2[0];


  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(65, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Dual Channel Mode"));

    ePaper.setTextSize(1);
    ePaper.setCursor(40, 160 );
    ePaper.print(F("A Single channel reading will not be performed if dual"));
    ePaper.setCursor(55, 170 );
    ePaper.print(F("channel mode readings are enabled for a channel."));

    ePaper.setCursor(40, 190 );
    ePaper.print(F("Single Mode Readings will still be performed on channel"));
    ePaper.setCursor(45, 200 );
    ePaper.print(F("A1 and A2 if Dual channel Reads are enabled. Disable"));
    ePaper.setCursor(110, 210 );
    ePaper.print(F("these channels if not needed."));

  }

  drawList(textList, maxCursor, 0, boxwidth, leftAdjust);
  //free(textList); // ensure allocated memory is freed

  // draw enable boxes based on if they are enabled or not. display "enabled" if the are, "disabled" if they are not
  for (int i = 0; i < MAX_ADS1115_INPUTS / 2; i++) {

    if ( ((ADS_BitMask_enable >> (i + i) ) & 0x01) == 0) {  // display channel is disabled if channel is set to disabled
      if ( (cursorPos - 1) == i)
        drawSettingChangeBox( &chanDis[0] , 0, i, boxwidth2, leftAdjust2, true);
      else
        drawSettingChangeBox( &chanDis[0] , 0, i, boxwidth2, leftAdjust2, false);
    }

    else if ( ((ADS_BitMask_single >> i) & 0x01) == 1) {  // if a channel is in single channel mode and enabled
      if ( (cursorPos - 1) == i)
        drawSettingChangeBox( &dis[0] , 0, i, boxwidth2, leftAdjust2, true);

      else
        drawSettingChangeBox( &dis[0] , 0, i, boxwidth2, leftAdjust2, false);
    }
    else {                                              // otherwise, display it as dual channel is enabled
      if ( (cursorPos - 1) == i)
        drawSettingChangeBox( &en[0] , 0, i, boxwidth2, leftAdjust2, true);
      else
        drawSettingChangeBox( &en[0] , 0, i, boxwidth2, leftAdjust2, false);
    }

  }

  inMenuNum = 122;   // update current menu value

}

//====================================================================================================================

void drawADC_PGAModes_Menu(bool refresh) {

  int16_t boxwidth = 160;
  int16_t boxwidth2 = 70;
  int16_t leftAdjust = -10;
  int16_t leftAdjust2 = 170;

  const char channelA0[] = "Channel A0:";
  const char channelA1[] = "Channel A1:";
  const char channelA2[] = "Channel A2:";
  const char channelA3[] = "Channel A3:";


  char numBuffer[5];
  memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 4;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &channelA0[0];
  textList[1] = &channelA1[0];
  textList[2] = &channelA2[0];
  textList[3] = &channelA3[0];



  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(95, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Set PGA Mode"));

    ePaper.setTextSize(1);
    ePaper.setCursor(40, 200 );
    ePaper.print(F("PGA mode controls the gain applied to each channel of"));
    ePaper.setCursor(170, 210 );
    ePaper.print(F("the ADC."));
  }

  drawList(textList, maxCursor, 0, boxwidth, leftAdjust);
  //free(textList); // ensure allocated memory is freed

  // draw enable boxes based on if they are enabled or not. display "enabled" if the are, "disabled" if they are not
  for (int i = 0; i < MAX_ADS1115_INPUTS; i++) {
    memset(numBuffer, '\0', sizeof(numBuffer));

    switch (ADS1115Gain[i]) {
      case 0:
        numBuffer[0] = '2';
        numBuffer[1] = '/';
        numBuffer[2] = '3';
        numBuffer[3] = 'x';

        break;
      case 1:
        numBuffer[0] = '1';
        numBuffer[1] = 'x';
        break;
      case 2:
        numBuffer[0] = '2';
        numBuffer[1] = 'x';
        break;
      case 3:
        numBuffer[0] = '4';
        numBuffer[1] = 'x';
        break;
      case 4:
        numBuffer[0] = '8';
        numBuffer[1] = 'x';
        break;
      case 5:
        numBuffer[0] = '1';
        numBuffer[1] = '6';
        numBuffer[2] = 'x';
        break;

      default:
        numBuffer[0] = 'N';
        numBuffer[1] = 'A';
        numBuffer[2] = 'N';
        break;

    }

    if ( (cursorPos - 1) == i)
      drawSettingChangeBox( &numBuffer[0] , 0, i, boxwidth2, leftAdjust2, true);
    else
      drawSettingChangeBox( &numBuffer[0] , 0, i, boxwidth2, leftAdjust2, false);
  }

  inMenuNum = 123;   // update current menu value

}


//---------------------------------------------------------------------

void drawDataSettingsMenu(bool refresh) {
  int16_t boxwidth = 295;
  int16_t leftAdjust = -25;
  const char save2Sd[] = "Remove SD Card";
  //const char sampleBuffer[] = "Set Sample Buffer Size";
  const char ClearData[] = "Clear Data On SD";
  //  const char channelA3[] = "Channel A3:";


  //char numBuffer[5];
  //memset(numBuffer, '\0', sizeof(numBuffer));
  maxCursor = 2;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &save2Sd[0];
  //textList[1] = &sampleBuffer[0];
  textList[1] = &ClearData[0];


  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(90, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Data Settings"));

  }

  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

  inMenuNum = 130;   // update current menu value

}


//====================================================================================================================


void drawDataSetting_RemoveSD_Menu(bool refresh) {
  int16_t boxwidth = 295;
  int16_t leftAdjust = -25;
  maxCursor = 1;

  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(70, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Remove SD Card"));

    ePaper.setTextSize(2);
    ePaper.setCursor(60, 80 );
    ePaper.print(F("Please wait until all"));
    ePaper.setCursor(60, 100 );
    ePaper.print(F("data is saved to the SD"));
    ePaper.setCursor(60, 120 );
    ePaper.print(F("card before removing."));
  }
  updateStatusBar();
  ePaper.refresh();


  //+++++++++++++++++++++++++++++
  if ( sampleCount > 0 ) {
    // run save to sd method here

    saveToSD();
    saveCO2ToSD();
  }
  //+++++++++++++++++++++++++++++

  ePaper.fillRect( 35, 75, 350, 90, WHITE); // clear previous text
  ePaper.setTextSize(2);
  ePaper.setCursor(70, 80 );
  ePaper.print(F("Saving Data Complete!"));
  ePaper.setCursor(50, 120 );
  ePaper.print(F("SD card is safe to remove."));

  //drawList(textList, 1, cursorPos, boxwidth, leftAdjust);
  ePaper.fillRect(160, 170, 65, 25, BLACK);
  ePaper.setCursor(170, 175 );
  ePaper.setTextColor(WHITE, BLACK);
  ePaper.print(F("Back"));


  //updateStatusBar();

  inMenuNum = 131;   // update current menu value

}

//====================================================================================================================

void drawClearSDCardMenu(bool refresh) {

  int16_t boxwidth = 85;
  int16_t leftAdjust = 70;
  int16_t yAdjust = 50;
  maxCursor = 2;

  const char yes[] = "Yes";
  const char no[] = "No";

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &yes[0];
  textList[1] = &no[0];

  if ( refresh ) {
    ePaper.clearDisplay();

    ePaper.setCursor(55, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Clear Data On SD"));
    ePaper.print('?');

    ePaper.setTextSize(2);
    ePaper.setCursor(65, 70 );
    ePaper.print(F("Warning: this cannot be"));
    ePaper.setCursor(155, 85 );
    ePaper.print(F("undone."));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust);

  inMenuNum = 133;    // update current menu value


}


//====================================================================================================================


void drawErrorHistoryMenu(bool refresh) {

  int16_t boxwidth = 90;
  int16_t leftAdjust = 80;
  int16_t yAdjust = 110;
  uint8_t errCountNumposition = 0;
  uint8_t errCountNumMax = 0;
  uint8_t count = 0;




  const char backText[] = "Back";
  const char subMenuADSText[] = "ADC Settings";
  const char subMenuDataText[] = "Data Settings";
  const char errHistText[] = "Error History";

  maxCursor = 1;

  // assign the starting memory addresses of the text labels to the TextList pointer to pointers.
  textList[0] = &backText[0];


  // setup display if this menu is not currently being displayed
  if ( refresh ) {
    ePaper.clearDisplay();
    ePaper.setCursor(80, 30 );
    ePaper.setTextSize(3);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.print(F("Error History"));
  }

  // draw menu boxes on display with box highlighted if cursor index matches
  drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust);

  errCountNumposition = fram.readByte(errCodeNumAddr ); // get error count number position
  errCountNumMax = fram.readByte(errCodeNumTotalAddr ); // get max number of error count stored

  // sanity check to prevent reading from FRAM adresses not used for error codes
  if ( errCountNumposition > errCodeMaxNum) {
    errCountNumposition = errCodeMaxNum;
  }

  ePaper.setTextSize(1);
  ePaper.setTextColor(BLACK, WHITE);


  if (errCountNumMax != 0) {
    if (errCountNumposition > 0) {

      // error history may not start at '0' since the history is not shifted allong in FRAM, instead a position counter is incremented and the oldest value is overwritten
      for (uint8_t i = errCountNumposition; i < errCountNumMax; i++) {
        ePaper.setCursor(90, 70 + (i - errCountNumposition) * 13 );

        ePaper.print(F("Err: "));
        ePaper.print(fram.readUInt16(errCodeHistoryAddr + i * 8 ), HEX); // print error code
        ePaper.print(space);
        ePaper.print(space);
        ePaper.print(F("Time: "));

        for ( uint8_t c = 0; c < 5; c++) {
          temp1 = fram.readByte(errCodeHistoryAddr + i * 8 + 2 + c);
          if ( temp1 < 10) {
            ePaper.print(zero);
          }
          ePaper.print(temp1);
          if (c < 2) {
            ePaper.print(':');
          }
          else if (c == 2) {
            ePaper.print(space);
          }
          else if ( c > 2) {
            ePaper.print('/');
          }

        }

        //print year
        ePaper.print(20);
        temp1 = fram.readByte(errCodeHistoryAddr + i * 8 + 7);
        if ( temp1 < 10) {
          ePaper.print(zero);
        }
        ePaper.print(temp1);

      }
    }
    count = errCountNumMax - errCountNumposition;
    for (uint8_t i = 0; i < errCountNumposition; i++) {
      ePaper.setCursor(90, 70 + (count + i) * 13 );
      ePaper.print(F("Err: "));
      ePaper.print(fram.readUInt16(errCodeHistoryAddr + (count + i) * 8 ), HEX); // print error code
      ePaper.print(space);
      ePaper.print(space);
      ePaper.print(F("Time: "));

      for ( uint8_t c = 0; c < 5; c++) {
        temp1 = fram.readByte(errCodeHistoryAddr + (count + i) * 8 + 2 + c);
        if ( temp1 < 10) {
          ePaper.print(zero);
        }
        ePaper.print(temp1);
        if (c < 2) {
          ePaper.print(':');
        }
        else if (c == 2) {
          ePaper.print(space);
        }
        else if ( c > 2) {
          ePaper.print('/');
        }

      }

      //print year
      ePaper.print(20);
      temp1 = fram.readByte(errCodeHistoryAddr + (count + i) * 8 + 7);
      if ( temp1 < 10) {
        ePaper.print(zero);
      }
      ePaper.print(temp1);

    }
  }

  else {
    ePaper.setCursor(90, 70);
    ePaper.print(F("No errors in History"));
  }

  inMenuNum = 140;    // update current menu value


}

//================ Display notification messages ============================================

void drawDataClearedMessage() {
  ePaper.fillRect(105, 60, 180, 60, WHITE);
  ePaper.drawRect(105, 60, 180, 60, BLACK);
  ePaper.drawRect(106, 61, 178, 58, BLACK);
  ePaper.setCursor(120, 80);
  ePaper.setTextSize(2);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.print(F("Data Cleared!"));
  ePaper.refresh();
  ePaper.refresh();
  delay(2000);


}

void drawDataClearFailedMessage() {
  ePaper.fillRect(80, 60, 250, 70, WHITE);
  ePaper.drawRect(80, 60, 250, 70, BLACK);
  ePaper.drawRect(81, 61, 248, 68, BLACK);
  ePaper.setCursor(90, 80);
  ePaper.setTextSize(2);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.print(F("!Data Clear Failed!"));

  ePaper.refresh();
  ePaper.refresh();
  delay(5000);
}

void drawSamplingStart() {
  //ePaper.clearDisplay();
  ePaper.fillRect(90, 60, 220, 70, WHITE);
  ePaper.drawRect(90, 60, 220, 70, BLACK);
  ePaper.drawRect(91, 61, 218, 68, BLACK);
  ePaper.setCursor(100, 80);
  ePaper.setTextSize(2);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.print(F("Sampling Started!"));
  //delay(1);
  ePaper.refresh();
  ePaper.refresh();
  delay(2000);
}

void drawSamplingInProgress() {

  ePaper.fillRect(95, 60, 200, 110, WHITE);
  ePaper.drawRect(95, 60, 200, 110, BLACK);
  ePaper.drawRect(96, 61, 198, 108, BLACK);
  ePaper.setCursor(115, 80);
  ePaper.setTextSize(2);
  ePaper.setTextColor(BLACK, WHITE);
  ePaper.print(F("Sampling In"));
  ePaper.setCursor(140, 100);
  ePaper.print(F("Progress."));
  ePaper.setCursor(120, 130);
  ePaper.print(F("Please Wait.."));

  ePaper.refresh();
  ePaper.refresh();
  //delay(1000);
}


/*
  -----------------------------------------------------------------------------------
*/




bool batteryGood() {
  float measuredvbat;

  PM->APBCMASK.reg |= 0x00010000; // enable the ADC Clock
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC

  for (int i = 0; i < 3 ; i++) {
    measuredvbat += analogRead(VBATPIN);
  }

  ADC->CTRLA.bit.ENABLE = 0;       // disable ADC
  PM->APBCMASK.reg &= ~0x00010000; // disable the ADC Clock

  measuredvbat /= 3; //apply average

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  measuredvbat -= 0.07; //add offset to measurement to get better value aof actual voltage
  //  ePaper.setTextSize(1);
  //  ePaper.setTextColor(BLACK, WHITE);
  //  ePaper.setCursor(170, 220);
  //
  //  ePaper.print("VBat: " );
  //  ePaper.println(measuredvbat);
  if (measuredvbat < 3.55) { // low battery voltage is assumed to be 3.55v for lithium ion battery
    return false;
  }
  else {
    return true;
  }
}

//======================================================================

void error(uint16_t errorCode) {
  uint16_t err;
  uint8_t ledCount = 1;
  uint8_t errCountNum = fram.readByte(errCodeNumAddr) ; // get error code position
  uint8_t errCountNumtotal;

  // error history total for while the device is on

  err = fram.readUInt16(0);
  err |= errorCode;
  fram.writeUInt16(0, err); //store error in FRAM for later checking
  if (debug) {
    printError();
  }
  if (displayOn) {
    ePaper.setTextSize(1);
    ePaper.setTextColor(BLACK, WHITE);
    ePaper.setCursor(135, 5);
    ePaper.print(F("ERR: "));
    ePaper.print(errorCode, HEX);
    ePaper.refresh();
  }

  // error history over all time
  errCountNumtotal = fram.readByte(errCodeNumTotalAddr ); // get max number of error count stored

  // write error code history to FRAM
  if (errCountNum > errCodeMaxNum) {
    errCountNum = 0;
  }

  fram.writeUInt16(errCodeHistoryAddr + errCountNum * 8 , errorCode); //store error in FRAM history for later checking
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 2, DS3234.getHour()); //store timestamps for error in FRAM for later checking
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 3, DS3234.getMinute());
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 4, DS3234.getSecond());
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 5, DS3234.getDate());
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 6, DS3234.getMonth());
  fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 7, DS3234.getYear());

  fram.writeByte(errCodeNumAddr, errCountNum + 1); // update error count number
  if (debug) {
    Serial.print(F("Err code FRAMwrite address: "));
    Serial.println(errCodeHistoryAddr + errCountNum * 8 + 7);
  }

  // increment the total number of errors stored in FRAM counter
  if (errCountNumtotal <= errCodeMaxNum) {
    fram.writeByte(errCodeNumTotalAddr,  errCountNum + 1);
  }


  if (debug) {
    Serial.print(F("error history: "));

    if (errCountNumtotal > 0) {
      for (uint8_t i = errCountNum; i < errCountNumtotal; i++) {
        Serial.print(F(" Err: "));
        Serial.print(fram.readUInt16(errCodeHistoryAddr + i * 8 ), HEX); // print error code
        Serial.print(space);
        Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 2)); // print time stamps
        Serial.print(':');
        Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 3));
        Serial.print(':');
        Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 4));
        Serial.print(space);
        Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 5));
        Serial.print('/');
        Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 6));
        Serial.print('/');
        Serial.println(fram.readByte(errCodeHistoryAddr + i * 8 + 7));

      }
    }
    for (uint8_t i = 0; i < errCountNum; i++) {
      Serial.print(F(" Err: "));
      Serial.print(fram.readUInt16(errCodeHistoryAddr + i * 8 ), HEX); // print error code
      Serial.print(space);
      Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 2)); // print time stamps
      Serial.print(':');
      Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 3));
      Serial.print(':');
      Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 4));
      Serial.print(space);
      Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 5));
      Serial.print('/');
      Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 6));
      Serial.print('/');
      Serial.println(fram.readByte(errCodeHistoryAddr + i * 8 + 7));

    }

  }




  if (!displayOn) {
    //err = errorCode; // reuse err for finding number of times to flash led
    while (!(errorCode & 1)) {
      errorCode = errorCode >> 1; //bit shift errorCode untill the least significant bit is 1
      ledCount++;
    }

    digitalWrite(PWRLED, HIGH); // pulse LED to show start of error status flashing
    delay(800);
    digitalWrite(PWRLED, LOW);
    delay(500);
    pwrLED(ledCount); // flash led x number of times to indicate error

    //double up Led flashes to catch error easier
    delay(1000);
    digitalWrite(PWRLED, HIGH); // pulse LED to show start of error status flashing
    delay(800);
    digitalWrite(PWRLED, LOW);
    delay(500);
    pwrLED(ledCount); // flash led x number of times to indicate error
  }

  //  if(errorCode > 0x00FF){ // if the error is a hardware critical error, the devices has failed and is put to sleep
  //    fail();
  //  }


}

//======================================================================

void fail() {
  // go to sleep until device gets reset
  //cli(); // disable interrupts - this should prevent device from waking
  //EIMSK &= (0 << INT0);
  startSleep();

}

//======================================================================
void printError() {

  Serial.print(F("ERR: "));
  Serial.println(fram.readUInt16(0), HEX);


}


void pwrLED(uint8_t flashAmount) {

  for (int i = 0; i < flashAmount; i++) {
    digitalWrite(PWRLED, HIGH);
    delay(10);
    digitalWrite(PWRLED, LOW);
    delay(380);
  }

}


uint32_t FreeRam() {
  uint32_t stackTop;
  uint32_t heapTop;

  // current position of the stack.
  stackTop = (uint32_t) &stackTop;

  // current position of heap.
  void* hTop = malloc(1);
  heapTop = (uint32_t) hTop;
  free(hTop);

  // The difference is the free, available ram.
  return stackTop - heapTop;
}


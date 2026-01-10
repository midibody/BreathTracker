//************************
//Common.h
//************************
#include <stdint.h>

#define MOTOR_PIN 1

#define MOTOR_SPEED_LOW     150
#define MOTOR_SPEED_NORMAL  200
#define MOTOR_SPEED_HIGH    255

#define SWITCH_PIN 0 // !! Wake up on pin level does NOT work with pin 9...

// --- Constants ---
#define VBAT_MINI (2.5f) // GPT untrusted value - put higher value when real batteries

#define PROGRAM_WAKEUP_PERIOD 100 // timer to run the program

// on BACK position alerts (DEBUG / TEST MODE -> Short values)
//#define TIMER_BACK_POSITION_GRACE_PERIOD (20 * 1000UL) // (5*60000UL)  // 60000UL = 1 mn.
//#define TIMER_BACK_POSITION_REPETITION (10 * 1000UL) // 10 seconds
//#define TIMER_BACK_POSITION_STARTUP_DELAY (30 * 1000UL) // at startup we let the person on the back without triggering vibration alerts,as he/she is not yet supposed to sleep deeply

// on BACK position alerts (REAL MODE)
#define TIMER_BACK_POSITION_GRACE_PERIOD (60 * 1000UL) // 1 mn - (5*60000UL)  // 60000UL = 1 mn.
#define TIMER_BACK_POSITION_REPETITION (10 * 1000UL) // 10 seconds - Back position alarm repetition
#define TIMER_BACK_POSITION_STARTUP_DELAY (5 * 60 * 1000UL) // 5 mn - at startup we let the person on the back without triggering vibration alerts,as he/she is not yet supposed to sleep deeply

#define TIMER_ADD_RECORD (60000UL) // 1 mn 

#define TIMER_LED_BLINK 0 // Zero-> disable blink (1000*10) // every 10 sec

#define N_RECORDS_MAX 1000 // 1 record per minute during 12h = 720 records

// movement & breath parameters
#define BREATH_MIN_PERIOD_MS    1500     //after detecting a breath we ignore next ones during this period to avoid duplicates counting 
#define GYRO_TH_BIG_MOVE        150.0f   // to set fMoving = true;
#define ROTATION_COOLDOWN_MS    3000    // duration in ms to ignore respiration after big rotation movement


bool fPrintDebug = true;
bool fPlotDebug = false;
bool fMaxTrace = false;

//**************************
// all possible positions detected
typedef enum : uint8_t {
  POS_BACK    = 2,
  POS_RIGHT   = 3,
  POS_LEFT    = 1,
  POS_STANDUP = 5,
  POS_STOMAC  = 4,
  POS_UNKNOWN = 0
} Position;

const char* aPosText[]={"??" , "LEFT", "BACK" , "RIGHT", "STOMAC", "STANDING"};

//=> !! WARNING: MUST be multiple of 4 bytes, to simplify management of Flash writing blocs (based on 'words'multiple of 4 bytes) !!
//=> 8 bytes -> 2 words
typedef struct __attribute__((packed, aligned(4))) { // keep those attribues packed & aligned
//typedef struct {
  uint16_t time;           // secondes depuis démarrage (0..43200)
  uint8_t  position;      // 0..3 (valeurs POS_*)
  uint8_t  breathPerMinut;  // 0..255
  uint8_t  breathMax; // 00..60, or more? max <255 sec...
  uint8_t  cMoves;
  uint16_t padding; // free for extra functional fields
}recordData_t;

static_assert(sizeof(recordData_t) == 8, "recordData_t must be 8 bytes");
static_assert(alignof(recordData_t) == 4, "recordData_t must be 4-byte aligned");

//!! WARNING: this structure size MUST be multiple of 4 bytes!!
//=> 12 bytes
typedef struct __attribute__((packed, aligned(4))) {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t magic3;
  uint16_t counter; // incrmental number to detect the last chain of records that was written on flash
  uint16_t cRecords; // # of data records after this header
  int8_t biasGx;
  int8_t biasGy;
  int8_t biasGz;
  uint8_t errorCode;
  uint8_t errorValue;
} recordHead_t;
static_assert(sizeof(recordHead_t) == 12, "recordHead_t must be 12 bytes");
static_assert(alignof(recordHead_t) == 4, "recordHead_t must be 4-byte aligned");



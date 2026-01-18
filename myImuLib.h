//*******************
// Accelero Gyro Lib
//*******************

// ============ IMU (XIAO BLE Sense) ============
// Selon ton core/lib, l’IMU peut être gérée via Seeed_Arduino_LSM6DS3
// Installe: "Seeed Arduino LSM6DS3" (Library Manager)r

// =====================================================
/* si carte sur le ventre, usb a gauche:
- sur le dos: az -> 1, ax et ay -> 0
- coté droit: ax->-1, az et ay ->0
- coté gauche: ax->1, ay et az->0
- debout -> ay ->-1, ax et az->0
- sur le ventre: az->-1, ax et ay ->0
*/

#include <LSM6DS3.h> // rechercher dans le menu croquis>gestionnaire de bibliotheque, avec mots cles: Seeed Arduino LSM6DS3

extern void motorStart(uint16_t duration, uint16_t pwm);
static LSM6DS3 imu(I2C_MODE, 0x6A); // adresse courante LSM6DS3

// VARIABLES GLOBALES
float ax,ay,az,gx_dps,gy_dps,gz_dps;
float ax_raw, ay_raw, az_raw;

int8_t biasGx =0, biasGy=0, biasGz = 0;

bool fMoving = false;
Position position = POS_UNKNOWN;
Position previousPosition = POS_UNKNOWN;
Position lastPosition = POS_UNKNOWN; // globale

// ---- State ----
static float lpX=0, lpY=0, lpZ=0;
static float prevBp=0;
static bool  breathHigh=false;

static uint16_t imuReg_CTRL_XL, imuReg_CTRL2_G; // IMU accelero & Gyro config registers

static uint32_t lastBreathMs=0;
static uint32_t lastRotationMs=0;
uint32_t breathMaxFromPreviousPeriod=0;
uint32_t lastBreathFromPreviousPeriod=0;

static unsigned long backPositionStartTime = 0;
static bool backPositionGracePeriodTimerRunning = false;
static bool backPositionRepetitionTimerRunning = false;
uint16_t cAlertsOnBack = 0;


uint16_t breathCount = 0;
uint32_t breathMax = 0;
uint8_t cMoves = 0; // Big movments
uint16_t cSmallMoves = 0;// small moves that block accelero counting breath 

extern uint16_t cRecords;
extern bool fCycleStart;

// Position enum (use yours if already defined)

// ---- Breath profiles ----
typedef struct {
  float alpha;     // LPF alpha
  float thH;       // high threshold on delta
  float thL;       // low threshold on delta
  float gyroVeto;  // max gyro norm to accept a breath
} BreathProfile;

// Profiles for thresholds depending on position (need to be more sensitive on BACK)
static const BreathProfile BREATH_PROFILE_BACK = {
  .alpha = 0.7f, //0.06f,=> changement pour nuit 12 jan. si alpha augmente, frequence de coupure du filtre passe bas augmente=> plus de peaks
  .thH = 0.6f, //1.0f, changement de 0.6 a 0.5 nuit du 14 jan
  .thL = -0.3f, //-1.0f, chgt de -0.5 a -0.3 nuit du 14 jan
  .gyroVeto = 100.0f // was 60
};

static const BreathProfile BREATH_PROFILE_SIDE = {
  .alpha = 0.8f, 
  .thH = 0.6f, // was 1.0f until Jan 14
  .thL = -0.6f, // was -1.0f until 14 jan
  .gyroVeto = 100.0f
};

static const float dt = 0.1f;  // 100ms entre échantillons

// Structure de données (C pur)
typedef struct {
  float s[3];      // s[0]=ancien, s[1]=moyen, s[2]=récent
  uint8_t idx;
  bool ready;      // ou uint8_t si bool non disponible
} Derivative2;

Derivative2 deriv;

SlidingSlope slope_detector; // regressionlineaire

MedianFilterN medianAx;
MedianFilterN medianAy;
MedianFilterN medianAz;

//***********************************
bool isBackPositionTimerExpired(void)
//***********************************
{
  uint32_t now;

    if (!backPositionGracePeriodTimerRunning && !backPositionRepetitionTimerRunning)
       return false;

    now= millis();

    if (backPositionGracePeriodTimerRunning )
     {
       if ( (now - backPositionStartTime) >= TIMER_BACK_POSITION_GRACE_PERIOD )// grace period expired, we trigger a new shorter timer to vibrate
        {
          backPositionGracePeriodTimerRunning = false; // optionnel : timer one-shot
          backPositionRepetitionTimerRunning = true; // we initiate a repetition timer
          backPositionStartTime = now; // restart a timer
          return true;
        }
     }
     else if (backPositionRepetitionTimerRunning ) // we check if the alert repetitive timer expired
     {
       if ( (now - backPositionStartTime) >= TIMER_BACK_POSITION_REPETITION )// we repeat until person moves off BACK position
        {
          backPositionStartTime = now; // restart a timer
          return true;
        }
     }


    return false;
}

//**************************************************
static inline float norm3(float x, float y, float z)
//**************************************************
{
  return sqrtf(x*x + y*y + z*z);
}

//***************************************************************
static inline const BreathProfile* getBreathProfile(Position pos)
{
  // Dos => profil dos, Côtés => profil côté
  if (pos == POS_BACK) return &BREATH_PROFILE_BACK;
  if (pos == POS_LEFT || pos == POS_RIGHT) return &BREATH_PROFILE_SIDE;

  // Fallback (unknown / autre): plus tolérant que dos, moins que côté
  return &BREATH_PROFILE_SIDE;
}

//************************
void checkBreathAccelGyro()
//************************
{
  const BreathProfile* p = getBreathProfile(position);
  uint32_t now = millis();

  // Gyro: détecte rotation / agitation
  float gNorm = norm3(gx_dps, gy_dps, gz_dps);

  // Cooldown post-rotation
  if (now - lastRotationMs < ROTATION_COOLDOWN_MS) {
    breathHigh = false;
    //if (fPrintDebug) serialPrintln("Ignore breath: now - lastRotationMs < ROTATION_COOLDOWN_MS");
    return;
  }

  if (gNorm > GYRO_TH_BIG_MOVE) {       // Big movement
    lastRotationMs = now;           // memorize current time of movement to start timer and ignore measures during ROTATION_COOLDOWN_MS
    breathHigh = false;             // reset state
    cMoves++;
    if (fPrintDebug) serialPrintln("Big move - Ignore breath: gNorm > GYRO_TH_BIG_MOVE");
    return;                         
  }


  // Gyro veto (thresholds depend on position) - smaller moves than GYRO_TH_MOVE
  if (gNorm > p->gyroVeto) {
    breathHigh = false;
    cSmallMoves++;
    if (fPrintDebug ) { serialPrint("Small move - Ignore breath: gNorm > p->gyroVeto:");serialPrintln(gNorm); }
    return;
  }


  //************** PROCESSING ***********************

  // 2) Filtrage passe-bas IIR 1er ordre, accel multi-axes (alpha dépend du profil)
  lpX = p->alpha * ax + (1.0f - p->alpha) * lpX;
  lpY = p->alpha * ay + (1.0f - p->alpha) * lpY;
  lpZ = p->alpha * az + (1.0f - p->alpha) * lpZ;

  //float bx = ax - lpX; // ecart par rapport a la moyenne
  //float by = ay - lpY;
  //float bz = az - lpZ;

  // !! Signe moins ou pas ?? pour inverser les directions de la pente
  float delta = -sliding_slope_update(&slope_detector, lpZ);

  // for debug...
  uint16_t d = (uint16_t)(delta * 100);
  if (acceleroMax < d) acceleroMax = d;

  // 6) detection respiration - Hystérésis + période mini (seuils dépendent du profil)
  if (!breathHigh && (delta > p->thH) ) 
    {
     if (now - lastBreathMs > BREATH_MIN_PERIOD_MS) {
      
      if (fCycleStart) // we are at the begining of a cycle, we store the largest breath from previous period. we will keep it as THE max only if it is higher than all the ones of this new priod
       {  
        if (lastBreathMs) lastBreathFromPreviousPeriod = now - lastBreathMs;
       }
      else // not the first breath counted for this period
       {
        if ((now - lastBreathMs > breathMax)) breathMax = now - lastBreathMs;
       }
      //  B-----B--B---#---B----B-#--B
      //Max     5          6         3
      //cBreath    6   0          0 

      if (fPrintDebug)  { serialPrint("* * BREATH: "); serialPrint ((now - lastBreathMs)/1000); serialPrint(" sec. Max during period = "); serialPrintln(breathMax/1000); }
      
      breathCount++;
      lastBreathMs = now;
      }

     breathHigh = true;
    } 

  else if (breathHigh && delta < p->thL) // delta decreases bellow threshold
   {
    breathHigh = false; // to allow to detect next breath
   }

 if (fPlotDebug)
  {
    // on peut utiliser aussi: Serial Plotter VSCode (Arduino extension)
  //serialPrint("MAX:"); serialPrint(0.03,3); serialPrint('\t');
  //serialPrint("MIN:"); serialPrint(-0.03,3); serialPrint('\t');

  //serialPrint("1-az_raw:"); serialPrint(1-az_raw,3); serialPrint('\t');
  serialPrint("1-az:"); serialPrint(1-az,3); serialPrint('\t');
  serialPrint("xx*-lpz:"); serialPrint(200*(7.98-lpZ),3); serialPrint('\t');

  //serialPrint("bz:"); serialPrint(bz,3); serialPrint('\t');

  //serialPrint("b:"); serialPrint(b,3); serialPrint('\t');
  serialPrint("delta:"); serialPrint(delta,3); serialPrint('\t');
  float v;
  if (lastBreathMs == now) v=1.0; else v = 0;
  serialPrint("Breath:"); serialPrint(v,3); serialPrint('\t');
  float f=(float)breathHigh; f=f*5;
  serialPrint("BreathHigh:"); serialPrint(breathHigh,3); serialPrint('\t');
  //serialPrint("domAxis:"); serialPrint(domAxis,3); serialPrint('\t');
  serialPrint("position:"); serialPrint(position); serialPrint('\t');
   serialPrintln();
  }

}

//***************************************************************************
static constexpr float TH_ENTER = 0.90f; // posture "claire"
static constexpr float TH_HOLD  = 0.75f; // posture "plutôt" -> on garde lastPos

Position detectPosition2(float ax, float ay, float az, Position lastPos)
//***************************************************************************
{
  const float g = sqrtf(ax*ax + ay*ay + az*az);

  const float nx = ax / g;
  const float ny = ay / g;
  const float nz = az / g;

  Position pos = POS_UNKNOWN;
  if (nz > 0.9f ) pos = POS_BACK;
  else if (nz < -0.9f) pos = POS_STOMAC;
  else if (nx > 0.4f) pos = POS_LEFT;
  else if (nx < -0.4f) pos = POS_RIGHT;
  else if (ny < -0.6f) pos = POS_STANDUP;

 // if (fPrintDebug) { 
 //   serialPrint("Accelero values: "); serialPrint (nx, 3); serialPrint (" | "); serialPrint (ny, 3); serialPrint (" | "); serialPrint (nz, 3); serialPrint (" | Pos: "); serialPrintln (aPosText[pos] );
 //   }
  return pos;
}

//***************************************************
void checkPosAndMovements() 
//***************************************************
{
  // read IMU Accelerometer
  ax_raw = imu.readFloatAccelX();
  ay_raw = imu.readFloatAccelY();
  az_raw = imu.readFloatAccelZ();

  // filter peaks(low path filter)
  ax = medianUpdate (medianAx,ax_raw);
  ay = medianUpdate (medianAy,ay_raw);
  az = medianUpdate (medianAz,az_raw);

  // read Gyro & remove bias
  gx_dps = imu.readFloatGyroX() - biasGx;
  gy_dps = imu.readFloatGyroY() - biasGy;
  gz_dps = imu.readFloatGyroZ() - biasGz;

  // Check position 
  position = detectPosition2(ax, ay, az, lastPosition);
  lastPosition = position;

  if ( position != previousPosition ) 
    {
      if(fPrintDebug) { serialPrint("> Moved from "); serialPrint(aPosText[previousPosition]) ; serialPrint ("-> "); serialPrintln(aPosText[position]); }
      if ( (position == POS_STANDUP) && (previousPosition != POS_UNKNOWN) ) {  motorStart(500,MOTOR_SPEED_NORMAL);  delay(200);  motorStart(500,MOTOR_SPEED_NORMAL); }
    }

  if ( millis() < TIMER_BACK_POSITION_STARTUP_DELAY) 
  {
    previousPosition = position;
    return; // at startup we let the person quiet even if on the back...
  }

  // ==== Decide back position actions / alerts ====
  if ((position == POS_BACK) && !backPositionGracePeriodTimerRunning && !backPositionRepetitionTimerRunning) // previous check : (previousPosition != POS_BACK) )
   {
      if (fPrintDebug) serialPrintln("* * WARNING -> Moved on BACK Position. Launch grace period timer before vibrations.");

      backPositionStartTime = millis();
      backPositionGracePeriodTimerRunning = true;
   }

   else if (position != POS_BACK) // stop timer of back position alert
    {
      backPositionGracePeriodTimerRunning = false;
      backPositionRepetitionTimerRunning = false;
      cAlertsOnBack = 0;

    }

  previousPosition = position;

  if ( ALERTS_ON_BACK && isBackPositionTimerExpired() )
    {
     if (fPrintDebug) serialPrintln("* * ALERT -> ON BACK Position. VIBRATIONS !");

     if (cAlertsOnBack == 0) // start super smooth
        {      motorStart(1000, MOTOR_SPEED_LOW); delay(500);  motorStart(1000, MOTOR_SPEED_LOW); }

     else if (cAlertsOnBack < COUNT_ALERTS_ON_BACK_BEFORE_MAX_VIBRATIONS) // start smooth
        {      motorStart(1000, MOTOR_SPEED_NORMAL); delay(500);  motorStart(1000, MOTOR_SPEED_NORMAL); }

     else // wake up!
      {      motorStart(1000,MOTOR_SPEED_HIGH);  delay(500);  motorStart(1000,MOTOR_SPEED_HIGH); delay(500);motorStart(1000,MOTOR_SPEED_HIGH);delay(500);motorStart(1000,MOTOR_SPEED_HIGH);}
     
     cAlertsOnBack++;
    }

//   if (fPrintDebug) { 
//    serialPrint("Accelero values: "); serialPrint (ax_raw, 3); serialPrint (" | "); serialPrint (ay_raw, 3); serialPrint (" | "); serialPrint (az_raw, 3); serialPrint (" | Pos: "); serialPrintln (aPosText[position] );
//    }

  //serialPrint(ax, 3); serialPrint('\t');  //serialPrint(ay, 3); serialPrint('\t');  //serialPrint(az, 3); serialPrint('\t');  //serialPrintln();  
  //serialPrint(gx_dps, 3); serialPrint('\t');  serialPrint(gy_dps, 3); serialPrint('\t');  serialPrint(gz_dps, 3); serialPrint('\t');  serialPrint(fMoving); serialPrint('\t');  serialPrintln(position); 
}

//*********************************************
bool calibrateGyro ()
//*********************************************
{
  const uint16_t SAMPLE_MS   = 10;     // 100 Hz
  const uint16_t DURATION_MS = 2000;   // 2 s
  const uint16_t N_TOTAL     = DURATION_MS / SAMPLE_MS;

  // Seuil "immobile" (deg/s). Ajustable.
  // - 0.5 à 1.5 deg/s : typiquement OK
  const float STILL_SUM_ABS_TH = 60.0f;   // |gx|+|gy|+|gz| < XX deg/s

  // Protection contre un "coup" ponctuel : si ça saute trop d’un sample à l’autre -> rejet
  const float JUMP_SUM_ABS_TH  = 20.0f;   // |Δgx|+|Δgy|+|Δgz| < 2 deg/s

  float sx = 0.0f, sy = 0.0f, sz = 0.0f;
  uint16_t used = 0, rej = 0;

  float pgx = 0, pgy = 0, pgz = 0;
  bool hasPrev = false;

// speedup accelero internal rate during calibration
imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G , LSM6DS3_ACC_GYRO_ODR_G_1660Hz | LSM6DS3_ACC_GYRO_FS_G_245dps | LSM6DS3_ACC_GYRO_FS_125_DISABLED ); //Gyyro register

//====== iterations to read values ================
  for (uint16_t i = 0; i < N_TOTAL; i++) 
    {
    float gx, gy, gz;

    gx = imu.readFloatGyroX();
    gy = imu.readFloatGyroY();
    gz = imu.readFloatGyroZ();

    float sumAbs = fabsf(gx) + fabsf(gy) + fabsf(gz);
  
    float jumpAbs = 0.0f;
    if (hasPrev) {
      jumpAbs = fabsf(gx - pgx) + fabsf(gy - pgy) + fabsf(gz - pgz);
    }
    pgx = gx; pgy = gy; pgz = gz; hasPrev = true;
    /*if (fPrintDebug)
     {
      serialPrint ("JumpAbs , sumAbs , gx , gy , gz :"); serialPrint (jumpAbs,3); serialPrint(" , "); serialPrint(sumAbs,3); serialPrint(" | ");
      serialPrint (gx,3); serialPrint(" , "); serialPrint (gy,3); serialPrint(" , ");serialPrint (gz,3); serialPrint(" , "); serialPrintln();
     }*/

    // Critère d'immobilité : faible vitesse + pas de "saut" brusque
    if (sumAbs < STILL_SUM_ABS_TH && jumpAbs < JUMP_SUM_ABS_TH) {

      sx += gx; sy += gy; sz += gz;
      used++;
    } else {
      rej++;
    }

    delay(SAMPLE_MS);
  }

  // Exige un minimum de samples valides (sinon biais pas fiable)
  // Ici: au moins 60% des 2s immobiles.
  if (used < (N_TOTAL * 60) / 100) {
    biasGx = 0; biasGy = 0; biasGz= 0;
    if (fPrintDebug) serialPrintln("ERREUR Calibration gyro. Trop de mouvements.");
    return false;
  }

  // we suppose that bias < 255... to be checked depending on the dps selected for configuration
  sx = round(sx / used);
  sy = round(sy / used);
  sz = round(sz / used);

  if ( sx<255 && sy<255 && sz<255)
   {
    biasGx = (int8_t)sx; biasGy = (int8_t)sy; biasGz = (int8_t)sz;
   }

  else 
  {
    if (fPrintDebug) { serialPrintln ("ERROR: Gyro bias is higher than 255. Capped to 255."); }
    if ( sx > 255) sx = 255;
    if ( sy > 255) sy = 255;
    if ( sz > 255) sz = 255;
  }

  // move back to default Gyro config values (slow down speed)
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G , imuReg_CTRL2_G); 

  if (fPrintDebug) 
  {
  //serialPrint ("> Calibration Gyro: "); serialPrint (biasGx, 3); serialPrint (" | "); serialPrint (biasGy, 3); serialPrint (" | "); serialPrintln (biasGz, 3);
  serialPrint ("> Calibration Gyro (int8): "); serialPrint (biasGx); serialPrint (" | "); serialPrint (biasGy); serialPrint (" | "); serialPrintln (biasGz);

  }
  return true;
}


//***********************
void initIMU()
//***********************
/* default config:IMU Reg CTRL_XL [0x10]-> 66 (01100110). Reg CTRL2_G[0x11]-> 6C (01101100)
Register Accelero (par defaut au demarrage):
| Bits   | Champ           | Valeur binaire | Valeur | Interprétation |
| ------ | --------------- | -------------- | ------ | -------------- |
| b7..b4 | **ODR_XL[3:0]** | `0110`         | 6      | **416 Hz**     |
| b3..b2 | **FS_XL[1:0]**  | `01`           | 1      | **±16 g**      |
| b1..b0 | **BW_XL[1:0]**  | `10`           | 2      | **100 Hz**     |

Register Gyro (par defaut au demarrage):
| Bits   | Champ          | Valeur binaire | Valeur | Interprétation         |
| ------ | -------------- | -------------- | ------ | ---------------------- |
| b7..b4 | **ODR_G[3:0]** | `0110`         | 6      | **416 Hz**             |
| b3..b2 | **FS_G[1:0]**  | `11`           | 3      | **±2000 dps**          |
| b1     | **FS_125**     | `0`            | —      | ±125 dps **désactivé** |
| b0     | —              | `0`            | —      | **Doit rester à 0**    |

*/

{
  char a[128];
char aBin1[9], aBin2[9];  
uint8_t v1,v2;

//derivative2_init(&deriv);
sliding_slope_init(&slope_detector);//regression lineaire

medianInit (medianAx);
medianInit (medianAy);
medianInit (medianAz);

// IMU
if (imu.begin() != 0) {
   if (fPrintDebug) serialPrintln("IMU init failed");
   return;
   
  } else {
    //if (fPrintDebug) serialPrintln("IMU init ok");
  }

if (fPrintDebug && fMaxTrace)
{
imu.readRegister(&v1, 0x10);
imu.readRegister(&v2, 0x11);
toBinary8(v1,aBin1);
toBinary8(v2,aBin2);

sprintf (a, "Initial IMU Reg CTRL_XL [0x10]-> %02X (%s). Reg CTRL2_G[0x11]-> %02X (%s)", v1, aBin1, v2,aBin2);
serialPrintln(a);
}

imuReg_CTRL_XL = LSM6DS3_ACC_GYRO_ODR_XL_13Hz | LSM6DS3_ACC_GYRO_FS_XL_2g | LSM6DS3_ACC_GYRO_BW_XL_50Hz;
imuReg_CTRL2_G = LSM6DS3_ACC_GYRO_ODR_G_26Hz | LSM6DS3_ACC_GYRO_FS_G_245dps | LSM6DS3_ACC_GYRO_FS_125_DISABLED; //Gyroscope full-scale at 125 dps->OFF.
// si on endort et reveil IMO, alors -> DRDY_MASK (CTRL4_C bit DRDY_MASK = 1) pour qu il n envoie pas de data avant stabilisation

imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL , imuReg_CTRL_XL );//Accelero register
imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G  , imuReg_CTRL2_G  ); // Gyro register 

if (fPrintDebug && fMaxTrace)
{
imu.readRegister(&v1, 0x10);
imu.readRegister(&v2, 0x11);
toBinary8(v1,aBin1);
toBinary8(v2,aBin2);

sprintf (a, "Initial Patch> IMU Reg CTRL_XL [0x10]-> %02X (%s). Reg CTRL2_G[0x11]-> %02X (%s)", v1, aBin1, v2,aBin2);
serialPrintln(a);
}
}

//********************
void disableIMU()
//********************
{
 // Stop FIFO first (optional but clean)
  //imu.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);

  // Accel power-down (CTRL1_XL: ODR_XL = 0)
  //imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);

  // Gyro power-down (CTRL2_G: ODR_G = 0)
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);

imu.fifoClear();
}

//********************
void enableIMU()
//********************
{
  char aBin1[9], aBin2[9];  
  uint8_t v1,v2;

// si on endort et reveil IMO, alors -> DRDY_MASK (CTRL4_C bit DRDY_MASK = 1) pour qu il n envoie pas de data avant stabilisation
imuReg_CTRL_XL = LSM6DS3_ACC_GYRO_ODR_XL_13Hz | LSM6DS3_ACC_GYRO_FS_XL_2g | LSM6DS3_ACC_GYRO_BW_XL_50Hz;
imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL , imuReg_CTRL_XL );//Accelero register

imuReg_CTRL2_G = LSM6DS3_ACC_GYRO_ODR_G_26Hz | LSM6DS3_ACC_GYRO_FS_G_245dps | LSM6DS3_ACC_GYRO_FS_125_DISABLED; //Gyroscope full-scale at 125 dps->OFF.
imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G  , imuReg_CTRL2_G  ); // Gyro register


imu.fifoClear(); //works but useless 
//delay(10);
// Code GPT qui ne marche pas
// 2) Petit délai de stabilisation (OBLIGATOIRE)
 // delay(1);   // datasheet: quelques ms suffisent à 13–26 Hz

  // 3) FLUSH FIFO (très important)
  //imu.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x00);

  // 4) Redémarrer FIFO (exemple : stream + ODR aligné)
  imu.writeRegister(    LSM6DS3_ACC_GYRO_FIFO_CTRL5,     LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2 | LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz ); // 10 to 6600 (see LSM6DS3.cpp)

delay(10);

}



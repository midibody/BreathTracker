//*******************
// Accelero Gyro Lib
//*******************

// ============ IMU (XIAO BLE Sense) ============

// Install: "Seeed Arduino LSM6DS3" (Library Manager)

// =====================================================
/* si carte sur le ventre, usb a gauche:
- sur le dos: az -> 1, ax et ay -> 0
- coté droit: ax->-1, az et ay ->0
- coté gauche: ax->1, ay et az->0
- debout -> ay ->-1, ax et az->0
- sur le ventre: az->-1, ax et ay ->0
*/

#include <LSM6DS3.h> // rechercher dans le menu croquis>gestionnaire de bibliotheque, avec mots cles: Seeed Arduino LSM6DS3
#include <Wire.h>
#include <math.h>

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


uint16_t breathCount = 0;
uint32_t breathMax = 0;
uint8_t cMoves = 0;

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

//Si alpha était plus grand :tu mangerais la respiration. Plus petit : trop lent → retard de phase
// Suggested profiles for "boîtier sur le torse" @ 100ms wakeup
static const BreathProfile BREATH_PROFILE_BACK = {
  .alpha = 0.6f, //0.06f,=> si alpha augmente, frequence de coupure du filtre passe bas augmente
  .thH = 1.0f, //0.030f,
  .thL = -1.0f, //0.012f,
  .gyroVeto = 60.0f
};

static const BreathProfile BREATH_PROFILE_SIDE = {
  .alpha = 0.8f, 
  .thH = 1.0f,
  .thL = -1.0f,
  .gyroVeto = 60.0f
};

// Constante globale
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

/*
//***************************************
void derivative2_init(Derivative2* d) {
  d->s[0] = 0.0f;
  d->s[1] = 0.0f;
  d->s[2] = 0.0f;
  d->idx = 0;
  d->ready = false;
}

// **************************************************
float derivative2_update(Derivative2* d, float x) {
  // Shift des échantillons
  d->s[0] = d->s[1];
  d->s[1] = d->s[2];
  d->s[2] = x;
  
  d->idx++;
  if (d->idx >= 3) {
    d->ready = true;
  }
  
  if (!d->ready) return 0.0f;
  
  // Dérivée seconde : d²x/dt² = (x[n] - 2*x[n-1] + x[n-2]) / dt²
  float d2 = (d->s[2] - 2.0f*d->s[1] + d->s[0]) / (dt * dt);
  
  return d2;
}

//******* pour derivee N ***********
#define DERIV_N 8   // 4, 6, 8...

static float buf[DERIV_N];
static uint8_t w = 0;
static bool full = false;

//************************
float derivativeN(float x)
//************************
{
  buf[w++] = x;
  if (w >= DERIV_N) {
    w = 0;
    full = true;
  }

  if (!full) return 0;

  float a = 0, b = 0;
  for (uint8_t i = 0; i < DERIV_N / 2; i++) {
    a += buf[i];
    b += buf[i + DERIV_N / 2];
  }

  return (b - a) / (DERIV_N / 2);
}
*/
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

  if (gNorm > GYRO_TH_BIG_MOVE) {       // Big movement
    lastRotationMs = now;           // memorize current time of movement to start timer and ignore measures during ROTATION_COOLDOWN_MS
    breathHigh = false;             // reset state
    cMoves++;
    //if (fPrintDebug) Serial.println("Ignore breath: gNorm > GYRO_TH_BIG_MOVE");
    return;                         
  }

  // Gyro veto (thresholds depend on position) - smaller moves than GYRO_TH_MOVE
  if (gNorm > p->gyroVeto) {
    breathHigh = false;
    //if (fPrintDebug ) { Serial.print("Ignore breath: gNorm > p->gyroVeto:");Serial.println(gNorm); }
    return;
  }

  // Cooldown post-rotation
  if (now - lastRotationMs < ROTATION_COOLDOWN_MS) {
    breathHigh = false;
    //if (fPrintDebug) Serial.println("Ignore breath: now - lastRotationMs < ROTATION_COOLDOWN_MS");
    return;
  }

  //************** PROCESSING ***********************

  // 2) Filtrage passe-bas IIR 1er ordre, accel multi-axes (alpha dépend du profil)
  lpX = p->alpha * ax + (1.0f - p->alpha) * lpX;
  lpY = p->alpha * ay + (1.0f - p->alpha) * lpY;
  lpZ = p->alpha * az + (1.0f - p->alpha) * lpZ;

  float bx = ax - lpX; // ecart par rapport a la moyenne
  float by = ay - lpY;
  float bz = az - lpZ;

  // !! Signe moins ou pas ?? pour inverser les directions de la pente
  float delta = -sliding_slope_update(&slope_detector, lpZ);

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

      if (fPrintDebug)  { Serial.print("* * BREATH: "); Serial.print ((now - lastBreathMs)/1000); Serial.print(" sec. Max during period = "); Serial.println(breathMax/1000); }
      
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
  //Serial.print("MAX:"); Serial.print(0.03,3); Serial.print('\t');
  //Serial.print("MIN:"); Serial.print(-0.03,3); Serial.print('\t');

  Serial.print("1-az_raw:"); Serial.print(1-az_raw,3); Serial.print('\t');
  Serial.print("1-az:"); Serial.print(1-az,3); Serial.print('\t');
  Serial.print("XX*-lpz:"); Serial.print(200*(7.98-lpZ),3); Serial.print('\t');

  Serial.print("bz:"); Serial.print(bz,3); Serial.print('\t');

  //Serial.print("b:"); Serial.print(b,3); Serial.print('\t');
  Serial.print("delta:"); Serial.print(delta,3); Serial.print('\t');
  float v;
  if (lastBreathMs == now) v=3.0; else v = 0;
  Serial.print("Breath:"); Serial.print(v,3); Serial.print('\t');
  float f=(float)breathHigh; f=f*5;
  Serial.print("BreathHigh:"); Serial.print(breathHigh,3); Serial.print('\t');
  //Serial.print("domAxis:"); Serial.print(domAxis,3); Serial.print('\t');
  Serial.print("position:"); Serial.print(position); Serial.print('\t');
   Serial.println();
  }

}

static constexpr float TH_ENTER = 0.90f; // posture "claire"
static constexpr float TH_HOLD  = 0.75f; // posture "plutôt" -> on garde lastPos

//***************************************************************************
Position detectPosition2(float ax, float ay, float az, Position lastPos)
//***************************************************************************
{
  const float g = sqrtf(ax*ax + ay*ay + az*az);
  //if (g < 1e-3f) return POS_UNKNOWN;

  const float nx = ax / g;
  const float ny = ay / g;
  const float nz = az / g;

  //position = POS_UNKNOWN;
  Position pos = POS_UNKNOWN;
  if (nz > 0.9f ) pos = POS_BACK;
  else if (nz < -0.9f) pos = POS_STOMAC;
  else if (nx > 0.4f) pos = POS_LEFT;
  else if (nx < -0.4f) pos = POS_RIGHT;
  else if (ny < -0.6f) pos = POS_STANDUP;

 // if (fPrintDebug) { 
 //   Serial.print("Accelero values: "); Serial.print (nx, 3); Serial.print (" | "); Serial.print (ny, 3); Serial.print (" | "); Serial.print (nz, 3); Serial.print (" | Pos: "); Serial.println (aPosText[pos] );
 //   }
  return pos;

  /*
  // scores (plus grand = mieux)
  float best = nz;        Position pos = POS_BACK;
  float s    = -nz;       if (s > best) { best = s; pos = POS_STOMAC; }
  s          = nx;        if (s > best) { best = s; pos = POS_LEFT; }
  s          = -nx;       if (s > best) { best = s; pos = POS_RIGHT; }
  s          = -ny;       if (s > best) { best = s; pos = POS_STANDUP; }

  // Décision
  if (best >= TH_ENTER) return pos;              // posture nette
  if (best >= TH_HOLD && lastPos != POS_UNKNOWN) return lastPos; // zone grise -> on ne change pas

  return POS_UNKNOWN;
  */
}


//***************************************************************************
Position detectPositionSimple(float ax, float ay, float az, Position lastPos)
//***************************************************************************
{
  const float g = sqrtf(ax*ax + ay*ay + az*az);
  if (g < 1e-3f) return POS_UNKNOWN;

  const float nx = ax / g;
  const float ny = ay / g;
  const float nz = az / g;

  // scores (plus grand = mieux)
  float best = nz;        Position pos = POS_BACK;
  float s    = -nz;       if (s > best) { best = s; pos = POS_STOMAC; }
  s          = nx;        if (s > best) { best = s; pos = POS_LEFT; }
  s          = -nx;       if (s > best) { best = s; pos = POS_RIGHT; }
  s          = -ny;       if (s > best) { best = s; pos = POS_STANDUP; }

  // Décision
  if (best >= TH_ENTER) return pos;              // posture nette
  if (best >= TH_HOLD && lastPos != POS_UNKNOWN) return lastPos; // zone grise -> on ne change pas
  return POS_UNKNOWN;
}

// Seuils sur composantes normalisées (cos(angle))
// 0.75 ~ 41° ; 0.65 ~ 49°  (à ajuster selon ton besoin)
//static constexpr float TH_ENTER = 0.70f;
static constexpr float TH_EXIT  = 0.65f;
//********************************************************************
Position detectPosition(float ax, float ay, float az, Position lastPos)
//********************************************************************
{
  // norme de l'accélération (|g|)
  const float g = sqrtf(ax*ax + ay*ay + az*az);

  if (fPrintDebug ) 
    { 
      Serial.print ("detectPosition. g = "); Serial.print (g, 3);Serial.print (" . ");
    }   
 
  if (g < 1e-3f) 
  {
    if (fPrintDebug ) Serial.print ("* * * g < 1e-3f -> POS UNKNOWN. ");
    return POS_UNKNOWN;
  }

  // normalisation : insensible aux unités et au gain
  const float nx = ax / g;
  const float ny = ay / g;
  const float nz = az / g;

  const float axa = fabsf(nx);
  const float aya = fabsf(ny);
  const float aza = fabsf(nz);

  // Hystérésis : si on est déjà dans une position, on accepte de "tenir"
  // tant que l'axe correspondant reste au-dessus de TH_EXIT.
  auto stillValid = [&](Position p)->bool {
    switch (p) {
      case POS_LEFT:   return nx >  TH_EXIT;
      case POS_RIGHT:  return nx < -TH_EXIT;
      case POS_BACK:   return nz >  TH_EXIT;
      case POS_STOMAC: return nz < -TH_EXIT;
      case POS_STANDUP:return ny < -TH_EXIT;
      default:         return false;
    }
  };
  if (stillValid(lastPos)) return lastPos;

  // Sinon, on (re)détecte avec TH_ENTER
  if (aya >= axa && aya >= aza) {
    if (ny < -TH_ENTER) return POS_STANDUP;
    return POS_UNKNOWN;
  } else if (aza >= axa && aza >= aya) {
    if (nz >  TH_ENTER) return POS_BACK;
    if (nz < -TH_ENTER) return POS_STOMAC;
    return POS_UNKNOWN;
  } else {
    if (nx >  TH_ENTER) return POS_LEFT;
    if (nx < -TH_ENTER) return POS_RIGHT;
    return POS_UNKNOWN;
  }
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

  //if (fPrintDebug) 
  //  { Serial.print ("Gyro values: "); Serial.print (gx_dps, 3); Serial.print (" | "); Serial.print (gy_dps, 3); Serial.print (" | "); Serial.println (gz_dps, 3); }
/* methode bancale:
  const float TH = 7.0f; //0.55f; // g (≈ cos 56°)
  
	float axa = fabsf(ax);
	float aya = fabsf(ay);
	float aza = fabsf(az);

	if (aya > axa && aya > aza) {
	  if (ay < -TH) position = POS_STANDUP;
	  else position = POS_UNKNOWN;
	}
	else if (aza > axa && aza > aya) {
	  if (az > TH) position = POS_BACK;
	  else if (az < -TH) position = POS_STOMAC;
	  else position = POS_UNKNOWN;
	}
	else {
	  if (ax > TH) position = POS_LEFT;
	  else if (ax < -TH) position = POS_RIGHT;
	  else position = POS_UNKNOWN;
	}
*/

  //Serial.println(position);

  // ---- Détection mouvement ----
  // Bouge si gyro ou delta accel dépasse seuil
  /*
  const float GYRO_TH = 25.0f;    // deg/s
  const float ACC_TH  = 0.20f;    // g (variations rapides)
  static float lastAx = 0, lastAy = 0, lastAz = 0;

  float dA = fabs(ax - lastAx) + fabs(ay - lastAy) + fabs(az - lastAz);
  fMoving = (fabs(gx_dps) > GYRO_TH) || (fabs(gy_dps) > GYRO_TH) || (fabs(gz_dps) > GYRO_TH) || (dA > ACC_TH);

  lastAx = ax; lastAy = ay; lastAz = az;
  */

  if ( (position != previousPosition) && fPrintDebug)
   { Serial.print("> Moved from "); Serial.print(aPosText[previousPosition]) ; Serial.print ("-> "); Serial.println(aPosText[position]); }
 
  if ( millis() < TIMER_BACK_POSITION_STARTUP_DELAY) 
  {
    previousPosition = position;
    return; // at startup we let the person quiet even if on the back...
  }

  // ==== Decide back position actions / alerts ====
  if ((position == POS_BACK) && !backPositionGracePeriodTimerRunning && !backPositionRepetitionTimerRunning) // previous check : (previousPosition != POS_BACK) )
   {
      if (fPrintDebug) Serial.println("* * WARNING -> Moved on BACK Position. Launch grace period timer before vibrations.");

      backPositionStartTime = millis();
      backPositionGracePeriodTimerRunning = true;

      //motorStart(1000,255);  delay(500);  motorStart(1000,255);      
   }

   else if (position != POS_BACK) // stop timer of back position alert
    {
      backPositionGracePeriodTimerRunning = false;
      backPositionRepetitionTimerRunning = false;
    }

  previousPosition = position;

  if (isBackPositionTimerExpired())
    {
     if (fPrintDebug) Serial.println("* * ALERT -> ON BACK Position. VIBRATIONS !");

      motorStart(1000,MOTOR_SPEED_HIGH);  delay(500);  motorStart(1000,MOTOR_SPEED_HIGH);
    }

//   if (fPrintDebug) { 
//    Serial.print("Accelero values: "); Serial.print (ax_raw, 3); Serial.print (" | "); Serial.print (ay_raw, 3); Serial.print (" | "); Serial.print (az_raw, 3); Serial.print (" | Pos: "); Serial.println (aPosText[position] );
//    }

  //Serial.print(ax, 3); Serial.print('\t');  //Serial.print(ay, 3); Serial.print('\t');  //Serial.print(az, 3); Serial.print('\t');  //Serial.println();  
  //Serial.print(gx_dps, 3); Serial.print('\t');  Serial.print(gy_dps, 3); Serial.print('\t');  Serial.print(gz_dps, 3); Serial.print('\t');  Serial.print(fMoving); Serial.print('\t');  Serial.println(position); 
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
      Serial.print ("JumpAbs , sumAbs , gx , gy , gz :"); Serial.print (jumpAbs,3); Serial.print(" , "); Serial.print(sumAbs,3); Serial.print(" | ");
      Serial.print (gx,3); Serial.print(" , "); Serial.print (gy,3); Serial.print(" , ");Serial.print (gz,3); Serial.print(" , "); Serial.println();
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
    if (fPrintDebug) Serial.println("ERREUR Calibration gyro. Trop de mouvements.");
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
    if (fPrintDebug) { Serial.println ("ERROR: Gyro bias is higher than 255. Capped to 255."); }
    if ( sx > 255) sx = 255;
    if ( sy > 255) sy = 255;
    if ( sz > 255) sz = 255;
  }

  // move back to default Gyro config values (slow down speed)
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G , imuReg_CTRL2_G); 

  if (fPrintDebug) 
  {
  //Serial.print ("> Calibration Gyro: "); Serial.print (biasGx, 3); Serial.print (" | "); Serial.print (biasGy, 3); Serial.print (" | "); Serial.println (biasGz, 3);
  Serial.print ("> Calibration Gyro (int8): "); Serial.print (biasGx); Serial.print (" | "); Serial.print (biasGy); Serial.print (" | "); Serial.println (biasGz);

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

//derivative2_init(&deriv);
sliding_slope_init(&slope_detector);//regression lineaire

medianInit (medianAx);
medianInit (medianAy);
medianInit (medianAz);

// I2C => necessaire pour imu?
Wire.begin();

// IMU
if (imu.begin() != 0) {
   if (fPrintDebug) Serial.println("IMU init failed");
   return;
   
  } else {
    //if (fPrintDebug) Serial.println("IMU init ok");
  }

uint8_t v1,v2;

if (fPrintDebug && fMaxTrace)
{
imu.readRegister(&v1, 0x10);
imu.readRegister(&v2, 0x11);
toBinary8(v1,aBin1);
toBinary8(v2,aBin2);

sprintf (a, "IMU Reg CTRL_XL [0x10]-> %02X (%s). Reg CTRL2_G[0x11]-> %02X (%s)", v1, aBin1, v2,aBin2);
Serial.println(a);
}

imuReg_CTRL_XL = LSM6DS3_ACC_GYRO_ODR_XL_13Hz | LSM6DS3_ACC_GYRO_FS_XL_2g | LSM6DS3_ACC_GYRO_BW_XL_50Hz;
imuReg_CTRL2_G = LSM6DS3_ACC_GYRO_ODR_G_26Hz | LSM6DS3_ACC_GYRO_FS_G_245dps | LSM6DS3_ACC_GYRO_FS_125_DISABLED; //Gyroscope full-scale at 125 dps->OFF.
// si on endort et reveil IMO, alors -> DRDY_MASK (CTRL4_C bit DRDY_MASK = 1) pour qu il n envoie pas de data avant stabilisation

imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL , imuReg_CTRL_XL );//Accelero register
imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G  , imuReg_CTRL2_G  ); // Gyro register

}

//********************
void disableIMU()
//********************
{
//imu.writeRegister(0x10, 0x00); // CTRL1_XL: accel OFF (ODR=0)
//imu.writeRegister(0x11, 0x00); // CTRL2_G : gyro  OFF (ODR=0)
//imu.disableAccel();
//imu.disableGyro();
}

//********************
void enableIMU()
//********************
{

//imu.writeRegister(0x10, 0x10); // accel: ODR=12.5Hz, ±2g (valeur typique)
//imu.writeRegister(0x11, 0x10); // gyro : ODR=12.5Hz, ±250 dps (valeur typique)
//imu.enableAccel();
//imu.enableGyro();
}



#include <nrf_sdm.h>

#define NO_CLICK 0
#define LONG_CLICK  1
#define SHORT_CLICK 2

static bool     gBtnLast      = false;
static bool     gBtnActive    = false;
static bool     gLongFired    = false;
static uint32_t gPressStartMs = 0;

static const uint32_t LONG_CLICK_MIN_MS = 1500;
static const uint32_t DEBOUNCE_MS       = 30;

//***************************************
void dump_memory(uint32_t addr, size_t n) 
//***************************************
{
  const uint8_t *p = (const uint8_t*)(uintptr_t)addr;
  for (size_t i = 0; i < n; i++) {
    if (i % 16 == 0) { Serial.printf("\n0x%08lX: ", (unsigned long)(addr + i)); }
    Serial.printf("%02X ", p[i]);
  }
  Serial.println();
}

/*
//***********************************************
static inline bool is_word_aligned(uint32_t addr) 
//***********************************************
// Vérifie alignement (NVMC écrit par mots de 32 bits)
{  return (addr & 0x3u) == 0;}
*/

//*************************************
void toBinary8(uint8_t v, char *buf) {
  for (int i = 7; i >= 0; i--) {
    buf[7 - i] = (v & (1 << i)) ? '1' : '0';
  }
  buf[8] = '\0';
}

//*******************
void flashLed(uint16_t c, uint16_t d)
{
  uint16_t i;

  for (i=0; i < c ; i++)
   {
    digitalWrite(LED_BUILTIN, LOW); // lOW -> light ON...
    delay(5);
    digitalWrite(LED_BUILTIN, HIGH);
    if (i<c-1) delay (d); 
   }
}
//=================================================
void motorStart(uint16_t duration, uint16_t pwm)
//=================================================
{
  if (pwm > 255) pwm = 255;

  //digitalWrite (MOTOR_PIN, HIGH);
  analogWrite(MOTOR_PIN, pwm);

  // Maintien pendant "duration" ms
  delay(duration);
  //digitalWrite (MOTOR_PIN, LOW);
  analogWrite(MOTOR_PIN, 0);

}

//********************************
// WARNING: GPT dit que le debounce ne marche pas vraiment, a revoir s il faut demander un code modifié...
bool fSwitchedPressed(uint16_t &v)
{
  v = NO_CLICK;

  const bool btn = (digitalRead(SWITCH_PIN) == LOW); 

  // Front montant : appui
  if (btn && !gBtnLast) {
    //Serial.println(">> CLICK-> Front montant");
    gPressStartMs = millis();
    gBtnActive = true;
    gLongFired = false;
  }

  // Pendant l'appui : déclenche LONG dès 2s (une seule fois)
  if (btn && gBtnActive && !gLongFired) {

    const uint32_t dt = millis() - gPressStartMs;
    if (dt >= LONG_CLICK_MIN_MS) {
      gLongFired = true;
      v = LONG_CLICK;
      gBtnLast = btn;
      //Serial.println(">> CLICK-> LONG");
      return true;
    }
  }

  // Front descendant : relâchement
  if (!btn && gBtnLast && gBtnActive) {
    const uint32_t dt = millis() - gPressStartMs;
    gBtnActive = false;

    // Si LONG déjà envoyé, on ne renvoie rien au relâchement
    if (gLongFired) {
      gBtnLast = btn;
      return false;
    }

    // Sinon, SHORT si pas rebond
    if (dt >= DEBOUNCE_MS) {
      v = SHORT_CLICK;
      gBtnLast = btn;
      //Serial.println(">> CLICK-> SHORT");
      return true;
    }
  }

  gBtnLast = btn;
  return false;
}


//**********************
bool serialIsActive()
//**********************
{
  // "Serial" vrai si l'USB CDC est up sur beaucoup de cores
  // DTR = terminal ouvert (Moniteur Série / PuTTY etc.)
  // Si dtr() n’existe pas sur ton core, commente la ligne et garde juste (bool)Serial.
  return (bool)Serial && Serial.dtr();
}

//******************************
bool isSoftDeviceEnabled(void) 
//******************************
{
  uint8_t enabled = 0;
  if (sd_softdevice_is_enabled(&enabled) != NRF_SUCCESS) {
    return false;
  }
  return (enabled != 0);
}
//**********************
bool wokeFromSystemOff()
//**********************
{
  return (NRF_POWER->RESETREAS & POWER_RESETREAS_OFF_Msk);
}

//*********************************
// MEDIAN FILTER
//*********************************
#define MEDIAN_N 5   // 3, 5, 7...

typedef struct {
  float buf[MEDIAN_N];
  uint8_t idx;
  bool full;
} MedianFilterN;

//*********************************
void medianInit(MedianFilterN &f) {
  f.idx = 0;
  f.full = false;
  for (uint8_t i = 0; i < MEDIAN_N; i++) f.buf[i] = 0.0f;
}

//*********************************
static inline void sortFloatArray(float *a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    float key = a[i];
    int8_t j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

//*********************************
float medianUpdate(MedianFilterN &f, float x) {
// to filter signal crasy peaks
  f.buf[f.idx++] = x;
  if (f.idx >= MEDIAN_N) {
    f.idx = 0;
    f.full = true;
  }

  if (!f.full) return x;  // pas assez d’échantillons

  float tmp[MEDIAN_N];
  for (uint8_t i = 0; i < MEDIAN_N; i++)
    tmp[i] = f.buf[i];

  sortFloatArray(tmp, MEDIAN_N);

  return tmp[MEDIAN_N / 2];
}

/*
#define DERIV_K 5   // 3 à 6 typiquement
//*********************************
static float hist[DERIV_K + 1];
static uint8_t hidx = 0;
static bool derivateKfull = false;

//*****************************
float derivativeK(float b)
{
  hist[hidx++] = b;
  if (hidx > DERIV_K) {
    hidx = 0;
    derivateKfull = true;
  }

  if (!derivateKfull) return 0.0f;

  float oldest = hist[hidx];           // b[n-k]
  float newest = hist[(hidx + DERIV_K) % (DERIV_K + 1)]; // b[n]

  return (newest - oldest) / DERIV_K;
}
*/

//***********LINEAR REGRESSION to detect variations of Accelero signal ***************
// Configuration
#define SLOPE_WINDOW_SIZE 10  // Nombre de points pour calculer la pente

// Structure pour la pente glissante
typedef struct {
  float buffer[SLOPE_WINDOW_SIZE];
  uint8_t index;
  bool full;
  float sum_x;      // Somme des indices (précalculée)
  float sum_x2;     // Somme des indices² (précalculée)
} SlidingSlope;

//******************************************
void sliding_slope_init(SlidingSlope* ss) {
//******************************************
  for (uint8_t i = 0; i < SLOPE_WINDOW_SIZE; i++) {
    ss->buffer[i] = 0.0f;
  }
  ss->index = 0;
  ss->full = false;
  
  // Précalcul des sommes (optimisation)
  // sum_x = 0 + 1 + 2 + ... + (N-1) = N*(N-1)/2
  ss->sum_x = (float)(SLOPE_WINDOW_SIZE * (SLOPE_WINDOW_SIZE - 1)) / 2.0f;
  
  // sum_x2 = 0² + 1² + 2² + ... + (N-1)² = N*(N-1)*(2N-1)/6
  ss->sum_x2 = (float)(SLOPE_WINDOW_SIZE * (SLOPE_WINDOW_SIZE - 1) * (2 * SLOPE_WINDOW_SIZE - 1)) / 6.0f;
}

//***********************************************
// Calcul de la pente (régression linéaire)
float sliding_slope_update(SlidingSlope* ss, float y) {
//***********************************************

  // Ajouter la nouvelle valeur dans le buffer circulaire
  ss->buffer[ss->index] = y;
  ss->index++;
  
  if (ss->index >= SLOPE_WINDOW_SIZE) {
    ss->index = 0;
    ss->full = true;
  }
  
  // Attendre d'avoir N points
  if (!ss->full) {
    return 0.0f;
  }
  
  // Calcul de la régression linéaire: y = a*x + b
  // Formule de la pente: a = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x²) - sum(x)²)
  
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  
  // Parcourir le buffer dans l'ordre chronologique
  for (uint8_t i = 0; i < SLOPE_WINDOW_SIZE; i++) {
    uint8_t buf_idx = (ss->index + i) % SLOPE_WINDOW_SIZE;
    float y_val = ss->buffer[buf_idx];
    
    sum_y += y_val;
    sum_xy += (float)i * y_val;
  }
  
  // Calcul de la pente
  float N = (float)SLOPE_WINDOW_SIZE;
  float numerator = N * sum_xy - ss->sum_x * sum_y;
  float denominator = N * ss->sum_x2 - ss->sum_x * ss->sum_x;
  
  //float slope = numerator / denominator; initial code from claude
  float slope = numerator; // no need to devide by a constant in our context

  return slope;
}

#include "nrf.h"

static volatile bool tick = false;

// 👇 Période en millisecondes (modifiable à chaud)
static volatile uint32_t tick_ms = 100;

// (optionnel) si tu veux imposer une limite de période
static constexpr uint32_t TICK_MS_MIN = 1;
static constexpr uint32_t TICK_MS_MAX = 512000; // ~512 s max (limite RTC 24-bit)

//***************************************
void setTickPeriodMs(uint32_t ms) 
//***************************************
// Change la période en ms (thread-safe simple)
{
  if (ms < TICK_MS_MIN) ms = TICK_MS_MIN;
  if (ms > TICK_MS_MAX) ms = TICK_MS_MAX;
  tick_ms = ms;
}

//***************************************
bool checkRtcEvent()
//***************************************
{
if (tick) 
	{
    tick = false;
	return true;
	}
else return false;
	
}

//***************************************
static bool lfclk_start_try(uint32_t src)
//***************************************
{
  NRF_CLOCK->LFCLKSRC = (src << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;

  for (uint32_t i = 0; i < 500000; i++) {
    if (NRF_CLOCK->EVENTS_LFCLKSTARTED) return true;
  }
  return false;
}

//***************************************
static void lfclk_start()
//***************************************
{
  if (!lfclk_start_try(CLOCK_LFCLKSRC_SRC_Xtal)) {
    lfclk_start_try(CLOCK_LFCLKSRC_SRC_RC);
  }
}

//***************************************
extern "C" void RTC2_IRQHandler(void)
//***************************************
{
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;

    tick = true;

    // ✅ conversion faite ICI : ms -> ticks RTC (LFCLK = 32768 Hz)
    uint32_t ms = tick_ms; // lecture volatile (atomique sur 32-bit nRF52)
    if (ms < TICK_MS_MIN) ms = TICK_MS_MIN;
    if (ms > TICK_MS_MAX) ms = TICK_MS_MAX;

    // ticks = ms * 32768 / 1000
    // we use maximum few seconds (in debug mode), sono risk of overflow here
    uint32_t period_ticks = (ms * 32768UL) / 1000UL;
    if (period_ticks == 0) period_ticks = 1;

    NRF_RTC2->CC[0] = (NRF_RTC2->CC[0] + period_ticks) & 0x00FFFFFF;
  }
}

//***************************************
static void rtc2_start()
//***************************************
{
  lfclk_start();

  NRF_RTC2->TASKS_STOP  = 1;
  NRF_RTC2->TASKS_CLEAR = 1;

  NRF_RTC2->PRESCALER = 0; // 32768 Hz

  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_SetPriority(RTC2_IRQn, 6);
  NVIC_EnableIRQ(RTC2_IRQn);

  // Premier compare : à partir de maintenant, selon tick_ms
  uint32_t ms = tick_ms;
  uint32_t first_ticks = (ms * 32768UL) / 1000UL;

  if (first_ticks == 0) first_ticks = 1;
  NRF_RTC2->CC[0] = (NRF_RTC2->COUNTER + first_ticks) & 0x00FFFFFF;

  NRF_RTC2->TASKS_START = 1;
}

//***************************************
static inline void sleep_until_irq()
//***************************************
//C’est un pattern standard recommandé par ARM pour garantir :pas d’événement fantôme, sommeil réel et déterministe, conso minimale
{
  
__SEV();   // force un événement connu
__WFE();   // le consomme
__WFE();   // dort proprement
}

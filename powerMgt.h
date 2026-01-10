
/* Xiao Sense has pins to read VBAT voltage by software, cool
definitions in variant.h:
#define VBAT_ENABLE             (14)    // Output LOW to enable reading of the BAT voltage.
                                        // https://wiki.seeedstudio.com/XIAO_BLE#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging

#define PIN_CHARGING_CURRENT    (22)    // Battery Charging current
                                        // https://wiki.seeedstudio.com/XIAO_BLE#battery-charging-current

#define PIN_VBAT                (32)    // Read the BAT voltage.
                                        // https://wiki.seeedstudio.com/XIAO_BLE#q3-what-are-the-considerations-when-using-xiao

definitions in variant.cpp:
PinDescription g_APinDescription[] = {
  ....
  // VBAT_READ / Analog
  { P0_31, NULL, NULL, NULL },     // D32/VBAT_READ  
  ....
  // VBAT
  { P0_14, NULL, NULL, NULL },     // D31/VBAT_ENABLE

*/

//**************
float readVBAT()
//**************
{
  analogReadResolution(12); // 0..4095

  // Active le pont diviseur (P0.14 à 0)
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  delayMicroseconds(200);   // suffisant, pas besoin de 5 ms

  uint16_t raw = analogRead(PIN_VBAT);

  // Coupe le pont (zéro fuite) : repasse en Hi-Z
  pinMode(VBAT_ENABLE, INPUT);

  // Conversion:
  // Vadc = raw/4095 * 3.6V
  // Vbat = Vadc / k, avec k = 510k/(1M+510k) = 0.337748
  const float k = 0.337748f;
  float vadc = (raw * 3.6f) / 4095.0f;
  float vbat = vadc / k;

  return vbat;
}


//*******************************
void gotoSystemOff_WakeupOnSwitch()
//******************************
{
  // Avoid Serial if usb is not active...
  if (fPrintDebug) {
    Serial.println("SYSTEMOFF called ...");
    Serial.flush();
    delay(20);

    //return;
  }

  // LED off (selon ton câblage, LOW n'est pas toujours OFF)
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, LOW);

// Turn LED off (depending on board wiring, LOW may not always mean OFF)
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, LOW);

  // --- 1) Configure D9 as input with pull-up (Arduino side) ---
  // Button assumed between D9 and GND
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // --- 2) Get the NRF GPIO number corresponding to Arduino D9 ---
  // Works on cores exposing g_ADigitalPinMap[]
  uint32_t nrf_pin = g_ADigitalPinMap[SWITCH_PIN];

  // --- 3) Clear any previous GPIO wake-up latches ---
  // If the pin is already LOW when entering SYSTEMOFF,
  // the MCU could wake up immediately without this.
  NRF_GPIO->LATCH = 0xFFFFFFFF;
  (void)NRF_GPIO->LATCH;

  // --- 4) Configure GPIO hardware SENSE ---
  // Wake up on LOW level, keep internal pull-up enabled
  NRF_GPIO->PIN_CNF[nrf_pin] =
      (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)  |
      (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);

  // --- 5) Enter SYSTEM OFF mode ---
  // CPU, RAM and peripherals are powered down
  // GPIO SENSE remains active and will trigger a RESET on wake-up
  NRF_POWER->SYSTEMOFF = 1;

  // Execution never reaches here
  while (1) { __WFE(); }

}
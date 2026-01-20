# SleepBreath

**Tracking Breathing, Sleep Position and Apnea Using Only an IMU**

SleepBreath is a small, battery-powered, open‑source sleep monitoring device based on a single inertial sensor (IMU). It extracts breathing rate, sleep position, movements, and suspected sleep apnea events by measuring low‑frequency micro‑movements of the human body.

The project is designed for experimentation, transparency, and reproducibility. No microphones, cameras, cloud services, or machine learning models are used.
the software is Arduino based in C.

---

## Key Features

* Breathing rate estimation (breaths per minute)
* Detection of suspected sleep apnea events (breathing pauses)
* Sleep position detection (back, sides, stomach, upright)
* Movements detection/counting and discrimination from breathing
* Optional vibration feedback to discourage sleeping on the back, as people tend to have apneas only when on the back
* Breath counters stored on Flash data, with readout directly from the Arduino IDE, by sending commands to the board
* Local data logging to internal flash memory (once per minute)
* Circular flash memory management to maximize flash lifetime

  

<img height="323" alt="image" src="https://github.com/user-attachments/assets/706c1a73-475a-484c-8c9d-2677173494c2" /> <img width="321" height="323" alt="image" src="https://github.com/user-attachments/assets/eec2eb92-bf1a-4dcd-afd9-9e24120603fb" />
<img width="600" height="230" alt="image" src="https://github.com/user-attachments/assets/97098383-4444-4e41-a7c8-a05990deb762" />


## Hardware Overview

* **MCU:** Seeed Studio XIAO nRF52840
* **IMU (integrated into the XIAO nRF52840:** LSM6DS3 (3‑axis accelerometer + 3‑axis gyroscope)
* Status LED
* Push button
* Vibration motor
* Battery powered
* Plastic enclosure (small electric box)

The hardware is intentionally minimal to keep power consumption low and ensure easy reproduction.

---

## Principle of Operation

### Breathing Detection

Each breathing cycle produces small, periodic mechanical movements of the torso. These movements are captured by the accelerometer at very low frequencies (typically below 1 Hz).

The firmware:

* samples acceleration data continuously
* filters out high‑frequency noise and gross movements
* extracts low‑frequency oscillations
* detects breathing cycles and computes respiration rate

 Data from accelerometer, with derivate computation to detect moves:
<img width="758" height="383" alt="image" src="https://github.com/user-attachments/assets/31b94dbc-5c9e-4ea9-9848-9a62d11e30ba" />

### Movement vs Breathing Separation

Breathing is distinguished from voluntary movements using:

* signal amplitude
* signal duration
* temporal regularity

Short, irregular, high‑amplitude signals are classified as movements. Low‑amplitude, periodic signals are classified as breathing.

### Sleep Position Detection

Sleep position is determined using the direction of the gravity vector measured by the accelerometer. The firmware classifies the dominant orientation into predefined positions (back, left side, right side, stomach, upright).

### Sleep Apnea Detection

Suspected apnea events are detected when:

* the breathing signal disappears or drops below a defined threshold
* for longer than a configurable duration

These events are logged for later offline analysis. The device does **not** provide medical diagnosis.

### Positional Therapy (Vibration Feedback)

If enabled, a vibration motor is triggered when the subject remains on their back for too long. This gentle feedback encourages a spontaneous change to a side position, a common technique used in positional therapy.

---

## Data Logging and Flash memory Management

* Data is recorded once per minute
* Logged data includes:

  * breathing rate
  * sleep position: LEFT, RIGHT, LEFT, STOMAC, STANDING
  * movements count
  * Max breathing duration per minut (to detect apnea events)

To preserve flash memory lifetime, the firmware implements:

* circular flash storage
* wear‑optimized erase/write strategy
* no repeated erase of the same flash sector

This allows long‑term use without premature flash degradation.

---

## Reading Data from Flash

Recorded data can be read directly using the Arduino IDE:

* via serial connection
* no external tools required
* no custom bootloader

This makes data extraction and analysis simple for developers and researchers.
Example of graph built from the captured data:
<img width="992" height="362" alt="image" src="https://github.com/user-attachments/assets/f93b9efc-53d9-4f7c-a097-62116f91d6d6" />

---

## Power and Autonomy

The device is designed for overnight operation on battery power. Low‑power modes of both the MCU and IMU are used whenever possible to minimize consumption.

---

## Project Status

* Firmware: functional and tested, certainly still some bugs or corner cases
* Hardware: stable prototype

---

## Disclaimer

This project is **not a medical device** and is not intended for diagnosis or treatment of any medical condition. It is provided for experimentation, learning, and personal data exploration only.

---

## License

This project is released under an open‑source license. See the LICENSE file for details.

---

## Related Article

A detailed technical article describing the design and results is available on Hackaday.io.

---

## Contributions

Contributions, discussions, and improvements are welcome. Feel free to open issues or pull requests.

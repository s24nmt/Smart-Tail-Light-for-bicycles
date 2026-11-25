# Smart-Tail-Light-for-bicycles
his firmware turns a Seeed Studio XIAO ESP32C6 and an MPU6050 6-axis IMU into a smart tail light controller for bicycles or two-wheelers driving an 8-LED WS2812B strip.
It combines motion/brake detection, non-volatile configuration storage, physical button controls, power management, and a built-in captive-portal web UI.
# Smart Braking & Motion Detection
Coordinate Remapping & Calibration: On startup, calibrateSensorInitial() averages 40 samples, remaps axes for a 90-degree mount rotation, biases gyro offsets within ±10 deg/s, and initializes pitch, roll, baseline acceleration, and gravity reference values.
Roll & Pitch Compensation: Tracks roll and pitch with complementary filters (0.992 for roll; adaptive 0.985 during normal riding and 0.996 during braking/counting for pitch), then projects out gravity (sin(estimatedPitch)) and compensates pitch rate using roll angle to reduce false triggers on turns.
False-Trigger Filtering: Filters out bumps/potholes
Debounced Brake State Machine
# Motion Wake & Auto-Off Timeout: 
Wakes sleeping lights when filteredAx or ay_g changes by more than 0.05 (motionThreshold), and turns lights off after timeoutSeconds (default 30s, adjustable 0–30s, with 0 disabling auto-off) of inactivity when not running the Soft AP.
# Lighting Modes & Visual Feedback 10 Lighting Modes
# Wi-Fi Soft AP, Captive Portal & Flash Persistence
Captive Portal Access Point: Long-pressing D3 for 2 seconds starts an open Wi-Fi access point named SmartBrake at 192.168.4.1 with a DNSServer redirecting captive-portal probe URLs (Android, iOS, macOS, Windows) and unknown paths to /. During Soft AP mode, LEDs blink blue (0, 0, 255) except during a 2-second preview after changing mode from the web UI.
# NVS Flash Persistence: 
Uses Preferences under namespace "taillight" to persist mode, bright, brkThr, brkRel, brkCnt, and tmOut across reboots and deep sleep cycles.
# Arduino IDE Setup
Install ESP32 Board Support:
Open Arduino IDE (2.x recommended) and go to File → Preferences.
Add Espressif's board package URL to Additional boards manager URLs if not already present:[https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json](https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)
Open Tools → Board → Boards Manager, search for esp32 by Espressif Systems, and install the latest version supporting ESP32-C6.
Select the Board & Port:
Go to Tools → Board → esp32 → XIAO_ESP32C6.
Connect the XIAO ESP32C6 via USB-C and select its serial Port under Tools → Port.
Set USB CDC On Boot to Enabled if you want serial output over the USB-C port.
Install Required Library:
Open Sketch → Include Library → Manage Libraries, search for Adafruit NeoPixel, and install it. (Wire.h, Preferences.h, WiFi.h, WebServer.h, DNSServer.h, esp_sleep.h, driver/rtc_io.h, and driver/gpio.h are built into the ESP32 Arduino core.)

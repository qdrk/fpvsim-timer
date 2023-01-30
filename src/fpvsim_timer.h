#if defined(ESP8266)
#include <ESP8266WiFi.h>

#define RSSI_PIN A0
#else
#include "WiFi.h"

#define RSSI_PIN 34
#endif

#include <sstream>
#include <SPI.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <Int64String.h>

// Time set to shutdown.
unsigned long shutdownMillis = 0;

// Reconnect timestamp millis.
unsigned long previousReconnectMillis = 0;
// Reconnection interval.
unsigned long reconnectInterval = 1000 * 5;
// Used to bookkeep whether connected to router.
bool isWifiConnected = false;
// SSID for AP mode.
char apSsid[11];

const int slaveSelectPin = SS; // Setup data pins for rx5808 comms
const int spiDataPin = MOSI;
const int spiClockPin = SCK;

// #define DEV_MODE

uint8_t INITIAL_RSSI_FILTER = 30;

// Leave offset will multiply this factor when in calibration mode.
uint16_t CALIBRATION_LEAVE_RSSI_FACTOR = 2;
// Maximum passes for calibration to be considered done.
uint8_t CALIBRATION_PASSES = 1;
// Calibration has to last at least 60 seconds.
uint32_t CALIBRATION_MIN_TIME_MICROS = 30 * 1000 * 1000;
// Each lap has to take at least 4 seconds.
uint32_t MIN_LAP_TIME_MICROS = 4 * 1000 * 1000;

struct SettingsType {
  uint16_t volatile vtxFreq = 5732;

  uint8_t volatile filterRatio = INITIAL_RSSI_FILTER;

  // The RSSI when quad is close.
  uint16_t volatile rssiPeak = 270;
  uint16_t volatile enterRssiOffset = 6;
  uint16_t volatile leaveRssiOffset = 27;

  // Id of the timer, 0 - 25.
  uint8_t volatile id = -1;

  char apIp[20];
  char localIp[20];
  char apSsid[30];

  // The router ssid and password.
  char routerSsid[32];
  char routerPwd[32];

  uint8_t version = 42;
} settings;

struct {
  // True when the quad is going through the gate
  bool volatile crossing = false;

  // Current unsmoothed rssi
  uint16_t volatile rssiRaw = 0;
  // Smoothed rssi value, needs to be a float for smoothing to work
  float volatile rssiSmoothed = 0;
  // int representation of the smoothed rssi value
  uint16_t volatile rssi = 0;

  // Rssi has to be above the enter rssi to count as crossing.
  uint16_t volatile enterRssiTrigger = 0;
  // Rssi has to fall below the leave rssi to count as leaving.
  uint16_t volatile leaveRssiTrigger = 0;

  // The peak raw rssi seen the current pass
  uint16_t volatile rssiPeakRaw = 0;
  // The peak smoothed rssi seen the current pass
  uint16_t volatile rssiPeak = 0;
  // The time of the peak raw rssi for the current pass
  uint64_t volatile rssiPeakRawTimeStamp = 0;

  // variables to track the loop time
  uint32_t volatile loopTime = 0;
  uint64_t volatile lastLoopTimeStamp = 0;

  // The float version of settings.filterRatio.
  float volatile filterRatioFloat = 0.0f;

  // The new vtx freq updated from user.
  uint16_t volatile newVtxFreq = 5732;

  // Whether in calibration mode right now, detecting peak rssi.
  bool volatile calibrationMode = false;

  uint32_t volatile calibrationStartMicros = 0;

  // How many passes has done in calibration mode.
  uint8_t volatile calibrationPasses = 0;

  // Whether a client has connected.
  bool volatile clientConnected = false;
} state;

struct {
  uint16_t volatile rssiPeakRaw;
  uint16_t volatile rssiPeak;
  uint64_t volatile timeStamp;
  uint8_t volatile lap;
} lastPass;

std::string settingsToJson() {
  std::stringstream ss;
  ss << "{" << std::endl;

  ss << "\"vtxFreq\":" << settings.vtxFreq << "," << std::endl;
  ss << "\"rssiPeak\":" << settings.rssiPeak << "," << std::endl;
  ss << "\"enterRssiOffset\":" << settings.enterRssiOffset << "," << std::endl;
  ss << "\"leaveRssiOffset\":" << settings.leaveRssiOffset << "," << std::endl;
  ss << "\"apSsid\":\"" << settings.apSsid << "\"," << std::endl;
  ss << "\"apIp\":\"" << settings.apIp << "\"," << std::endl;
  ss << "\"localIp\":\"" << settings.localIp << "\"," << std::endl;

  ss << "\"routerSsid\":\"" << settings.routerSsid << "\"," << std::endl;
  ss << "\"routerPwd\":\"" << settings.routerPwd << "\"" << std::endl;

  ss << "}";

  return ss.str();
}

// Calculate rx5808 register hex value for given frequency in MHz
uint16_t freqMhzToRegVal(uint16_t freqInMhz) {
  uint16_t tf, N, A;
  tf = (freqInMhz - 479) / 2;
  N = tf / 32;
  A = tf % 32;
  return (N << 7) + A;
}

// Functions for the rx5808 module
void SERIAL_SENDBIT1() {
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(300);
  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(300);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(300);
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(300);
}
void SERIAL_SENDBIT0() {
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(300);
  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(300);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(300);
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(300);
}
void SERIAL_ENABLE_LOW() {
  delayMicroseconds(100);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(100);
}
void SERIAL_ENABLE_HIGH() {
  delayMicroseconds(100);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(100);
}

// Set the frequency given on the rx5808 module
void setRxModule(int frequency) {
  Serial.print("Setup rx5808 frequency to: ");
  Serial.println(frequency);

  uint8_t i; // Used in the for loops

  // Get the hex value to send to the rx module
  uint16_t vtxHex = freqMhzToRegVal(frequency);

  // bit bash out 25 bits of data / Order: A0-3, !R/W, D0-D19 / A0=0, A1=0,
  // A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH();
  delay(2);
  SERIAL_ENABLE_LOW();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();

  for (i = 20; i > 0; i--)
    SERIAL_SENDBIT0(); // Remaining zeros

  SERIAL_ENABLE_HIGH(); // Clock the data in
  delay(2);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table, 20 bytes of register data
  // are sent, but the MSB 4 bits are zeros register address = 0x1, write,
  // data0-15=vtxHex data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT1(); // Register 0x1
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  SERIAL_SENDBIT1(); // Write to register

  // D0-D15, note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--) {
    if (vtxHex & 0x1) { // Is bit high or low?
      SERIAL_SENDBIT1();
    } else {
      SERIAL_SENDBIT0();
    }
    vtxHex >>= 1; // Shift bits along to check the next one
  }

  for (i = 4; i > 0; i--) // Remaining D16-D19
    SERIAL_SENDBIT0();

  SERIAL_ENABLE_HIGH(); // Finished clocking data in
  delay(2);

  digitalWrite(slaveSelectPin, LOW);
  digitalWrite(spiClockPin, LOW);
  digitalWrite(spiDataPin, LOW);

  settings.vtxFreq = frequency;
  Serial.println("rx5808 set.");
}

// Read the RSSI value for the current channel
int rssiRead() { return analogRead(RSSI_PIN); }

void printWifiInfo() {
  Serial.print("IP address for network ");
  Serial.print(settings.routerSsid);
  Serial.print(" : ");
  Serial.println(WiFi.localIP());
  Serial.print("IP address for network ");
  Serial.print(apSsid);
  Serial.print(" : ");
  Serial.print(WiFi.softAPIP());
  Serial.println("");
}
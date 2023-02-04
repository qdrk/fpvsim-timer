#include "fpvsim_timer.h"

AsyncWebServer server(80);
AsyncEventSource events("/events");

void commitEeprom() {
#ifdef DEV_MODE
  Serial.println("Write EEPROM.");
#endif

  EEPROM.put(0, settings);
  EEPROM.commit();
}

volatile bool settingsUpdated = false;
volatile uint64_t lastSettingsUpdateTime = 0;

void updateRssiTrigger() {
  state.enterRssiTrigger =
      settings.rssiPeak * (1.0 - settings.enterRssiOffset / 100.0);
  state.leaveRssiTrigger =
      settings.rssiPeak * (1.0 - settings.leaveRssiOffset / 100.0);
  state.filterRatioFloat = settings.filterRatio / 1000.0f;

#ifdef DEV_MODE
  // NOTE: have to use %d instead of %s here to avoid crashing.
  Serial.printf_P("peak: %d, enter: %d, leave: %d, filter: %d \n", settings.rssiPeak,
                  state.enterRssiTrigger, state.leaveRssiTrigger, settings.filterRatio);
#endif

  settingsUpdated = true;
}

void setApSsid() {
  strcpy(apSsid, "fpvsim- \0");
  apSsid[7] = char('a' + settings.id);
}

void setupServer() {
  setApSsid();

  // Begin Access Point
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid);

  // Begin WiFi
  if (strlen(settings.routerSsid) > 0) {
    WiFi.begin(settings.routerSsid, settings.routerPwd);

    // Connecting to WiFi...
    previousReconnectMillis = millis();
    Serial.print("Connecting to ");
    Serial.print(settings.routerSsid);
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
      counter += 1;
      delay(100);
      Serial.print(".");
      if (counter > 70) {
        Serial.println("Failed to connect to router. Skipping.");
        // This seems to improve network stability when WIFI failed.
        WiFi.disconnect();
        break;
      }
    }
  }

  // Connected to WiFi
  Serial.println();
  Serial.println("Connected!");
  printWifiInfo();

  // Dump network settings.
  strcpy(settings.apSsid, apSsid);
  strcpy(settings.localIp, WiFi.localIP().toString().c_str());
  strcpy(settings.apIp, WiFi.softAPIP().toString().c_str());

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Private-Network", "*");

  // Settings.
  server.on("/api/v1/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    state.clientConnected = true;

    request->send(200, "text/json", settingsToJson().c_str());
  });

  updateRssiTrigger();

  server.on("/api/v1/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("rssiPeak") ||
        !request->hasParam("enterRssiOffset") ||
        !request->hasParam("leaveRssiOffset")) {
      request->send(400, "text/plain", "Invalid params");
      return;
    }

    settings.rssiPeak =
        std::atoi(request->getParam("rssiPeak")->value().c_str());
    settings.enterRssiOffset =
        std::atoi(request->getParam("enterRssiOffset")->value().c_str());
    settings.leaveRssiOffset =
        std::atoi(request->getParam("leaveRssiOffset")->value().c_str());

    // Update filter ratio if in the request, compatible with older client.
    if (request->hasParam("filterRatio")) {
      settings.filterRatio =
          std::atoi(request->getParam("filterRatio")->value().c_str());
    }

    // Update logRssi if in the request, compatible with older client.
    if (request->hasParam("logRssi")) {
      settings.logRssi =
          strcmp(request->getParam("filterRatio")->value().c_str(), "true") == 0;
    }

    updateRssiTrigger();
    commitEeprom();

    Serial.print("Updated settings:");
    Serial.println(settingsToJson().c_str());

    request->send(200, "text/json", settingsToJson().c_str());
  });

  server.on("/api/v1/wifisettings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("routerSsid") ||
        !request->hasParam("routerPwd")) {
      request->send(400, "text/plain", "Invalid params");
      return;
    }

    strcpy(settings.routerSsid, request->getParam("routerSsid")->value().c_str());
    strcpy(settings.routerPwd, request->getParam("routerPwd")->value().c_str());

    commitEeprom();

    Serial.print("Updated settings:");
    Serial.println(settingsToJson().c_str());

    request->send(200, "text/json", settingsToJson().c_str());

    shutdownMillis = millis();
  });


  // Calibration.
  // Somehow, PUT fails with CORS, POST works.
  server.on("/api/v1/start", HTTP_POST, [](AsyncWebServerRequest *request) {
    settings.rssiPeak = 0;
    state.calibrationMode = true;
    state.calibrationPasses = 0;
    state.calibrationStartMicros = micros();

    Serial.println(">>>> Start calibration");

    events.send("started", "calibration", state.calibrationStartMicros);
    request->send(200, "text/json", settingsToJson().c_str());
  });


  // Frequency.
  server.on("/api/v1/setFrequency", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              if (!request->hasParam("frequency")) {
                request->send(400, "text/plain", "No frequency");
                return;
              }

              AsyncWebParameter *p = request->getParam("frequency");
              int frequency = std::atoi(p->value().c_str());

              state.newVtxFreq = frequency;

              request->send(200, "text/json", settingsToJson().c_str());
            });

  // Setup events.
  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it gat is: %u\n",
                    client->lastId());
    }
    // Send event with message "hello!", id current millis
    // and set reconnect delay to 1 second.
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);

  server.begin();

  Serial.println("HTTP server started");
}


void setup() {
  Serial.begin(115200); // Start serial for output/debugging
  Serial.println();
  Serial.println();

  // commit 256 bytes of ESP8266 flash (for "EEPROM" emulation)
  // this step actually loads the content (256 bytes) of flash into
  // a 256-byte-array cache in RAM
  EEPROM.begin(256);

  // read bytes (i.e. sizeof(data) from "EEPROM"),
  // in reality, reads from byte-array cache
  // cast bytes into structure called data
  SettingsType settingsPref;
  EEPROM.get(0, settingsPref);

  // A heuristics to check EEPROM has been set, since this var is never changed.
  if (settingsPref.version == 42
      // Old default.
      || settingsPref.filterRatio == 10) {
    Serial.println("Loading from EEPROM.");
    settings = settingsPref;
    // So we don't accidentally reset the vtx freq.
    state.newVtxFreq = settings.vtxFreq;

    Serial.print("VTX: ");
    Serial.println(settings.vtxFreq);

    Serial.print("Rssi peak: ");
    Serial.println(settings.rssiPeak);
  } else {
    Serial.println("EEPROM not set.");
  }

  // If no existing id, generate one.
  if (settings.id < 0 || settings.id > 25) {
    // Get a number from 0 to 25.
    settings.id = random(26);
    commitEeprom();
  }

  Serial.print("Timer id: ");
  Serial.println(settings.id);

  // RX5808 comms.
  // SPI.begin();
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(spiDataPin, OUTPUT);
  pinMode(spiClockPin, OUTPUT);

  digitalWrite(slaveSelectPin, HIGH);

  while (!Serial) {
  }; // Wait for the Serial port to initialise
  Serial.println("Serial ready...");

  setRxModule(settings.vtxFreq);

  setupServer();
}

uint64_t lastRssiSendTime = 0;
// #if defined(ESP8266)
const uint32_t rssiSendInterval = 2000 * 1000;
// #else
// // 500ms per rssi.
// const uint32_t rssiSendInterval = 3000 * 1000;
// #endif

void loop() {
  // Shutdown after 1s.
  if (shutdownMillis != 0 && millis() - shutdownMillis > 1000) {
    // Restart the server.
    ESP.restart();
  }

  // If WiFi is down, try reconnecting every reconnectInterval.
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousReconnectMillis >= reconnectInterval
        && strlen(settings.routerSsid) > 0) {
#ifdef DEV_MODE
      Serial.println("Reconnecting to WiFi...");
#endif
      isWifiConnected = false;
      WiFi.disconnect();
      WiFi.reconnect();
      previousReconnectMillis = currentMillis;
    }
  } else if (!isWifiConnected) {
    isWifiConnected = true;
    strcpy(settings.localIp, WiFi.localIP().toString().c_str());

#ifdef DEV_MODE
    Serial.println("Wifi is connected.");
    printWifiInfo();
#endif
  }

  // If no client has connected, no need to loop.
  if (!state.clientConnected) {
    return;
  }

  if (state.newVtxFreq != settings.vtxFreq) {
    setRxModule(state.newVtxFreq);

    commitEeprom();
  }

  uint32_t previousLoopTimestamp = state.lastLoopTimeStamp;
  state.lastLoopTimeStamp = micros();
  state.loopTime = state.lastLoopTimeStamp - previousLoopTimestamp;

  // Two settings update has to be larger than 1s interval.
  if (settingsUpdated &&
      state.lastLoopTimeStamp - lastSettingsUpdateTime > 1000 * 1000) {
    events.send(settingsToJson().c_str(), "settings", millis());
    lastSettingsUpdateTime = state.lastLoopTimeStamp;
    settingsUpdated = false;
  }

  state.rssiRaw = rssiRead();

  state.rssiSmoothed = (state.filterRatioFloat * (float)state.rssiRaw) +
                       ((1.0f - state.filterRatioFloat) * state.rssiSmoothed);
  state.rssi = (int)state.rssiSmoothed;

  // Measure peaks, only measure when in calibration mode.
  if (state.calibrationMode && state.rssi > settings.rssiPeak) {
    settings.rssiPeak = state.rssi;
    updateRssiTrigger();
    commitEeprom();
  }
  // Measure end.

  if (state.lastLoopTimeStamp - lastRssiSendTime > rssiSendInterval) {
    String rssiMsg = String(state.rssi) + " " + int64String(state.lastLoopTimeStamp);
    // + " " + String(settings.rssiPeak)
    // + " " + String(settings.enterRssiOffset)
    // + " " + String(settings.leaveRssiOffset);
#ifdef DEV_MODE
    Serial.print("RSSI:");
    Serial.println(rssiMsg);
    Serial.print("Loop time micros: ");
    Serial.println(state.loopTime);
#endif
    events.send(rssiMsg.c_str(), "rssi", state.lastLoopTimeStamp);

    lastRssiSendTime = state.lastLoopTimeStamp;
  }

  if (!state.crossing &&
      state.rssi > state.enterRssiTrigger
      /**
       * Make sure the next crossing only happens after MIN lap time.
       *
       * To avoid the following cases:
       * 0. Last passed
       * 1. Within MIN_LAP_TIME_MILLIS RSSI jump high, crossing again
       * 2. What if that crossing rssi is too high, and no later RSSI passes
       *    that?
       *
       * So what we should check here is the next crossing should not happen
       * until MIN_LAP_TIME_MILLIS, instead of checking the leave.
       *
       * This also makes sure we don't get hang by an overly large crossing
       * RSSI.
       * */
      && (state.lastLoopTimeStamp - lastPass.timeStamp * 1000 >
          MIN_LAP_TIME_MICROS)) {
    state.crossing = true; // Quad is going through the gate
    Serial.println("Crossing = True");
  }

  if (state.crossing) {
    state.rssiPeak = max(state.rssiPeak, state.rssi);

    // Find the peak rssi and the time it occured during a crossing event
    // Use the raw value to account for the delay in smoothing.
    if (state.rssiRaw > state.rssiPeakRaw) {
      state.rssiPeakRaw = state.rssiRaw;
      state.rssiPeakRawTimeStamp = state.lastLoopTimeStamp / 1000;
    }

    uint16_t leaveRssiTrigger = state.leaveRssiTrigger;
    if (state.calibrationMode) {
      // Use lower trigger for leave when in calibration mode.
      leaveRssiTrigger -=
          (CALIBRATION_LEAVE_RSSI_FACTOR - 1) * settings.leaveRssiOffset;
    }

    // See if we have left the gate.
    if (state.rssi < leaveRssiTrigger) {
      uint64_t lastPassTimestamp = lastPass.timeStamp;

      lastPass.rssiPeakRaw = state.rssiPeakRaw;
      lastPass.rssiPeak = state.rssiPeak;
      lastPass.timeStamp = state.rssiPeakRawTimeStamp;
      lastPass.lap = lastPass.lap + 1;

      uint64_t interval = lastPass.timeStamp - lastPassTimestamp;

      // In case some weird overflow happens.
      if (lastPass.timeStamp < lastPassTimestamp) {
        interval = 0;
      }

      String msg = String(lastPass.lap) + " " +
                   int64String(interval) + " " +
                   String(lastPass.rssiPeak) + " " +
                   // The peak timestamp will be the initial timestamp of next
                   // timing round.
                   int64String(lastPass.timeStamp) + " " +
                   int64String(state.lastLoopTimeStamp);

      Serial.printf_P("Crossing = False >>>>>> %s\n", msg.c_str());
      events.send(msg.c_str(), "newtime", state.lastLoopTimeStamp);

      state.crossing = false;
      state.rssiPeakRaw = 0;
      state.rssiPeak = 0;

      if (state.calibrationMode) {
        state.calibrationPasses += 1;

#ifdef DEV_MODE
        Serial.println(">>>> Calibration pass");
#endif

        if (state.calibrationPasses >= CALIBRATION_PASSES &&
            (state.lastLoopTimeStamp - state.calibrationStartMicros >
             CALIBRATION_MIN_TIME_MICROS)) {
          state.calibrationMode = false;
          events.send("ended", "calibration", state.lastLoopTimeStamp);

#ifdef DEV_MODE
          Serial.println(">>>> Calibration done");
#endif
        }
      }
    }
  }

#if defined(ESP8266)
  delay(8);
#else
  delay(1);
#endif
}

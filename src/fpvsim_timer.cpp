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

void initApSsidIfNeeded() {
  // If already inited, just return.
  if (strlen(settings.apSsid) > 0) {
    return;
  }

  strcpy(settings.apSsid, "fpvsim- \0");
  settings.apSsid[7] = char('a' + settings.id); 
  settings.apPwd[0] = 0;

  commitEeprom();
}

void setupServer() {
  initApSsidIfNeeded();

  // Begin Access Point
  WiFi.mode(WIFI_AP_STA);
  if (strlen(settings.apPwd) > 0) {
    WiFi.softAP(settings.apSsid, settings.apPwd);
  } else {
    WiFi.softAP(settings.apSsid);
  }

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
        !request->hasParam("routerPwd") ||
        !request->hasParam("apSsid") ||
        !request->hasParam("apPwd")) {
      request->send(400, "text/plain", "Invalid params");
      return;
    }

    // TODO: handle over-length.
    strcpy(settings.routerSsid, request->getParam("routerSsid")->value().c_str());
    strcpy(settings.routerPwd, request->getParam("routerPwd")->value().c_str());
    strcpy(settings.apSsid, request->getParam("apSsid")->value().c_str());
    strcpy(settings.apPwd, request->getParam("apPwd")->value().c_str());

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

  commitEeprom();
}

void loop() {

}

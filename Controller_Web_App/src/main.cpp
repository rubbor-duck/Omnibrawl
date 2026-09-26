#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// The ESP32 creates this Wi-Fi network. The password must be at least 8 characters.
const char *AP_SSID = "ESP32-Robot";
const char *AP_PASSWORD = "123456789";

AsyncWebServer server(80);

// The web-server callback and loop() can run on different ESP32 CPU cores.
// This lock keeps the X, Y, and timestamp values together as one command.
portMUX_TYPE joystickMutex = portMUX_INITIALIZER_UNLOCKED;

// The most recent joystick command. Each axis ranges from -100 to 100.
volatile int joystickX = 0;
volatile int joystickY = 0;
volatile unsigned long lastJoystickUpdate = 0;

// Stop the robot if the controller stops sending updates unexpectedly.
const unsigned long COMMAND_TIMEOUT_MS = 500;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Omnibrawl Controller</title>
  <style>
    * {
      box-sizing: border-box;
    }

    body {
      min-height: 100vh;
      margin: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 1.25rem;
      overflow: hidden;
      background: #111827;
      color: #f9fafb;
      font-family: Arial, sans-serif;
      text-align: center;
    }

    h1,
    p {
      margin: 0;
    }

    #joystick {
      position: relative;
      width: min(72vw, 360px);
      aspect-ratio: 1;
      border: 4px solid #64748b;
      border-radius: 50%;
      background:
        linear-gradient(#334155 0 0) center / 2px 100% no-repeat,
        linear-gradient(90deg, #334155 0 0) center / 100% 2px no-repeat,
        #1e293b;
      box-shadow: inset 0 0 30px #0f172a;
      cursor: grab;
      touch-action: none;
      user-select: none;
    }

    #joystick.active {
      cursor: grabbing;
      border-color: #38bdf8;
    }

    #knob {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 30%;
      aspect-ratio: 1;
      border-radius: 50%;
      background: #38bdf8;
      box-shadow: 0 6px 18px rgb(0 0 0 / 45%);
      transform: translate(-50%, -50%);
      pointer-events: none;
    }

    #values {
      min-width: 13rem;
      padding: 0.75rem 1rem;
      border-radius: 0.75rem;
      background: #1e293b;
      font: 1.25rem monospace;
    }

    .hint {
      color: #94a3b8;
      font-size: 0.9rem;
    }
  </style>
</head>
<body>
  <h1>Omnibrawl Controller</h1>
  <div id="joystick" aria-label="Robot movement joystick">
    <div id="knob"></div>
  </div>
  <p id="values">X: 0&nbsp;&nbsp;Y: 0</p>
  <p class="hint">Drag with a mouse or one finger. Release to stop.</p>

  <script>
    const joystick = document.getElementById("joystick");
    const knob = document.getElementById("knob");
    const values = document.getElementById("values");

    let activePointerId = null;
    let x = 0;
    let y = 0;
    let lastSentAt = 0;
    const SEND_INTERVAL_MS = 50;
    const HEARTBEAT_INTERVAL_MS = 100;

    function sendCommand(force = false) {
      const now = performance.now();
      if (!force && now - lastSentAt < SEND_INTERVAL_MS) {
        return;
      }

      lastSentAt = now;
      fetch(`/joystick?x=${x}&y=${y}`, {
        cache: "no-store",
        keepalive: true
      }).catch(() => {
        // The ESP32 timeout stops the robot if communication is lost.
      });
    }

    function updateJoystick(clientX, clientY) {
      const rect = joystick.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;
      const maximumDistance = (rect.width - knob.offsetWidth) / 2;

      let offsetX = clientX - centerX;
      let offsetY = clientY - centerY;
      const distance = Math.hypot(offsetX, offsetY);

      if (distance > maximumDistance) {
        const scale = maximumDistance / distance;
        offsetX *= scale;
        offsetY *= scale;
      }

      knob.style.transform =
        `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;

      x = Math.round((offsetX / maximumDistance) * 100);
      // Screen coordinates increase downward, so invert Y to make forward positive.
      y = Math.round((-offsetY / maximumDistance) * 100);
      values.textContent = `X: ${x}  Y: ${y}`;
      sendCommand();
    }

    function stopJoystick() {
      if (activePointerId === null && x === 0 && y === 0) {
        return;
      }

      activePointerId = null;
      x = 0;
      y = 0;
      knob.style.transform = "translate(-50%, -50%)";
      joystick.classList.remove("active");
      values.textContent = "X: 0  Y: 0";
      sendCommand(true);
    }

    joystick.addEventListener("pointerdown", event => {
      if (activePointerId !== null) {
        return;
      }

      activePointerId = event.pointerId;
      joystick.setPointerCapture(event.pointerId);
      joystick.classList.add("active");
      updateJoystick(event.clientX, event.clientY);
    });

    joystick.addEventListener("pointermove", event => {
      if (event.pointerId === activePointerId) {
        updateJoystick(event.clientX, event.clientY);
      }
    });

    joystick.addEventListener("pointerup", event => {
      if (event.pointerId === activePointerId) {
        stopJoystick();
      }
    });

    joystick.addEventListener("pointercancel", stopJoystick);
    joystick.addEventListener("lostpointercapture", stopJoystick);
    joystick.addEventListener("contextmenu", event => event.preventDefault());
    window.addEventListener("blur", stopJoystick);
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) {
        stopJoystick();
      }
    });

    // Continue sending while held still so the ESP32 knows the controller is alive.
    setInterval(() => {
      if (activePointerId !== null) {
        sendCommand(true);
      }
    }, HEARTBEAT_INTERVAL_MS);
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println();
  Serial.println("Wi-Fi access point started.");
  Serial.print("Network name: ");
  Serial.println(AP_SSID);
  Serial.print("Controller address: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.print("Webpage requested by ");
    Serial.println(request->client()->remoteIP());

    AsyncWebServerResponse *response =
        request->beginResponse(200, "text/html", INDEX_HTML);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on("/joystick", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("x") || !request->hasParam("y")) {
      Serial.print("Invalid joystick request from ");
      Serial.println(request->client()->remoteIP());
      request->send(400, "text/plain", "Missing x or y value");
      return;
    }

    const int newX =
        constrain(request->getParam("x")->value().toInt(), -100, 100);
    const int newY =
        constrain(request->getParam("y")->value().toInt(), -100, 100);
    bool commandChanged;

    portENTER_CRITICAL(&joystickMutex);
    commandChanged = newX != joystickX || newY != joystickY;
    joystickX = newX;
    joystickY = newY;
    lastJoystickUpdate = millis();
    portEXIT_CRITICAL(&joystickMutex);

    // Heartbeats repeat the same command, so only print values that change.
    if (commandChanged) {
      Serial.printf("Joystick command received: X: %d, Y: %d\n", newX, newY);
    }

    request->send(204, "text/plain", "");
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    Serial.print("Unknown page requested by ");
    Serial.print(request->client()->remoteIP());
    Serial.print(": ");
    Serial.println(request->url());
    request->send(404, "text/plain", "Not found");
  });

  server.begin();
  lastJoystickUpdate = millis();
}

void loop() {
  static uint8_t previousDeviceCount = 0;
  const uint8_t deviceCount = WiFi.softAPgetStationNum();

  if (deviceCount != previousDeviceCount) {
    if (deviceCount > previousDeviceCount) {
      Serial.printf("Device connected to Wi-Fi. Connected devices: %u\n",
                    deviceCount);
    } else {
      Serial.printf("Device disconnected from Wi-Fi. Connected devices: %u\n",
                    deviceCount);
    }
    previousDeviceCount = deviceCount;
  }

  bool commandTimedOut = false;
  const unsigned long now = millis();

  portENTER_CRITICAL(&joystickMutex);
  if ((joystickX != 0 || joystickY != 0) &&
      now - lastJoystickUpdate > COMMAND_TIMEOUT_MS) {
    joystickX = 0;
    joystickY = 0;
    commandTimedOut = true;
  }
  portEXIT_CRITICAL(&joystickMutex);

  if (commandTimedOut) {
    Serial.println("Controller timed out; command reset to X: 0, Y: 0");
  }

  // Later, joystickX and joystickY will be converted into individual motor speeds here.
}

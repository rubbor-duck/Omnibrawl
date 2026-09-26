#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// The ESP32 creates this Wi-Fi network. The password must be at least 8 characters.
const char *AP_SSID = "ESP32-Robot";
const char *AP_PASSWORD = "123456789";

AsyncWebServer server(80);

// The web-server callback and loop() can run on different ESP32 CPU cores.
// This lock keeps the X, Y, Z, and timestamp values together as one command.
portMUX_TYPE joystickMutex = portMUX_INITIALIZER_UNLOCKED;

// X and Y range from -100 to 100. Z is relative heading from -180 to 180 degrees.
volatile int joystickX = 0;
volatile int joystickY = 0;
volatile int joystickZ = 0;
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

    #joystick.tilt-active {
      cursor: not-allowed;
      border-color: #a78bfa;
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

    #tilt-button {
      padding: 0.75rem 1.25rem;
      border: 0;
      border-radius: 0.75rem;
      background: #38bdf8;
      color: #082f49;
      font-size: 1rem;
      font-weight: bold;
      cursor: pointer;
    }

    #tilt-button.enabled {
      background: #a78bfa;
      color: #2e1065;
    }

    #sensor-status {
      min-height: 1.2rem;
      max-width: min(90vw, 32rem);
      color: #fbbf24;
      font-size: 0.9rem;
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
  <p id="values">X: 0&nbsp;&nbsp;Y: 0&nbsp;&nbsp;Z: 0&deg;</p>
  <button id="tilt-button" type="button">Enable Tilt Control</button>
  <p id="sensor-status" role="status"></p>
  <p class="hint">Drag with a mouse or one finger, or enable phone tilt control.</p>

  <script>
    const joystick = document.getElementById("joystick");
    const knob = document.getElementById("knob");
    const values = document.getElementById("values");
    const tiltButton = document.getElementById("tilt-button");
    const sensorStatus = document.getElementById("sensor-status");

    let activePointerId = null;
    let x = 0;
    let y = 0;
    let z = 0;
    let tiltEnabled = false;
    let tiltCalibration = null;
    let orientationReceived = false;
    let lastSentAt = 0;
    const SEND_INTERVAL_MS = 50;
    const HEARTBEAT_INTERVAL_MS = 100;
    const MAX_TILT_DEGREES = 30;

    function sendCommand(force = false) {
      const now = performance.now();
      if (!force && now - lastSentAt < SEND_INTERVAL_MS) {
        return;
      }

      lastSentAt = now;
      fetch(`/joystick?x=${x}&y=${y}&z=${z}`, {
        cache: "no-store",
        keepalive: true
      }).catch(() => {
        // The ESP32 timeout stops the robot if communication is lost.
      });
    }

    function renderCommand() {
      const maximumDistance = (joystick.clientWidth - knob.offsetWidth) / 2;
      const offsetX = (x / 100) * maximumDistance;
      const offsetY = (-y / 100) * maximumDistance;

      knob.style.transform =
        `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;
      values.textContent = `X: ${x}  Y: ${y}  Z: ${z}\u00b0`;
    }

    function limitToCircle(nextX, nextY) {
      const magnitude = Math.hypot(nextX, nextY);
      if (magnitude > 100) {
        const scale = 100 / magnitude;
        nextX *= scale;
        nextY *= scale;
      }
      return { x: Math.round(nextX), y: Math.round(nextY) };
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

      x = Math.round((offsetX / maximumDistance) * 100);
      // Screen coordinates increase downward, so invert Y to make forward positive.
      y = Math.round((-offsetY / maximumDistance) * 100);
      z = 0;
      renderCommand();
      sendCommand();
    }

    function stopJoystick() {
      if (activePointerId === null && x === 0 && y === 0 && z === 0) {
        return;
      }

      activePointerId = null;
      x = 0;
      y = 0;
      z = 0;
      joystick.classList.remove("active");
      renderCommand();
      sendCommand(true);
    }

    function getScreenAdjustedTilt(beta, gamma) {
      const screenAngle = screen.orientation?.angle ?? window.orientation ?? 0;

      if (screenAngle === 90) {
        return { x: beta, y: -gamma };
      }
      if (screenAngle === 180 || screenAngle === -180) {
        return { x: -gamma, y: -beta };
      }
      if (screenAngle === 270 || screenAngle === -90) {
        return { x: -beta, y: gamma };
      }
      return { x: gamma, y: beta };
    }

    function getHeading(event) {
      // iPhones expose a compass heading through this WebKit property.
      if (Number.isFinite(event.webkitCompassHeading)) {
        return event.webkitCompassHeading;
      }
      if (Number.isFinite(event.alpha)) {
        // Convert the standard counterclockwise alpha angle to clockwise heading.
        return (360 - event.alpha) % 360;
      }
      return null;
    }

    function normalizeHeading(angle) {
      return ((angle + 540) % 360) - 180;
    }

    function handleOrientation(event) {
      if (!tiltEnabled || !Number.isFinite(event.beta) ||
          !Number.isFinite(event.gamma)) {
        return;
      }

      const tilt = getScreenAdjustedTilt(event.beta, event.gamma);
      const heading = getHeading(event);
      orientationReceived = true;

      if (tiltCalibration === null) {
        tiltCalibration = { x: tilt.x, y: tilt.y, heading };
        sensorStatus.textContent =
          "Tilt control active. The current position is the neutral position.";
        x = 0;
        y = 0;
        z = 0;
        renderCommand();
        sendCommand(true);
        return;
      }

      const limited = limitToCircle(
        ((tilt.x - tiltCalibration.x) / MAX_TILT_DEGREES) * 100,
        ((tiltCalibration.y - tilt.y) / MAX_TILT_DEGREES) * 100
      );
      x = limited.x;
      y = limited.y;
      z = heading === null || tiltCalibration.heading === null
        ? 0
        : Math.round(normalizeHeading(heading - tiltCalibration.heading));

      renderCommand();
      sendCommand();
    }

    async function enableTiltControl() {
      if (!("DeviceOrientationEvent" in window)) {
        sensorStatus.textContent =
          "This browser does not provide phone orientation data.";
        return;
      }

      tiltButton.disabled = true;
      sensorStatus.textContent = "Requesting motion sensor permission...";

      try {
        if (typeof DeviceOrientationEvent.requestPermission === "function") {
          const permission = await DeviceOrientationEvent.requestPermission(true);
          if (permission !== "granted") {
            sensorStatus.textContent = "Motion sensor permission was denied.";
            return;
          }
        }

        stopJoystick();
        tiltEnabled = true;
        tiltCalibration = null;
        orientationReceived = false;
        joystick.classList.add("tilt-active");
        tiltButton.classList.add("enabled");
        tiltButton.textContent = "Disable Tilt Control";
        sensorStatus.textContent =
          "Hold the phone in a comfortable neutral position...";
        window.addEventListener("deviceorientation", handleOrientation);

        setTimeout(() => {
          if (tiltEnabled && !orientationReceived) {
            sensorStatus.textContent = window.isSecureContext
              ? "No orientation data received. Check this browser's motion permissions."
              : "No orientation data received. This browser may require an HTTPS page.";
          }
        }, 2500);
      } catch (error) {
        sensorStatus.textContent =
          `Could not start tilt control: ${error.message}`;
      } finally {
        tiltButton.disabled = false;
      }
    }

    function disableTiltControl(message = "Tilt control disabled.") {
      if (!tiltEnabled) {
        return;
      }

      tiltEnabled = false;
      tiltCalibration = null;
      window.removeEventListener("deviceorientation", handleOrientation);
      joystick.classList.remove("tilt-active");
      tiltButton.classList.remove("enabled");
      tiltButton.textContent = "Enable Tilt Control";
      sensorStatus.textContent = message;
      stopJoystick();
    }

    tiltButton.addEventListener("click", () => {
      if (tiltEnabled) {
        disableTiltControl();
      } else {
        enableTiltControl();
      }
    });

    joystick.addEventListener("pointerdown", event => {
      if (activePointerId !== null || tiltEnabled) {
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
    window.addEventListener("blur", () => {
      if (tiltEnabled) {
        disableTiltControl("Tilt control stopped when the page lost focus.");
      } else {
        stopJoystick();
      }
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) {
        if (tiltEnabled) {
          disableTiltControl("Tilt control stopped when the page was hidden.");
        } else {
          stopJoystick();
        }
      }
    });

    window.addEventListener("orientationchange", () => {
      if (tiltEnabled) {
        tiltCalibration = null;
        sensorStatus.textContent =
          "Screen rotated. Hold still while tilt control recalibrates...";
      }
    });

    // Continue sending while held still so the ESP32 knows the controller is alive.
    setInterval(() => {
      if (activePointerId !== null || tiltEnabled) {
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
    const int newZ = request->hasParam("z")
                         ? constrain(request->getParam("z")->value().toInt(),
                                     -180, 180)
                         : 0;
    bool commandChanged;

    portENTER_CRITICAL(&joystickMutex);
    commandChanged =
        newX != joystickX || newY != joystickY || newZ != joystickZ;
    joystickX = newX;
    joystickY = newY;
    joystickZ = newZ;
    lastJoystickUpdate = millis();
    portEXIT_CRITICAL(&joystickMutex);

    // Heartbeats repeat the same command, so only print values that change.
    if (commandChanged) {
      Serial.printf("Control command received: X: %d, Y: %d, Z: %d degrees\n",
                    newX, newY, newZ);
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
  if ((joystickX != 0 || joystickY != 0 || joystickZ != 0) &&
      now - lastJoystickUpdate > COMMAND_TIMEOUT_MS) {
    joystickX = 0;
    joystickY = 0;
    joystickZ = 0;
    commandTimedOut = true;
  }
  portEXIT_CRITICAL(&joystickMutex);

  if (commandTimedOut) {
    Serial.println("Controller timed out; command reset to X: 0, Y: 0, Z: 0");
  }

  // Later, X/Y can drive motor speed and Z can control robot heading here.
}

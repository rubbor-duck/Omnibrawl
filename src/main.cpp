#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

//TODO: Change Compass to work when the tilt controls are disabled
// Change to https so it works with iphone

// Matrixes for the motor translations
BLA::Matrix<3,2> A; // "A" is the cos/sine x and y translation for each wheel
BLA::Matrix<2> v;   // "v" is where the x and y data from the joystick goes into

volatile int motor_0 = 0;
volatile int motor_120 = 0;
volatile int motor_240 = 0;

// The ESP32 creates this Wi-Fi network. The password must be at least 8 characters.
const char *AP_SSID = "ESP32-Robot";
const char *AP_PASSWORD = "123456789";

AsyncWebServer server(80);

// The web-server callback and loop() can run on different ESP32 CPU cores.
// This lock keeps X, Y, Z, heading validity, and timestamp together.
portMUX_TYPE joystickMutex = portMUX_INITIALIZER_UNLOCKED;

// X and Y range from -100 to 100. Z is magnetic heading from 0 to 359 degrees.
volatile int joystickX = 0;
volatile int joystickY = 0;
volatile int joystickZ = 0;
volatile bool headingValid = false;
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

    #compass-panel {
      position: fixed;
      top: 0.75rem;
      left: 0.75rem;
      width: 9rem;
      padding: 0.6rem;
      border-radius: 0.9rem;
      background: rgb(15 23 42 / 90%);
      box-shadow: 0 5px 18px rgb(0 0 0 / 35%);
      z-index: 2;
    }

    #compass {
      position: relative;
      width: 6.5rem;
      aspect-ratio: 1;
      margin: 0 auto 0.4rem;
      border: 3px solid #64748b;
      border-radius: 50%;
      background:
        linear-gradient(#334155 0 0) center / 1px 100% no-repeat,
        linear-gradient(90deg, #334155 0 0) center / 100% 1px no-repeat,
        #1e293b;
      touch-action: none;
      user-select: none;
    }

    #compass.manual {
      border-color: #fbbf24;
      cursor: grab;
    }

    #north-needle {
      position: absolute;
      inset: 0.45rem;
      transform: rotate(0deg);
      pointer-events: none;
    }

    #north-needle::before {
      content: "";
      position: absolute;
      left: calc(50% - 2px);
      top: 0.6rem;
      width: 4px;
      height: calc(50% - 0.6rem);
      border-radius: 3px;
      background: #38bdf8;
    }

    #north-needle::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 0;
      border-right: 0.38rem solid transparent;
      border-bottom: 0.7rem solid #38bdf8;
      border-left: 0.38rem solid transparent;
      transform: translateX(-50%);
    }

    #forward-marker {
      position: absolute;
      left: 50%;
      top: 1.4rem;
      color: white;
      font-size: 1.1rem;
      transform: translateX(-50%);
      pointer-events: none;
    }

    #compass-center {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 0.55rem;
      aspect-ratio: 1;
      border: 2px solid #f8fafc;
      border-radius: 50%;
      background: #0f172a;
      transform: translate(-50%, -50%);
      pointer-events: none;
    }

    #heading-readout,
    #heading-source {
      margin-top: 0.25rem;
      font-size: 0.75rem;
    }

    #heading-source {
      color: #94a3b8;
    }

    #calibrate-north {
      width: 100%;
      margin-top: 0.45rem;
      padding: 0.45rem;
      border: 0;
      border-radius: 0.5rem;
      background: #fbbf24;
      color: #422006;
      font-weight: bold;
      cursor: pointer;
    }

    #calibration-hint {
      margin-top: 0.35rem;
      color: #fbbf24;
      font-size: 0.68rem;
      line-height: 1.2;
    }

    [hidden] {
      display: none !important;
    }

    @media (max-width: 600px) {
      #compass-panel {
        width: 7.4rem;
      }

      #compass {
        width: 5.25rem;
      }
    }

    .hint {
      color: #94a3b8;
      font-size: 0.9rem;
    }
  </style>
</head>
<body>
  <aside id="compass-panel" aria-label="Magnetic compass">
    <div id="compass">
      <div id="north-needle"></div>
      <div id="forward-marker" title="Phone forward">&#9650;</div>
      <div id="compass-center"></div>
    </div>
    <p id="heading-readout">Heading: --</p>
    <p id="heading-source">Compass inactive</p>
    <button id="calibrate-north" type="button" hidden>Calibrate North</button>
    <p id="calibration-hint" hidden>Drag the blue needle toward magnetic north, then calibrate.</p>
  </aside>
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
    const compass = document.getElementById("compass");
    const northNeedle = document.getElementById("north-needle");
    const headingReadout = document.getElementById("heading-readout");
    const headingSource = document.getElementById("heading-source");
    const calibrateNorthButton = document.getElementById("calibrate-north");
    const calibrationHint = document.getElementById("calibration-hint");

    let activePointerId = null;
    let x = 0;
    let y = 0;
    let z = 0;
    let headingIsValid = false;
    let tiltEnabled = false;
    let tiltCalibration = null;
    let orientationReceived = false;
    let absoluteHeadingReceived = false;
    let latestRelativeHeading = null;
    let manualHeadingOffset = null;
    let manualNorthNeedleAngle = 0;
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
      fetch(`/joystick?x=${x}&y=${y}&z=${z}&headingValid=${headingIsValid ? 1 : 0}`, {
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
      const displayedHeading = headingIsValid ? `${z}\u00b0` : "--";
      values.textContent = `X: ${x}  Y: ${y}  Z: ${displayedHeading}`;
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
      if (activePointerId === null && x === 0 && y === 0 && z === 0 &&
          !headingIsValid) {
        return;
      }

      activePointerId = null;
      x = 0;
      y = 0;
      z = 0;
      headingIsValid = false;
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

    function normalize360(angle) {
      return ((angle % 360) + 360) % 360;
    }

    function getHeadingReading(event) {
      // Alpha is counterclockwise, while compass headings increase clockwise.
      // Keeping beta/gamma out of this calculation prevents movement tilt from
      // changing the direction reported by the phone's magnetometer.
      const relativeHeading = Number.isFinite(event.alpha)
        ? normalize360(360 - event.alpha)
        : null;

      // Safari provides a clockwise magnetic heading directly.
      if (Number.isFinite(event.webkitCompassHeading) &&
          event.webkitCompassHeading >= 0) {
        return {
          absolute: normalize360(event.webkitCompassHeading),
          relative: relativeHeading,
          source: "Automatic magnetic heading"
        };
      }

      // Other browsers must explicitly identify their Earth-referenced data.
      if ((event.type === "deviceorientationabsolute" || event.absolute === true) &&
          relativeHeading !== null) {
        return {
          absolute: relativeHeading,
          relative: relativeHeading,
          source: "Automatic absolute heading"
        };
      }

      return { absolute: null, relative: relativeHeading, source: null };
    }

    function hideManualCalibration() {
      compass.classList.remove("manual");
      calibrateNorthButton.hidden = true;
      calibrationHint.hidden = true;
    }

    function showManualCalibration() {
      if (!tiltEnabled || absoluteHeadingReceived) {
        return;
      }

      compass.classList.add("manual");
      calibrateNorthButton.hidden = false;
      calibrationHint.hidden = false;
      headingSource.textContent = "Manual north calibration needed";
      northNeedle.style.transform =
        `rotate(${manualNorthNeedleAngle}deg)`;
    }

    function displayHeading(heading, source) {
      z = Math.round(normalize360(heading)) % 360;
      headingIsValid = true;
      const northAngle = normalize360(360 - z);
      northNeedle.style.transform = `rotate(${northAngle}deg)`;
      headingReadout.textContent = `Heading: ${z}\u00b0`;
      headingSource.textContent = source;
    }

    function clearHeadingDisplay() {
      z = 0;
      headingIsValid = false;
      northNeedle.style.transform = "rotate(0deg)";
      headingReadout.textContent = "Heading: --";
      headingSource.textContent = "Compass inactive";
      hideManualCalibration();
    }

    function handleOrientation(event) {
      if (!tiltEnabled || !Number.isFinite(event.beta) ||
          !Number.isFinite(event.gamma)) {
        return;
      }

      const tilt = getScreenAdjustedTilt(event.beta, event.gamma);
      const heading = getHeadingReading(event);
      orientationReceived = true;
      latestRelativeHeading = heading.relative;

      if (heading.absolute !== null) {
        absoluteHeadingReceived = true;
        manualHeadingOffset = null;
        hideManualCalibration();
        displayHeading(heading.absolute, heading.source);
      } else if (manualHeadingOffset !== null && heading.relative !== null) {
        displayHeading(
          heading.relative + manualHeadingOffset,
          "Manually calibrated magnetic heading"
        );
      }

      if (tiltCalibration === null) {
        tiltCalibration = { x: tilt.x, y: tilt.y };
        sensorStatus.textContent =
          "Tilt control active. The current position is the neutral position.";
        x = 0;
        y = 0;
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
        absoluteHeadingReceived = false;
        latestRelativeHeading = null;
        manualHeadingOffset = null;
        manualNorthNeedleAngle = 0;
        clearHeadingDisplay();
        joystick.classList.add("tilt-active");
        tiltButton.classList.add("enabled");
        tiltButton.textContent = "Disable Tilt Control";
        sensorStatus.textContent =
          "Hold the phone in a comfortable neutral position...";
        window.addEventListener("deviceorientation", handleOrientation);
        window.addEventListener("deviceorientationabsolute", handleOrientation);

        setTimeout(() => {
          if (tiltEnabled && !orientationReceived) {
            sensorStatus.textContent = window.isSecureContext
              ? "No orientation data received. Check this browser's motion permissions."
              : "No orientation data received. This browser may require an HTTPS page.";
          } else if (tiltEnabled && !absoluteHeadingReceived &&
                     manualHeadingOffset === null) {
            showManualCalibration();
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
      window.removeEventListener("deviceorientationabsolute", handleOrientation);
      joystick.classList.remove("tilt-active");
      tiltButton.classList.remove("enabled");
      tiltButton.textContent = "Enable Tilt Control";
      sensorStatus.textContent = message;
      stopJoystick();
      clearHeadingDisplay();
    }

    function updateManualNorthNeedle(clientX, clientY) {
      const rect = compass.getBoundingClientRect();
      const offsetX = clientX - (rect.left + rect.width / 2);
      const offsetY = clientY - (rect.top + rect.height / 2);
      manualNorthNeedleAngle = Math.round(
        normalize360(Math.atan2(offsetX, -offsetY) * 180 / Math.PI)
      );
      northNeedle.style.transform =
        `rotate(${manualNorthNeedleAngle}deg)`;
    }

    compass.addEventListener("pointerdown", event => {
      if (!compass.classList.contains("manual")) {
        return;
      }
      compass.setPointerCapture(event.pointerId);
      updateManualNorthNeedle(event.clientX, event.clientY);
    });

    compass.addEventListener("pointermove", event => {
      if (compass.classList.contains("manual") &&
          compass.hasPointerCapture(event.pointerId)) {
        updateManualNorthNeedle(event.clientX, event.clientY);
      }
    });

    calibrateNorthButton.addEventListener("click", () => {
      if (latestRelativeHeading === null) {
        sensorStatus.textContent =
          "No relative heading is available for manual calibration.";
        return;
      }

      const selectedPhoneHeading = normalize360(360 - manualNorthNeedleAngle);
      manualHeadingOffset = normalize360(
        selectedPhoneHeading - latestRelativeHeading
      );
      hideManualCalibration();
      displayHeading(
        latestRelativeHeading + manualHeadingOffset,
        "Manually calibrated magnetic heading"
      );
      sensorStatus.textContent =
        "Manual north calibration saved for this control session.";
      renderCommand();
      sendCommand(true);
    });

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

// function skeleton ready for matrix multiplication implementation
void motor_matrix_translation(int x, int y, int z){
v = {x, y};

BLA::Matrix<3, 1> D = A * v;


}



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
    const bool newHeadingValid = request->hasParam("headingValid") &&
                                 request->getParam("headingValid")->value() == "1";
    const int newZ = newHeadingValid && request->hasParam("z")
                         ? constrain(request->getParam("z")->value().toInt(),
                                     0, 359)
                         : 0;
    bool commandChanged;

    portENTER_CRITICAL(&joystickMutex);
    commandChanged =
        newX != joystickX || newY != joystickY || newZ != joystickZ ||
        newHeadingValid != headingValid;
    joystickX = newX;
    joystickY = newY;
    joystickZ = newZ;
    headingValid = newHeadingValid;
    lastJoystickUpdate = millis();
    portEXIT_CRITICAL(&joystickMutex);

    // Heartbeats repeat the same command, so only print values that change.
    if (commandChanged) {
      if (newHeadingValid) {
        Serial.printf("Control command received: X: %d, Y: %d, heading: %d degrees\n",
                      newX, newY, newZ);
      } else {
        Serial.printf("Control command received: X: %d, Y: %d, heading unavailable\n",
                      newX, newY);
      }
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

  // Matrixs filled up with setup data
  // {Cos(1), Sin(0), Cos(120), Sin(120), Cos(240), Sin(240)}
  A = {1, 0, -0.5, 0.866025403784, -0.5, -0.866025403784};
  v.Fill(0);
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
  if ((joystickX != 0 || joystickY != 0 || headingValid) &&
      now - lastJoystickUpdate > COMMAND_TIMEOUT_MS) {
    joystickX = 0;
    joystickY = 0;
    joystickZ = 0;
    headingValid = false;
    commandTimedOut = true;
  }
  portEXIT_CRITICAL(&joystickMutex);

  if (commandTimedOut) {
    Serial.println("Controller timed out; movement stopped and heading invalidated");
  }

  // Later, X/Y can drive motor speed and Z can control robot heading here.
}

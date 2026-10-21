#include "web_server.hpp"

#include <Arduino.h>
#include <esp_https_server.h>
#include <cstdlib>
#include <cstring>
#include <WiFi.h>
#include <LittleFS.h>

namespace {
constexpr char AP_SSID[] = "ESP32-Robot";
constexpr char AP_PASSWORD[] = "123456789";
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

httpd_handle_t server = nullptr;
portMUX_TYPE joystickMutex = portMUX_INITIALIZER_UNLOCKED;
ControlCommand currentCommand;
unsigned long lastJoystickUpdate = 0;
uint8_t previousDeviceCount = 0;

// Original webpage (including JS, compass and tilt controls), unchanged.
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

  esp_err_t handleRoot(httpd_req_t *request) {
      httpd_resp_set_type(request, "text/html");
      httpd_resp_set_hdr(request, "Cache-Control", "no-store");
      return httpd_resp_send(request, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
  }

  esp_err_t handleJoystick(httpd_req_t *request) {
      char query[128];
      const size_t queryLength = httpd_req_get_url_query_len(request);

      if (queryLength == 0 || queryLength >= sizeof(query) ||
          httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
          return httpd_resp_send_err(
              request, HTTPD_400_BAD_REQUEST, "Invalid query string");
      }

      char xText[8] = {};
      char yText[8] = {};

      if (httpd_query_key_value(query, "x", xText, sizeof(xText)) != ESP_OK ||
          httpd_query_key_value(query, "y", yText, sizeof(yText)) != ESP_OK) {
          return httpd_resp_send_err(
              request, HTTPD_400_BAD_REQUEST, "Missing x or y value");
      }

      const int newX = constrain(atoi(xText), -100, 100);
      const int newY = constrain(atoi(yText), -100, 100);

      char headingValidText[2] = {};
      const bool newHeadingValid =
          httpd_query_key_value(
              query, "headingValid", headingValidText,
              sizeof(headingValidText)) == ESP_OK &&
          strcmp(headingValidText, "1") == 0;

      char zText[8] = {};
      int newZ = 0;
      if (newHeadingValid &&
          httpd_query_key_value(query, "z", zText, sizeof(zText)) == ESP_OK) {
          newZ = constrain(atoi(zText), 0, 359);
      }

      bool commandChanged;
      portENTER_CRITICAL(&joystickMutex);
      commandChanged = newX != currentCommand.x ||
                      newY != currentCommand.y ||
                      newZ != currentCommand.heading ||
                      newHeadingValid != currentCommand.headingValid;
      currentCommand.x = newX;
      currentCommand.y = newY;
      currentCommand.heading = newZ;
      currentCommand.headingValid = newHeadingValid;
      lastJoystickUpdate = millis();
      portEXIT_CRITICAL(&joystickMutex);

      if (commandChanged) {
          if (newHeadingValid) {
              Serial.printf(
                  "Control command received: X: %d, Y: %d, heading: %d degrees\n",
                  newX, newY, newZ);
          } else {
              Serial.printf(
                  "Control command received: X: %d, Y: %d, heading unavailable\n",
                  newX, newY);
          }
      }

      httpd_resp_set_status(request, "204 No Content");
      return httpd_resp_send(request, nullptr, 0);
  }
} // namespace

namespace WebServer {

void begin() {
    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAPConfig(localIP, gateway, subnet)) {
        Serial.println("Failed to configure the Wi-Fi access point IP.");
        return;
    }

    if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.println("Failed to start the Wi-Fi access point.");
        return;
    }

    Serial.println();
    Serial.println("Wi-Fi access point started.");
    Serial.print("Network name: ");
    Serial.println(AP_SSID);
    Serial.print("Controller address: https://");
    Serial.println(WiFi.softAPIP());

    if (!LittleFS.begin(false)) {
        Serial.println("LittleFS mount failed.");
        return;
    }

    File certFile = LittleFS.open("/certs/server.crt", "r");
    File keyFile = LittleFS.open("/certs/server.key", "r");

    if (!certFile || !keyFile) {
        Serial.println("Could not open the HTTPS certificate or private key.");
        return;
    }

    String serverCert = certFile.readString();
    String serverKey = keyFile.readString();
    certFile.close();
    keyFile.close();

    if (serverCert.length() == 0 || serverKey.length() == 0) {
        Serial.println("The HTTPS certificate or private key is empty.");
        return;
    }

    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();

    // This SDK version names the server-certificate field cacert_pem.
    config.cacert_pem =
        reinterpret_cast<const uint8_t *>(serverCert.c_str());
    config.cacert_len = serverCert.length() + 1;
    config.prvtkey_pem =
        reinterpret_cast<const uint8_t *>(serverKey.c_str());
    config.prvtkey_len = serverKey.length() + 1;

    // Two TLS connections allow page loading and controller requests.
    config.httpd.max_open_sockets = 2;

    esp_err_t result = httpd_ssl_start(&server, &config);
    if (result != ESP_OK) {
        Serial.printf("HTTPS server failed to start: %s\n",
                      esp_err_to_name(result));
        server = nullptr;
        return;
    }

    httpd_uri_t rootUri{};
    rootUri.uri = "/";
    rootUri.method = HTTP_GET;
    rootUri.handler = handleRoot;
    rootUri.user_ctx = nullptr;

    httpd_uri_t joystickUri{};
    joystickUri.uri = "/joystick";
    joystickUri.method = HTTP_GET;
    joystickUri.handler = handleJoystick;
    joystickUri.user_ctx = nullptr;

    if (httpd_register_uri_handler(server, &rootUri) != ESP_OK ||
        httpd_register_uri_handler(server, &joystickUri) != ESP_OK) {
        Serial.println("Failed to register an HTTPS route.");
        httpd_ssl_stop(server);
        server = nullptr;
        return;
    }

    portENTER_CRITICAL(&joystickMutex);
    lastJoystickUpdate = millis();
    portEXIT_CRITICAL(&joystickMutex);
}

void update() {
    const uint8_t deviceCount = WiFi.softAPgetStationNum();
    if (deviceCount != previousDeviceCount) {
        if (deviceCount > previousDeviceCount) {
            Serial.printf("Device connected to Wi-Fi. Connected devices: %u\n", deviceCount);
        } else {
            Serial.printf("Device disconnected from Wi-Fi. Connected devices: %u\n", deviceCount);
        }
        previousDeviceCount = deviceCount;
    }

    bool commandTimedOut = false;
    const unsigned long now = millis();
    portENTER_CRITICAL(&joystickMutex);
    if ((currentCommand.x != 0 || currentCommand.y != 0 || currentCommand.headingValid) &&
        now - lastJoystickUpdate > COMMAND_TIMEOUT_MS) {
        currentCommand = ControlCommand{};
        commandTimedOut = true;
    }
    portEXIT_CRITICAL(&joystickMutex);

    if (commandTimedOut) {
        Serial.println("Controller timed out; movement stopped and heading invalidated");
    }
}

ControlCommand getCommand() {
    ControlCommand snapshot;
    portENTER_CRITICAL(&joystickMutex);
    snapshot = currentCommand;
    portEXIT_CRITICAL(&joystickMutex);
    return snapshot;
}

} // namespace WebServer

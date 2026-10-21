#pragma once

// Public interface for the Wi-Fi controller. No HTTP types escape this module.
struct ControlCommand {
    int x = 0;
    int y = 0;
    int heading = 0;
    bool headingValid = false;
};

namespace WebServer {
    void begin();                // Start the Wi-Fi AP and HTTP server.
    void update();               // Check connections and command timeout.
    ControlCommand getCommand(); // Atomically read the latest controller command.
}

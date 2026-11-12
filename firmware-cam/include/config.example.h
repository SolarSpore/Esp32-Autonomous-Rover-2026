#pragma once

// Copy this file to config.h and replace every CHANGE_ME.
// config.h holds secrets: never commit it.

#include <Arduino.h>
#include <IPAddress.h>

// Your Wi-Fi network. The board tries it first.
const char* const sta_ssid     = "CHANGE_ME";
const char* const sta_password = "CHANGE_ME";

// The board's own fallback access point, started if your Wi-Fi is not
// reachable. Use a different name from the rover's access point.
// The password needs at least 8 characters.
const char* const ap_ssid      = "CHANGE_ME";
const char* const ap_password  = "CHANGE_ME";

// The board's name on the network, and its OTA host name.
const char* const mdnsHostname = "CHANGE_ME";

// Password for over-the-air updates. Must match the OTA_PASSWORD
// environment variable you upload with.
const char* const ota_password = "CHANGE_ME";

// Optional fixed address. Leave this false and reserve the address in
// your router instead (the same way as for the rover), or set it to true
// and check the three values below match your network.
const bool      use_static_ip  = false;
const IPAddress static_ip(192, 168, 1, 27);
const IPAddress static_gateway(192, 168, 1, 1);
const IPAddress static_subnet(255, 255, 255, 0);
// Copy this file, rename the copy to secrets.h (in this same src/ folder), and
// fill in your own values. src/secrets.h is listed in .gitignore so your real
// credentials are never committed.

// Optional: a home Wi-Fi network for the device to also join. Leave the name
// empty ("") to use only the device's own access point.
#define SECRET_WIFI_SSID "WIFI_Network_Name"
#define SECRET_WIFI_PASSWORD "WIFI_PSK"

// This device's own access point (what you connect to day to day).
// The password is only used when AP_REQUIRE_PASSWORD is 1 in config.h.
// WPA2 requires 8+ characters.
#define SECRET_AP_SSID "TeslaBMS"
#define SECRET_AP_PASSWORD "changeme123"

// Only used when WEBUI_REQUIRE_AUTH is 1 in config.h.
#define SECRET_WEBUI_USER "admin"
#define SECRET_WEBUI_PASS "changeme123"

// Copy this file to src/secrets.h and fill in your own values.
// src/secrets.h is listed in .gitignore so your real credentials are never committed.

#define SECRET_MQTT_SERVER_IP "192.168.1.2"
#define SECRET_MQTT_USER "mqtt_user"
#define SECRET_MQTT_PASSWORD "mqtt_password"

#define SECRET_WIFI_SSID "WIFI_Network_Name"
#define SECRET_WIFI_PASSWORD "WIFI_PSK"

// This device's own access point (what you connect to day to day).
// The password is only used when AP_REQUIRE_PASSWORD is 1 in config.h.
// WPA2 requires 8+ characters.
#define SECRET_AP_SSID "TeslaBMS"
#define SECRET_AP_PASSWORD "changeme123"

#define SECRET_FTP_SERVER_IP "192.168.1.3"
#define SECRET_FTP_USER "ftpuser"
#define SECRET_FTP_PASSWORD "ftpuser_password"

// Only used when WEBUI_REQUIRE_AUTH is 1 in config.h.
#define SECRET_WEBUI_USER "admin"
#define SECRET_WEBUI_PASS "changeme123"

#define SECRET_MQTT_TOPIC "homeassistant/sensor/bms/"
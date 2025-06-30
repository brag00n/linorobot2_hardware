#ifndef DEVICEWIFI_H
#define DEVICEWIFI_H

#include "Device.hpp"
#include <FS.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "MySyslog.h"

class DeviceWifi: public Device {
public:
    struct DeviceWifi_t: Device::Device_t {
        String mode;
        String ssid;
        String ipAddress;
        String macAddress;
        int rssi;
    }wifiData;

    void init(const char* name) override;
    DeviceWifi_t* getData() override { return &wifiData; };
    void setSta(const char* ssid, const char* password){sta_ssid = ssid; sta_password = password;};
    void setAp(const char* ssid, const char* password){ap_ssid = ssid; ap_password = password;};
    bool isReady() override { return isConnected; };
    void read() override;
private:
    void onConnected();
    void onDisconnected();
    void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
    bool wifiModeOnBoot();
    bool wifiModeSTA(const char* input_APssid, const char* input_APpassword,const char* input_STAssid, const char* input_STApassword);
    bool wifiModeAP(const char* input_ssid, const char* input_password);
    bool wifiModeAPSTA(const char* input_APssid, const char* input_APpassword,const char* input_STAssid, const char* input_STApassword);
    long connectionStartTime;
    long connectionTimeout = 15000;
    bool wifiConfigFound = false;
    byte WIFI_CURRENT_MODE = -1;
    byte WIFI_MODE_ON_BOOT = 2; // 0: OFF, 1: AP, 2: STA, 3: AP+STA (default mode after first wifi connection succeed)
    const char *ap_ssid = "liniorobot";
    const char *ap_password = "#012zAyb@y";
    const char *sta_ssid = "<SSID>";
    const char *sta_password = "<PASSWORD>";
    bool defaultModeToAPSTA = true;
    bool isConnected = false;
};
#endif // DEVICEWIFI_H

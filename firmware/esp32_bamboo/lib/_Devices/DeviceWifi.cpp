
#include "DeviceWifi.hpp"

void DeviceWifi::init(const char* name) {
	Device::init(name);
	if (isSyslog) syslog(LOG_DEBUG, "   DeviceWifi::init\n");
	WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) {
		onWiFiEvent(event, info);
	});
	isConnected = wifiModeOnBoot();
}

void DeviceWifi::read()  {
	Device::read();
	if (isReady()){
		if (isSyslog) syslog(LOG_DEBUG, "   DeviceWifi::read\n");
		getData()->mode = WiFi.getMode() == WIFI_MODE_AP ? "AP" : "STA";
		getData()->ipAddress = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
		getData()->macAddress = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPmacAddress() : WiFi.macAddress();
		getData()->rssi = WiFi.RSSI();
		getData()->ssid = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPSSID() : WiFi.SSID();
	} else {
		if (isSyslog) syslog(LOG_DEBUG, "   DeviceWifi::read: not connected\n");
		Device::setError(1, "Error on read: DeviceWifi is not connected");
	}
}

void DeviceWifi::onConnected() {
	isConnected = true;
	read();
	if (isSyslog) syslog(LOG_DEBUG, ("   Connected to Wifi on SSID:" + getData()->ssid +", IP Address: " + getData()->ipAddress + "\n").c_str());
	read();
}

void DeviceWifi::onDisconnected() {
	isConnected = false;
	if (isSyslog) syslog(LOG_INFO, "   Disconnected from WiFi\n");
}

void DeviceWifi::onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
	switch (event) {
		case SYSTEM_EVENT_STA_GOT_IP:
			onConnected();
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			onDisconnected();
			break;
		case SYSTEM_EVENT_AP_START:
			onConnected();
			break;
		case SYSTEM_EVENT_AP_STOP:
			onDisconnected();
			break;
		default:
			if (isSyslog) syslog(LOG_DEBUG, ("   Wifi event not managed: " + String(event) + "\n").c_str());
			break;
	}
}

bool DeviceWifi::wifiModeOnBoot() {
	bool funcStatus = false;
	switch(WIFI_MODE_ON_BOOT) {
	case 0: 
		funcStatus = true;
		WIFI_CURRENT_MODE = 0;
		WiFi.mode(WIFI_OFF);
		getData()->mode = "OFF";
		break;
	case 1:
		funcStatus = wifiModeAP(ap_ssid, ap_password);
		break;
	case 2:
		funcStatus = wifiModeSTA(ap_ssid, ap_password, sta_ssid, sta_password);
		break;
	case 3:
		funcStatus = wifiModeAPSTA(ap_ssid, ap_password, sta_ssid, sta_password);
		break;
	}
	return funcStatus;
}

// set wifi as AP mode.
bool DeviceWifi::wifiModeAP(const char* input_ssid, const char* input_password) {
	WiFi.disconnect();
    if (isSyslog) syslog(LOG_DEBUG, "   wifi mode on boot: AP\n");
	// WiFi.mode(WIFI_AP);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(input_ssid, input_password);
    isConnected=WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    
    if (isSyslog) syslog(LOG_DEBUG, "   AP mode starts...\n");
	if (isSyslog) syslog(LOG_DEBUG, ("      SSID: " + String(input_ssid) + ", Password: " + String(input_password) + "\n").c_str());
	if (isSyslog) syslog(LOG_DEBUG, ("      AP Address: " + WiFi.softAPIP().toString() + "\n").c_str());
	read();
	return true;
}

// set wifi as STA mode.
bool DeviceWifi::wifiModeSTA(const char* input_APssid, const char* input_APpassword,const char* input_STAssid, const char* input_STApassword) {

	WiFi.disconnect();

	WiFi.mode(WIFI_STA);
	WiFi.begin(input_STAssid, input_STApassword);
	connectionStartTime = millis();

	if (isSyslog) syslog(LOG_DEBUG, ("   STA mode starts: connecting to " + String(input_STAssid)+", password: " + String(input_STApassword) + "...\n").c_str());

	while (WiFi.status() != WL_CONNECTED) {
		unsigned long currentTime = millis();
		delay(500);

		if (currentTime - connectionStartTime >= connectionTimeout) {
			WIFI_CURRENT_MODE = -1;
            if (isSyslog) syslog(LOG_INFO, "   STA connection timeout. Activating AP...\n");
			return wifiModeAP(input_APssid, input_APpassword);
			break;
		}
	}

	if (isSyslog) syslog(LOG_DEBUG, "   STA connection succeed.\n");
	read();
	return true;
}

// set wifi as STA mode.
bool DeviceWifi::wifiModeAPSTA(const char* input_APssid, const char* input_APpassword,const char* input_STAssid, const char* input_STApassword) {

	WiFi.disconnect();

	WiFi.mode(WIFI_AP_STA);
	if (isSyslog) syslog(LOG_DEBUG, ("   STA mode starts: connecting to " + String(input_STAssid)+", password: " + String(input_STApassword) + "...\n").c_str());
	WiFi.softAP(input_APssid, input_APpassword);

	if (isSyslog) syslog(LOG_DEBUG, ("   AP mode starts: connecting to " + String(input_APssid)+", password: " + String(input_APpassword) + "...\n").c_str());
	WiFi.begin(input_STAssid, input_STApassword);
	connectionStartTime = millis();

	while (WiFi.status() != WL_CONNECTED) {
		unsigned long currentTime = millis();
		delay(500);

		if (currentTime - connectionStartTime >= connectionTimeout) {
			WIFI_CURRENT_MODE = -1;
            if (isSyslog) syslog(LOG_INFO, "   STA connection timeout. Activating AP only...\n");
			return wifiModeAP(input_APssid, input_APpassword);
			break;
		}
	}

	if (isSyslog) syslog(LOG_DEBUG, "   STA connection succeed.\n");
	read();
	return true;
}

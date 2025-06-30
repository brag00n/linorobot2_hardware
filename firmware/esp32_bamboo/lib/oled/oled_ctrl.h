// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
#include <Adafruit_SSD1306.h>
#include "MySyslog.h"
// #include "Device.hpp"

// class DeviceOled: public Device {
// public:
//     struct Device_t{
//         String line0;
//         String line1;
//         String line2;
//         String line3;
//     } dataOled;

//   Device* init() override {
//     return this;
//   }

//   Device* read() override {
//     // Nothing to read
//     return this;
//   };

//   Device* write() override {
//     return this;
//   };

//   Device* setData(Device::Device_t *pData) override {
//     data.id = pData->id;
//     data.name = pData->name;
//     // screenLine_0 = pData.line0;
//     // screenLine_1 = pData.line1;
//     // screenLine_2 = pData.line2;
//     // screenLine_3 = pData.line3;
//     return this;
//   };

//   Device* setData(const char *line0, const char *line1, const char *line2, const char *line3) {
//     return this;
//   };

//   Device::Device_t* getData() {
//     data.data = &dataOled;
//     return &data; 
//   };

//   bool isReady() override {
//     // Always ready for OLED
//     return true;
//   };

//   bool isError() {
//     // OLED does not have an error state
//     return false;
//   };

//   DeviceError_t getError() {
//     DeviceError_t error;
//     error.error_code = 0; // No specific error code for OLED
//     error.error_message = "No Error";
//     return error;
//   };
//   void setError(DeviceError_t pError) {
//     // OLED does not have an error state, so this function does nothing
//   };

// };

// // void test() {
// //   DeviceOled oled;
// //   oled.init ();
// //   DeviceOled::Device_t tempData;
// //   oled.getData()->id = 1;
// //   oled.getData()->name = "OLED Test";
// //   (oled.getData()->data)->line1 = "Hello, World!";
// //   oled.write();
// // }
bool screenDefaultMode = true;

unsigned long currentTimeMillis = millis();
unsigned long lastTimeMillis = millis();

// default
String screenLine_0;
String screenLine_1;
String screenLine_2;
String screenLine_3;

// custom
String customLine_0;
String customLine_1;
String customLine_2;
String customLine_3;

// #include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// init oled ctrl functions.
void init_oled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if (isSyslog) syslog(LOG_DEBUG,"   SSD1306 allocation failed\n");
  }else{
    if (isSyslog) syslog(LOG_DEBUG,"   SSD1306 allocation success\n");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
}


// Updata all data and flash the screen.
void oled_update() {
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(screenLine_0);
  display.println(screenLine_1);
  display.println(screenLine_2);
  display.println(screenLine_3);

  display.display();
}

// dev info update on oled.
// void oledInfoUpdate() {
//   currentTimeMillis = millis();
//   if (currentTimeMillis - lastTimeMillis > 10000) {
//     inaDataUpdate();
//     lastTimeMillis = currentTimeMillis;
//   } else {
//     return;
//   }
//   if (!screenDefaultMode) {
//     return;
//   }
//   // inaDataUpdate();
//   screenLine_3 = "V:"+String(loadVoltage_V);
//   oled_update();
  
// }

// oled ctrl.
void oledCtrl(byte inputLineNum, String inputMegs) {
  screenDefaultMode = false;
  switch (inputLineNum) {
  case 0: customLine_0 = inputMegs;break;
  case 1: customLine_1 = inputMegs;break;
  case 2: customLine_2 = inputMegs;break;
  case 3: customLine_3 = inputMegs;break;
  }
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(customLine_0);
  display.println(customLine_1);
  display.println(customLine_2);
  display.println(customLine_3);

  display.display();
}

// set oled as default.
void setOledDefault(){
  screenDefaultMode = true;
  //inaDataUpdate();
  //screenLine_3 = "V:"+String(loadVoltage_V);
  oled_update();
  lastTimeMillis = currentTimeMillis;
}
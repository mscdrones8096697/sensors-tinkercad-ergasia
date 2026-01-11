C++
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"           // Needed for brownout fix
#include "soc/rtc_cntl_reg.h"  // Needed for brownout fix

// --- WiFi Settings ---
const char* ssid = "Robot_Car_WiFi";
const char* password = "12345678";

// --- Camera Pin Definition (AI THINKER Model) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer(); // Function prototype for the web server

void setup() {
  // 1. DISABLE BROWNOUT DETECTOR
  // Critical for robot cars: prevents ESP32 from resetting when motors draw high current
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  Serial.begin(115200); // Serial for debugging (USB)
  
  // 2. Initialize Serial Communication with Arduino
  // RX=14, TX=15. Connect ESP32 TX(15) -> Arduino RX(11) | ESP32 RX(14) -> Arduino TX(12)
  Serial2.begin(9600, SERIAL_8N1, 14, 15); 
  
  // 3. Setup WiFi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Camera Stream Ready! Connect to: http://");
  Serial.println(IP);

  // 4. Configure Camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Optimization for low memory (PSRAM check)
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize Camera
  esp_err_t err = esp_camera_init(&config);
  if (err!= ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  // Start the Web Server (This function handles the HTML and button clicks)
  // Note: The full implementation of startCameraServer() is usually in the "app_httpd.cpp"
  // file included in standard ESP32-CAM examples. You would modify it to send
  // Serial2.write('F'), Serial2.write('B'), etc., when buttons are pressed on the webpage.
  startCameraServer();
}

void loop() {
  // Main loop does nothing; everything is handled by FreeRTOS tasks (Server/Camera)
  delay(10000);
}

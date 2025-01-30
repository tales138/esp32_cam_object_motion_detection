#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "FS.h"
#include "LittleFS.h"

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_TTGO_T_CAMERA_V05
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"
#include "SSD1306.h"
#include "OLEDDisplayUi.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp32-hal-psram.h"

//#include "C:\Users\tales\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\idf-release_v5.1-b6b4727c58\esp32\include\esp_psram\include\esp32\himem.h"
//#include "C:\Users\tales\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\idf-release_v5.1-b6b4727c58\esp32\include\esp_psram\include\esp_private\esp_psram_extram.h"

#define ENABLE_OLED

#ifdef ENABLE_OLED
#include "SSD1306.h"
#define OLED_ADDRESS 0x3c
#define I2C_SDA 21
#define I2C_SCL 22
#define CONFIG_SPIRAM_SUPPORT 1 
#define CONFIG_SPIRAM_USE_MALLOC 1
#define CONFIG_SPIRAM_USE_CAPS_ALLOC 0
SSD1306Wire display(OLED_ADDRESS, I2C_SDA, I2C_SCL, GEOMETRY_128_64);
#endif
// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "GalaxyA14117B";
const char *password = "4321asdf";

void startCameraServer();
void setupLedFlash(int pin);
void info(char *msg);

void info(char *msg)
{
    display.clear();
    display.drawString(128 / 2, 32 / 2, msg);
    display.display();
}
void setup() {
  //esp_psram_init();
  
  display.init();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.display();
  info("Starting serial...");
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  //config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.pixel_format = /*PIXFORMAT_RGB565*/PIXFORMAT_GRAYSCALE; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  info("Starting camera...");
  Serial.println("Starting camera...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //s->set_pixformat(s,PIXFORMAT_YUV422);
  s->set_special_effect(s,2);
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
  info("Connecting...");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  display.clear();
  display.drawString(128 / 2, 32 / 2, WiFi.localIP().toString());
  
  
  display.display();

  
  //Serial.print(esp_psram_get_size());

 /* if (psramFound()){
      Serial.print("PSRAM found");
    }
  if(esp_psram_is_initialized()){
    Serial.print("PSRAM initi");
  }*/
  //esp_psram_extram_test();
  //esp_spiram_test();
  // Tamanho do bloco de memória a ser alocado
  //int size = 128*1024; // 1 KB

  // Aloca memória (em geral, você pode usar malloc diretamente aqui)
  //void* ptr = (unsigned char *) ps_malloc(size);
  //unsigned char *ptr = (unsigned char *) ps_malloc(size);
 /* if (ptr == NULL) {
    Serial.println("Falha ao alocar memória.");
    //return;
  }*
/*
  // Inicializa a memória alocada com um valor específico
  memset(ptr, 0xAA, size);
  /*if (CONFIG_SPIRAM_SUPPORT && (CONFIG_SPIRAM_USE_CAPS_ALLOC || CONFIG_SPIRAM_USE_MALLOC)){
    Serial.println(" alocar memória.");
  }*/
  // Mostra o endereço da memória alocada
  /*Serial.printf("Memória alocada em: %p\n", ptr);

  // Usa a memória alocada (neste exemplo, apenas imprime o valor inicial)
  for (size_t i = 0; i < size; i++) {
    Serial.printf("Byte %zu: %02X\n", i, ((uint8_t *)ptr)[i]);
  }

  // Libera a memória alocada
  free(ptr);*/
  if (psramFound()) {
     Serial.print("psramFound \n");
  }
    /*Serial.print("PSRAM size: ");
    if(esp_psram_init()){
       Serial.print("init \n");
    }
    Serial.print(esp_psram_get_size());
    
    uint16_t* p_buffer_a = (uint16_t*) heap_caps_malloc( 30700, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM );
    if(p_buffer_a != NULL){
        Serial.println("\nPSRAM is correctly initialized");
    }else{
        Serial.println("PSRAM not available");
    }*/
    //acc_data_all[1] = 'a';
}

void loop() {
    

    
    /*uint32_t totalHeap = ESP.getHeapSize();
    uint32_t freeHeap = ESP.getFreeHeap();

    Serial.printf("Tamanho total do heap: %u bytes\n", totalHeap);
    Serial.printf("Memória heap livre: %u bytes\n", freeHeap);
    
    // Aproximar a RAM total (não é uma medida exata, pois pode variar)
    Serial.printf("Aproximação da RAM total: %u bytes\n", totalHeap + (totalHeap - freeHeap));
    Serial.print("PSRAM size: ");
    Serial.print(ESP.getPsramSize());
    Serial.print("flash size: ");
    Serial.print(ESP.getFlashChipSize());
    if (psramFound()){
      Serial.print("PSRAM found");
    }*/
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}

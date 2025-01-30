// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#include <Arduino.h>
#endif

// Face Detection will not work on boards without (or with disabled) PSRAM
#ifdef BOARD_HAS_PSRAM
// Face Recognition takes upward from 15 seconds per frame on chips other than ESP32S3
// Makes no sense to have it enabled for them
#if CONFIG_IDF_TARGET_ESP32S3
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 1
#define CONFIG_ESP_FACE_DETECT_ENABLED      1
#else
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
#define CONFIG_ESP_FACE_DETECT_ENABLED      0
#endif
#else
#define CONFIG_ESP_FACE_DETECT_ENABLED      0
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED

#include <vector>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s16.hpp"
#include "face_recognition_112_v1_s8.hpp"
#pragma GCC diagnostic error "-Wformat"
#pragma GCC diagnostic warning "-Wstrict-aliasing"

#define QUANT_TYPE 0  //if set to 1 => very large firmware, very slow, reboots when streaming...

#define FACE_ID_SAVE_NUMBER 7
#endif

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
#endif
#define FACE_COLOR_GREEN  0x0000FF00
// Enable LED FLASH setting
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

// LED FLASH setup
#if CONFIG_LED_ILLUMINATOR_ENABLED

#define LED_LEDC_GPIO            22  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

#endif

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

#if CONFIG_ESP_FACE_DETECT_ENABLED

static int8_t detection_enabled = 0;

// #if TWO_STAGE
// static HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
// static HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
// #else
// static HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
// #endif

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;

#if QUANT_TYPE
// S16 model
FaceRecognition112V1S16 recognizer;
#else
// S8 model
FaceRecognition112V1S8 recognizer;
#endif
#endif

#endif

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static void rgb_print(fb_data_t *fb, uint32_t color, const char *str) {
  fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(fb_data_t *fb, uint32_t color, const char *format, ...) {
  char loc_buf[64];
  char *temp = loc_buf;
  int len;
  va_list arg;
  va_list copy;
  va_start(arg, format);
  va_copy(copy, arg);
  len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
  va_end(copy);
  if (len >= sizeof(loc_buf)) {
    temp = (char *)malloc(len + 1);
    if (temp == NULL) {
      return 0;
    }
  }
  vsnprintf(temp, len + 1, format, arg);
  va_end(arg);
  rgb_print(fb, color, temp);
  if (len > 64) {
    free(temp);
  }
  return len;
}
#endif
static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id) {
  int x, y, w, h;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0) {
    color = FACE_COLOR_RED;
  } else if (face_id > 0) {
    color = FACE_COLOR_GREEN;
  }
  if (fb->bytes_per_pixel == 2) {
    //color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
    color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800);
  }
  int i = 0;
  for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++) {
    // rectangle box
    x = (int)prediction->box[0];
    y = (int)prediction->box[1];
    w = (int)prediction->box[2] - x + 1;
    h = (int)prediction->box[3] - y + 1;
    if ((x + w) > fb->width) {
      w = fb->width - x;
    }
    if ((y + h) > fb->height) {
      h = fb->height - y;
    }
    fb_gfx_drawFastHLine(fb, x, y, w, color);
    fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(fb, x, y, h, color);
    fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);
#if TWO_STAGE
    // landmarks (left eye, mouth left, nose, right eye, mouth right)
    int x0, y0, j;
    for (j = 0; j < 10; j += 2) {
      x0 = (int)prediction->keypoint[j];
      y0 = (int)prediction->keypoint[j + 1];
      fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
    }
#endif
  }
}

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int run_face_recognition(fb_data_t *fb, std::list<dl::detect::result_t> *results) {
  std::vector<int> landmarks = results->front().keypoint;
  int id = -1;

  Tensor<uint8_t> tensor;
  tensor.set_element((uint8_t *)fb->data).set_shape({fb->height, fb->width, 3}).set_auto_free(false);

  int enrolled_count = recognizer.get_enrolled_id_num();

  if (enrolled_count < FACE_ID_SAVE_NUMBER && is_enrolling) {
    id = recognizer.enroll_id(tensor, landmarks, "", true);
    log_i("Enrolled ID: %d", id);
    rgb_printf(fb, FACE_COLOR_CYAN, "ID[%u]", id);
  }

  face_info_t recognize = recognizer.recognize(tensor, landmarks);
  if (recognize.id >= 0) {
    rgb_printf(fb, FACE_COLOR_GREEN, "ID[%u]: %.2f", recognize.id, recognize.similarity);
  } else {
    rgb_print(fb, FACE_COLOR_RED, "Intruder Alert!");
  }
  return recognize.id;
}
#endif
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en) {  // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted) {
    log_e("BMP Conversion failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  res = httpd_resp_send(req, (const char *)buf, buf_len);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
#endif
  log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
  return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
  
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();              // or it won't be visible in the frame. A better way to do this is needed.
  enable_led(false);
 
  
#else
  fb = esp_camera_fb_get();
  
#endif

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

#if CONFIG_ESP_FACE_DETECT_ENABLED
  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  bool s;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  bool detected = false;
#endif
  int face_id = 0;
  if (!detection_enabled || fb->width > 400) {
#endif
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    size_t fb_len = 0;
#endif
    if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
      fb_len = fb->len;
#endif
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = {req, 0};
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
      fb_len = jchunk.len;
#endif
    }
    esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    int64_t fr_end = esp_timer_get_time();
#endif
    log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
#if CONFIG_ESP_FACE_DETECT_ENABLED
  }

  jpg_chunking_t jchunk = {req, 0};

  if (fb->format == PIXFORMAT_RGB565
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
      && !recognition_enabled
#endif
  ) {
#if TWO_STAGE
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
    HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
    std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
    std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
#else
    HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
    std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif
    if (results.size() > 0) {
      fb_data_t rfb;
      rfb.width = fb->width;
      rfb.height = fb->height;
      rfb.data = fb->buf;
      rfb.bytes_per_pixel = 2;
      rfb.format = FB_RGB565;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
      detected = true;
#endif
      draw_face_boxes(&rfb, &results, face_id);
    }
    s = fmt2jpg_cb(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 90, jpg_encode_stream, &jchunk);
    esp_camera_fb_return(fb);
  } else {
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;
    out_buf = (uint8_t *)malloc(out_len);
    if (!out_buf) {
      log_e("out_buf malloc failed");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if (!s) {
      free(out_buf);
      log_e("To rgb888 failed");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }

    fb_data_t rfb;
    rfb.width = out_width;
    rfb.height = out_height;
    rfb.data = out_buf;
    rfb.bytes_per_pixel = 3;
    rfb.format = FB_BGR888;

#if TWO_STAGE
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
    HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
    std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
    std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
#else
    HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
    std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
#endif

    if (results.size() > 0) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
      detected = true;
#endif
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
      if (recognition_enabled) {
        face_id = run_face_recognition(&rfb, &results);
      }
#endif
      draw_face_boxes(&rfb, &results, face_id);
    }

    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    free(out_buf);
  }

  if (!s) {
    log_e("JPEG compression failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("FACE: %uB %ums %s%d", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
  return res;
#endif
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];
#if CONFIG_ESP_FACE_DETECT_ENABLED
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  bool detected = false;
  int64_t fr_ready = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;
  int64_t fr_face = 0;
  int64_t fr_start = 0;
#endif
  int face_id = 0;
  size_t out_len = 0, out_width = 0, out_height = 0;
  uint8_t *out_buf = NULL;
  bool s = false;
#if TWO_STAGE
  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#else
  HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
#endif
#endif

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = true;
  enable_led(true);
#endif

  while (true) {
#if CONFIG_ESP_FACE_DETECT_ENABLED
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    detected = false;
#endif
    face_id = 0;
#endif

    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
#if CONFIG_ESP_FACE_DETECT_ENABLED
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      fr_face = fr_start;
#endif
      if (!detection_enabled || fb->width > 400) {
#endif
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            log_e("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
#if CONFIG_ESP_FACE_DETECT_ENABLED
      } else {
        if (fb->format == PIXFORMAT_RGB565
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
            && !recognition_enabled
#endif
        ) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
          fr_ready = esp_timer_get_time();
#endif
#if TWO_STAGE
          std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
          std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
#else
          std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
          fr_face = esp_timer_get_time();
          fr_recognize = fr_face;
#endif
          if (results.size() > 0) {
            fb_data_t rfb;
            rfb.width = fb->width;
            rfb.height = fb->height;
            rfb.data = fb->buf;
            rfb.bytes_per_pixel = 2;
            rfb.format = FB_RGB565;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            detected = true;
#endif
            draw_face_boxes(&rfb, &results, face_id);
          }
          s = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!s) {
            log_e("fmt2jpg failed");
            res = ESP_FAIL;
          }
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
          fr_encode = esp_timer_get_time();
#endif
        } else {
          out_len = fb->width * fb->height * 3;
          out_width = fb->width;
          out_height = fb->height;
          out_buf = (uint8_t *)malloc(out_len);
          if (!out_buf) {
            log_e("out_buf malloc failed");
            res = ESP_FAIL;
          } else {
            s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
            esp_camera_fb_return(fb);
            fb = NULL;
            if (!s) {
              free(out_buf);
              log_e("To rgb888 failed");
              res = ESP_FAIL;
            } else {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
              fr_ready = esp_timer_get_time();
#endif

              fb_data_t rfb;
              rfb.width = out_width;
              rfb.height = out_height;
              rfb.data = out_buf;
              rfb.bytes_per_pixel = 3;
              rfb.format = FB_BGR888;

#if TWO_STAGE
              std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
              std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
#else
              std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
              fr_face = esp_timer_get_time();
              fr_recognize = fr_face;
#endif

              if (results.size() > 0) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                detected = true;
#endif
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
                if (recognition_enabled) {
                  face_id = run_face_recognition(&rfb, &results);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                  fr_recognize = esp_timer_get_time();
#endif
                }
#endif
                draw_face_boxes(&rfb, &results, face_id);
              }
              s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
              free(out_buf);
              if (!s) {
                log_e("fmt2jpg failed");
                res = ESP_FAIL;
              }
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
              fr_encode = esp_timer_get_time();
#endif
            }
          }
        }
      }
#endif
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    int64_t ready_time = (fr_ready - fr_start) / 1000;
    int64_t face_time = (fr_face - fr_ready) / 1000;
    int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    int64_t process_time = (fr_encode - fr_start) / 1000;
#endif

    int64_t frame_time = fr_end - last_frame;
    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
#if CONFIG_ESP_FACE_DETECT_ENABLED
      ", %u+%u+%u+%u=%u %s%d"
#endif
      ,
      (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time, 1000.0 / avg_frame_time
#if CONFIG_ESP_FACE_DETECT_ENABLED
      ,
      (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time, (detected) ? "DETECTED " : "", face_id
#endif
    );
  }

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = false;
  enable_led(false);
#endif

  return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;
      return ESP_OK;
    }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);
  log_i("%s = %d", variable, val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);
  }
#if CONFIG_LED_ILLUMINATOR_ENABLED
  else if (!strcmp(variable, "led_intensity")) {
    led_duty = val;
    if (isStreaming) {
      enable_led(true);
    }
  }
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
  else if (!strcmp(variable, "face_detect")) {
    detection_enabled = val;
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    if (!detection_enabled) {
      recognition_enabled = 0;
    }
#endif
  }
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
  else if (!strcmp(variable, "face_enroll")) {
    is_enrolling = !is_enrolling;
    log_i("Enrolling: %s", is_enrolling ? "true" : "false");
  } else if (!strcmp(variable, "face_recognize")) {
    recognition_enabled = val;
    if (recognition_enabled) {
      detection_enabled = val;
    }
  }
#endif
#endif
  else {
    log_i("Unknown command: %s", variable);
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, s, reg, 0xFFF);  //12 bit
    }
    p += print_reg(p, s, 0x3406, 0xFF);

    p += print_reg(p, s, 0x3500, 0xFFFF0);  //16 bit
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);   //10 bit
    p += print_reg(p, s, 0x350c, 0xFFFF);  //16 bit

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF);  //9 bit
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }

  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#if CONFIG_LED_ILLUMINATOR_ENABLED
  p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
  p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
#if CONFIG_ESP_FACE_DETECT_ENABLED
  p += sprintf(p, ",\"face_detect\":%u", detection_enabled);
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
  p += sprintf(p, ",\"face_enroll\":%u,", is_enrolling);
  p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
#endif
#endif
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);
  log_i("Set XCLK: %d MHz", xclk);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK
      || httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  int val = atoi(_val);
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0) {
    return httpd_resp_send_500(req);
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;
  }
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY,
    totalX, totalY, outputX, outputY, scale, binning
  );
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    if (s->id.PID == OV3660_PID) {
      return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    } else if (s->id.PID == OV5640_PID) {
      return httpd_resp_send(req, (const char *)index_ov5640_html_gz, index_ov5640_html_gz_len);
    } else {
      return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
    }
  } else {
    log_e("Camera sensor not found");
    return httpd_resp_send_500(req);
  }
}
/*static esp_err_t subtraction_handler(httpd_req_t *req) {
    camera_fb_t *fb1 = NULL;
    camera_fb_t *fb2 = NULL;
    esp_err_t res = ESP_OK;

    // Captura do primeiro frame
    fb1 = esp_camera_fb_get();
    if (!fb1) {
        log_e("Camera capture failed for first frame");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for first frame");
        return ESP_FAIL;
    }

    // Adiciona um atraso antes de capturar o segundo frame
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Captura do segundo frame
    fb2 = esp_camera_fb_get();
    if (!fb2) {
        log_e("Camera capture failed for second frame");
        esp_camera_fb_return(fb1);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for second frame");
        return ESP_FAIL;
    }

    // Verifica se ambos os frames possuem o mesmo tamanho e formato
    if (fb1->width != fb2->width || fb1->height != fb2->height || fb1->format != fb2->format) {
        log_e("Frames have different dimensions or formats");
        esp_camera_fb_return(fb1);
        esp_camera_fb_return(fb2);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Frames have different dimensions or formats");
        return ESP_FAIL;
    }

    // Subtração dos dois frames
    size_t frame_size = fb1->len;
    uint8_t *result_buf = (uint8_t *)malloc(frame_size);
    if (!result_buf) {
        log_e("Memory allocation for result frame failed");
        esp_camera_fb_return(fb1);
        esp_camera_fb_return(fb2);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Memory allocation for result frame failed");
        return ESP_FAIL;
    }

    // Realiza a subtração dos frames
    for (size_t i = 0; i < frame_size; i++) {
        int diff = fb2->buf[i] - fb1->buf[i];
        result_buf[i] = diff < 0 ? 0 : diff;
    }

    // Libera os frames capturados
    esp_camera_fb_return(fb1);
    esp_camera_fb_return(fb2);

    // Converte o resultado para JPEG
    camera_fb_t fb_result = {
        .buf = result_buf,
        .len = frame_size,
        .width = fb2->width,
        .height = fb2->height,
        .format = fb2->format,
        .timestamp = fb2->timestamp
    };

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;

    bool jpeg_converted = frame2jpg(&fb_result, 90, &jpg_buf, &jpg_len);
    free(result_buf);

    if (!jpeg_converted) {
        log_e("JPEG conversion failed");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: JPEG conversion failed");
        return ESP_FAIL;
    }

    // Envia a imagem JPEG como resposta HTTP
    res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    free(jpg_buf);

    if (res != ESP_OK) {
        log_e("Failed to send the image");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Failed to send the image");
        return ESP_FAIL;
    }

    return ESP_OK;
}*/
static esp_err_t capture_test_handler(httpd_req_t *req) {
  
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed");
        return ESP_FAIL;
    }
    
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);

    return ESP_OK;
}
#define CAPTURE_DELAY_MS 5000  // Atraso em milissegundos entre as capturas

static esp_err_t subtraction_handler(httpd_req_t *req) {
    camera_fb_t *fb1 = NULL;
    camera_fb_t *fb2 = NULL;
    esp_err_t res = ESP_OK;

    // Captura do primeiro frame
    fb1 = esp_camera_fb_get();
    if (!fb1) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for first frame");
        return ESP_FAIL;
    }

    // Adiciona um pequeno atraso antes de capturar o segundo frame
    vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));

    // Captura do segundo frame
    fb2 = esp_camera_fb_get();
    if (!fb2) {
        esp_camera_fb_return(fb1);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for second frame");
        return ESP_FAIL;
    }

    // Verifica se ambos os frames possuem o mesmo tamanho e formato
    if (fb1->width != fb2->width || fb1->height != fb2->height || fb1->format != fb2->format) {
        ESP_LOGE("CAMERA", "Frames have different dimensions or formats");
        esp_camera_fb_return(fb1);
        esp_camera_fb_return(fb2);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Frames have different dimensions or formats");
        return ESP_FAIL;
    }

    // Subtração dos dois frames
    size_t frame_size = fb1->len;
    uint8_t *result_buf = (uint8_t *)malloc(frame_size);
    if (!result_buf) {
        ESP_LOGE("CAMERA", "Memory allocation for result frame failed");
        esp_camera_fb_return(fb1);
        esp_camera_fb_return(fb2);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Memory allocation for result frame failed");
        return ESP_FAIL;
    }

    // Realiza a subtração dos frames
    for (size_t i = 0; i < frame_size; i++) {
        int diff = fb2->buf[i] - fb1->buf[i];
        result_buf[i] = diff < 0 ? 0 : diff;
    }

    // Libera os frames capturados
    esp_camera_fb_return(fb1);
    esp_camera_fb_return(fb2);

    // Converte o resultado para JPEG
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;

    bool jpeg_converted = fmt2jpg(result_buf, frame_size, fb2->width, fb2->height, fb2->format, 90, &jpg_buf, &jpg_len);
    free(result_buf);

    if (!jpeg_converted) {
        ESP_LOGE("CAMERA", "JPEG conversion failed");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: JPEG conversion failed");
        return ESP_FAIL;
    }

    // Envia a imagem JPEG como resposta HTTP
    res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    free(jpg_buf);

    if (res != ESP_OK) {
        ESP_LOGE("CAMERA", "Failed to send the image");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Failed to send the image");
        return ESP_FAIL;
    }

    return ESP_OK;
}

#include <JPEGENC.h>
JPEGENC jpgenc;
static esp_err_t capture_two_frames_handler1(httpd_req_t *req) {
    JPEGENCODE enc;
    uint8_t *out_jpg   = NULL;
    size_t out_jpg_len1 = 0, out_jpg_len2;
    camera_fb_t *fb1 = esp_camera_fb_get();
    sensor_t *s = esp_camera_sensor_get();
    s->set_pixformat(s, PIXFORMAT_YUV422);
    out_jpg = (uint8_t *) malloc(65536);
    jpgenc.open(out_jpg, 65536);
    jpgenc.encodeBegin(&enc, fb1->width, fb1->height, JPEGE_PIXEL_YUV422, JPEGE_SUBSAMPLE_420, JPEGE_Q_MED);
    jpgenc.addFrame(&enc, fb1->buf, fb1->width * 2);
    out_jpg_len2 = jpgenc.close(); 
    httpd_resp_set_type(req, "image/jpeg");
   // esp_err_t res = httpd_resp_send(req, (const char *)fb2->buf, fb2->len);
    esp_err_t res = httpd_resp_send(req, (const char *)out_jpg, out_jpg_len2);
    esp_camera_fb_return(fb1);
    return res;
    //free(out_jpg_len2);
}

static esp_err_t capture_two_frames_handler(httpd_req_t *req) {
    camera_fb_t *fb1 = esp_camera_fb_get();
    camera_fb_t *aux = NULL;
    camera_fb_t *aux2 = NULL;
    if (!fb1) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for first frame");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));  // Use o valor que funcionou antes
    aux = fb1;

    esp_camera_fb_return(fb1);
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Pequeno atraso para reinicializar
   
    camera_fb_t *fb2 = esp_camera_fb_get();
    aux2 = fb2;
    if (!fb2) {
        esp_camera_fb_return(fb1);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for second frame");
        return ESP_FAIL;
    }

    // Apenas para teste, enviamos o segundo frame
    httpd_resp_set_type(req, "image/jpeg");
   // esp_err_t res = httpd_resp_send(req, (const char *)fb2->buf, fb2->len);
    esp_err_t res = httpd_resp_send(req, (const char *)aux2->buf, aux2->len);
    esp_camera_fb_return(fb1);
    esp_camera_fb_return(fb2);

    return res;
}
#include "img_converters.h" 
#include "esp_log.h"

// Novo callback para a função frame2jpg_cb
static size_t jpg_encode_stream1(void *arg, size_t index, const void *data, size_t len) {
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index) {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_and_subtract_handler(httpd_req_t *req) {
    camera_fb_t *fb1 = esp_camera_fb_get();
    camera_fb_t *fb2 = NULL;
    camera_fb_t *fb1_aux = NULL;
    camera_fb_t *fb2_aux = NULL;
    esp_err_t res = ESP_OK;
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    if (!fb1) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for first frame");
        return ESP_FAIL;
    }

    // Armazenar o primeiro frame em uma variável auxiliar e liberar o recurso de fb1
    fb1_aux = (camera_fb_t *)malloc(4000);
    memcpy(fb1_aux->buf, fb1->buf, fb1->len);
    esp_camera_fb_return(fb1);

    vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));  // Atraso antes de capturar o segundo frame

    fb2 = esp_camera_fb_get();
    if (!fb2) {
        esp_camera_fb_return(fb1_aux);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed for second frame");
        return ESP_FAIL;
    }

    // Armazenar o segundo frame em uma variável auxiliar
    fb2_aux = (camera_fb_t *)malloc(4000);
    memcpy(fb1_aux->buf, fb2->buf, fb2->len);
    esp_camera_fb_return(fb2);
    // Verificar se ambos os frames possuem o mesmo tamanho e formato
    if (fb1_aux->width != fb2_aux->width || fb1_aux->height != fb2_aux->height) {
        esp_camera_fb_return(fb1_aux);
        esp_camera_fb_return(fb2_aux);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Frames have different dimensions");
        return ESP_FAIL;
    }

    // Subtração dos dois frames
    size_t frame_size = fb1_aux->len;
    uint8_t *result_bufaux = (uint8_t *)malloc(frame_size);
    camera_fb_t *result_buf = NULL;
   /* if (!result_buf) {
        esp_camera_fb_return(fb1_aux);
        esp_camera_fb_return(fb2_aux);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Memory allocation for result frame failed");
        return ESP_FAIL;
    }*/
    
    // Realiza a subtração dos frames
    for (size_t i = 0; i < frame_size; i++) {
        int diff = fb2_aux->buf[i] - fb1_aux->buf[i];
        result_bufaux[i] = diff < 0 ? 0 : diff;
        Serial.print("1 \n");
        Serial.print(fb1_aux->buf[i]);
        Serial.print("\n");
        Serial.print(fb2_aux->buf[i]);
        Serial.print("\n");
        Serial.print("2 \n");
    }
    result_buf->buf = result_bufaux;
    // Liberar os frames capturados
    esp_camera_fb_return(fb1_aux);
    esp_camera_fb_return(fb2_aux);

    // Convertendo o resultado para JPEG
    uint8_t *jpg_buf = (uint8_t *)malloc(frame_size);
    size_t jpg_len = 0;

    // Criar um frame temporário para armazenar o resultado da subtração
    /*camera_fb_t fb_result = {
        .buf = result_buf,
        .len = frame_size,
        .width = fb1_aux->width,
        .height = fb1_aux->height,
        .format = PIXFORMAT_JPEG, // Use o formato correto aqui
        .timestamp = 0
    };
*/
   // bool jpeg_converted = fmt2jpg(fb_result.buf, fb_result.len, fb_result.width, fb_result.height, fb_result.format, 80, &jpg_buf, &jpg_len);
    bool bmp_converted = frame2bmp(result_buf, &jpg_buf, &jpg_len);
    //free(result_buf->buf);

    if (!bmp_converted) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: JPEG conversion failed");
        return ESP_FAIL;
    }
     // Configura a resposta HTTP
   httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=subtracted.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    // Envia a imagem JPEG como resposta HTTP
    res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
    free(jpg_buf);

    if (res != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Failed to send the image");
        return ESP_FAIL;
    }

    return res;
}


static esp_err_t capture_and_test_jpeg_handler(httpd_req_t *req) {
    uint32_t totalHeap = ESP.getFreeHeap();
    
    size_t max_jpg_len = 6000;  // Exemplo de 200 KB, ajuste conforme necessário
    //uint8_t *jpg_buf = (uint8_t *) heap_caps_malloc(max_jpg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *jpg_buf = (uint8_t *)  malloc(max_jpg_len);
    //uint8_t *jpg_buf = (uint8_t *) heap_caps_malloc(5802, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    //uint8_t *jpg_buf = NULL;
    /*if (!jpg_buf) {
        //esp_camera_fb_return(fb);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: memoria allocation for JPEG buffer failed");
        return ESP_FAIL;
    }*/
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed");
        return ESP_FAIL;
    }
   
    // Defina o tamanho máximo esperado do buffer JPEG. 
    // Um tamanho razoável para uma imagem QVGA pode ser ajustado conforme necessário.
   
    uint32_t freeHeap = ESP.getFreeHeap();
    size_t jpg_len = 0;

    // Converte o frame para JPEG
    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format,10, &jpg_buf, &jpg_len);
    
    // Verifica se a conversão para JPEG foi bem-sucedida
    if (!jpeg_converted || jpg_buf == NULL) {
        // Incluindo informações de depuração na resposta de erro
        char err_msg[256];
        snprintf(err_msg, sizeof(err_msg),
                 "Error: JPEG conversion failed\n"
                 "JPEG Buffer Length: %d\n"
                 "orginal Buffer Length: %d\n"
                 "Image Width: %d\n"
                 "Image Height: %d\n"
                 "Frame Buffer Length: %d\n"
                 "Format: %d\n"
                 "Total heap: %d\n"
                 "Free heap: %d\n",
                 jpg_buf,fb->buf, fb->width, fb->height, fb->len,fb->format,totalHeap,freeHeap);

        free(jpg_buf);
        esp_camera_fb_return(fb);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, err_msg);
        return ESP_FAIL;
    }

    // Configura a resposta HTTP
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=test.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Envia a imagem JPEG como resposta HTTP
    esp_err_t res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);

    if (res != ESP_OK) {
        esp_camera_fb_return(fb);
        free(jpg_buf); // Libere o buffer JPEG em caso de erro
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Failed to send the image");
        return ESP_FAIL;
    }

    // Libera a memória do buffer JPEG
    free(jpg_buf);
    esp_camera_fb_return(fb);

    return ESP_OK;
}
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BMP_HEADER_SIZE 54  // Cabeçalho BMP fixo de 54 bytes

// Função para criar o cabeçalho BMP
void create_bmp_header(uint8_t *header, size_t width, size_t height, size_t data_size) {
    memset(header, 0, BMP_HEADER_SIZE);

    // Cabeçalho de Arquivo (14 bytes)
    header[0] = 'B';
    header[1] = 'M';  // Assinatura BMP
    *(uint32_t *)(header + 2) = BMP_HEADER_SIZE + data_size;  // Tamanho total do arquivo
    *(uint32_t *)(header + 10) = BMP_HEADER_SIZE;  // Offset para dados da imagem

    // Cabeçalho de Informação (40 bytes)
    *(uint32_t *)(header + 14) = 40;  // Tamanho do cabeçalho de informação
    *(uint32_t *)(header + 18) = width;  // Largura da imagem
    *(uint32_t *)(header + 22) = height;  // Altura da imagem
    *(uint16_t *)(header + 26) = 1;  // Número de planos de cor
    *(uint16_t *)(header + 28) = 24;  // Bits por pixel
    *(uint32_t *)(header + 30) = 0;  // Compressão (0 para nenhum)
    *(uint32_t *)(header + 34) = data_size;  // Tamanho dos dados da imagem
    *(uint32_t *)(header + 38) = 2835;  // Resolução horizontal (pixels por metro)
    *(uint32_t *)(header + 42) = 2835;  // Resolução vertical (pixels por metro)
    *(uint32_t *)(header + 46) = 0;  // Número de cores na paleta (0 para padrão)
    *(uint32_t *)(header + 50) = 0;  // Número de cores importantes (0 para todas)
}

// Função para manipular a solicitação HTTP e enviar a imagem BMP
static esp_err_t capture_and_test_bmp_handler(httpd_req_t *req) {
    size_t max_bmp_len = 10000;  // Ajuste o tamanho conforme necessário
    uint8_t *bmp_buf = (uint8_t *) malloc(max_bmp_len);
    if (!bmp_buf) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Memory allocation for BMP buffer failed");
        return ESP_FAIL;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Camera capture failed");
        free(bmp_buf);
        return ESP_FAIL;
    }

    size_t bmp_data_size = fb->width * fb->height * 3;  // Assumindo 24 bits por pixel
    size_t bmp_total_size = BMP_HEADER_SIZE + bmp_data_size;

    if (bmp_total_size > max_bmp_len) {
        free(bmp_buf);
        esp_camera_fb_return(fb);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Buffer size too small for BMP image");
        return ESP_FAIL;
    }

    // Cria o cabeçalho BMP
    create_bmp_header(bmp_buf, fb->width, fb->height, bmp_data_size);

    // Copia os dados da imagem para o buffer BMP
    uint8_t *image_data = bmp_buf + BMP_HEADER_SIZE;
    for (int y = 0; y < fb->height; y++) {
        for (int x = 0; x < fb->width; x++) {
            // Assumindo que os dados da imagem estão em RGB565
            uint16_t pixel = ((uint16_t *) fb->buf)[y * fb->width + x];
            uint8_t r = (pixel >> 11) & 0xF8;
            uint8_t g = (pixel >> 5) & 0xFC;
            uint8_t b = (pixel & 0xF8);
            image_data[((fb->height - y - 1) * fb->width + x) * 3 + 0] = b;
            image_data[((fb->height - y - 1) * fb->width + x) * 3 + 1] = g;
            image_data[((fb->height - y - 1) * fb->width + x) * 3 + 2] = r;
        }
    }

    // Configura a resposta HTTP
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=test.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Envia a imagem BMP como resposta HTTP
    esp_err_t res = httpd_resp_send(req, (const char *)bmp_buf, bmp_total_size);

    // Libera a memória
    free(bmp_buf);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Error: Failed to send the image");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t stream_handler1(httpd_req_t *req) {
    camera_fb_t *fb1 = NULL;
    camera_fb_t *fb2 = NULL;
    camera_fb_t *fb1_aux = NULL;
    camera_fb_t *fb2_aux = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];

    static int64_t last_frame = 0;
    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = true;
    enable_led(true);
#endif
    Serial.begin(115200);
    Serial.setDebugOutput(true);
   
    while (true) {
        // Captura o primeiro frame
        fb1 = esp_camera_fb_get();
        Serial.print("FRAME1");
        if (!fb1) {
            log_e("Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        
        // Armazena o primeiro frame em uma variável auxiliar e libera o recurso
        fb1_aux = fb1;
        esp_camera_fb_return(fb1);
        fb1 = NULL;

        //vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));  // Atraso antes de capturar o segundo frame
        Serial.print("FRAME2");
        // Captura o segundo frame
        fb2 = esp_camera_fb_get();
        if (!fb2) {
            esp_camera_fb_return(fb1_aux);
            log_e("Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // Armazena o segundo frame em uma variável auxiliar e libera o recurso
        fb2_aux = fb2;
        esp_camera_fb_return(fb2);
        fb2 = NULL;

        // Verificar se ambos os frames possuem o mesmo tamanho e formato
        if (fb1_aux->width != fb2_aux->width || fb1_aux->height != fb2_aux->height) {
            esp_camera_fb_return(fb1_aux);
            esp_camera_fb_return(fb2_aux);
            log_e("Frames have different dimensions");
            res = ESP_FAIL;
            break;
        }

        // Subtração dos dois frames
        size_t frame_size = fb1_aux->len;
        uint8_t *result_buf = (uint8_t *)malloc(frame_size);
        if (!result_buf) {
            esp_camera_fb_return(fb1_aux);
            esp_camera_fb_return(fb2_aux);
            log_e("Memory allocation for result frame failed");
            res = ESP_FAIL;
            break;
        }

        // Realiza a subtração dos frames
        for (size_t i = 0; i < frame_size; i++) {
            int diff = fb2_aux->buf[i] - fb1_aux->buf[i];
            //result_buf[i] = diff < 0 ? 0 : diff;
            result_buf[i] =  diff;
            Serial.print("1 \n");
            Serial.print(fb1_aux->buf[i]);
            Serial.print("\n");
            Serial.print(fb2_aux->buf[i]);
            Serial.print("\n");
            Serial.print("2 \n");
            
        }
        
        // Liberar os frames capturados
        esp_camera_fb_return(fb1_aux);
        esp_camera_fb_return(fb2_aux);

        // Convertendo o resultado para JPEG
        bool jpeg_converted = fmt2jpg(result_buf, frame_size, fb1_aux->width, fb1_aux->height, PIXFORMAT_GRAYSCALE, 80, &_jpg_buf, &_jpg_buf_len);
        
        free(result_buf);

        if (!jpeg_converted) {
            log_e("JPEG conversion failed");
            res = ESP_FAIL;
            break;
        }

        // Configura a resposta HTTP
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, fb1_aux->timestamp.tv_sec, fb1_aux->timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK) {
            log_e("Send frame failed");
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        frame_time /= 1000;
        last_frame = fr_end;

        log_i("MJPG: %uB %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = false;
    enable_led(false);
#endif

    return res;
}

static esp_err_t capture_and_subtract_handler2(httpd_req_t *req) {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    camera_fb_t *fb1 = esp_camera_fb_get();
    esp_err_t res = ESP_OK;
    
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    
    if (!fb1) {
        // Handle error
    }

    // Capture first frame
    camera_fb_t *fb1_aux = (camera_fb_t *)malloc(4000);
    //fb1_aux->buf = (uint8_t *)malloc(fb1->len);
   // memcpy(fb1_aux->buf, fb1->buf, fb1->len);
    esp_camera_fb_return(fb1);
    
    vTaskDelay(pdMS_TO_TICKS(CAPTURE_DELAY_MS));

    // Capture second frame
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        // Handle error
    }

    camera_fb_t *fb2_aux = (camera_fb_t *)malloc(4000);
    fb2_aux->buf = (uint8_t *)malloc(fb2->len);
    //memcpy(fb2_aux->buf, fb2->buf, fb2->len);
    esp_camera_fb_return(fb2);
    
    // Check dimensions
    if (fb1_aux->width != fb2_aux->width || fb1_aux->height != fb2_aux->height) {
        // Handle error
    }

    // Subtract frames
    size_t frame_size = fb1_aux->len;
    uint8_t *result_bufaux = (uint8_t *)malloc(frame_size);

    for (size_t i = 0; i < frame_size; i++) {
        int diff = fb2_aux->buf[i] - fb1_aux->buf[i];
         result_bufaux[i] =  diff;
         Serial.print("1 \n");
         Serial.print(fb1_aux->buf[i]);
         Serial.print("\n");
         Serial.print(fb2_aux->buf[i]);
         Serial.print("\n");
         Serial.print("2 \n");
    }

    // Free auxiliary frames
    free(fb1_aux->buf);
    free(fb2_aux->buf);
    free(fb1_aux);
    free(fb2_aux);

    // Convert to BMP
    // Handle conversion and response setup here...

    return res;
}
#include "FS.h" // Para SPIFFS
#include "SPIFFS.h"
#include "JPEGDecoder.h"

bool subtract_images(const char* img1_path, const char* img2_path, const char* result_path) {
    File file1 = SPIFFS.open(img1_path, FILE_READ);
    File file2 = SPIFFS.open(img2_path, FILE_READ);
    File result_file = SPIFFS.open(result_path, FILE_WRITE);
   
    if (!file1 || !file2 || !result_file) {
        Serial.println("Erro ao abrir arquivos de imagem");
        return false;
    }

    size_t len1 = file1.size();
    size_t len2 = file2.size();

    /*if (len1 != len2) {
        Serial.println("Tamanhos das imagens não são iguais");
        return false;
    }
*/
    // Aloca buffers para as imagens
    uint8_t* img1_buffer = (uint8_t*)malloc(4*1024);
    uint8_t* img2_buffer = (uint8_t*)malloc(4*1024);
    //uint8_t* result_buffer = (uint8_t*)malloc(len1);

    if (!img1_buffer || !img2_buffer /*|| !result_buffer*/) {
        //if (img1_buffer) free(img1_buffer);
        //if (img2_buffer) free(img2_buffer);
        //if (result_buffer) free(result_buffer);
        Serial.println("Erro ao alocar memória para as imagens");
        return false;
    }

    // Lê os dados das duas imagens
    if (file1.read(img1_buffer, len1) != len1 || file2.read(img2_buffer, len2) != len2) {
        Serial.println("Erro ao ler os dados das imagens");
        free(img1_buffer);
        free(img2_buffer);
        //free(result_buffer);
        return false;
    }
    uint8_t* result_buffer = (uint8_t*)malloc(len1);
    // Subtração pixel a pixel
    for (size_t i = 0; i < len1; i++) {
        result_buffer[i] = abs(img1_buffer[i] - img2_buffer[i]);
    }

    // Salva a imagem resultante
    result_file.write(result_buffer, len1);
    
    // Libera a memória alocada
    free(img1_buffer);
    free(img2_buffer);
    free(result_buffer);

    return true;
}

static esp_err_t capture_and_subtract_handler3(httpd_req_t *req) {
    // Inicializa SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Erro ao iniciar SPIFFS");
        return ESP_FAIL;
    }

    Serial.begin(115200);
    Serial.setDebugOutput(true);
    size_t freeHeap = esp_get_free_heap_size();
    Serial.println("Memoria livre antes da foto \n ");
    Serial.println(freeHeap);
    // Captura o primeiro frame
    camera_fb_t *fb1 = esp_camera_fb_get();
    size_t freeHeap2 = esp_get_free_heap_size();
    Serial.println("memoria livre apos foto \n");
    Serial.println(freeHeap2);
    if (!fb1) {
        Serial.println("Erro ao capturar o primeiro frame");
        return ESP_FAIL;
    }

    // Salva o primeiro frame
    File file1 = SPIFFS.open("/frame1.raw", FILE_WRITE);
    if (!file1) {
        Serial.println("Erro ao abrir ou criar frame1.raw");
        esp_camera_fb_return(fb1);
        return ESP_FAIL;
    }
    file1.write(fb1->buf, fb1->len);
    file1.close();
    esp_camera_fb_return(fb1); // Libera a memória do frame

    // Captura o segundo frame
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (!fb2) {
        Serial.println("Erro ao capturar o segundo frame");
        return ESP_FAIL;
    }

    // Salva o segundo frame
    File file2 = SPIFFS.open("/frame2.raw", FILE_WRITE);
    if (!file2) {
        Serial.println("Erro ao abrir ou criar frame2.raw");
        esp_camera_fb_return(fb2);
        return ESP_FAIL;
    }
    file2.write(fb2->buf, fb2->len);
    file2.close();
    esp_camera_fb_return(fb2); // Libera a memória do frame

    // Realiza a subtração das duas imagens
    if (!subtract_images("/frame1.raw", "/frame2.raw", "/result.raw")) {
        Serial.println("Erro na subtração de imagens");
        return ESP_FAIL;
    }

    return ESP_OK;
}
static esp_err_t capture_and_subtract_handler4(httpd_req_t *req) {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    Serial.println("Memoria livre antes da foto \n ");
    Serial.println(freeHeap);
    // Captura o primeiro frame
    camera_fb_t *fb1 = esp_camera_fb_get();
    uint8_t *frame1_copy = (uint8_t *)malloc(fb1->len);
    Serial.println("fb1 len \n");
    Serial.println(fb1->len);
    for(uint8_t i=0;i<fb1->len;i++){
      frame1_copy[i] = fb1->buf[i];
      //Serial.println(i);
    }

    //memcpy(frame1_copy, fb1->buf, fb1->len);
    esp_camera_fb_return(fb1);
    size_t freeHeap2 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    camera_fb_t *fb2 = esp_camera_fb_get();
    //uint8_t *frame2_copy = (uint8_t *)malloc(fb2->len);
    Serial.println("memoria livre apos foto \n");
    Serial.println(frame1_copy[1]);
    Serial.println("buffer len \n");
   // Serial.println(fb1->len);
    Serial.println(fb2->len);
    Serial.println(fb2->buf[1]);
    for(uint8_t i=0;i<20;i++){
      Serial.println("buffer 1");
      Serial.println(frame1_copy[i]);
      Serial.println("\n");
      Serial.println("buffer2");
      Serial.println(fb2->buf[i]);
    
    } 
    esp_camera_fb_return(fb1);
    esp_camera_fb_return(fb2);
    return ESP_OK;
}





uint8_t *  jpg_buf = NULL;
size_t  len = NULL;
size_t  wt = NULL;
size_t  he = NULL;
int cont = 0;

typedef struct {
    int x_min, x_max, y_min, y_max;
    int pixel_count;
} Region;

// Função iterativa para expandir uma região conectada
void expand_region(uint8_t *buffer, int buf_len, int width, int start_x, int start_y, Region *region) {
    const int MAX_QUEUE = 2048; // Tamanho máximo da fila
    int queue[MAX_QUEUE][2];   // Fila para armazenar coordenadas (x, y)
    int front = 0, back = 0;   // Indicadores da fila

    // Adiciona o pixel inicial à fila
    queue[back][0] = start_x;
    queue[back][1] = start_y;
    back++;

    while (front != back) {
        // Remove o próximo pixel da fila
        int x = queue[front][0];
        int y = queue[front][1];
        front = (front + 1) % MAX_QUEUE;

        int index = y * width + x;
        if (x < 0 || y < 0 || x >= width || y >= buf_len / width || buffer[index] != 255) {
            continue;
        }

        // Marca o pixel como processado
        buffer[index] = 0;
        region->pixel_count++;

        // Atualiza os limites da região
        if (x < region->x_min) region->x_min = x;
        if (x > region->x_max) region->x_max = x;
        if (y < region->y_min) region->y_min = y;
        if (y > region->y_max) region->y_max = y;

        // Adiciona os vizinhos à fila
        if (x + 1 < width) { queue[back][0] = x + 1; queue[back][1] = y; back = (back + 1) % MAX_QUEUE; }
        if (x - 1 >= 0)    { queue[back][0] = x - 1; queue[back][1] = y; back = (back + 1) % MAX_QUEUE; }
        if (y + 1 < buf_len / width) { queue[back][0] = x; queue[back][1] = y + 1; back = (back + 1) % MAX_QUEUE; }
        if (y - 1 >= 0)    { queue[back][0] = x; queue[back][1] = y - 1; back = (back + 1) % MAX_QUEUE; }
    }
}

int detect_regions(uint8_t *buffer, int buf_len, int width, Region *regions, int max_regions, int min_pixels) {
    int region_count = 0;
    int height = buf_len / width;

    // Conta o total de pixels brancos
    int total_white_pixels = 0;
    /*for (int i = 0; i < buf_len; i++) {
        if (buffer[i] == 255) {
            total_white_pixels++;
        }
    }*/

    for (int i = 0; i < buf_len; i++) {
        if (buffer[i] == 255) {// Pixel branco encontrado
             total_white_pixels++; 
            if (region_count >= max_regions) break;

            int y = i / width;
            int x = i % width;

            Region region = {x, x, y, y, 0};
            expand_region(buffer, buf_len, width, x, y, &region);

            // Filtra regiões muito pequenas
            if (region.pixel_count >= min_pixels) {
                // Verifica se a região ocupa uma parte significativa do total de pixels brancos
                float region_fraction = (float)region.pixel_count / total_white_pixels;
                if (region_fraction < 0.9) { // Exclui se ocupar 90% ou mais dos pixels brancos
                    regions[region_count++] = region;
                } else {
                    printf("Região descartada: ocupa %.2f%% dos pixels brancos.\n", region_fraction * 100);
                }
            }
        }
    }

    return region_count;
}


static Region* process_image(uint8_t *buffer,size_t buf_len,size_t width) {
    //uint8_t *buffer = fb->buf; // Assuma que fb contém a imagem binária
    //int buf_len = fb->len;
    //int width = fb->width;

    //Region regions[10];
    printf("\n inicio1 \n");
    Region *regions = NULL;
    regions = new Region[10];
    printf("\n inicio2 \n");
    //Region regios = NULL; // Máximo de 10 regiões
    int num_regions = detect_regions(buffer, buf_len, width, regions, 10, 50);

    printf("Regiões detectadas: %d\n", num_regions);
    printf("process imgage complete");
    /*for (int i = 0; i < num_regions; i++) {
        printf("Região %d: x_min=%d, x_max=%d, y_min=%d, y_max=%d, pixel_count=%d\n",
               i + 1, regions[i].x_min, regions[i].x_max, regions[i].y_min, regions[i].y_max, regions[i].pixel_count);
    }*/
    
    return regions;
}

static int detectObjectsInMotion(uint8_t* binaryImage, int width, int height) {
    //const int MAX_LABELS = 256; // Limite de regiões a detectar
    int labels[width * height]; // Matriz de rótulos
    int labelCount = 1;
    int tes = 1;
    // Array para armazenar tamanhos das regiões
    int regionSizes[240 * 240];
    printf("inicio");
    // Algoritmo de rotulagem simples (4 conectados)
    for (int y = 0; y < height; y++) {
       printf("inicio1");
        for (int x = 0; x < width; x++) {
            if (binaryImage[y * width + x] > 0) {
                int left = (x > 0) ? labels[y * width + (x - 1)] : 0;
                int top = (y > 0) ? labels[(y - 1) * width + x] : 0;

                if (left > 0) {
                    labels[y * width + x] = left;
                } else if (top > 0) {
                    labels[y * width + x] = top;
                } else {
                    labelCount++;
                    labels[y * width + x] =labelCount ;
                }

                // Incrementa o tamanho da região
                regionSizes[labels[y * width + x]]++;
            }
        }
    }
    return labelCount;
 
}
static int detectObjectsInMotion1(uint8_t* binaryImage){
  int labelc = 1;
  int labels[240 * 240];
  for (int y = 0; y < 240; y++) {
    for (int x = 0; x < 240; x++) {
      if(binaryImage[y * 240 + x] == 255){
        int left = (x > 0) ? labels[y * 240 + (x - 1)] : 0;
        int top = (y > 0) ? labels[(y - 1) * 240 + x] : 0;

        labels[labelc] = binaryImage[y * 240 + x];
        labelc++;
      } 
    }
  }
  return labelc;
}


#define MAX_LABELS 256 // Limite de rótulos, ajustável para a resolução da imagem

typedef struct {
    int parent[MAX_LABELS];
    int rank[MAX_LABELS];
} LabelEquivalence;

// Inicializa o sistema de equivalência
void initEquivalence(LabelEquivalence* eq) {
    for (int i = 0; i < MAX_LABELS; i++) {
        eq->parent[i] = i;
        eq->rank[i] = 0;
    }
}

// Encontra o rótulo raiz com compressão de caminho
int find(LabelEquivalence* eq, int x) {
    if (eq->parent[x] != x) {
        eq->parent[x] = find(eq, eq->parent[x]);
    }
    return eq->parent[x];
}

// Une dois rótulos
void unionLabels(LabelEquivalence* eq, int x, int y) {
    int rootX = find(eq, x);
    int rootY = find(eq, y);

    if (rootX != rootY) {
        if (eq->rank[rootX] > eq->rank[rootY]) {
            eq->parent[rootY] = rootX;
        } else if (eq->rank[rootX] < eq->rank[rootY]) {
            eq->parent[rootX] = rootY;
        } else {
            eq->parent[rootY] = rootX;
            eq->rank[rootX]++;
        }
    }
}

// Função principal para 4-conectados
int connectedComponents4(uint8_t* binaryImage, int width, int height, int* labels) {
    LabelEquivalence eq;
    initEquivalence(&eq);

    int currentLabel = 1;

    // Passagem 1: Atribuição de rótulos
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (binaryImage[y * width + x] > 0) { // Pixel branco
                int left = (x > 0) ? labels[y * width + (x - 1)] : 0;
                int top = (y > 0) ? labels[(y - 1) * width + x] : 0;

                if (left > 0 || top > 0) {
                    // Adota o menor rótulo entre os vizinhos
                    int smallestLabel = (left > 0) ? left : top;
                    if (top > 0 && top < smallestLabel) {
                        smallestLabel = top;
                    }
                    labels[y * width + x] = smallestLabel;

                    // Une os rótulos equivalentes
                    if (left > 0 && left != smallestLabel) {
                        unionLabels(&eq, left, smallestLabel);
                    }
                    if (top > 0 && top != smallestLabel) {
                        unionLabels(&eq, top, smallestLabel);
                    }
                } else {
                    // Novo rótulo
                    labels[y * width + x] = currentLabel++;
                }
            } else {
                labels[y * width + x] = 0; // Fundo
            }
        }
    }

    // Passagem 2: Atualização dos rótulos finais
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (labels[y * width + x] > 0) {
                labels[y * width + x] = find(&eq, labels[y * width + x]);
            }
        }
    }

    return currentLabel - 1; // Número total de componentes
}

void dilate(uint8_t* image, int width, int height) {
    uint8_t* temp = (uint8_t*)malloc(width * height);
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            uint8_t maxPixel = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    maxPixel = max(maxPixel, image[(y + dy) * width + (x + dx)]);
                }
            }
            temp[y * width + x] = maxPixel;
        }
    }
    // Define as bordas como zero
    for (int x = 0; x < width; x++) {
        temp[x] = 0;                     // Linha superior
        temp[(height - 1) * width + x] = 0; // Linha inferior
    }
    for (int y = 0; y < height; y++) {
        temp[y * width] = 0;             // Coluna esquerda
        temp[y * width + (width - 1)] = 0; // Coluna direita
    }

    memcpy(image, temp, width * height);
    free(temp);
}


   //printf("labelCount: ");
    //Serial.print(labelCount);
   
    // Análise de regiões
    /*for (int i = 1; i < labelCount; i++) {
        if (regionSizes[i] > 50) { // Filtra regiões pequenas
            Serial.printf("Região %d: %d pixels\n", i, regionSizes[i]);
            // Você pode calcular e armazenar centroides ou áreas aqui
        }
    }*/
#define MAX_CONTOURS 500
#define MAX_CONTOUR_POINTS 1000

struct Contour {
    int points[MAX_CONTOUR_POINTS][2];
    int size;
};

int detectContoursNoModify(const uint8_t* binaryImage, int width, int height, Contour* contours, int maxContours) {
    int numContours = 0;
    
    // Matriz de pixels visitados
    bool* visited = (bool*)malloc(width * height * sizeof(bool));
    if (!visited) {
        printf("Erro: Falha ao alocar memória para matriz visited.\n");
        return 0;
    }
    memset(visited, false, width * height * sizeof(bool));

    // Movimentos para os 8 vizinhos (vizinhança de 8 conectados)
    const int dx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    const int dy[8] = {0, -1, -1, 0, 1, 1, 1, 0};

    // Função para verificar se um pixel é válido
    auto isValidPixel = [&](int x, int y) {
        return (x >= 0 && x < width && y >= 0 && y < height &&
                binaryImage[y * width + x] > 0 && !visited[y * width + x]);
    };

    // Itera sobre todos os pixels da imagem
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Ignora pixels já processados ou que não são parte de um contorno
            if (!isValidPixel(x, y)) continue;

            // Verifica limite de contornos
            if (numContours >= maxContours) {
                printf("Aviso: Número máximo de contornos atingido.\n");
                free(visited);
                return numContours;
            }

            // Inicia um novo contorno
            Contour& contour = contours[numContours++];
            contour.size = 0;

            int startX = x, startY = y;
            int curX = x, curY = y, prevDir = 7;

            do {
                // Adiciona ponto ao contorno
                if (contour.size < MAX_CONTOUR_POINTS) {
                    contour.points[contour.size][0] = curX;
                    contour.points[contour.size][1] = curY;
                    contour.size++;
                }

                // Marca pixel como processado
                visited[curY * width + curX] = true;

                // Procura próximo pixel do contorno
                bool foundNext = false;
                for (int d = 0; d < 8; d++) {
                    int dir = (prevDir + d) % 8; // Verifica na ordem circular
                    int nextX = curX + dx[dir];
                    int nextY = curY + dy[dir];

                    if (isValidPixel(nextX, nextY)) {
                        curX = nextX;
                        curY = nextY;
                        prevDir = (dir + 6) % 8; // Ajusta direção para continuar o contorno
                        foundNext = true;
                        break;
                    }
                }

                if (!foundNext) break; // Sem próximo pixel, contorno termina

            } while (curX != startX || curY != startY); // Sai do loop quando retorna ao ponto inicial
        }
    }

    free(visited); // Libera a memória da matriz visited
    return numContours;
}
#include <stack>
// Função para contar regiões conectadas (8-conectados)
int countRegions(uint8_t* image, int width, int height) {
    // Matriz para marcar os pixels visitados
    bool* visited = (bool*)malloc(width * height * sizeof(bool));
    if (!visited) {
        // Tratar erro de alocação de memória
        return -1;
    }
    memset(visited, false, width * height * sizeof(bool));

    int regionCount = 0;

    // Função lambda para verificar se um pixel é válido
    auto isValidPixel = [&](int x, int y) {
        return (x >= 0 && x < width && y >= 0 && y < height &&
                image[y * width + x] == 255 && !visited[y * width + x]);
    };

    // Movimentos para os 8 vizinhos
    const int dx[8] = {-1, -1, 0, 1, 1,  1,  0, -1};
    const int dy[8] = { 0, -1, -1, -1, 0,  1,  1,  1};

    // Percorre todos os pixels da imagem
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Se o pixel atual não é válido, pula para o próximo
            if (!isValidPixel(x, y)) continue;

            // Inicia uma nova região
            regionCount++;

            // Pilha para o Flood Fill iterativo
            std::stack<std::pair<int, int>> stack;
            stack.push({x, y});

            while (!stack.empty()) {
                auto [curX, curY] = stack.top();
                stack.pop();

                // Marca o pixel como visitado
                visited[curY * width + curX] = true;

                // Verifica os vizinhos
                for (int i = 0; i < 8; i++) {
                    int nextX = curX + dx[i];
                    int nextY = curY + dy[i];

                    if (isValidPixel(nextX, nextY)) {
                        stack.push({nextX, nextY});
                        visited[nextY * width + nextX] = true;
                    }
                }
            }
        }
    }

    free(visited); // Libera a memória alocada
    return regionCount;
}

// Estrutura para armazenar uma bounding box
struct BoundingBox {
    int minX, minY; // Coordenadas mínimas
    int maxX, maxY; // Coordenadas máximas
};

// Função para detectar regiões conectadas e calcular as bounding boxes
std::vector<BoundingBox> detectRegionsWithBoundingBoxes(uint8_t* image, int width, int height) {
    // Matriz para marcar os pixels visitados
    bool* visited = (bool*)ps_malloc(width * height * sizeof(bool));
    if (!visited) {
        // Tratar erro de alocação de memória
        return {};
    }
    memset(visited, false, width * height * sizeof(bool));

    std::vector<BoundingBox> boundingBoxes;

    // Função lambda para verificar se um pixel é válido
    auto isValidPixel = [&](int x, int y) {
        return (x >= 0 && x < width && y >= 0 && y < height &&
                image[y * width + x] == 255 && !visited[y * width + x]);
    };

    // Movimentos para os 8 vizinhos
    const int dx[8] = {-1, -1, 0, 1, 1,  1,  0, -1};
    const int dy[8] = { 0, -1, -1, -1, 0,  1,  1,  1};

    // Percorre todos os pixels da imagem
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Se o pixel atual não é válido, pula para o próximo
            if (!isValidPixel(x, y)) continue;

            // Cria uma nova bounding box inicializada com valores extremos
            BoundingBox box = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};

            // Pilha para o Flood Fill iterativo
            std::stack<std::pair<int, int>> stack;
            stack.push({x, y});

            while (!stack.empty()) {
                auto [curX, curY] = stack.top();
                stack.pop();

                // Marca o pixel como visitado
                visited[curY * width + curX] = true;

                // Atualiza a bounding box
                box.minX = std::min(box.minX, curX);
                box.minY = std::min(box.minY, curY);
                box.maxX = std::max(box.maxX, curX);
                box.maxY = std::max(box.maxY, curY);

                // Verifica os vizinhos
                for (int i = 0; i < 8; i++) {
                    int nextX = curX + dx[i];
                    int nextY = curY + dy[i];

                    if (isValidPixel(nextX, nextY)) {
                        stack.push({nextX, nextY});
                        visited[nextY * width + nextX] = true;
                    }
                }
            }

            // Adiciona a bounding box detectada à lista
            boundingBoxes.push_back(box);
        }
    }

    free(visited); // Libera a memória alocada
    return boundingBoxes;
}
void applyMeanFilter(uint8_t* image, uint8_t* output, int width, int height, int kernelSize) {
    int halfKernel = kernelSize / 2;

    // Itera sobre cada pixel da imagem
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int sum = 0;
            int count = 0;

            // Varre a janela do kernel
            for (int ky = -halfKernel; ky <= halfKernel; ky++) {
                for (int kx = -halfKernel; kx <= halfKernel; kx++) {
                    int nx = x + kx;
                    int ny = y + ky;

                    // Verifica se o pixel vizinho está dentro dos limites da imagem
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        sum += image[ny * width + nx];
                        count++;
                    }
                }
            }

            // Calcula a média e armazena no pixel de saída
            output[y * width + x] = sum / count;
        }
    }
}

// Filtro da média aplicado diretamente na imagem
void applyMeanFilterInPlace(uint8_t* image, int width, int height, int kernelSize) {
    int halfKernel = kernelSize / 2;

    // Buffer temporário para armazenar uma linha suavizada
    uint8_t* tempRow = (uint8_t*) ps_malloc(width * sizeof(uint8_t));
    if (!tempRow) {
        // Erro de alocação
        return;
    }

    // Itera sobre cada linha da imagem
    for (int y = 0; y < height; y++) {
        // Processa cada pixel da linha atual
        for (int x = 0; x < width; x++) {
            int sum = 0;
            int count = 0;

            // Varre a janela do kernel
            for (int ky = -halfKernel; ky <= halfKernel; ky++) {
                for (int kx = -halfKernel; kx <= halfKernel; kx++) {
                    int nx = x + kx;
                    int ny = y + ky;

                    // Verifica se o pixel vizinho está dentro dos limites
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        sum += image[ny * width + nx];
                        count++;
                    }
                }
            }

            // Calcula a média e armazena no buffer temporário
            tempRow[x] = sum / count;
        }

        // Copia a linha suavizada de volta para a imagem
        memcpy(&image[y * width], tempRow, width);
    }

    free(tempRow); // Libera o buffer temporário
}
static esp_err_t capture_and_subtract_handler5(httpd_req_t *req) {
  
  esp_err_t res = ESP_OK;
  printf("\n memoria livre %d",ESP.getFreeHeap());
  if(cont == 0){
     camera_fb_t *fb = esp_camera_fb_get();
     jpg_buf = (uint8_t *) ps_malloc(fb->len);
     jpg_buf = fb->buf;
     len = fb->len;
     wt = fb->width;
     he = fb->height;
     esp_camera_fb_return(fb);
     Serial.print("buffer par \n");
  }
  else{
  Contour contours[MAX_CONTOURS];
  camera_fb_t *fb2 = esp_camera_fb_get();
  len = fb2->len;
  uint8_t * subtraction_buffer = NULL;
  subtraction_buffer = (uint8_t *) ps_malloc(fb2->len);
  printf("controle0 \n");
  //applyMeanFilterInPlace(fb2->buf,240,240,3);
  for (int i = 0; i < len; i++) {
    uint8_t diff = abs(fb2->buf[i] - jpg_buf[i]); // Subtração absoluta de cada pixel
    
    if (diff > 70) {
      diff = 255; // Branco
    } else {
      diff = 0; // Preto
    }
    subtraction_buffer[i] = (uint8_t)diff; // Armazenando o resultado da subtração
  }
  //int number = detectObjectsInMotion(subtraction_buffer,240,240);
  dilate(subtraction_buffer,240,240);
  int* labels = (int*) ps_malloc(240 * 240 * sizeof(int));

  // Inicializa o array
  for (int i = 0; i < 240 * 240; i++) {
      labels[i] = 0;
  }
  //int numComponents = connectedComponents4(subtraction_buffer, 240, 240, labels);
  int numComponents = countRegions(subtraction_buffer,240,240);
  std::vector<BoundingBox> boxes = detectRegionsWithBoundingBoxes(subtraction_buffer,240,240); 
 /* for (size_t i = 0; i < boxes.size(); i++) {
        printf("Região %zu: MinX=%d, MinY=%d, MaxX=%d, MaxY=%d\n",
               i + 1, boxes[i].minX, boxes[i].minY, boxes[i].maxX, boxes[i].maxY);
  }*/
  //int number = detectObjectsInMotion1(subtraction_buffer);
 
  
  //Region* reg = process_image(subtraction_buffer,fb2->len,fb2->width);
    
    /*int x =  boxes[0].minX;
    int y =  boxes[0].minY;
    int w =  boxes[0].maxX - x + 1;
    int h =  boxes[0].maxY - y + 1;*/
    //printf("%d %d %d %d ",x,y,reg[0].x_max,reg[0].y_max);
    uint32_t color = FACE_COLOR_GREEN;
    fb_data_t *fb_aux = NULL;
    printf("controle1 \n");
    //fb_aux = (fb_data_t *) heap_caps_malloc(fb2->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    fb_aux = (fb_data_t *) ps_malloc(fb2->len);
    fb_format_t forma = FB_GRAY;
    printf("\n controle2 \n");
    fb_aux->width = fb2->width;
    fb_aux->height = fb2->height;
    fb_aux->bytes_per_pixel = 1;
    fb_aux->format = forma;
    fb_aux->data = fb2->buf;
    printf("\n controle3 \n");
    //fb_gfx_drawFastHLine(fb_aux, 207, 235, 213-207 + 1, color);
     if(boxes.size() > 0){
      for (size_t i = 0; i < boxes.size(); i++) {
        int w =  boxes[i].maxX - boxes[i].minX + 1;
        int h =  boxes[i].maxY - boxes[i].minY + 1;
        
        fb_gfx_drawFastHLine(fb_aux, boxes[i].minX, boxes[i].minY, w, color);
        fb_gfx_drawFastHLine(fb_aux, boxes[i].minX, boxes[i].minY+h, w, color);
        fb_gfx_drawFastVLine(fb_aux, boxes[i].minX, boxes[i].minY, h, color);
        fb_gfx_drawFastVLine(fb_aux, boxes[i].minX+w-1, boxes[i].minY, h, color);
      }
     }
    /*
    int x =  boxes[0].minX;
    int y =  boxes[0].minY;
    int w =  boxes[0].maxX - x + 1;
    int h =  boxes[0].maxY - y + 1;
    
    fb_gfx_drawFastHLine(fb_aux, x, y+h-1, w, color);
    fb_gfx_drawFastVLine(fb_aux, x, y, h, color);
    fb_gfx_drawFastVLine(fb_aux, x+w-1, y, w, color);*/
    //delete[] reg;
  
  //heap_caps_free(jpg_buf);
  //jpg_buf = NULL;
  //jpg_buf = (uint8_t *) heap_caps_malloc(fb2->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  jpg_buf = (uint8_t *) ps_malloc(fb2->len);
  memcpy(jpg_buf,fb2->buf,fb2->len);
  printf("\n controle4 \n");
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  jpg_chunking_t jchunk = {req, 0};
  //res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
  res = fmt2jpg_cb(fb_aux->data, len, fb_aux->width, fb_aux->height, PIXFORMAT_GRAYSCALE, 90, jpg_encode_stream, &jchunk);
  printf("\n controle5 \n");
  
  esp_camera_fb_return(fb2);
  free(subtraction_buffer);
  free(fb_aux); 
  }
  cont++;
 
  return res;
}

/*for (size_t i = 0; i < 20; i+=2) {
    // Cada pixel em RGB565 ocupa 2 bytes
    uint16_t pixel = (fb->buf[i] << 8) | fb->buf[i+1];
    
    // Divida o valor do pixel RGB565 em R, G, B
    uint8_t r = (pixel >> 11) & 0x1F;
    uint8_t g = (pixel >> 5) & 0x3F;
    uint8_t b = pixel & 0x1F;

    // Agora você tem os valores de R, G e B para o pixel
    Serial.print("R: "); Serial.print(r);
    Serial.print(" G: "); Serial.print(g);
    Serial.print(" B: "); Serial.println(b);
  }*/
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t subtraction_uri = {
    .uri = "/subtraction",
    .method = HTTP_GET,
    .handler = capture_and_subtract_handler5/*capture_two_frames_handler1*/,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif

  };
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
  recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");

  // load ids from flash partition
  recognizer.set_ids_from_flash();
#endif
  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &subtraction_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash(int pin) {
#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcAttach(pin, 5000, 8);
#else
  log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
}

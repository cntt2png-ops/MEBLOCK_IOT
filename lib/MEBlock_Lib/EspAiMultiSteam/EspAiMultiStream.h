#ifndef ESP_AI_MULTI_STREAM_H
#define ESP_AI_MULTI_STREAM_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include <driver/i2s.h>


#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#define I2S_SCK 14
#define I2S_WS  21
#define I2S_SD  2
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000


class EspAiMultiStream {
public:
    EspAiMultiStream();
    void beginWiFi(const char* ssid, const char* pass);
    bool beginCamera(String tmCamModelLink, bool flipV = false, bool flipH = false);
    bool beginAudio(String tmAudioModelLink);
    void beginServer();
    void loop();

    int getCamAiResult();
    int getAudioAiResult();

    int _latestCamResult;
    int _latestAudioResult;

private:
    AsyncWebServer server;
    AsyncWebSocket wsCamera; 
    AsyncWebSocket wsAudio; 

    bool _cameraEnabled;
    bool _audioEnabled;
    
    String _tmCamModelLink;
    String _tmAudioModelLink;

    unsigned long _lastCamTime; 
};

#endif
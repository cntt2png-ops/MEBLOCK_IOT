#ifndef ESP_AI_MULTI_STREAM_H
#define ESP_AI_MULTI_STREAM_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include <driver/i2s.h>

class EspAiMultiStream {
public:
    EspAiMultiStream();
    void beginWiFi(const char* ssid, const char* pass);
    bool beginCamera(String tmCamModelLink, bool flipV = false, bool flipH = false);
    bool beginAudio(String tmAudioModelLink);
    void beginServer();
    void loop();

    String getCamAiResult();
    int getCamAiScore();
    
    String getAudioAiResult();
    int getAudioAiScore();

    String _latestCamResult;
    int _latestCamScore;
    String _latestAudioResult;
    int _latestAudioScore;

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
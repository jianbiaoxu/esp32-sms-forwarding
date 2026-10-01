#pragma once

#include <ESPAsyncWebServer.h>

void modemsPostController(AsyncWebServerRequest* request, uint8_t* data,
                          size_t len, size_t index, size_t total);

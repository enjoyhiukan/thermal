#include "esphome.h"
#include "Melopero_AMG8833.h"
#include "math.h"
#include "ESPAsyncWebServer.h"
#include "ArduinoJson.h"
#include "ESPAsyncTCP.h"
#include <iostream>
#include <sstream>
#include <string>

using namespace esphome;

template < typename Type > std::string to_str (const Type & t)
{
  std::ostringstream os;
  os << t;
  return os.str ();
}

static const int SOBEL_X[3][3] =
    { { -1, 0, 1 },
      { -2, 0, 2 },
      { -1, 0, 1 } };

static const int SOBEL_Y[3][3] =
  { { -1, -2, -1 },
    { 0,  0,  0 },
    { 1,  2,  1 } };

static const float THRESHOLD = 4.0;
std::string heatmap;
std::string heatmap_complete;
float temps[8][8];

class AMG8833Sensor : public PollingComponent{
 Sensor *device_temp_ {nullptr};
 BinarySensor *occupancy_ {nullptr};
 
 //TextSensor *heatmap_a_ {nullptr};
 //TextSensor *heatmap_b_ {nullptr};
 
 public:
  // constructor
  //GridEYE grideye;

  Melopero_AMG8833 sensor;
  AsyncWebServer server;

  //AMG8833Sensor(Sensor *device_temp, BinarySensor *occupancy, TextSensor *heatmap_a, TextSensor *heatmap_b) : device_temp_(device_temp), occupancy_(occupancy), heatmap_a_(heatmap_a), heatmap_b_(heatmap_b), server(81) {}
  AMG8833Sensor(Sensor *device_temp, BinarySensor *occupancy) : device_temp_(device_temp), occupancy_(occupancy), server(81) {}
  float get_setup_priority() const override { return esphome::setup_priority::BUS; }
  
  void setup() override {
    // This will be called by App.setup()
    //grideye.begin();
    this->set_update_interval(1000);
    ESP_LOGD("AMG8833", "Resetting sensor ... ");
    int statusCode = sensor.resetFlagsAndSettings();
    ESP_LOGD("AMG8833", sensor.getErrorDescription(statusCode).c_str());

    ESP_LOGD("AMG8833", "Setting FPS ... ");
    statusCode = sensor.setFPSMode(FPS_MODE::FPS_10);
    ESP_LOGD("AMG8833", sensor.getErrorDescription(statusCode).c_str());

    serverInit();
    
  }
  void update() override {
    // This will be called every "update_interval" milliseconds.
    float gradients[8][8];
    bool state;
       
    readTemps();
    //applySobel(temps, gradients);
    applySobel(gradients);
    state = meetsThreshold(gradients);
    int statusCode = sensor.updateThermistorTemperature();
    ESP_LOGD("AMG8833", sensor.getErrorDescription(statusCode).c_str());

    device_temp_->publish_state(sensor.thermistorTemperature);
    occupancy_->publish_state(state);
    //heatmap_->publish_state(heatmap);
    //heatmap_a_->publish_state("23.00,22.75,22.25,23.25,23.25,23.50,23.50,23.50,22.75,23.00,23.00,23.00,23.00,22.75,22.75,22.75,23.25,22.75,23.00,22.75,22.50,23.25,23.50,23.25,22.50,22.75,22.75,22.75,23.00,23.00,22.50,22.50");
    //heatmap_b_->publish_state("22.00,22.50,22.75,22.25,22.75,22.75,23.25,22.50,21.75,22.50,22.50,22.75,22.50,22.50,22.50,23.00,22.50,22.25,21.75,21.75,22.50,22.50,22.75,23.00,21.75,22.50,22.00,21.50,22.25,22.00,22.50,23.00");
    //heatmap_a_->publish_state("23.00,22.75,22.25,23.25,23.25,23.50,23.50,23.50,22.75,23.00,23.00,23.00,23.00,22.75,22.75,22.75,23.25,22.75,23.00,22.75,22.50,23.25,23.50,23.25,22.50,22.75,22.75,22.75,23.00,23.00,22.50,22.50,22.00,22.50,22.75,22.25,22.75,22.75,23.25,22.50,21.75,22.50,22.50,22.75,22.50,22.50,22.50,23.00,22.50,22.25,21.75,21.75,22.50,22.50,22.75,23.00,21.75,22.50,22.00,21.50,22.25,22.00,22.50,23.00");
    // HTTP server
    //server.handleClient();

  }

private:
// sub-routines
  void readTemps() {
    int statusCode = sensor.updatePixelMatrix();
    ESP_LOGD("AMG8833", sensor.getErrorDescription(statusCode).c_str());
    /*for (int x = 0; x < 8; x++){
      for (int y = 0; y < 8; y++){
        temps[y][x] = sensor.pixelMatrix[y][x];
      }
    }*/
    memmove( temps, sensor.pixelMatrix, sizeof(temps) );
    //temps = sensor.pixelMatrix;
  }

  //void applySobel(float temps[8][8], float gradients[8][8]) {
  void applySobel(float gradients[8][8]) {
    //int statusCode = sensor.updatePixelMatrix();
    //ESP_LOGD("AMG8833", sensor.getErrorDescription(statusCode).c_str());
    for (int y = 0; y < 8; y++) {
      for (int x = 0; x < 8; x++) {
        float magX = 0;
        float magY = 0;
        float gradient = 0;


        for (int yk = 0; yk < 3; yk++) {
          for (int xk = 0; xk < 3; xk++) {
            magX += temps[clamp(y + yk - 1, 0, 7)][clamp(x + xk - 1, 0, 7)] * SOBEL_X[yk][xk];
            magY += temps[clamp(y + yk - 1, 0, 7)][clamp(x + xk - 1, 0, 7)] * SOBEL_Y[yk][xk];
          }
        }

        gradients[y][x] = sqrt(pow(magX, 2) + pow(magY, 2));
        if ((y==0)&&(x==0)){
          heatmap = to_str (temps[y][x]);
        } else{
          heatmap += ",";
          heatmap += to_str (temps[y][x]);
        }
      }
    }
    heatmap_complete = heatmap;
  }

  bool meetsThreshold(float values[8][8]) {
    for (int y = 0; y < 8; y++){
      for (int x = 0; x < 8; x++){
        if (values[y][x] >= THRESHOLD) {
          return true;
        }
      }
    }

    return false;
  }

  int clamp(int d, int min, int max) {
    const int t = d < min ? min : d;
    return t > max ? max : t;
  }

  /*void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  }*/
  
  void serverInit() {
    server.on("/raw", HTTP_GET, [](AsyncWebServerRequest *request){
        StaticJsonBuffer<512> doc;
        JsonObject& obj = doc.createObject();
        obj["sensor"] = "AMG8833";
        obj["rows"] = 8;
        obj["cols"] = 8;
        obj["data"] = heatmap_complete;
        //heatmap;
        String output;
        obj.printTo(output);
        request->send(200, "application/json", output.c_str());
    });
    
    //server.onNotFound(notFound);
    server.begin();
  }

};
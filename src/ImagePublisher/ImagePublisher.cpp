#include "ImagePublisher.h"
#include <img_converters.h>
#include <ros.h>
#include <sensor_msgs/CompressedImage.h>


ImagePublisher::ImagePublisher() : publisher(nullptr), image(nullptr), cameraConfig(nullptr), buffer(nullptr), jpgBuffer(nullptr), jpgBufferLength(0) {
}

ImagePublisher::~ImagePublisher() {
  freeBuffers();

  if (publisher) {
    delete publisher;
    publisher = nullptr;
  }
  if (image) {
    delete image;
    image = nullptr;
  }
}

void ImagePublisher::init(ros::NodeHandle &node, const char *topic, camera_config_t &cameraConfig) {
  this->cameraConfig = &cameraConfig;
  setupCamera();

  image = new sensor_msgs::CompressedImage();
  image->format = "JPEG/COMPRESSED";

  publisher = new ros::Publisher(topic, image);
  node.advertise(*publisher);
}

void ImagePublisher::publish() {

  int32_t status = captureImage();
  if (status == ESP_OK) {
    image->data = jpgBuffer;
    image->data_length = jpgBufferLength;
    publisher->publish(image);
  }

  freeBuffers();
}

void ImagePublisher::setupCamera() {

  // camera init
  esp_err_t err = esp_camera_init(cameraConfig);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    delay(500);
    ESP.restart();
    return;
  }

  // What to do with this?
  sensor_t *sensor = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (sensor->id.PID == OV3660_PID) {
    sensor->set_vflip(sensor, 1);//flip it back
    sensor->set_brightness(sensor, 1);//up the blightness just a bit
    sensor->set_saturation(sensor, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
}

esp_err_t ImagePublisher::captureImage() {
  esp_err_t status = ESP_OK;
  buffer = esp_camera_fb_get();

  if (!buffer) {
    Serial.println("Camera capture failed");
    status = ESP_FAIL;

  } else if (buffer->format == PIXFORMAT_JPEG) {

//    Serial.println("buffer jpeg");
    jpgBufferLength = buffer->len;
    jpgBuffer = buffer->buf;

  } else {

    Serial.println("buffer raw");
    bool converted = frame2jpg(buffer, 80, &jpgBuffer, &jpgBufferLength);

    esp_camera_fb_return(buffer);
    buffer = nullptr;

    if (!converted) {
      Serial.println("JPEG compression failed");
      status = ESP_FAIL;
    }

  }

  if (status == ESP_OK) {
//    Serial.printf("buffer length: %d\n", jpgBufferLength);
  }
  return status;
}


void ImagePublisher::freeBuffers() {
  if (buffer) {
    esp_camera_fb_return(buffer);
    buffer = nullptr;
    jpgBuffer = nullptr;
  } else if (jpgBuffer) {
    free(jpgBuffer);
    jpgBuffer = nullptr;
  }
}

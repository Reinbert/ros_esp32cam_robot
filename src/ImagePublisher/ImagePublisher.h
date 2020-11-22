#ifndef ROS_IMAGEPUBLISHER_H
#define ROS_IMAGEPUBLISHER_H

#include <Arduino.h>
#include <esp_camera.h>
#include <ros.h>

// Forward declare ROS classes
namespace sensor_msgs {
class CompressedImage;
}

class ImagePublisher {
public:
  ImagePublisher();
  virtual ~ImagePublisher();
  void init(ros::NodeHandle &node, const char *topic, camera_config_t &cameraConfig);
  void publish();

protected:

  void setupCamera();
  esp_err_t captureImage();
  void freeBuffers();


  ros::Publisher *publisher;
  sensor_msgs::CompressedImage *image;

  camera_config_t *cameraConfig;
  camera_fb_t *buffer;
  uint8_t *jpgBuffer;
  size_t jpgBufferLength;
};

#endif

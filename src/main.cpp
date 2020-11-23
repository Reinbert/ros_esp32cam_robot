#include <Arduino.h>
#include <cmath>
#include <WiFi.h>
#include <IPAddress.h>
#include <esp_camera.h>
#include <DateTime.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Timer.h>
#include <Logger.h>
#include "ImagePublisher/ImagePublisher.h"

// Include correct camera pins
#define CAMERA_MODEL_AI_THINKER
#include <camera_pins.h>

// PINS
#define LED_FLASH 4
#define PIN_FORWARD 12
#define PIN_BACKWARD 13
#define PIN_SERVO 15

// PWM Channels
#define PWM_CHANNEL_FORWARD LEDC_CHANNEL_2
#define PWM_CHANNEL_BACKWARD LEDC_CHANNEL_3
#define PWM_CHANNEL_SERVO LEDC_CHANNEL_4

// MIN and MAX values for PWM
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_16_BIT

// These values are determined by experiment and may differ on your system
#define PWM_MOTOR_MIN 5000    // The value where the motor starts moving
#define PWM_MOTOR_MAX 65535   // Full on
#define PWM_SERVO_LEFT 2100   // 90° left
#define PWM_SERVO_RIGHT 7800  // 90° right


// You can also define your wifi access in 'platformio.ini' or by setting environment variables.
// https://docs.platformio.org/en/latest/projectconf/section_env_build.html#build-flags
// https://docs.platformio.org/en/latest/envvars.html#envvar-PLATFORMIO_BUILD_FLAGS
// If you do, be sure to escape the quotes (\"), e.g.: -D WIFI_SSID=\"YOUR_SSID\"
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif


namespace esp32cam {

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

// ROS
IPAddress serialServer(10, 0, 1, 10);
ros::NodeHandle *node;
ros::Subscriber<geometry_msgs::Twist> *cmdvelSub;
ros::Subscriber<std_msgs::Bool> *flashSub;
ros::Publisher *fpsPub;
ros::Publisher *logPub;
ros::Publisher *streamPub;
std_msgs::Float32 *fpsMsg;
std_msgs::String *logMsg;
//sensor_msgs::Image *streamMsg;

// ROS topics
String chipId = "ESP32_";
String cmdVelTopic; // chipId + /cmd_vel
String streamTopic; // chipId + /stream
String flashTopic;  // chipId + /flash
String fpsTopic;    // chipId + /fps
String logTopic;    // chipId + /log

camera_config_t cameraConfig;
ImagePublisher *imagePublisher;

// Variables
bool wifiConnected = false;
bool rosConnected = false;
bool movement = false;
float linear, angular = 0;

// Timing
#define AVERAGE_ALPHA 0.7f
float frameDuration = 0;
uint32_t lastFrameTime = 0;
uint32_t lastCmdVelMessage = 0;

// Tickers
//#define MAX_CMD_VEL_INTERVAL 1000 // If there are no new CMD_VEL message during this interval, the robot stops

// Function definitions
void onCmdVel(const geometry_msgs::Twist &msg);
void onFlash(const std_msgs::Bool &msg);
void stop();

void handleMovement();
void checkConnection();

void publishImage();
void publishFps();
void publishLog(const char *format, ...);

float fmap(float val, float in_min, float in_max, float out_min, float out_max);


// Setup functions
// -------------------------------------------------------------------

void setupPins() {
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Flash LED
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW);

  // Forward and backward PWM channels
  ledcSetup(PWM_CHANNEL_FORWARD, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BACKWARD, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_SERVO, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(PIN_FORWARD, PWM_CHANNEL_FORWARD);
  ledcAttachPin(PIN_BACKWARD, PWM_CHANNEL_BACKWARD);
  ledcAttachPin(PIN_SERVO, PWM_CHANNEL_SERVO);

  ledcWrite(PWM_CHANNEL_FORWARD, 0);
  ledcWrite(PWM_CHANNEL_BACKWARD, 0);
  ledcWrite(PWM_CHANNEL_SERVO, (PWM_SERVO_RIGHT + PWM_SERVO_LEFT) / 2);
}

void setupSerial() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();
  delay(500);
}

void setupWifi() {
  // Setting persistent to false disables writing wifi credentials to EEPROM everytime.
  WiFi.persistent(false);
  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  chipId.concat(WiFi.macAddress());
  chipId.replace(":", "");
  Serial.printf("ChipId: %s\n", chipId.c_str());

  DateTimeClass time(BUILD_TIME);
  Serial.printf("Build:  %s\n", time.toISOString().c_str());
}

void setupCamera() {
  cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  cameraConfig.ledc_timer = LEDC_TIMER_0;
  cameraConfig.pin_d0 = Y2_GPIO_NUM;
  cameraConfig.pin_d1 = Y3_GPIO_NUM;
  cameraConfig.pin_d2 = Y4_GPIO_NUM;
  cameraConfig.pin_d3 = Y5_GPIO_NUM;
  cameraConfig.pin_d4 = Y6_GPIO_NUM;
  cameraConfig.pin_d5 = Y7_GPIO_NUM;
  cameraConfig.pin_d6 = Y8_GPIO_NUM;
  cameraConfig.pin_d7 = Y9_GPIO_NUM;
  cameraConfig.pin_xclk = XCLK_GPIO_NUM;
  cameraConfig.pin_pclk = PCLK_GPIO_NUM;
  cameraConfig.pin_vsync = VSYNC_GPIO_NUM;
  cameraConfig.pin_href = HREF_GPIO_NUM;
  cameraConfig.pin_sscb_sda = SIOD_GPIO_NUM;
  cameraConfig.pin_sscb_scl = SIOC_GPIO_NUM;
  cameraConfig.pin_pwdn = PWDN_GPIO_NUM;
  cameraConfig.pin_reset = RESET_GPIO_NUM;
  cameraConfig.xclk_freq_hz = 20000000;
  cameraConfig.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    cameraConfig.frame_size = FRAMESIZE_UXGA;
    cameraConfig.jpeg_quality = 10;
    cameraConfig.fb_count = 2;
  } else {
    cameraConfig.frame_size = FRAMESIZE_SVGA;
    cameraConfig.jpeg_quality = 12;
    cameraConfig.fb_count = 1;
  }
}

void setupTopics() {

  cmdVelTopic = chipId + "/cmd_vel";
  flashTopic = chipId + "/flash";
  fpsTopic = chipId + "/fps";
  logTopic = chipId + "/log";
  streamTopic = chipId + "/stream";
}

void setupRos() {
  Serial.printf("ROS serial server: %s\n", serialServer.toString().c_str());

  // Connect to ros-serial socket server and init node. (Using default port of 11411)
  node = new ros::NodeHandle();
  node->getHardware()->setConnection(serialServer);
  node->initNode();

  // Subscribers
  cmdvelSub = new ros::Subscriber<geometry_msgs::Twist>(cmdVelTopic.c_str(), &onCmdVel);
  node->subscribe(*cmdvelSub);

  flashSub = new ros::Subscriber<std_msgs::Bool>(flashTopic.c_str(), &onFlash);
  node->subscribe(*flashSub);

  // Publishers
  fpsMsg = new std_msgs::Float32();
  fpsPub = new ros::Publisher(fpsTopic.c_str(), fpsMsg);
  node->advertise(*fpsPub);

  logMsg = new std_msgs::String();
  logPub = new ros::Publisher(logTopic.c_str(), logMsg);
  node->advertise(*logPub);

  imagePublisher = new ImagePublisher();
  imagePublisher->init(*node, streamTopic.c_str(), cameraConfig);
}

void setupTickers() {
  Timer.setInterval(&Logger, 5, -1, -10);
  Timer.setInterval(checkConnection, 1000);
  Timer.setInterval(handleMovement, 10);
  Timer.setInterval(publishFps, 1000);
  Timer.setInterval(publishImage, 33);
}

// ROS callbacks
// -------------------------------------------------------------------

void onFlash(const std_msgs::Bool &msg) {
  digitalWrite(LED_FLASH, msg.data); // false -> off, true -> on
  publishLog("Flash: %s", msg.data ? "ON" : "OFF");
}

// Receive messages and store them. They are handled once per frame in main loop.
void onCmdVel(const geometry_msgs::Twist &msg) {
  // Cap values at [-1 .. 1]
  linear = constrain(msg.linear.x, -1, 1);
  angular = constrain(msg.angular.z, -1, 1);

  lastCmdVelMessage = millis();
  movement = true;
}

// Robot control
// -------------------------------------------------------------------
// Stops in next frame
void stop() {
  linear = 0;
  movement = true;
}

// RC car drive:
// linear controls forward and backward movement of motor
// angular controls the steering servo
void handleMovement() {
#ifdef MAX_CMD_VEL_INTERVAL
  // If defined, interval between two cmd_vel messages must not exceed this value or else the robot stops
  if (millis() > lastCmdVelMessage + MAX_CMD_VEL_INTERVAL)
    stop();
#endif

  if (!movement)
    return;

  auto pwmMotor = (uint16_t) fmap(std::fabs(linear), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
  auto pwmServo = (uint16_t) fmap(angular, -1, 1, PWM_SERVO_RIGHT, PWM_SERVO_LEFT);
  bool forward = linear > 0;
  bool backward = linear < 0;

  ledcWrite(PWM_CHANNEL_FORWARD, pwmMotor * forward);
  ledcWrite(PWM_CHANNEL_BACKWARD, pwmMotor * backward);

  // Don't allow steering while there's no movement because the robot can't handle it well.
  if (forward || backward) {
    ledcWrite(PWM_CHANNEL_SERVO, pwmServo);
  }

  publishLog("%d, %d, %d, %d", pwmMotor, pwmServo, forward, backward);
  movement = false;
}

// ROS
// -------------------------------------------------------------------

void publishImage() {
  if (rosConnected) {
    imagePublisher->publish();
  }
}

void publishFps() {
  if (frameDuration > 0) {
    fpsMsg->data = 1000.0f / frameDuration;
    fpsPub->publish(fpsMsg);
//    Serial.printf("%f\n", fpsMsg->data);
  }
}

void publishLog(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, 256, format, args);
  va_end(args);
  logMsg->data = buffer;
  logPub->publish(logMsg);
  Serial.printf("%s\n", buffer);
}

void measureFramerate() {
  uint32_t now = millis();
  uint32_t duration = now - lastFrameTime;
  lastFrameTime = now;

  // Calculate Exponential moving average of frame time
  // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
  frameDuration = duration * AVERAGE_ALPHA + (frameDuration * (1 - AVERAGE_ALPHA));
}

// Check connection functions
// -------------------------------------------------------------------

bool checkWifi() {
  if (wifiConnected) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      stop();

      Serial.println("WiFi disconnected");
    }
  } else {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;

      Serial.println("WiFi connected");
      Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
      Serial.printf("IP:   %s\n", WiFi.localIP().toString().c_str());
    }
  }
  return wifiConnected;
}

bool checkROS() {

  if (rosConnected) {
    if (!node->connected()) {
      rosConnected = false;
      stop();

      Serial.println("ROS disconnected");
      digitalWrite(LED_BUILTIN, !rosConnected); // false -> on, true -> off
    }
  } else {
    if (node->connected()) {
      rosConnected = true;

      Serial.println("ROS connected");
      digitalWrite(LED_BUILTIN, !rosConnected); // false -> on, true -> off
    }
  }
  return rosConnected;
}

void checkConnection() {
  if (checkWifi()) {
    checkROS();
  }
}

// Main functions
// -------------------------------------------------------------------

void setup() {
  setupPins();
  setupSerial();
  setupWifi();
  setupCamera();
  setupTopics();
  setupRos();
  setupTickers();
}

// Note: regular loop function gets called way too fast, so we run our own loop here
void loop() {
  measureFramerate();
  node->spinOnce();
  Timer.loop();
}


// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
}

void setup() {
  esp32cam::setup();
}

void loop() {
  esp32cam::loop();
}

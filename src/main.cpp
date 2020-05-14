#include <Arduino.h>
#include <WiFi.h>
#include <IPAddress.h>
#include <esp_camera.h>
#include <cmath>
#include <ESP32Ticker.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

// Include correct camera pins
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// PINS
#define LED_BUILTIN 33
#define LED_FLASH 4
#define PIN_FORWARD 12
#define PIN_BACKWARD 13
#define PIN_SERVO 15

// PWM Channels
#define PWM_CHANNEL_FORWARD 0
#define PWM_CHANNEL_BACKWARD 1
#define PWM_CHANNEL_SERVO 2

// MIN and MAX values for PWM
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16

// These values are determined by experiment and may differ on your system
#define PWM_MOTOR_MIN 5000    // The value where the motor starts moving
#define PWM_MOTOR_MAX 65535   // Full on
#define PWM_SERVO_LEFT 2100   // 90° left
#define PWM_SERVO_RIGHT 7800  // 90° right

// Debugging defines
#define ENABLE_CAMERA


// You can also define your wifi access via platformio.ini or environment variables
// https://docs.platformio.org/en/latest/projectconf/section_env_build.html#build-flags
// https://docs.platformio.org/en/latest/envvars.html#envvar-PLATFORMIO_BUILD_FLAGS
// If you do, be sure to escape the quotes (\"), e.g.: -D WIFI_SSID=\"YOUR_SSID\"
#ifndef WIFI_SSID
  #define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS "YOUR_PASSWORD"
#endif

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
sensor_msgs::Image *stream;

// ROS topics
char id[64];
char cmdvelTopic[64]; // id + /cmd_vel
char streamTopic[64]; // id + /stream
char flashTopic[64];  // id + /flash
char fpsTopic[64];    // id + /fps
char logTopic[64];    // id + /log

bool connected = false;
bool movement = false;
float linear, angular = 0;

// Timing
#define AVERAGE_ALPHA 0.7f
float frameDuration = 0;
unsigned long lastFrameTime = 0;
unsigned long lastCmdVelMessage = 0;

// Tickers
//#define MAX_CMD_VEL_INTERVAL 1000 // If there are no new CMD_VEL message during this interval, the robot stops
#define FPS_PUBLISH_TIME 1          // in seconds
#define LOOP_TIME 0.01              // Loop runs way too fast on ESP32. Running the loop every 10 ms is enough
Ticker *fpsTicker;
Ticker *loopTicker;

// Function definitions
void startCameraServer();
void onCmdVel(const geometry_msgs::Twist &msg);
void onFlash(const std_msgs::Bool &msg);
void stop();
void publishFps();
void publishLog(const char *format, ...);
void _loop();
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

  // Debug
//  Serial.println(ssid);
//  Serial.println(password);

  Serial.println("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWifi connected");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   http://");
  Serial.println(WiFi.localIP());
}

void setupTopics() {
  uint64_t mac = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  snprintf(id, 64, "ESP32_%04X%08X", (uint16_t) (mac >> 32u), (uint32_t) mac);
//  snprintf(cmdvelTopic, 64, "/cmd_vel");
  snprintf(cmdvelTopic, 64, "%s/cmd_vel", id);
  snprintf(flashTopic, 64, "%s/flash", id);
  snprintf(fpsTopic, 64, "%s/fps", id);
  snprintf(logTopic, 64, "%s/log", id);
  snprintf(streamTopic, 64, "%s/stream", id);
  Serial.print("ID:   ");
  Serial.println(id);
}

void setupRos() {
  Serial.print("Connecting to ROS serial server @ ");
  Serial.println(serialServer.toString());

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  node = new ros::NodeHandle();
  node->getHardware()->setConnection(serialServer);
  node->initNode();

  // Subscribers
  cmdvelSub = new ros::Subscriber<geometry_msgs::Twist>(cmdvelTopic, &onCmdVel);
  node->subscribe(*cmdvelSub);

  flashSub = new ros::Subscriber<std_msgs::Bool>(flashTopic, &onFlash);
  node->subscribe(*flashSub);

  // Publishers
  fpsMsg = new std_msgs::Float32();
  fpsPub = new ros::Publisher(fpsTopic, fpsMsg);
  node->advertise(*fpsPub);

  logMsg = new std_msgs::String();
  logPub = new ros::Publisher(logTopic, logMsg);
  node->advertise(*logPub);

//  stream = new sensor_msgs::Image();
//  streamPub = new ros::Publisher(streamTopic, stream);
//  node->advertise(*streamPub);
}

void setupCamera() {

#ifdef ENABLE_CAMERA

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

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }


  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(500);
    ESP.restart();
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#endif
}

void setupTickers() {
  fpsTicker = new Ticker();
  fpsTicker->attach(FPS_PUBLISH_TIME, publishFps);

  loopTicker = new Ticker();
  loopTicker->attach(LOOP_TIME, _loop);
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


// Loop functions (run every frame)
// -------------------------------------------------------------------

bool rosConnected() {
  // If value changes, notify via LED and console.
  bool conn = node->connected();
  if (connected != conn) {
    connected = conn;
    if (!connected)
      stop();

    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    publishLog(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

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

void measureFramerate() {
  auto now = millis();
  auto duration = now - lastFrameTime;
  lastFrameTime = now;

  // Calculate Exponential moving average of frame time
  // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
  frameDuration = duration * AVERAGE_ALPHA + (frameDuration * (1 - AVERAGE_ALPHA));
}

// Publish functions
// -------------------------------------------------------------------

void publishFps() {
  if (frameDuration > 0) {
    fpsMsg->data = 1000.0f / frameDuration;
    fpsPub->publish(fpsMsg);
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

// Main functions
// -------------------------------------------------------------------

void setup() {
  setupPins();
  setupSerial();
  setupWifi();
  setupCamera();
  setupTopics();
  setupRos();

#ifdef ENABLE_CAMERA
  startCameraServer();
#endif
  setupTickers();
}

// Note: regular loop function gets called way too fast, so we run our own loop here
void _loop() {
  measureFramerate();
  node->spinOnce();
  rosConnected();
  handleMovement();
}

void loop() { }

// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
  float divisor = (in_max - in_min);
  if (divisor == 0) {
    return -1; //AVR returns -1, SAM returns 0
  }
  return (val - in_min) * (out_max - out_min) / divisor + out_min;
}

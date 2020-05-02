#include <Arduino.h>
#include <WiFi.h>
#include <IPAddress.h>
#include <esp_camera.h>
#include <cmath>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Include correct camera pins
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// PINS
#define LED_BUILTIN 33
#define LED_FLASH 4
#define PIN_FORWARD 2
#define PIN_BACKWARD 14
#define PIN_SERVO 15

// PWM Channels
#define PWM_CHANNEL_FORWARD 0
#define PWM_CHANNEL_BACKWARD 1
#define PWM_CHANNEL_SERVO 2

// MIN and MAX values for PWM
#define PWM_MOTOR_MIN 0
#define PWM_MOTOR_MAX 255
#define PWM_SERVO_MIN 9
#define PWM_SERVO_MAX 30

// If there are no new CMD_VEL message during this interval, the robot stops
//#define MAX_CMD_VEL_INTERVAL 1000


// Define your own credentials via platformio.ini
const char *ssid = "SSID";
const char *password = "PASSWORD";


// ROS
IPAddress serialServer(192, 168, 0, 12);
ros::NodeHandle *node;
ros::Subscriber<geometry_msgs::Twist> *cmdvelSub;
ros::Subscriber<std_msgs::Bool> *flashSub;
ros::Publisher *idPub;
ros::Publisher *streamPub;
std_msgs::String *info;
sensor_msgs::Image *stream;

// ROS topics
char id[19];
char cmdvelTopic[19 + 8]; // id + /cmd_vel
char streamTopic[19 + 7]; // id + /stream
char flashTopic[19 + 6];  // id + /flash

bool connected = false;
bool movement = false;
float linear, angular = 0;
uint64_t lastCmdVelMessage = 0;

// Function definitions
void onCmdVel(const geometry_msgs::Twist &msg);
void onFlash(const std_msgs::Bool &msg);
void stop();
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
  ledcSetup(PWM_CHANNEL_FORWARD, 50, 8);
  ledcSetup(PWM_CHANNEL_BACKWARD, 50, 8);
  ledcSetup(PWM_CHANNEL_SERVO, 50, 8);

  ledcAttachPin(PIN_FORWARD, PWM_CHANNEL_FORWARD);
  ledcAttachPin(PIN_BACKWARD, PWM_CHANNEL_BACKWARD);
  ledcAttachPin(PIN_SERVO, PWM_CHANNEL_SERVO);

  ledcWrite(PWM_CHANNEL_FORWARD, 0);
  ledcWrite(PWM_CHANNEL_BACKWARD, 0);
  ledcWrite(PWM_CHANNEL_SERVO, (PWM_SERVO_MIN + PWM_SERVO_MAX) / 2);
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

void setupRos() {
  Serial.print("Connecting to ROS serial server @ ");
  Serial.println(serialServer.toString());

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  node = new ros::NodeHandle();
  node->getHardware()->setConnection(serialServer);
  node->initNode();

  cmdvelSub = new ros::Subscriber<geometry_msgs::Twist>(cmdvelTopic, &onCmdVel);
  node->subscribe(*cmdvelSub);

  flashSub = new ros::Subscriber<std_msgs::Bool>(flashTopic, &onFlash);
  node->subscribe(*flashSub);

//  info = new std_msgs::String();
//  idPub = new ros::Publisher(id, info);
//  node->advertise(*idPub);
//
//  stream = new sensor_msgs::Image();
//  streamPub = new ros::Publisher(streamTopic, stream);
//  node->advertise(*streamPub);
}

void setupCamera() {

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
}

void getROSTopics() {
  uint64_t mac = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  snprintf(id, 19, "ESP32_%04X%08X", (uint16_t) (mac >> 32u), (uint32_t) mac);
  snprintf(cmdvelTopic, 19 + 8, "%s/cmd_vel", id);
  snprintf(streamTopic, 19 + 7, "%s/stream", id);
  snprintf(flashTopic, 19 + 6, "%s/flash", id);
  Serial.print("ID:   ");
  Serial.println(id);
}

// ROS callbacks
// -------------------------------------------------------------------

void onFlash(const std_msgs::Bool &msg) {
  digitalWrite(LED_FLASH, msg.data); // false -> off, true -> on
  Serial.printf("Flash: %s\n", msg.data ? "ON" : "OFF");
}

// Receive messages and store them. They are handled once per frame in main loop.
void onCmdVel(const geometry_msgs::Twist &msg) {
  // Cap values at [-1 .. 1]
  linear = constrain(msg.linear.x, -1, 1);
  angular = constrain(msg.angular.z, -1, 1);

  lastCmdVelMessage = millis();
  movement = true;
}


// Loop functions
// -------------------------------------------------------------------


bool rosConnected() {
  // If value changes, notify via LED and console.
  bool conn = node->connected();
  if (connected != conn) {
    connected = conn;
    if (!connected)
      stop();

    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
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

  auto pwmMotor = (uint8_t) fmap(std::fabs(linear), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
  auto pwmServo = (uint8_t) fmap(angular, -1, 1, PWM_SERVO_MIN, PWM_SERVO_MAX);

  ledcWrite(PWM_CHANNEL_FORWARD, pwmMotor * (linear > 0));
  ledcWrite(PWM_CHANNEL_BACKWARD, pwmMotor * (linear < 0));
  ledcWrite(PWM_CHANNEL_SERVO, pwmServo);
  Serial.printf("%f, %f, %d, %d\n", linear, angular, pwmMotor, pwmServo);

  movement = false;
}


// Main functions
// -------------------------------------------------------------------

void setup() {
  setupPins();
  setupSerial();
  setupWifi();
//  setupCamera();
  getROSTopics();
  setupRos();
//  startCameraServer();
}

void loop() {
  rosConnected();
  node->spinOnce();
  handleMovement();
}

// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
  float divisor = (in_max - in_min);
  if (divisor == 0) {
    return -1; //AVR returns -1, SAM returns 0
  }
  return (val - in_min) * (out_max - out_min) / divisor + out_min;
}




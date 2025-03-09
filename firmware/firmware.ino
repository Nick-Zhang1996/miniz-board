#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Servo.h>
#define PI 3.14159265

#include "isr_timer.h"
#include "network.h"
#include "packet.h"
#include "pwm.h"
#include "status_light.h"
#include "tfmini_s.h"

uint16_t dist1, dist2, dist3, dist4;

// front left right back
TFMiniS front_sensor(0x13);
TFMiniS left_sensor(0x12);
TFMiniS right_sensor(0x10);
TFMiniS back_sensor(0x11);
///////////////////

unsigned int localPort = 28840;
unsigned long last_packet_ts = 0;
int encoder_s_pin = 14;

Servo steerServo;

const int DRIVE_AIN1 = 4;
const int DRIVE_AIN2 = 5;
const int DRIVE_PWMA = 6;
const int DRIVE_STBY = 7;

volatile float throttle = 0.0;
// left positive, radians
volatile float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_pos = 117;       // steer setpoint @ full left
float full_left_angle = 20.0;    // steer angle @ full left
float full_right_pos = 63;       // steer setpoint @ full right
float full_right_angle = -20.0;  // steer angle @ full left
float failsafe_angle = 90;       // mid point

float steering_deadzone_rad = 1.0 / 180.0 * PI;
volatile unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;
long servo_ts = 0;
bool flag_failsafe = false;
StatusLed led;

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef _SAMD21_ADC_COMPONENT_
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
#endif

  Serial.begin(115200);

  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // Wait for 1 second before retrying
    Serial.print(".");
  }

  // Print network data once connected
  Serial.println("\nConnected to Wi-Fi!");
  printWifiData();
  printCurrentNet();

  // Start UDP communication
  Udp.begin(localPort);  // Begin UDP on the specified local port
  Serial.print("Local port: ");
  Serial.println(localPort);

  Wire.begin();
  Wire.setClock(400000);

  // Serial.println("\nTFMini-S I2C Reader");

  front_sensor.begin();
  left_sensor.begin();
  right_sensor.begin();
  back_sensor.begin();

  ///////////////////

  // Enable motor driver
  pinMode(DRIVE_AIN1, OUTPUT);
  pinMode(DRIVE_AIN2, OUTPUT);
  pinMode(DRIVE_PWMA, OUTPUT);
  pinMode(DRIVE_STBY, OUTPUT);
  digitalWrite(DRIVE_STBY, HIGH);
  steerServo.attach(10);

  pinMode(encoder_s_pin, INPUT);

  led.init();
  led.off();
  setupWifi();
  Udp.begin(localPort);

  PWM::setup();
}

void blinkTwice() {
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
}

void setupWifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }

  // while (status != WL_CONNECTED) {
  //   Serial.print("Attempting to connect to WPA SSID: ");
  //   Serial.println(ssid);
  //   status = WiFi.begin(ssid, pass);
  //   delay(10000);
  // }
  led.blink();

  printCurrentNet();
  printWifiData();
}

unsigned long periodic_print_1hz_ts = 0;

void loop() {
  static unsigned long lastRead = 0;

  if (millis() - lastRead >= 20) {  ///////////////////////////////////////////////////////////////////////////////   send/1s, need to change from 1000 to 10 for 100hz

    // Serial.println("\n--- Reading Sensors ---");

    if (front_sensor.readDistance(dist1)) {
      // Serial.print("Front Distance=");
      // Serial.print(dist1);
      // Serial.println(" cm");
    } else {
      Serial.println("Failed to read front sensor");
    }

    if (left_sensor.readDistance(dist2)) {
      // Serial.print("Left Distance=");
      // Serial.print(dist2);
      // Serial.println(" cm");
    } else {
      Serial.println("Failed to read left sensor");
    }

    if (right_sensor.readDistance(dist3)) {
      // Serial.print("Right Distance=");
      // Serial.print(dist3);
      // Serial.println(" cm");
    } else {
      Serial.println("Failed to read right sensor");
    }

    if (back_sensor.readDistance(dist4)) {
      // Serial.print("Back Distance=");
      // Serial.print(dist4);
      // Serial.println(" cm");
    } else {
      Serial.println("Failed to read back sensor");
    }

    lastRead = millis();
  }

  led.update();
  //  Serial.println(millis() - loop_time);
  //  loop_time = millis();
  int packet_size = Udp.parsePacket();

  // process incoming packet
  if (packet_size) {
    if (packet_size != PACKET_SIZE) {
      Serial.println("err packet size");
    }

    // Serial.print("From ");
    //IPAddress remoteIp = Udp.remoteIP();
    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE) {
      Serial.print("err reading packet size ");
      Serial.println(len);
    }
    // Serial.println("parsing packet");
    parsePacket();
    last_packet_ts = millis();
    flag_failsafe = false;
    led.on();
    // response is handled by packet parser
  }

  if (millis() - last_packet_ts > 100 && !flag_failsafe) {
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    // Serial.print(millis());
    // Serial.println(" failsafe");
    led.blink();
  }
  PIDControl();
}

// using global variable throttle and steering
// set pwm for servo and throttle
void actuateThrottle() {
  // throttle control
  if (abs(throttle) < throttle_deadzone) {
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
  } else {

    int pwmValue = abs(throttle) * 255;
    pwmValue = constrain(pwmValue, 0, 255);

    //    Serial.println(pwmValue);

    if (throttle > 0) {
      digitalWrite(DRIVE_AIN1, HIGH);
      digitalWrite(DRIVE_AIN2, LOW);
      analogWrite(DRIVE_PWMA, pwmValue);
    } else {
      digitalWrite(DRIVE_AIN1, LOW);
      digitalWrite(DRIVE_AIN2, HIGH);
      analogWrite(DRIVE_PWMA, pwmValue);
    }
  }
}

// for steering rack
void PIDControl() {
  float dt = (float)(micros() - last_pid_ts) / 1e6;
  last_pid_ts = micros();

  if (flag_failsafe) {
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
    steerServo.write(failsafe_angle);

    return;
  }

  actuateThrottle();

  float target_steer_deg = steering * 180. / PI;
  float target_pos = steeringPosition(target_steer_deg);
  float constrained_pos = constrain(target_pos, full_right_pos, full_left_pos);

  if (millis() - servo_ts > 20) {
    steerServo.write(constrained_pos);
    servo_ts = millis();

    //    Serial.println(target_pos);
  }
}

float steeringPosition(float steering_deg) {
  return fmap(steering_deg, full_right_angle, full_left_angle, full_right_pos, full_left_pos);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

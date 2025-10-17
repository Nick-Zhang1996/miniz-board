#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define PI 3.14159265

#include "network.h"
#include "packet.h"
#include "status_light.h"

// TODO use SAMD timer interrupt to achieve higher control frequency

unsigned int localPort = 2390;
unsigned long last_packet_ts = 0;

// board pin layout
int encoder_s_pin = 14;

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 2;
// right
int steer_fwd_pin = 3;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

// electronics calibration
// [-1,1]
float throttle = 0.0;
// left positive, radians
float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle_rad = 26.1/180.0*PI;
float full_right_angle_rad = -26.1/180.0*PI;
float full_left_encoder_value = 630.0;
float full_right_encoder_value = 410.0;
float steering_deadzone_rad = 1.0/180.0*PI;
unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;

bool flag_failsafe = false;
StatusLed led;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  pinMode(encoder_s_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);
  pinMode(drive_rev_pin, OUTPUT);
  pinMode(drive_fwd_pin, OUTPUT);
  led.init();
  led.off();
  setupWifi();
  Udp.begin(localPort);
}

void blinkTwice(){
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);

}

void setupWifi(){
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  led.blink();
  
  printCurrentNet();
  printWifiData();
}

int no_packet_count = 0;
int last_seq_no = 0;

void loop() {
  led.update();
//  Serial.println(millis() - loop_time);
//  loop_time = millis();
  unsigned long t0 = millis();
  int packet_size = Udp.parsePacket();
  if (millis()-t0 > 2){
    Serial.print("---- unexpected latency ---");
    Serial.println(millis()-t0);
  }
  
  // process incoming packet
  if (packet_size) {
    //Serial.print("no_packet_count =  ");
    //Serial.println(no_packet_count);
    no_packet_count = 0;

    if (packet_size != PACKET_SIZE){ Serial.println("err packet size"); }

    IPAddress remoteIp = Udp.remoteIP();

    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE){ Serial.println("err reading packet size"); }
    // NOTE no reply packet
    parsePacket();
    Packet *p = (Packet *) in_buffer;
    //Serial.print(" seq no ");
    //Serial.println(p->seq_no);
    Serial.print(".");
    if (p->seq_no - last_seq_no > 1){
      Serial.println("--------- missing packet -------");
      Serial.print("expected :");
      Serial.print(last_seq_no+1);
      Serial.print("actual :");
      Serial.print(p->seq_no);
    }
    last_seq_no = p->seq_no;
    last_packet_ts = millis();
    flag_failsafe = false;
    led.on();
    // response is handled by packet parser
  } else {
    no_packet_count ++;
  }
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



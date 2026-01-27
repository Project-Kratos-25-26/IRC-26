#include <micro_ros_arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "esp_system.h"
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32_multi_array.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int l1 = 0;
int l2 = 0;
int auger = 0;
int mixer = 0;
int plate = 0;
int p1 = 0;
int p2 = 0;
int p3 = 0;
int p4 = 0;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg0 = (const std_msgs__msg__Int32MultiArray*)msgin;
  l1=msg0->data.data[0]; //l1
  l2=msg0->data.data[1]; //l2
  mixer=msg0->data.data[2]; //mixer
  auger=msg0->data.data[3]; //auger
  p1=msg0->data.data[4]; //p1
  plate=msg0->data.data[5]; //plate
  p2=msg0->data.data[6]; //p2
  p3=msg0->data.data[7]; //p3
  p4=msg0->data.data[8]; //p4
  //digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  
}
int l1_dir=26;
int l1_speed=25;
int l2_dir=12;
int l2_speed=14;
int pump1_dir=4;
int pump1_speed=0;
int pump2_dir=23;//23 23 19 
int pump2_speed=19;//19 2 23
int pump3_dir=18;//18 18 2
int pump3_speed=2;//2 19 18  
int pump4_dir=16;
int pump4_speed=17;
int plate_dir=33;//
int plate_speed=32;
int auger_dir=27;
int auger_speed=13;
int mixer_dir=5;
int mixer_speed=15;
bool bevelEN=1;
bool isTopPressed=0;
bool isBotPressed=0;
bool isRightPressed=0;
bool isLeftPressed=0;

// #define LIMIT_TOP_PIN 21
// #define LIMIT_BOTTOM_PIN 22
// #define LIMIT_LEFT_PIN 34
// #define LIMIT_RIGHT_PIN 35

void wait_for_agent()
{
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) 
    delay(1000);
}
void setup() {
  set_microros_transports();
  
  pinMode(l1_dir,OUTPUT);
  pinMode(l1_speed, OUTPUT);
  pinMode(l2_dir, OUTPUT);
  pinMode(l2_speed, OUTPUT);
  pinMode(pump1_dir, OUTPUT);
  pinMode(pump1_speed, OUTPUT);
  pinMode(pump2_dir, OUTPUT);
  pinMode(pump2_speed, OUTPUT);
  pinMode(pump3_dir, OUTPUT);
  pinMode(pump3_speed, OUTPUT);
  pinMode(pump4_dir, OUTPUT);
  pinMode(pump4_speed, OUTPUT);
  pinMode(auger_dir, OUTPUT);
  pinMode(auger_speed, OUTPUT);
  pinMode(mixer_dir, OUTPUT);
  pinMode(mixer_speed, OUTPUT);
  pinMode(plate_dir,OUTPUT);
  pinMode(plate_speed,OUTPUT);
  // pinMode(LIMIT_TOP_PIN, INPUT_PULLUP);
  // pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);
  // pinMode(LIMIT_LEFT_PIN,INPUT);
  // pinMode(LIMIT_RIGHT_PIN,INPUT);

  analogWrite(l1_speed, 0);
  analogWrite(l2_speed, 0);
  analogWrite(pump1_speed, 0);
  analogWrite(pump2_speed, 0);
  analogWrite(pump3_speed, 0);
  analogWrite(pump4_speed, 0);
  analogWrite(mixer_speed, 0);
  analogWrite(auger_speed, 0);
  analogWrite(plate_speed, 0);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  msg.data.data = (int32_t*)malloc(9 * sizeof(int32_t)); 
  msg.data.size = 9;
  msg.data.capacity = 9;

  allocator = rcl_get_default_allocator();

  //create init_options
  wait_for_agent();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mymotors"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  //link1
  // isTopPressed=digitalRead(LIMIT_TOP_PIN);
  // isBotPressed=digitalRead(LIMIT_BOTTOM_PIN);
  // isLeftPressed=digitalRead(LIMIT_LEFT_PIN);
  // isRightPressed=digitalRead(LIMIT_RIGHT_PIN);

  if(l1>0)
  {
    digitalWrite(l1_dir,HIGH);
    analogWrite(l1_speed, l1);
  }
  else if(l1 == 0)
  {
    analogWrite(l1_speed, 0);
  }
  else
  {
    digitalWrite(l1_dir, LOW);
    analogWrite(l1_speed, -l1);
  }
  
  //link2
  if(l2>0)
  {
    digitalWrite(l2_dir,LOW);
    analogWrite(l2_speed,l2);
  }

  else if(l2 == 0)
  {
    analogWrite(l2_speed, 0);
  }
  
  else
  { 
    digitalWrite(l2_dir, HIGH);
    analogWrite(l2_speed, -l2);
  }

  //pump1

  if(p1==0)
  {
    digitalWrite(pump1_dir,HIGH);
    analogWrite(pump1_speed,0);
  }
  else
  {
    digitalWrite(pump1_dir,HIGH);
    analogWrite(pump1_speed,255);
  }
  //pump 2
  if(p2==0)
  {
    digitalWrite(pump2_dir,LOW);
    analogWrite(pump2_speed,0);
  }
  else
  {
    digitalWrite(pump2_dir,LOW);
    analogWrite(pump2_speed,255);
  }
  //pump 3
  if(p3==0)
  {
    digitalWrite(pump3_dir,LOW);
    analogWrite(pump3_speed,0);
  }
  else
  {
    digitalWrite(pump3_dir,LOW);
    analogWrite(pump3_speed,255);
  }
  //pump 4
  if(p4==0)
  {
    digitalWrite(pump4_dir,LOW);
    analogWrite(pump4_speed,0);
  }
  else
  {
    digitalWrite(pump4_dir,LOW);
    analogWrite(pump4_speed,255);
  }

  //plate
  if(plate>0) {
    digitalWrite(plate_dir,HIGH);
    analogWrite(plate_speed, plate);
  }

  else if(plate<0)
  {
    digitalWrite(plate_dir,LOW);
    analogWrite(plate_speed,-plate);
    
  }

  else
  {
    digitalWrite(plate_dir,LOW);
    analogWrite(plate_speed,0);
  }
  
// auger
  if(auger>0) {
    digitalWrite(auger_dir,HIGH);
    analogWrite(auger_speed,auger);
  }
  else if(auger<0)
  {
    digitalWrite(auger_dir,LOW);
    analogWrite(auger_speed,-auger);
  }
  else
  {
    digitalWrite(auger_dir,LOW);
    analogWrite(auger_speed,0);
  }
  //mixer
  if(mixer>0) {
    digitalWrite(mixer_dir,HIGH);
    analogWrite(mixer_speed,mixer);
  }
  else if(mixer<0)
  {
    digitalWrite(mixer_dir,LOW);
    analogWrite(mixer_speed,-mixer);
  }
  else
  {
    digitalWrite(mixer_dir,LOW);
    analogWrite(mixer_speed,0);
  }

  if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
  esp_restart();
  }

}
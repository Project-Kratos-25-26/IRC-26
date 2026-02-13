#include <micro_ros_arduino.h>
#include "esp_system.h"
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Cytron_SmartDriveDuo.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>


rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define IN1 27
#define IN2 14
#define IN3 12

Cytron_SmartDriveDuo motor_back(SERIAL_SIMPLIFIED, IN1, 115200);
Cytron_SmartDriveDuo motor_mid(SERIAL_SIMPLIFIED, IN2, 115200);
Cytron_SmartDriveDuo motor_front(SERIAL_SIMPLIFIED, IN3, 115200);
void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg0 = (const std_msgs__msg__Float32MultiArray *)msgin;
  msg.data.data[0]=msg0->data.data[0];
  msg.data.data[1]=msg0->data.data[1];
  msg.data.data[2]=msg0->data.data[2];
  msg.data.data[3]=msg0->data.data[3];
  msg.data.data[4]=msg0->data.data[4];
  msg.data.data[5]=msg0->data.data[5];
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}
void wait_for_agent()
{
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) 
    delay(1000);
}
void setup() 
{
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  delay(2000);
  
  msg.data.data = (float_t*)malloc(6 * sizeof(float_t)); 
  msg.data.size = 6;
  msg.data.capacity = 6;
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "rover"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  motor_back.control(msg.data.data[5], msg.data.data[4]);
  motor_mid.control(msg.data.data[3], msg.data.data[2]);
  motor_front.control(msg.data.data[1], msg.data.data[0]);

  if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
  esp_restart();
  }
}

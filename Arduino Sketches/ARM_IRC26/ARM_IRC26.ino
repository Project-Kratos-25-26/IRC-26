#include <micro_ros_arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rcl_publisher_t publisher;
std_msgs__msg__Int32 drop_status;
std_msgs__msg__Int32MultiArray msg;
std_msgs__msg__Int32 msg_drop;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define DEADZONE 10

int l1 = 0;
int l2 = 0;
int base = 0;
int gripper = 0;
int rbevel = 0;
int lbevel = 0;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
   RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &drop_status, NULL));
  }
}
void subscription_callback1(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg0 = (const std_msgs__msg__Int32MultiArray*)msgin;
  l1=msg0->data.data[0];
  l2=msg0->data.data[1];
  lbevel=msg0->data.data[2];
  rbevel=msg0->data.data[3];
  base=msg0->data.data[4];
  gripper=msg0->data.data[5];
  //digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  
}
void subscription_callback2(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg0 = (const std_msgs__msg__Int32*)msgin;
  msg_drop.data=msg0->data;


  //digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  
}
int l1_dir=26;
int l1_speed=25;
int l2_dir=12;
int l2_speed=14;
int base_dir=33;
int base_speed=32;
int finger_dir=4;
int finger_speed=0;
int bevel1_dir=27;
int bevel1_speed=13;
int bevel2_dir=5;
int bevel2_speed=15;
#define DROP_TOP_PIN 21
#define DROP_BOTTOM_PIN 22
int dropmotor_dir=16;
int dropmotor_speed=17;

void setup() {
  set_microros_transports();
  
  pinMode(l1_dir,OUTPUT);
  pinMode(l1_speed, OUTPUT);
  pinMode(l2_dir, OUTPUT);
  pinMode(l2_speed, OUTPUT);
  pinMode(base_dir, OUTPUT);
  pinMode(base_speed, OUTPUT);
  pinMode(bevel1_dir, OUTPUT);
  pinMode(bevel1_speed, OUTPUT);
  pinMode(bevel2_dir, OUTPUT);
  pinMode(bevel2_speed, OUTPUT);
  pinMode(finger_dir,OUTPUT);
  pinMode(finger_speed,OUTPUT);
  pinMode(dropmotor_dir,OUTPUT);
  pinMode(dropmotor_speed,OUTPUT);
  pinMode(DROP_TOP_PIN, INPUT_PULLUP);
  pinMode(DROP_BOTTOM_PIN, INPUT_PULLUP);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  msg_drop.data=2; 
  msg.data.data = (int32_t*)malloc(6 * sizeof(int32_t)); 
  msg.data.size = 6;
  msg.data.capacity = 6;

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "drop_status"));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mymotors"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "dropmotor"));

  const unsigned int timer_timeout = 1500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg, &subscription_callback1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg_drop, &subscription_callback2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); //100Hz

  //link1
    if(msg_drop.data==1)
  {
    if(digitalRead(DROP_TOP_PIN)==LOW)
    {
      digitalWrite(dropmotor_dir,HIGH);
      analogWrite(dropmotor_speed,0);
      drop_status.data=1;
    }
    else{
      digitalWrite(dropmotor_dir,HIGH);
      analogWrite(dropmotor_speed,255);
      drop_status.data=2;
    }
  }
  else if (msg_drop.data==0)
  {
    if(digitalRead(DROP_BOTTOM_PIN)==LOW)
    {
      digitalWrite(dropmotor_dir,LOW);
      analogWrite(dropmotor_speed,0);
      drop_status.data=0;
    }
    else{
      digitalWrite(dropmotor_dir,LOW);
      analogWrite(dropmotor_speed,255);
      drop_status.data=2;
    }
  }
  else
  {
    digitalWrite(dropmotor_dir,HIGH);
    analogWrite(dropmotor_speed,0);
  }
  if(l1>0)
  {
    digitalWrite(l1_dir,HIGH);
    analogWrite(l1_speed,l1);
  }
  else
  {
    digitalWrite(l1_dir,LOW);
    analogWrite(l1_speed,-l1);
  }
  
  //link2
  if(l2>0)
  {
    digitalWrite(l2_dir,LOW);
    analogWrite(l2_speed,l2);
  }
  else
  {
    digitalWrite(l2_dir,HIGH);
    analogWrite(l2_speed,-l2);
  }
 
  //base_yaw
  if(base>0)
  {
    digitalWrite(base_dir,HIGH);
    analogWrite(base_speed,base);
  }
  else if(base<0)
  {
    digitalWrite(base_dir,LOW);
    analogWrite(base_speed,-base);
  }
  else
  {
    digitalWrite(base_dir,LOW);
    analogWrite(base_speed,0);
  }

  //end_effector
  if(gripper>0) {
    digitalWrite(finger_dir,HIGH);
    analogWrite(finger_speed,gripper);
  }
  else if(gripper<0)
  {
    digitalWrite(finger_dir,LOW);
    analogWrite(finger_speed,-gripper);
  }
  else
  {
    digitalWrite(finger_dir,LOW);
    analogWrite(finger_speed,0);
  }

  if(abs(lbevel) < DEADZONE){
    analogWrite(bevel1_speed, 0);
    analogWrite(bevel2_speed, 0);
  }

  else{
    //bevel left
  if(lbevel>0)
  {
    digitalWrite(bevel1_dir, LOW);
     analogWrite(bevel1_speed,lbevel);
  }
  else
  {
    digitalWrite(bevel1_dir,HIGH);
    analogWrite(bevel1_speed,-lbevel);
  }
    
    //bevel right 
  if(rbevel>0)
  {
    digitalWrite(bevel2_dir,LOW);
    analogWrite(bevel2_speed,rbevel);
  }
  else
  {
    digitalWrite(bevel2_dir,HIGH);
    analogWrite(bevel2_speed,-rbevel);
  }

  }
}
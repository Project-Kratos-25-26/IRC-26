/*
 * ESP32 #2 - Camera Servo Subscriber (ROS2)
 * Receives servo angles from encoder and controls 6 servos
 * NOTE: Servos 4 and 5 (pins 22 and 25) are 360-degree continuous rotation servos
 * 
 * HARDWARE SETUP:
 * Servos:
 * - Servo 1 Signal → GPIO 18 (Camera 1) - Standard servo
 * - Servo 2 Signal → GPIO 19 (Camera 2) - Standard servo
 * - Servo 3 Signal → GPIO 22 (Camera 3) - Standard servo
 * - Servo 4 Signal → GPIO 21 (Camera 4) - 360° CONTINUOUS ROTATION
 * - Servo 5 Signal → GPIO 25 (Camera 5) - 360° CONTINUOUS ROTATION
 * - Servo 6 Signal → GPIO 26 (Camera 6) - Standard servo
 * - All Servo GND → Common GND
 * - All Servo Power → External 5-6V power supply
 * 
 * LIBRARIES NEEDED:
 * - micro_ros_arduino
 * - ESP32Servo by Kevin Harrington
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>

// ============= SERVO CONFIGURATION =============
const int SERVO1_PIN = 18;  // Camera 1 servo (standard)
const int SERVO2_PIN = 19;  // Camera 2 servo (standard)
const int SERVO3_PIN = 22;  // Camera 3 servo (standard)
const int SERVO4_PIN = 21;  // Camera 4 servo (360° CONTINUOUS)
const int SERVO5_PIN = 26;  // Camera 5 servo (360° CONTINUOUS)
const int SERVO6_PIN = 25;  // Camera 6 servo (standard)
const int SERVO7_PIN = 13;  // Camera 7 servo (standard)

// LED for status indication
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
// ================================================

// Servo objects
Servo servo1;  // Camera 1 (standard)
Servo servo2;  // Camera 2 (standard)
Servo servo3;  // Camera 3 (standard)
Servo servo4;  // Camera 4 (360° continuous)
Servo servo5;  // Camera 5 (360° continuous)
Servo servo6;  // Camera 6 (standard)
Servo servo7;  // Camera 7

// Individual servo angles - each camera maintains its position
float servo1_angle = 90.0;
float servo2_angle = 90.0;
float servo3_angle = 90.0;
float servo4_angle = 90.0;  // For 360° servo: 70=CCW, 90=STOP, 110=CW
float servo5_angle = 90.0;  // For 360° servo: 70=CCW, 90=STOP, 110=CW
float servo6_angle = 90.0;
float servo7_angle = 90.0;

// Active servo selection (received from publisher)
int activeServo1 = 0;
int activeServo2 = 1;

// micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Function to write to servo 4 (360° continuous rotation servo on pin 22)
void writeServo4(float angle) {
  // Only accept 70, 90, or 110 as valid angles
  // 70 = Counter-clockwise rotation
  // 90 = Stop
  // 110 = Clockwise rotation
  if (angle == 70.0 || angle == 90.0 || angle == 110.0) {
    servo4.write(angle);
    servo4_angle = angle;
  } else {
    // Default to stop if invalid angle received
    servo4.write(90);
    servo4_angle = 90.0;
  }
}

// Function to write to servo 5 (360° continuous rotation servo on pin 25)
void writeServo5(float angle) {
  // Only accept 70, 90, or 110 as valid angles
  // 70 = Counter-clockwise rotation
  // 90 = Stop
  // 110 = Clockwise rotation
  if (angle == 70.0 || angle == 90.0 || angle == 110.0) {
    servo5.write(angle);
    servo5_angle = angle;
  } else {
    // Default to stop if invalid angle received
    servo5.write(90);
    servo5_angle = 90.0;
  }
}

// Callback - receives servo angles and active servos from encoder ESP32
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //if (msg->data.size >= 8) {
    // Receive all eight values
    float recv_servo1_angle = msg->data.data[0];
    float recv_servo2_angle = msg->data.data[1];
    float recv_servo3_angle = msg->data.data[2];
    float recv_servo4_angle = msg->data.data[3];  // 360° servo
    float recv_servo5_angle = msg->data.data[4];  // 360° servo
    float recv_servo6_angle = msg->data.data[5];
    float recv_servo7_angle = msg->data.data[6];
    int recv_activeServo1 = (int)msg->data.data[7];
    int recv_activeServo2 = (int)msg->data.data[8];

    // Update local variables
    servo1_angle = recv_servo1_angle;
    servo2_angle = recv_servo2_angle;
    servo3_angle = recv_servo3_angle;
    servo7_angle = recv_servo7_angle;
    // servo4_angle handled by writeServo4
    // servo5_angle handled by writeServo5
    servo6_angle = recv_servo6_angle;
    activeServo1 = recv_activeServo1;
    activeServo2 = recv_activeServo2;
    
    // Write to servos
    servo1.write(servo1_angle);  // Standard servo
    servo2.write(servo2_angle);  // Standard servo
    servo3.write(servo3_angle);  // Standard servo
    writeServo4(recv_servo4_angle);  // 360° continuous servo
    writeServo5(recv_servo5_angle);  // 360° continuous servo
    servo6.write(servo6_angle);  // Standard servo
    servo7.write(servo7_angle);  // Standard servo
    
    // Visual feedback
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //}
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ========== SERVO SETUP ==========

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Standard servos
  servo1.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  
  servo2.setPeriodHertz(50);
  servo2.attach(SERVO2_PIN, 500, 2400);
  
  servo3.setPeriodHertz(50);
  servo3.attach(SERVO3_PIN, 500, 2400);
  
  // 360° continuous rotation servo (pin 22)
  servo4.setPeriodHertz(50);
  servo4.attach(SERVO4_PIN, 500, 2400);
  
  // 360° continuous rotation servo (pin 25)
  servo5.setPeriodHertz(50);
  servo5.attach(SERVO5_PIN, 500, 2400);
  
  servo6.setPeriodHertz(50);
  servo6.attach(SERVO6_PIN, 500, 2400);

  servo7.setPeriodHertz(50);
  servo7.attach(SERVO7_PIN, 500, 2400);
  
  // Initialize standard servos to center
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);  // Initialize 360° servo to STOP position
  servo5.write(90);  // Initialize 360° servo to STOP position
  servo6.write(90);
  servo7.write(90);
 
  // ========== ROS2 SETUP ==========

  
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "camera_servo_node", "", &support));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/camera_angles"));
  
  // Allocate message memory
  msg.data.capacity = 9;
  msg.data.size = 9;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, 
    &subscription_callback, ON_NEW_DATA));
  
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  // Process ROS messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "std_msgs/msg/u_int8_multi_array.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Float32MultiArray actuator_positions;
std_msgs__msg__Float32MultiArray system_currents;
std_msgs__msg__UInt8MultiArray diagnostics;
std_msgs__msg__Float32MultiArray system_temperatures; 
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool led_state = false;
int32_t x = 0;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void *msgin){
  //const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *) msgin;
  //x = msg->data.data[0];
  led_state = !led_state;
  digitalWrite(13, led_state ? HIGH : LOW);
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg2, NULL));
  }
}

void setup() {
  // Configure serial transport
  pinMode(13, OUTPUT);
  Serial.begin(1000000);
  set_microros_serial_transports(Serial);
  delay(500);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "actuator_positions"));
  
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "toggle_LED"));
  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));
  
    msg.data.capacity = 10;
msg.data.size = 0;
msg.data.data = (int32_t*) malloc(sizeof(int32_t) * msg.data.capacity);

  msg2.data = 10;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
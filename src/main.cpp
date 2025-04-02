#include "main.hpp"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void *msgin){
 
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg2, NULL));
  }
}

void setup() {
  Serial.begin(1000000);
  set_microros_serial_transports(Serial);
  delay(500);
  allocator = rcl_get_default_allocator();
  //Create the supporter
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&teensy_hexabot_node, "teensy_hexabot_node", "", &support));
  //Publisher init for actuator_positions
  RCCHECK(rclc_publisher_init_default(
    &actuator_positions_publisher,
    &teensy_hexabot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/teensy/actuator_positions"));
  //Publisher init for system_currents
  RCCHECK(rclc_publisher_init_default(
    &system_currents_publisher,
    &teensy_hexabot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/teensy/system_currents"));
  //Publisher init for system_temperatures
  RCCHECK(rclc_publisher_init_default(
    &system_temperatures_publisher,
    &teensy_hexabot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/teensy/system_temperatures"));
  //Publisher init for diagnostics
  RCCHECK(rclc_publisher_init_default(
    &diagnostics_publisher,
    &teensy_hexabot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/teensy/diagnostics"));
  //Subscriber init for actuator duty cycles
  RCCHECK(rclc_subscription_init_default(
      &actuator_positions_subscriber,
      &teensy_hexabot_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/hexabot_driver/actuator_duty_cycle"));
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
    &actuator_positions_subscriber,
    &actuator_positions_command,
    &subscription_callback,
    ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
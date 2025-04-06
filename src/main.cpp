#include "main.hpp"
//testß

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
  //Sample the actuator positions and format it into ros2 message
  //Sample the system currents and format it into ros2 message
  //Samle the system temperatures and format it into ros2 messge
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&actuator_positions_publisher, &actuator_positions_feedback, NULL));
    RCSOFTCHECK(rcl_publish(&system_currents_publisher, &system_currents, NULL));
    RCSOFTCHECK(rcl_publish(&system_temperatures_publisher, &system_temperatures, NULL));
    RCSOFTCHECK(rcl_publish(&diagnostics_publisher, &diagnostics, NULL));
    RCSOFTCHECK(rcl_publish(&actuator_positions_publisher, &actuator_positions_feedback, NULL));
  }
}

void setup() {
  Serial.begin(1000000);
  
  delay(500);
  init_microros();

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void init_microros(){

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  //Create the supporter
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  //Create the teensy node
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
  //Subscriber init for communication heart beat
  RCCHECK(rclc_subscription_init_default(
      &communication_beat_subscriber,
      &teensy_hexabot_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/hexabot_driver/heart_beat"))
  //Create timer for publishing the data
  const unsigned int timer_timeout = 20; //this is time in millisecond, so the timer will fire 50 times in a second
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  //Create executor to handle timer and subscription feedbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &actuator_positions_subscriber,
    &actuator_positions_command,
    &subscription_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &communication_beat_subscriber,
    &heart_beat_message,
    &subscription_callback,
    ON_NEW_DATA));
  //Allocate memory for messages
  actuator_positions_feedback.data.capacity = 8;
  actuator_positions_feedback.data.data = (float*) malloc(sizeof(float) * actuator_positions_feedback.data.capacity);
  actuator_positions_feedback.data.size = 0;
  system_currents.data.capacity = 9;
  system_currents.data.data = (float*) malloc(sizeof(float) * system_currents.data.capacity);
  system_currents.data.size = 0;
  diagnostics.data.capacity = 10; //change this based on the number of flags we are sending.
  diagnostics.data.data = (uint8_t*) malloc(sizeof(uint8_t) * diagnostics.data.capacity);
  diagnostics.data.size = 0;
  system_temperatures.data.capacity = 2;
  system_temperatures.data.data = (float*) malloc(sizeof(float) * system_temperatures.data.capacity);
  actuator_positions_command.data.capacity = 8;
  actuator_positions_command.data.data = (float*) malloc(sizeof(float) * actuator_positions_command.data.capacity);
  actuator_positions_command.data.size = 8;  
}

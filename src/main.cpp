#include "main.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
static bool error_loop_first_entry = true;
static bool driver_communication_flag = true;
static bool reconnecting = false;

void testing_comm();
void init_microros();
// Error handle loop
void error_loop() {
  if(error_loop_first_entry){
    //add functions to turn off power and trigger emergency
    error_loop_first_entry = false;
  }
  //SCB_AIRCR = 0x05FA0004; 
  destroy_microros();
  delay(10);
  init_microros();
}

void subscription_callback(const void *msgin){
 
}
void heart_beat_callback(const void *msgin){
  last_heartbeat_time = millis();
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  //Sample the actuator positions and format it into ros2 message
  //Sample the system currents and format it into ros2 message
  //Samle the system temperatures and format it into ros2 messge
  testing_comm();
  if (timer != NULL) {
    rcl_publish(&actuator_positions_publisher, &actuator_positions_feedback, NULL);
    rcl_publish(&system_currents_publisher, &system_currents, NULL);
    rcl_publish(&system_temperatures_publisher, &system_temperatures, NULL);
    rcl_publish(&diagnostics_publisher, &diagnostics, NULL);
    }
}

void setup() {
  teensy_setup();
  message_memory_allocation();
  Serial.begin(1000000);
  pinMode(13, OUTPUT);
  init_microros();
  digitalWrite(13, HIGH);
  
}

void loop() {
  
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  digitalWrite(communication_LED_blue_pin, LOW);
  if(millis() - last_heartbeat_time > 200){
    digitalWrite(communication_LED_green_pin, LOW);
    digitalWrite(communication_LED_red_pin, HIGH);
    if(!reconnecting){
      driver_communication_flag = false;
      reconnecting = true;
      destroy_microros();
      delay(10);
      init_microros();
    }
    //call a function that stops the actuators here.
  }
  else{
    // if(!driver_communication_flag){
    //   driver_communication_flag = true;
    //   //call a function that restarts the actuators here.

    // }
    digitalWrite(communication_LED_blue_pin, LOW);
    digitalWrite(communication_LED_red_pin, LOW);
    digitalWrite(communication_LED_green_pin, HIGH);
    
  }
}

void init_microros(){
  //bool correct = false;
  //digitalWrite(power_LED_red_pin, LOW); //remove - for test only

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  //Create the supporter
  rcl_ret_t ret;
  do{
    digitalWrite(communication_LED_blue_pin, HIGH);
    digitalWrite(communication_LED_red_pin,LOW);
    digitalWrite(communication_LED_green_pin,LOW);
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    delay(1000);
  } while(ret != RCL_RET_OK);
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
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
      "/hexabot_driver/heart_beat"));
  //Create timer for publishing the data
  const unsigned int timer_timeout = 1000; //this is time in millisecond, so the timer will fire 50 times in a second
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
    &heart_beat_callback,
    ON_NEW_DATA));
  //driver_communication_flag = false;
  last_heartbeat_time = millis();
  reconnecting = false;
}

void destroy_microros() {
  // Stop executor first to avoid running callbacks during teardown
  rclc_executor_fini(&executor);

  // Finalize all timers
  rcl_timer_fini(&timer);

  // Finalize all subscriptions
  rcl_subscription_fini(&actuator_positions_subscriber, &teensy_hexabot_node);
  rcl_subscription_fini(&communication_beat_subscriber, &teensy_hexabot_node);

  // Finalize all publishers
  rcl_publisher_fini(&actuator_positions_publisher, &teensy_hexabot_node);
  rcl_publisher_fini(&system_currents_publisher, &teensy_hexabot_node);
  rcl_publisher_fini(&system_temperatures_publisher, &teensy_hexabot_node);
  rcl_publisher_fini(&diagnostics_publisher, &teensy_hexabot_node);

  // Finalize node
  rcl_node_fini(&teensy_hexabot_node);

  // Finalize support structure
  rclc_support_fini(&support);
  digitalWrite(power_LED_red_pin, HIGH); //remove - for test only
}

void message_memory_allocation(){
  //Allocate memory for messages
  actuator_positions_feedback.data.capacity = 8;
  actuator_positions_feedback.data.data = (float*) malloc(sizeof(float) * actuator_positions_feedback.data.capacity);
  actuator_positions_feedback.data.size = 0;
  system_currents.data.capacity = 9;
  system_currents.data.data = (float*) malloc(sizeof(float) * system_currents.data.capacity);
  system_currents.data.size = 0;
  diagnostics.data.capacity = 14; //change this based on the number of flags we are sending.
  diagnostics.data.data = (uint8_t*) malloc(sizeof(uint8_t) * diagnostics.data.capacity);
  diagnostics.data.size = 0;
  system_temperatures.data.capacity = 2;
  system_temperatures.data.data = (float*) malloc(sizeof(float) * system_temperatures.data.capacity);
  actuator_positions_command.data.capacity = 8;
  actuator_positions_command.data.data = (float*) malloc(sizeof(float) * actuator_positions_command.data.capacity);
  actuator_positions_command.data.size = 8;  
}
void teensy_setup(){
  pinMode(power_LED_blue_pin, OUTPUT);
  pinMode(power_LED_green_pin, OUTPUT);
  pinMode(power_LED_red_pin, OUTPUT);
  pinMode(communication_LED_blue_pin, OUTPUT);
  pinMode(communication_LED_green_pin, OUTPUT);
  pinMode(communication_LED_red_pin, OUTPUT);
  pinMode(switch_control_pin, OUTPUT);
  pinMode(voltage_translator_control_pin, OUTPUT);
  pinMode(estop_signal_pin, INPUT_PULLUP); //the pullup is to prevent false interrupts on floating states.
  pinMode(switch_status_pin, INPUT);
  pinMode(main_power_feedback_pin, INPUT);
  pinMode(total_current_feedback_pin, INPUT);
  pinMode(temperature_pin, INPUT);
  pinMode(servo_position_control_pin, OUTPUT);
  pinMode(servo_power_control_pin, OUTPUT);
  pinMode(servo_current_feedback_pin, INPUT);
  pinMode(actuator_1_position_control_pin, OUTPUT);
  pinMode(actuator_2_position_control_pin, OUTPUT);
  pinMode(actuator_3_position_control_pin, OUTPUT);
  pinMode(actuator_4_position_control_pin, OUTPUT);
  pinMode(actuator_4_position_control_pin, OUTPUT);
  pinMode(actuator_5_position_control_pin, OUTPUT);
  pinMode(actuator_6_position_control_pin, OUTPUT);
  pinMode(actuator_1_power_control_pin, OUTPUT);
  pinMode(actuator_2_power_control_pin, OUTPUT);
  pinMode(actuator_3_power_control_pin, OUTPUT);
  pinMode(actuator_4_power_control_pin, OUTPUT);
  pinMode(actuator_5_power_control_pin, OUTPUT);
  pinMode(actuator_6_power_control_pin, OUTPUT);
  pinMode(actuator_1_position_feedback_pin,INPUT);
  pinMode(actuator_2_position_feedback_pin,INPUT);
  pinMode(actuator_3_position_feedback_pin,INPUT);
  pinMode(actuator_4_position_feedback_pin,INPUT);
  pinMode(actuator_5_position_feedback_pin,INPUT);
  pinMode(actuator_6_position_feedback_pin,INPUT);
  pinMode(actuator_1_current_feedback_pin,INPUT);
  pinMode(actuator_2_current_feedback_pin,INPUT);
  pinMode(actuator_3_current_feedback_pin,INPUT);
  pinMode(actuator_4_current_feedback_pin,INPUT);
  pinMode(actuator_5_current_feedback_pin,INPUT);
  pinMode(actuator_6_current_feedback_pin,INPUT);

  //Adjusting the frequency and resolution of the actuator control pins
  analogWriteFrequency(actuator_1_position_control_pin, 1000);
  analogWriteFrequency(actuator_2_position_control_pin, 1000);
  analogWriteFrequency(actuator_3_position_control_pin, 1000);
  analogWriteFrequency(actuator_4_position_control_pin, 1000);
  analogWriteFrequency(actuator_5_position_control_pin, 1000);
  analogWriteFrequency(actuator_6_position_control_pin, 1000);

//analogWriteFrequency(servo_position_control_pin, 1000) //adjust this frequency based on servo data sheet
  analogWriteResolution(15);
  //Emergency Stop Interrupt
  //attachInterrupt(digitalPinToInterrupt(estop_signal_pin), emergency_ISR, CHANGE); // change pin to name

}

void testing_comm(){
  actuator_positions_feedback.data.data[0] = 10;
  actuator_positions_feedback.data.data[1] = 11;
  actuator_positions_feedback.data.data[2] = 12;
  actuator_positions_feedback.data.data[3] = 13;
  actuator_positions_feedback.data.data[4] = 14;
  actuator_positions_feedback.data.data[5] = 15;
  actuator_positions_feedback.data.data[6] = 16;
  actuator_positions_feedback.data.data[7] = 3.14;
  actuator_positions_feedback.data.size = 8;

  system_currents.data.data[0] = 0;
  system_currents.data.data[1] = 1;
  system_currents.data.data[2] = 2;
  system_currents.data.data[3] = 3;
  system_currents.data.data[4] = 4;
  system_currents.data.data[5] = 5;
  system_currents.data.data[6] = 6;
  system_currents.data.data[7] = 7;
  system_currents.data.data[8] = 3.14;
  system_currents.data.size = 9;

  diagnostics.data.data[0] = 0;
  diagnostics.data.data[1] = 1;
  diagnostics.data.data[2] = 2;
  diagnostics.data.data[3] = 3;
  diagnostics.data.data[4] = 4;
  diagnostics.data.data[5] = 5;
  diagnostics.data.data[6] = 6;
  diagnostics.data.data[7] = 7;
  diagnostics.data.data[8] = 8;
  diagnostics.data.data[9] = 9;
  diagnostics.data.data[10] = 10;
  diagnostics.data.data[11] = 11;
  diagnostics.data.data[12] = 12;
  diagnostics.data.data[13] = 5; //because the type is uint
  diagnostics.data.size = 14;
  system_temperatures.data.data[0] = 0;
  system_temperatures.data.data[1] = 1;
  system_temperatures.data.data[2] = 3.14;
  system_temperatures.data.size = 3;
}
#include "main.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void emergency_sequance();
void teensy_setup();
void message_memory_allocation();
void init_microros();
void destroy_microros();
void heart_beat_callback();
void start_power();
void stop_power();
void get_diagnostics();
void get_actuator_positions();
void get_currents();
void get_temperatures();
void set_acuator_positions();
void check_main_power();
void publish_data();
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

void actuator_commands_callback(const void *msgin){
  if(power_avaliable_flag){
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    actuator_positions_command = *msg;
  }
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
  check_main_power();
  if(millis() - last_heartbeat_time > 200){
    digitalWrite(communication_LED_green_pin, LOW);
    digitalWrite(communication_LED_red_pin, HIGH);
    if(!reconnecting){
      get_actuator_positions();
      stop_power();
      emergency_sequance();
      driver_communication_flag = false;
      reconnecting = true;
      destroy_microros();
      delay(10);
      init_microros();
    }
    //call a function that stops the actuators here.
  }
  //Communication is good, check for emergency flags and 
  //main power before doing anything else.
  else{
    digitalWrite(communication_LED_blue_pin, LOW);
    digitalWrite(communication_LED_red_pin, LOW);
    digitalWrite(communication_LED_green_pin, HIGH);
    if(!emergency_flag && power_avaliable_flag){
      get_diagnostics();
      get_currents();
      get_actuator_positions();
      get_temperatures();
      //publish_data();
      set_actuator_positions();
    }
    else if(emergency_flag){
      //Save current positions first then turn off actuators.
      get_actuator_positions();
      stop_power();
      emergency_sequance(); 
    }
    
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
    &actuator_commands_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &communication_beat_subscriber,
    &heart_beat_message,
    &heart_beat_callback,
    ON_NEW_DATA));

  last_heartbeat_time = millis();
  reconnecting = false;
  start_power();
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
  attachInterrupt(digitalPinToInterrupt(estop_signal_pin), emergency_ISR, CHANGE); // change pin to name
  digitalWriteFast(switch_control_pin, HIGH);
}

void start_power(){
  digitalWriteFast(voltage_translator_control_pin,HIGH);
  digitalWriteFast(actuator_1_power_control_pin,HIGH);
  digitalWriteFast(actuator_2_power_control_pin,HIGH);
  digitalWriteFast(actuator_3_power_control_pin,HIGH);
  digitalWriteFast(actuator_4_power_control_pin,HIGH);
  digitalWriteFast(actuator_5_power_control_pin,HIGH);
  digitalWriteFast(actuator_6_power_control_pin,HIGH);
  digitalWriteFast(servo_power_control_pin,HIGH);
  digitalWriteFast(switch_control_pin, LOW);
  delay(100);
  digitalWriteFasst(switch_control_pin,HIGH);
  delay(100);
  return;
}
void stop_power(){
  digitalWriteFast(voltage_translator_control_pin, LOW);
  digitalWriteFast(actuator_1_power_control_pin,LOW);
  digitalWriteFast(actuator_2_power_control_pin,LOW);
  digitalWriteFast(actuator_3_power_control_pin,LOW);
  digitalWriteFast(actuator_4_power_control_pin,LOW);
  digitalWriteFast(actuator_5_power_control_pin,LOW);
  digitalWriteFast(actuator_6_power_control_pin,LOW);
  digitalWriteFast(servo_power_control_pin,LOW);
  digitalWriteFast(switch_control_pin, LOW);
  delay(100);
  digitalWriteFasst(switch_control_pin,HIGH);
  delay(100);
  return;
}

void get_diagnosstics(){
  diagnostics.data.data[0] = digitalReadFast(main_power_feedback_pin);
  diagnostics.data.data[1] = digitalReadFast(switch_status_pin);
  diagnostics.data.data[2] = digitalReadFast(estop_signal_pin);
  diagnostics.data.data[3] = digitalReadFast(voltage_translator_control_pin);
  diagnostics.data.data[4] = digitalReadFast(actuator_1_power_control_pin);
  diagnostics.data.data[5] = digitalReadFast(actuator_2_power_control_pin);
  diagnostics.data.data[6] = digitalReadFast(actuator_3_power_control_pin);
  diagnostics.data.data[7] = digitalReadFast(actuator_4_power_control_pin);
  diagnostics.data.data[8] = digitalReadFast(actuator_5_power_control_pin);
  diagnostics.data.data[9] = digitalReadFast(actuator_6_power_control_pin);
  diagnostics.data.data[10] = digitalReadFast(servo_power_control_pin);
  diagnostics.data.data[11] = 0; //maybe edit later
  diagnostics.data.data[12] = 0; //maybe edit later
  diagnostics.data.data[13] = 5; //changed to 5 since type is uint
  diagnostics.data.size = 14;
  return;
}
void get_currents(){ // TBD if 32757.0 should change to 1023
  // current calculations
  const float Voffset = 1.65;
  const float sensitivity = 0.132;
  float voltage;

  system_currents.data.size = 8;
  
  voltage = (static_cast<float>(analogRead(actuator_1_current_feedback_pin)) / 32767.0) * 3.3; // conversion to voltage
  system_currents.data.data[0] = (voltage - Voffset) / Sensitivity; // calculate current

  voltage = (static_cast<float>(analogRead(actuator_2_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[1] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(actuator_3_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[2] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(actuator_4_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[3] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(actuator_5_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[4] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(actuator_6_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[5] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(servo_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[6] = (voltage - Voffset) / Sensitivity;

  voltage = (static_cast<float>(analogRead(total_current_feedback_pin)) / 32757.0) * 3.3;
  system_currents.data.data[7] = (voltage - Voffset) / Sensitivity;

  //OMAR WORK
  return;
}
void get_temperatures(){

  //constexpr int temp_sensor_pin = temperature_pin;
  int raw_temperature_reading = analogRead(temperature_pin);


  //OMAR WORK
  return;
}
void get_actuator_positions(){
  return;
}
void emergency_ISR(){
  bool current_state = digitalRead(estop_signal_pin);
  //DIAG pin went from LOW to HIGH = E-stop is disengaged
  if(current_state){
    emergency_flag = false;
    digitalWriteFast(power_LED_red_pin,LOW);
    digitalWriteFast(power_LED_green_pin,HIGH);
    digitalWriteFast(power_LED_blue_pin,LOW);
    start_power();
  }
  //DIAG pin went from HIGH to LOW = E-stop is engaged
  else{
    emergency_flag = true;
    digitalWriteFast(power_LED_red_pin,LOW);
    digitalWriteFast(power_LED_green_pin,LOW);
    digitalWriteFast(power_LED_blue_pin,HIGH);
  }
  
  return;
}
void emergency_sequence(){
  actuator_1_duty_cycle = (actuator_1_position_feedback / 30) * 32757;
  actuator_2_duty_cycle = (actuator_2_position_feedback / 30) * 32575;
  actuator_3_duty_cycle = (actuator_3_position_feedback / 30) * 32575;
  actuator_4_duty_cycle = (actuator_4_position_feedback / 30) * 32575;
  actuator_5_duty_cycle = (actuator_5_position_feedback / 30) * 32575;
  actuator_6_duty_cycle = (actuator_6_position_feedback / 30) * 32575;
  //DO WE NEED TO DO SOMETHING ABOUT THE SERVO????
  //Assign those duty cycles to actuators
  set_acuator_positions();
}
void set_actuator_positions(){
  analogWrite(actuator_1_position_control_pin, actuator_1_duty_cycle);
  analogWrite(actuator_2_position_control_pin, actuator_2_duty_cycle);
  analogWrite(actuator_3_position_control_pin, actuator_3_duty_cycle);
  analogWrite(actuator_4_position_control_pin, actuator_4_duty_cycle);
  analogWrite(actuator_5_position_control_pin, actuator_5_duty_cycle);
  analogWrite(actuator_6_position_control_pin, actuator_6_duty_cycle);
  //analogWrite(servo_position_control_pin, servo_duty_cycle);
  return;
}
void check_main_power(){
  power_avaliable_flag = digitalReadFast(main_power_feedback_pin);
  if(!emergency_flag){
    if(!power_avaliable_flag){
      digitalWriteFast(power_LED_red_pin,HIGH);
      digitalWriteFast(power_LED_green_pin,LOW);
      digitalWriteFast(power_LED_blue_pin,LOW);
    }
    else{
      digitalWriteFast(power_LED_red_pin,LOW);
      digitalWriteFast(power_LED_green_pin,HIGH);
      digitalWriteFast(power_LED_blue_pin,LOW);
    }
  }
  return;
}
void publish_data(){
  rcl_publish(&actuator_positions_publisher, &actuator_positions_feedback, NULL);
  rcl_publish(&system_currents_publisher, &system_currents, NULL);
  rcl_publish(&system_temperatures_publisher, &system_temperatures, NULL);
  rcl_publish(&diagnostics_publisher, &diagnostics, NULL);
  return;
}
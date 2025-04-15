#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/empty.h>
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

//All the publishers
rcl_publisher_t actuator_positions_publisher;
rcl_publisher_t system_currents_publisher;
rcl_publisher_t system_temperatures_publisher;
rcl_publisher_t diagnostics_publisher;
rcl_publisher_t platform_pose_publisher; //May Remove
//All the subscribers
rcl_subscription_t actuator_positions_subscriber;
rcl_subscription_t communication_beat_subscriber;
//Global messages for publishing and subscriping.
std_msgs__msg__Float32MultiArray actuator_positions_feedback;
std_msgs__msg__Float32MultiArray system_currents;
std_msgs__msg__UInt8MultiArray diagnostics;
std_msgs__msg__Float32MultiArray system_temperatures;
std_msgs__msg__Float32MultiArray actuator_positions_command; 
std_msgs__msg__Empty heart_beat_message;
//rclc and node initiation.
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t teensy_hexabot_node;
rcl_timer_t timer;
rcl_ret_t ret;
//Naming scheme follows meaningful name + "pin" to indicate a constant for 
//pin assignment to the Teensy. Anything else is a variable used in dynamically in the code
//////// WE NEED TO ADD ALL MICROROS ASSIGNMENTS HERE INCLUDING THE MACROS.
//LEDs pin assignmnets (Digital Output)
constexpr int power_LED_blue_pin = 29;
constexpr int power_LED_green_pin = 30;
constexpr int power_LED_red_pin = 31;
constexpr int communication_LED_blue_pin = 32;
constexpr int communication_LED_green_pin = 33;
constexpr int communication_LED_red_pin = 34;
//Diagnostic pin assignments (Mixed)
constexpr int switch_control_pin = 15;
constexpr int voltage_translator_control_pin = 28;
constexpr int estop_signal_pin = 35;
constexpr int switch_status_pin = 36;
constexpr int main_power_feedback_pin = 37;
constexpr int total_current_feedback_pin = 38;
constexpr int temperature_pin = 26;
//Servo pin assignments (Mixed)
constexpr int servo_position_control_pin = 0;
constexpr int servo_power_control_pin = 24;
constexpr int servo_current_feedback_pin = 39;
//Actuator position control pin assignmnets (PWM)
constexpr int actuator_1_position_control_pin = 6;
constexpr int actuator_2_position_control_pin = 5;
constexpr int actuator_3_position_control_pin = 4;
constexpr int actuator_4_position_control_pin = 3;
constexpr int actuator_5_position_control_pin = 2;
constexpr int actuator_6_position_control_pin = 1;
//Actuator power control pin assignmnets (Digital Output)
constexpr int actuator_3_power_control_pin = 7;
constexpr int actuator_2_power_control_pin = 8;
constexpr int actuator_1_power_control_pin = 9;
constexpr int actuator_4_power_control_pin = 10;
constexpr int actuator_5_power_control_pin = 11;
constexpr int actuator_6_power_control_pin = 12;
//Actuator position feedback pin assignmnets (Analog Input)
constexpr int actuator_1_position_feedback_pin = 23;
constexpr int actuator_2_position_feedback_pin = 22;
constexpr int actuator_3_position_feedback_pin = 21;
constexpr int actuator_4_position_feedback_pin = 20;
constexpr int actuator_5_position_feedback_pin = 19;
constexpr int actuator_6_position_feedback_pin = 18;
//Actuator current feedback pin assignmnets (Analog Input)
constexpr int actuator_1_current_feedback_pin = 17;
constexpr int actuator_2_current_feedback_pin = 16;
constexpr int actuator_3_current_feedback_pin = 15;
constexpr int actuator_4_current_feedback_pin = 14;
constexpr int actuator_5_current_feedback_pin = 41;
constexpr int actuator_6_current_feedback_pin = 40;

//Globals for position feedback in millimeters.
float actuator_1_position_feedback = 0;
float actuator_2_position_feedback = 0;
float actuator_3_position_feedback = 0;
float actuator_4_position_feedback = 0;
float actuator_5_position_feedback = 0;
float actuator_6_position_feedback = 0;
float servo_position_feedback = 0;
//Globals for duty cycles (in 0 to 32... analog range) to the actuators
int actuator_1_duty_cycle = 0;
int actuator_2_duty_cycle = 0;
int actuator_3_duty_cycle = 0;
int actuator_4_duty_cycle = 0;
int actuator_5_duty_cycle = 0;
int actuator_6_duty_cycle = 0;
int servo_duty_cycle = 0;

unsigned long last_heartbeat_time = 0;
//void emergency_ISR();
//void emergency_sequance();
//void recover_from_emergency();
void teensy_setup();
void testing_comm();
void message_memory_allocation();
void init_microros();
void destroy_microros();
void heart_beat_callback();
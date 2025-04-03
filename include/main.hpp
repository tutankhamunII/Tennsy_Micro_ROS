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


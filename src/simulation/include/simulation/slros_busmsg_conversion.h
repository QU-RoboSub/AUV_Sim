#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include "Simulation_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(std_msgs::msg::Float32MultiArray& msgPtr, SL_Bus_std_msgs_Float32MultiArray const* busPtr);
void convertToBus(SL_Bus_std_msgs_Float32MultiArray* busPtr, const std_msgs::msg::Float32MultiArray& msgPtr);

void convertFromBus(std_msgs::msg::MultiArrayDimension& msgPtr, SL_Bus_std_msgs_MultiArrayDimension const* busPtr);
void convertToBus(SL_Bus_std_msgs_MultiArrayDimension* busPtr, const std_msgs::msg::MultiArrayDimension& msgPtr);

void convertFromBus(std_msgs::msg::MultiArrayLayout& msgPtr, SL_Bus_std_msgs_MultiArrayLayout const* busPtr);
void convertToBus(SL_Bus_std_msgs_MultiArrayLayout* busPtr, const std_msgs::msg::MultiArrayLayout& msgPtr);

void convertFromBus(std_msgs::msg::String& msgPtr, SL_Bus_std_msgs_String const* busPtr);
void convertToBus(SL_Bus_std_msgs_String* busPtr, const std_msgs::msg::String& msgPtr);


#endif

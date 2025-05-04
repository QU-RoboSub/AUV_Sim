#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_std_msgs_Float32MultiArray and std_msgs::msg::Float32MultiArray

void convertFromBus(std_msgs::msg::Float32MultiArray& msgPtr, SL_Bus_std_msgs_Float32MultiArray const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float32MultiArray");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
  convertFromBus(msgPtr.layout, &busPtr->layout);
}

void convertToBus(SL_Bus_std_msgs_Float32MultiArray* busPtr, const std_msgs::msg::Float32MultiArray& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float32MultiArray");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBus(&busPtr->layout, msgPtr.layout);
}


// Conversions between SL_Bus_std_msgs_MultiArrayDimension and std_msgs::msg::MultiArrayDimension

void convertFromBus(std_msgs::msg::MultiArrayDimension& msgPtr, SL_Bus_std_msgs_MultiArrayDimension const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertFromBusVariablePrimitiveArray(msgPtr.label, busPtr->label, busPtr->label_SL_Info);
  msgPtr.size =  busPtr->size;
  msgPtr.stride =  busPtr->stride;
}

void convertToBus(SL_Bus_std_msgs_MultiArrayDimension* busPtr, const std_msgs::msg::MultiArrayDimension& msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayDimension");

  convertToBusVariablePrimitiveArray(busPtr->label, busPtr->label_SL_Info, msgPtr.label, slros::EnabledWarning(rosMessageType, "label"));
  busPtr->size =  msgPtr.size;
  busPtr->stride =  msgPtr.stride;
}


// Conversions between SL_Bus_std_msgs_MultiArrayLayout and std_msgs::msg::MultiArrayLayout

void convertFromBus(std_msgs::msg::MultiArrayLayout& msgPtr, SL_Bus_std_msgs_MultiArrayLayout const* busPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  msgPtr.data_offset =  busPtr->data_offset;
  convertFromBusVariableNestedArray(msgPtr.dim, busPtr->dim, busPtr->dim_SL_Info);
}

void convertToBus(SL_Bus_std_msgs_MultiArrayLayout* busPtr, const std_msgs::msg::MultiArrayLayout& msgPtr)
{
  const std::string rosMessageType("std_msgs/MultiArrayLayout");

  busPtr->data_offset =  msgPtr.data_offset;
  convertToBusVariableNestedArray(busPtr->dim, busPtr->dim_SL_Info, msgPtr.dim, slros::EnabledWarning(rosMessageType, "dim"));
}


// Conversions between SL_Bus_std_msgs_String and std_msgs::msg::String

void convertFromBus(std_msgs::msg::String& msgPtr, SL_Bus_std_msgs_String const* busPtr)
{
  const std::string rosMessageType("std_msgs/String");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
}

void convertToBus(SL_Bus_std_msgs_String* busPtr, const std_msgs::msg::String& msgPtr)
{
  const std::string rosMessageType("std_msgs/String");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
}


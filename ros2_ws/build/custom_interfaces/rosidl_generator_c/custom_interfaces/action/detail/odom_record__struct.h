// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:action/OdomRecord.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__ODOM_RECORD__STRUCT_H_
#define CUSTOM_INTERFACES__ACTION__DETAIL__ODOM_RECORD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_Goal
{
  uint8_t structure_needs_at_least_one_member;
} custom_interfaces__action__OdomRecord_Goal;

// Struct for a sequence of custom_interfaces__action__OdomRecord_Goal.
typedef struct custom_interfaces__action__OdomRecord_Goal__Sequence
{
  custom_interfaces__action__OdomRecord_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'list_of_odoms'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_Result
{
  geometry_msgs__msg__Point__Sequence list_of_odoms;
} custom_interfaces__action__OdomRecord_Result;

// Struct for a sequence of custom_interfaces__action__OdomRecord_Result.
typedef struct custom_interfaces__action__OdomRecord_Result__Sequence
{
  custom_interfaces__action__OdomRecord_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_Result__Sequence;


// Constants defined in the message

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_Feedback
{
  float current_total;
} custom_interfaces__action__OdomRecord_Feedback;

// Struct for a sequence of custom_interfaces__action__OdomRecord_Feedback.
typedef struct custom_interfaces__action__OdomRecord_Feedback__Sequence
{
  custom_interfaces__action__OdomRecord_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "custom_interfaces/action/detail/odom_record__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__OdomRecord_Goal goal;
} custom_interfaces__action__OdomRecord_SendGoal_Request;

// Struct for a sequence of custom_interfaces__action__OdomRecord_SendGoal_Request.
typedef struct custom_interfaces__action__OdomRecord_SendGoal_Request__Sequence
{
  custom_interfaces__action__OdomRecord_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} custom_interfaces__action__OdomRecord_SendGoal_Response;

// Struct for a sequence of custom_interfaces__action__OdomRecord_SendGoal_Response.
typedef struct custom_interfaces__action__OdomRecord_SendGoal_Response__Sequence
{
  custom_interfaces__action__OdomRecord_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} custom_interfaces__action__OdomRecord_GetResult_Request;

// Struct for a sequence of custom_interfaces__action__OdomRecord_GetResult_Request.
typedef struct custom_interfaces__action__OdomRecord_GetResult_Request__Sequence
{
  custom_interfaces__action__OdomRecord_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_GetResult_Response
{
  int8_t status;
  custom_interfaces__action__OdomRecord_Result result;
} custom_interfaces__action__OdomRecord_GetResult_Response;

// Struct for a sequence of custom_interfaces__action__OdomRecord_GetResult_Response.
typedef struct custom_interfaces__action__OdomRecord_GetResult_Response__Sequence
{
  custom_interfaces__action__OdomRecord_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"

// Struct defined in action/OdomRecord in the package custom_interfaces.
typedef struct custom_interfaces__action__OdomRecord_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__OdomRecord_Feedback feedback;
} custom_interfaces__action__OdomRecord_FeedbackMessage;

// Struct for a sequence of custom_interfaces__action__OdomRecord_FeedbackMessage.
typedef struct custom_interfaces__action__OdomRecord_FeedbackMessage__Sequence
{
  custom_interfaces__action__OdomRecord_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__OdomRecord_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__ODOM_RECORD__STRUCT_H_

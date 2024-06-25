// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "scout_msgs/msg/detail/scout_status__struct.h"
#include "scout_msgs/msg/detail/scout_status__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool scout_msgs__msg__scout_actuator_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * scout_msgs__msg__scout_actuator_state__convert_to_py(void * raw_ros_message);
bool scout_msgs__msg__scout_light_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * scout_msgs__msg__scout_light_state__convert_to_py(void * raw_ros_message);
bool scout_msgs__msg__scout_light_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * scout_msgs__msg__scout_light_state__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool scout_msgs__msg__scout_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("scout_msgs.msg._scout_status.ScoutStatus", full_classname_dest, 40) == 0);
  }
  scout_msgs__msg__ScoutStatus * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // linear_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->linear_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vehicle_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vehicle_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // control_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_code = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // actuator_states
    PyObject * field = PyObject_GetAttrString(_pymsg, "actuator_states");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'actuator_states'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = 4;
    scout_msgs__msg__ScoutActuatorState * dest = ros_message->actuator_states;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!scout_msgs__msg__scout_actuator_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // light_control_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_control_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->light_control_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // front_light_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_light_state");
    if (!field) {
      return false;
    }
    if (!scout_msgs__msg__scout_light_state__convert_from_py(field, &ros_message->front_light_state)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // rear_light_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_light_state");
    if (!field) {
      return false;
    }
    if (!scout_msgs__msg__scout_light_state__convert_from_py(field, &ros_message->rear_light_state)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * scout_msgs__msg__scout_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ScoutStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("scout_msgs.msg._scout_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ScoutStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  scout_msgs__msg__ScoutStatus * ros_message = (scout_msgs__msg__ScoutStatus *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->linear_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vehicle_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->vehicle_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->control_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // actuator_states
    PyObject * field = NULL;
    size_t size = 4;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    scout_msgs__msg__ScoutActuatorState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->actuator_states[i]);
      PyObject * pyitem = scout_msgs__msg__scout_actuator_state__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "actuator_states", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_control_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->light_control_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_control_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_light_state
    PyObject * field = NULL;
    field = scout_msgs__msg__scout_light_state__convert_to_py(&ros_message->front_light_state);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_light_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_light_state
    PyObject * field = NULL;
    field = scout_msgs__msg__scout_light_state__convert_to_py(&ros_message->rear_light_state);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_light_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
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
#include "scout_msgs/msg/detail/scout_light_cmd__struct.h"
#include "scout_msgs/msg/detail/scout_light_cmd__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool scout_msgs__msg__scout_light_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("scout_msgs.msg._scout_light_cmd.ScoutLightCmd", full_classname_dest, 45) == 0);
  }
  scout_msgs__msg__ScoutLightCmd * ros_message = _ros_message;
  {  // cmd_ctrl_allowed
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_ctrl_allowed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cmd_ctrl_allowed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // front_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->front_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // front_custom_value
    PyObject * field = PyObject_GetAttrString(_pymsg, "front_custom_value");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->front_custom_value = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rear_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rear_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rear_custom_value
    PyObject * field = PyObject_GetAttrString(_pymsg, "rear_custom_value");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rear_custom_value = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * scout_msgs__msg__scout_light_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ScoutLightCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("scout_msgs.msg._scout_light_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ScoutLightCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  scout_msgs__msg__ScoutLightCmd * ros_message = (scout_msgs__msg__ScoutLightCmd *)raw_ros_message;
  {  // cmd_ctrl_allowed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cmd_ctrl_allowed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_ctrl_allowed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->front_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front_custom_value
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->front_custom_value);
    {
      int rc = PyObject_SetAttrString(_pymessage, "front_custom_value", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rear_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rear_custom_value
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rear_custom_value);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rear_custom_value", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
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
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__struct.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_feature_contents__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[97];
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
    assert(strncmp("ti_mmwave_rospkg_msgs.msg._radar_micro_doppler_feature_contents.RadarMicroDopplerFeatureContents", full_classname_dest, 96) == 0);
  }
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * ros_message = _ros_message;
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
  {  // f_low
    PyObject * field = PyObject_GetAttrString(_pymsg, "f_low");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->f_low = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // f_up
    PyObject * field = PyObject_GetAttrString(_pymsg, "f_up");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->f_up = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bw_pwr
    PyObject * field = PyObject_GetAttrString(_pymsg, "bw_pwr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bw_pwr = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mean_freq
    PyObject * field = PyObject_GetAttrString(_pymsg, "mean_freq");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mean_freq = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // med_freq
    PyObject * field = PyObject_GetAttrString(_pymsg, "med_freq");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->med_freq = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // s_entropy
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_entropy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_entropy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_feature_contents__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RadarMicroDopplerFeatureContents */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ti_mmwave_rospkg_msgs.msg._radar_micro_doppler_feature_contents");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RadarMicroDopplerFeatureContents");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * ros_message = (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents *)raw_ros_message;
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
  {  // f_low
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->f_low);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f_low", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // f_up
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->f_up);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f_up", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bw_pwr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bw_pwr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bw_pwr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mean_freq
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mean_freq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mean_freq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // med_freq
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->med_freq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "med_freq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_entropy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_entropy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_entropy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

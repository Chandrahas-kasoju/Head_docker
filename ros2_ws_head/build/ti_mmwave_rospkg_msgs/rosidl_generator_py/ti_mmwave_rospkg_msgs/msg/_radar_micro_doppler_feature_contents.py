# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadarMicroDopplerFeatureContents(type):
    """Metaclass of message 'RadarMicroDopplerFeatureContents'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ti_mmwave_rospkg_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ti_mmwave_rospkg_msgs.msg.RadarMicroDopplerFeatureContents')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radar_micro_doppler_feature_contents
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radar_micro_doppler_feature_contents
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radar_micro_doppler_feature_contents
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radar_micro_doppler_feature_contents
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radar_micro_doppler_feature_contents

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RadarMicroDopplerFeatureContents(metaclass=Metaclass_RadarMicroDopplerFeatureContents):
    """Message class 'RadarMicroDopplerFeatureContents'."""

    __slots__ = [
        '_header',
        '_f_low',
        '_f_up',
        '_bw_pwr',
        '_mean_freq',
        '_med_freq',
        '_s_entropy',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'f_low': 'float',
        'f_up': 'float',
        'bw_pwr': 'float',
        'mean_freq': 'float',
        'med_freq': 'float',
        's_entropy': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.f_low = kwargs.get('f_low', float())
        self.f_up = kwargs.get('f_up', float())
        self.bw_pwr = kwargs.get('bw_pwr', float())
        self.mean_freq = kwargs.get('mean_freq', float())
        self.med_freq = kwargs.get('med_freq', float())
        self.s_entropy = kwargs.get('s_entropy', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.f_low != other.f_low:
            return False
        if self.f_up != other.f_up:
            return False
        if self.bw_pwr != other.bw_pwr:
            return False
        if self.mean_freq != other.mean_freq:
            return False
        if self.med_freq != other.med_freq:
            return False
        if self.s_entropy != other.s_entropy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def f_low(self):
        """Message field 'f_low'."""
        return self._f_low

    @f_low.setter
    def f_low(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'f_low' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'f_low' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._f_low = value

    @builtins.property
    def f_up(self):
        """Message field 'f_up'."""
        return self._f_up

    @f_up.setter
    def f_up(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'f_up' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'f_up' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._f_up = value

    @builtins.property
    def bw_pwr(self):
        """Message field 'bw_pwr'."""
        return self._bw_pwr

    @bw_pwr.setter
    def bw_pwr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bw_pwr' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bw_pwr' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bw_pwr = value

    @builtins.property
    def mean_freq(self):
        """Message field 'mean_freq'."""
        return self._mean_freq

    @mean_freq.setter
    def mean_freq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mean_freq' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'mean_freq' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._mean_freq = value

    @builtins.property
    def med_freq(self):
        """Message field 'med_freq'."""
        return self._med_freq

    @med_freq.setter
    def med_freq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'med_freq' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'med_freq' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._med_freq = value

    @builtins.property
    def s_entropy(self):
        """Message field 's_entropy'."""
        return self._s_entropy

    @s_entropy.setter
    def s_entropy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_entropy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 's_entropy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._s_entropy = value

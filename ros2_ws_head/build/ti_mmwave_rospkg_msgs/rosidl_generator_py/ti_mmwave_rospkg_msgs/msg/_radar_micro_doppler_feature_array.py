# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadarMicroDopplerFeatureArray(type):
    """Metaclass of message 'RadarMicroDopplerFeatureArray'."""

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
                'ti_mmwave_rospkg_msgs.msg.RadarMicroDopplerFeatureArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radar_micro_doppler_feature_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radar_micro_doppler_feature_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radar_micro_doppler_feature_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radar_micro_doppler_feature_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radar_micro_doppler_feature_array

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ti_mmwave_rospkg_msgs.msg import RadarMicroDopplerFeatureContents
            if RadarMicroDopplerFeatureContents.__class__._TYPE_SUPPORT is None:
                RadarMicroDopplerFeatureContents.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RadarMicroDopplerFeatureArray(metaclass=Metaclass_RadarMicroDopplerFeatureArray):
    """Message class 'RadarMicroDopplerFeatureArray'."""

    __slots__ = [
        '_header',
        '_num_tracks',
        '_track',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'num_tracks': 'uint32',
        'track': 'sequence<ti_mmwave_rospkg_msgs/RadarMicroDopplerFeatureContents>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ti_mmwave_rospkg_msgs', 'msg'], 'RadarMicroDopplerFeatureContents')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.num_tracks = kwargs.get('num_tracks', int())
        self.track = kwargs.get('track', [])

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
        if self.num_tracks != other.num_tracks:
            return False
        if self.track != other.track:
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
    def num_tracks(self):
        """Message field 'num_tracks'."""
        return self._num_tracks

    @num_tracks.setter
    def num_tracks(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_tracks' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_tracks' field must be an unsigned integer in [0, 4294967295]"
        self._num_tracks = value

    @builtins.property
    def track(self):
        """Message field 'track'."""
        return self._track

    @track.setter
    def track(self, value):
        if __debug__:
            from ti_mmwave_rospkg_msgs.msg import RadarMicroDopplerFeatureContents
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, RadarMicroDopplerFeatureContents) for v in value) and
                 True), \
                "The 'track' field must be a set or sequence and each value of type 'RadarMicroDopplerFeatureContents'"
        self._track = value

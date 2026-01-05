# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ti_mmwave_rospkg_msgs:msg/RadarTrackContents.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadarTrackContents(type):
    """Metaclass of message 'RadarTrackContents'."""

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
                'ti_mmwave_rospkg_msgs.msg.RadarTrackContents')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radar_track_contents
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radar_track_contents
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radar_track_contents
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radar_track_contents
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radar_track_contents

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


class RadarTrackContents(metaclass=Metaclass_RadarTrackContents):
    """Message class 'RadarTrackContents'."""

    __slots__ = [
        '_header',
        '_tid',
        '_posx',
        '_posy',
        '_posz',
        '_velx',
        '_vely',
        '_velz',
        '_accx',
        '_accy',
        '_accz',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'tid': 'uint32',
        'posx': 'float',
        'posy': 'float',
        'posz': 'float',
        'velx': 'float',
        'vely': 'float',
        'velz': 'float',
        'accx': 'float',
        'accy': 'float',
        'accz': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.tid = kwargs.get('tid', int())
        self.posx = kwargs.get('posx', float())
        self.posy = kwargs.get('posy', float())
        self.posz = kwargs.get('posz', float())
        self.velx = kwargs.get('velx', float())
        self.vely = kwargs.get('vely', float())
        self.velz = kwargs.get('velz', float())
        self.accx = kwargs.get('accx', float())
        self.accy = kwargs.get('accy', float())
        self.accz = kwargs.get('accz', float())

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
        if self.tid != other.tid:
            return False
        if self.posx != other.posx:
            return False
        if self.posy != other.posy:
            return False
        if self.posz != other.posz:
            return False
        if self.velx != other.velx:
            return False
        if self.vely != other.vely:
            return False
        if self.velz != other.velz:
            return False
        if self.accx != other.accx:
            return False
        if self.accy != other.accy:
            return False
        if self.accz != other.accz:
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
    def tid(self):
        """Message field 'tid'."""
        return self._tid

    @tid.setter
    def tid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tid' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'tid' field must be an unsigned integer in [0, 4294967295]"
        self._tid = value

    @builtins.property
    def posx(self):
        """Message field 'posx'."""
        return self._posx

    @posx.setter
    def posx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'posx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'posx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._posx = value

    @builtins.property
    def posy(self):
        """Message field 'posy'."""
        return self._posy

    @posy.setter
    def posy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'posy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'posy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._posy = value

    @builtins.property
    def posz(self):
        """Message field 'posz'."""
        return self._posz

    @posz.setter
    def posz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'posz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'posz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._posz = value

    @builtins.property
    def velx(self):
        """Message field 'velx'."""
        return self._velx

    @velx.setter
    def velx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velx = value

    @builtins.property
    def vely(self):
        """Message field 'vely'."""
        return self._vely

    @vely.setter
    def vely(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vely' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vely' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vely = value

    @builtins.property
    def velz(self):
        """Message field 'velz'."""
        return self._velz

    @velz.setter
    def velz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velz = value

    @builtins.property
    def accx(self):
        """Message field 'accx'."""
        return self._accx

    @accx.setter
    def accx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'accx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._accx = value

    @builtins.property
    def accy(self):
        """Message field 'accy'."""
        return self._accy

    @accy.setter
    def accy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'accy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._accy = value

    @builtins.property
    def accz(self):
        """Message field 'accz'."""
        return self._accz

    @accz.setter
    def accz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'accz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._accz = value

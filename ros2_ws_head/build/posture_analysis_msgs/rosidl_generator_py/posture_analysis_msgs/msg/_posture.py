# generated from rosidl_generator_py/resource/_idl.py.em
# with input from posture_analysis_msgs:msg/Posture.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Posture(type):
    """Metaclass of message 'Posture'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': 0,
        'LYING_DOWN': 1,
        'SITTING': 2,
        'STANDING': 3,
        'UPRIGHT': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('posture_analysis_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'posture_analysis_msgs.msg.Posture')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__posture
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__posture
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__posture
            cls._TYPE_SUPPORT = module.type_support_msg__msg__posture
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__posture

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'LYING_DOWN': cls.__constants['LYING_DOWN'],
            'SITTING': cls.__constants['SITTING'],
            'STANDING': cls.__constants['STANDING'],
            'UPRIGHT': cls.__constants['UPRIGHT'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_Posture.__constants['UNKNOWN']

    @property
    def LYING_DOWN(self):
        """Message constant 'LYING_DOWN'."""
        return Metaclass_Posture.__constants['LYING_DOWN']

    @property
    def SITTING(self):
        """Message constant 'SITTING'."""
        return Metaclass_Posture.__constants['SITTING']

    @property
    def STANDING(self):
        """Message constant 'STANDING'."""
        return Metaclass_Posture.__constants['STANDING']

    @property
    def UPRIGHT(self):
        """Message constant 'UPRIGHT'."""
        return Metaclass_Posture.__constants['UPRIGHT']


class Posture(metaclass=Metaclass_Posture):
    """
    Message class 'Posture'.

    Constants:
      UNKNOWN
      LYING_DOWN
      SITTING
      STANDING
      UPRIGHT
    """

    __slots__ = [
        '_posture_class',
        '_uprightness_score',
    ]

    _fields_and_field_types = {
        'posture_class': 'int32',
        'uprightness_score': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.posture_class = kwargs.get('posture_class', int())
        self.uprightness_score = kwargs.get('uprightness_score', int())

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
        if self.posture_class != other.posture_class:
            return False
        if self.uprightness_score != other.uprightness_score:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def posture_class(self):
        """Message field 'posture_class'."""
        return self._posture_class

    @posture_class.setter
    def posture_class(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'posture_class' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'posture_class' field must be an integer in [-2147483648, 2147483647]"
        self._posture_class = value

    @builtins.property
    def uprightness_score(self):
        """Message field 'uprightness_score'."""
        return self._uprightness_score

    @uprightness_score.setter
    def uprightness_score(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'uprightness_score' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'uprightness_score' field must be an integer in [-2147483648, 2147483647]"
        self._uprightness_score = value

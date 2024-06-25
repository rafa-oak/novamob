# generated from rosidl_generator_py/resource/_idl.py.em
# with input from scout_msgs:msg/ScoutLightCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ScoutLightCmd(type):
    """Metaclass of message 'ScoutLightCmd'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'LIGHT_CONST_OFF': 0,
        'LIGHT_CONST_ON': 1,
        'LIGHT_BREATH': 2,
        'LIGHT_CUSTOM': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('scout_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'scout_msgs.msg.ScoutLightCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__scout_light_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__scout_light_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__scout_light_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__scout_light_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__scout_light_cmd

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LIGHT_CONST_OFF': cls.__constants['LIGHT_CONST_OFF'],
            'LIGHT_CONST_ON': cls.__constants['LIGHT_CONST_ON'],
            'LIGHT_BREATH': cls.__constants['LIGHT_BREATH'],
            'LIGHT_CUSTOM': cls.__constants['LIGHT_CUSTOM'],
        }

    @property
    def LIGHT_CONST_OFF(self):
        """Message constant 'LIGHT_CONST_OFF'."""
        return Metaclass_ScoutLightCmd.__constants['LIGHT_CONST_OFF']

    @property
    def LIGHT_CONST_ON(self):
        """Message constant 'LIGHT_CONST_ON'."""
        return Metaclass_ScoutLightCmd.__constants['LIGHT_CONST_ON']

    @property
    def LIGHT_BREATH(self):
        """Message constant 'LIGHT_BREATH'."""
        return Metaclass_ScoutLightCmd.__constants['LIGHT_BREATH']

    @property
    def LIGHT_CUSTOM(self):
        """Message constant 'LIGHT_CUSTOM'."""
        return Metaclass_ScoutLightCmd.__constants['LIGHT_CUSTOM']


class ScoutLightCmd(metaclass=Metaclass_ScoutLightCmd):
    """
    Message class 'ScoutLightCmd'.

    Constants:
      LIGHT_CONST_OFF
      LIGHT_CONST_ON
      LIGHT_BREATH
      LIGHT_CUSTOM
    """

    __slots__ = [
        '_cmd_ctrl_allowed',
        '_front_mode',
        '_front_custom_value',
        '_rear_mode',
        '_rear_custom_value',
    ]

    _fields_and_field_types = {
        'cmd_ctrl_allowed': 'boolean',
        'front_mode': 'uint8',
        'front_custom_value': 'uint8',
        'rear_mode': 'uint8',
        'rear_custom_value': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd_ctrl_allowed = kwargs.get('cmd_ctrl_allowed', bool())
        self.front_mode = kwargs.get('front_mode', int())
        self.front_custom_value = kwargs.get('front_custom_value', int())
        self.rear_mode = kwargs.get('rear_mode', int())
        self.rear_custom_value = kwargs.get('rear_custom_value', int())

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
        if self.cmd_ctrl_allowed != other.cmd_ctrl_allowed:
            return False
        if self.front_mode != other.front_mode:
            return False
        if self.front_custom_value != other.front_custom_value:
            return False
        if self.rear_mode != other.rear_mode:
            return False
        if self.rear_custom_value != other.rear_custom_value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd_ctrl_allowed(self):
        """Message field 'cmd_ctrl_allowed'."""
        return self._cmd_ctrl_allowed

    @cmd_ctrl_allowed.setter
    def cmd_ctrl_allowed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cmd_ctrl_allowed' field must be of type 'bool'"
        self._cmd_ctrl_allowed = value

    @builtins.property
    def front_mode(self):
        """Message field 'front_mode'."""
        return self._front_mode

    @front_mode.setter
    def front_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'front_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'front_mode' field must be an unsigned integer in [0, 255]"
        self._front_mode = value

    @builtins.property
    def front_custom_value(self):
        """Message field 'front_custom_value'."""
        return self._front_custom_value

    @front_custom_value.setter
    def front_custom_value(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'front_custom_value' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'front_custom_value' field must be an unsigned integer in [0, 255]"
        self._front_custom_value = value

    @builtins.property
    def rear_mode(self):
        """Message field 'rear_mode'."""
        return self._rear_mode

    @rear_mode.setter
    def rear_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rear_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rear_mode' field must be an unsigned integer in [0, 255]"
        self._rear_mode = value

    @builtins.property
    def rear_custom_value(self):
        """Message field 'rear_custom_value'."""
        return self._rear_custom_value

    @rear_custom_value.setter
    def rear_custom_value(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rear_custom_value' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rear_custom_value' field must be an unsigned integer in [0, 255]"
        self._rear_custom_value = value

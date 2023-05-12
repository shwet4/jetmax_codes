# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from jetmax_control/JetMax.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class JetMax(genpy.Message):
  _md5sum = "98e79b4f27f832f857f4f7315fd89046"
  _type = "jetmax_control/JetMax"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 x
float32 y
float32 z
float32 joint1
float32 joint2
float32 joint3
float32 servo1
float32 servo2
float32 servo3
float32 pwm1
float32 pwm2
bool    sucker
"""
  __slots__ = ['x','y','z','joint1','joint2','joint3','servo1','servo2','servo3','pwm1','pwm2','sucker']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z,joint1,joint2,joint3,servo1,servo2,servo3,pwm1,pwm2,sucker

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(JetMax, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.joint1 is None:
        self.joint1 = 0.
      if self.joint2 is None:
        self.joint2 = 0.
      if self.joint3 is None:
        self.joint3 = 0.
      if self.servo1 is None:
        self.servo1 = 0.
      if self.servo2 is None:
        self.servo2 = 0.
      if self.servo3 is None:
        self.servo3 = 0.
      if self.pwm1 is None:
        self.pwm1 = 0.
      if self.pwm2 is None:
        self.pwm2 = 0.
      if self.sucker is None:
        self.sucker = False
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.joint1 = 0.
      self.joint2 = 0.
      self.joint3 = 0.
      self.servo1 = 0.
      self.servo2 = 0.
      self.servo3 = 0.
      self.pwm1 = 0.
      self.pwm2 = 0.
      self.sucker = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_11fB().pack(_x.x, _x.y, _x.z, _x.joint1, _x.joint2, _x.joint3, _x.servo1, _x.servo2, _x.servo3, _x.pwm1, _x.pwm2, _x.sucker))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 45
      (_x.x, _x.y, _x.z, _x.joint1, _x.joint2, _x.joint3, _x.servo1, _x.servo2, _x.servo3, _x.pwm1, _x.pwm2, _x.sucker,) = _get_struct_11fB().unpack(str[start:end])
      self.sucker = bool(self.sucker)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_11fB().pack(_x.x, _x.y, _x.z, _x.joint1, _x.joint2, _x.joint3, _x.servo1, _x.servo2, _x.servo3, _x.pwm1, _x.pwm2, _x.sucker))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 45
      (_x.x, _x.y, _x.z, _x.joint1, _x.joint2, _x.joint3, _x.servo1, _x.servo2, _x.servo3, _x.pwm1, _x.pwm2, _x.sucker,) = _get_struct_11fB().unpack(str[start:end])
      self.sucker = bool(self.sucker)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_11fB = None
def _get_struct_11fB():
    global _struct_11fB
    if _struct_11fB is None:
        _struct_11fB = struct.Struct("<11fB")
    return _struct_11fB

# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from viso2_ros/VisoInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class VisoInfo(genpy.Message):
  _md5sum = "765500d8b83bf74f7715c6e2e8e89092"
  _type = "viso2_ros/VisoInfo"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# Internal information on the
# viso2 algorithm parameters
# and results

Header header

# True if the previous iteration of viso2
# was not able to complete the matching process
# therefore the visual odometer is re-started.
bool got_lost

# True if in the next run the reference 
# frame will be changed. This is the case
# when the number of inliers drops below
# a threshold or the previous motion estimate
# failed in last motion estimation.
bool change_reference_frame

# info from motion estimator
bool motion_estimate_valid
int32 num_matches
int32 num_inliers

# runtime of last iteration in seconds
float64 runtime

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','got_lost','change_reference_frame','motion_estimate_valid','num_matches','num_inliers','runtime']
  _slot_types = ['std_msgs/Header','bool','bool','bool','int32','int32','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,got_lost,change_reference_frame,motion_estimate_valid,num_matches,num_inliers,runtime

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VisoInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.got_lost is None:
        self.got_lost = False
      if self.change_reference_frame is None:
        self.change_reference_frame = False
      if self.motion_estimate_valid is None:
        self.motion_estimate_valid = False
      if self.num_matches is None:
        self.num_matches = 0
      if self.num_inliers is None:
        self.num_inliers = 0
      if self.runtime is None:
        self.runtime = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.got_lost = False
      self.change_reference_frame = False
      self.motion_estimate_valid = False
      self.num_matches = 0
      self.num_inliers = 0
      self.runtime = 0.

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3B2id().pack(_x.got_lost, _x.change_reference_frame, _x.motion_estimate_valid, _x.num_matches, _x.num_inliers, _x.runtime))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 19
      (_x.got_lost, _x.change_reference_frame, _x.motion_estimate_valid, _x.num_matches, _x.num_inliers, _x.runtime,) = _get_struct_3B2id().unpack(str[start:end])
      self.got_lost = bool(self.got_lost)
      self.change_reference_frame = bool(self.change_reference_frame)
      self.motion_estimate_valid = bool(self.motion_estimate_valid)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3B2id().pack(_x.got_lost, _x.change_reference_frame, _x.motion_estimate_valid, _x.num_matches, _x.num_inliers, _x.runtime))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 19
      (_x.got_lost, _x.change_reference_frame, _x.motion_estimate_valid, _x.num_matches, _x.num_inliers, _x.runtime,) = _get_struct_3B2id().unpack(str[start:end])
      self.got_lost = bool(self.got_lost)
      self.change_reference_frame = bool(self.change_reference_frame)
      self.motion_estimate_valid = bool(self.motion_estimate_valid)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3B2id = None
def _get_struct_3B2id():
    global _struct_3B2id
    if _struct_3B2id is None:
        _struct_3B2id = struct.Struct("<3B2id")
    return _struct_3B2id
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I

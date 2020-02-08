; Auto-generated. Do not edit!


(cl:in-package fourth_proj-msg)


;//! \htmlinclude cmdmsg.msg.html

(cl:defclass <cmdmsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vl
    :reader vl
    :initarg :vl
    :type cl:float
    :initform 0.0)
   (vr
    :reader vr
    :initarg :vr
    :type cl:float
    :initform 0.0))
)

(cl:defclass cmdmsg (<cmdmsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmdmsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmdmsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fourth_proj-msg:<cmdmsg> is deprecated: use fourth_proj-msg:cmdmsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <cmdmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-msg:header-val is deprecated.  Use fourth_proj-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vl-val :lambda-list '(m))
(cl:defmethod vl-val ((m <cmdmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-msg:vl-val is deprecated.  Use fourth_proj-msg:vl instead.")
  (vl m))

(cl:ensure-generic-function 'vr-val :lambda-list '(m))
(cl:defmethod vr-val ((m <cmdmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-msg:vr-val is deprecated.  Use fourth_proj-msg:vr instead.")
  (vr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmdmsg>) ostream)
  "Serializes a message object of type '<cmdmsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmdmsg>) istream)
  "Deserializes a message object of type '<cmdmsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vl) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vr) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmdmsg>)))
  "Returns string type for a message object of type '<cmdmsg>"
  "fourth_proj/cmdmsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmdmsg)))
  "Returns string type for a message object of type 'cmdmsg"
  "fourth_proj/cmdmsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmdmsg>)))
  "Returns md5sum for a message object of type '<cmdmsg>"
  "4f54d6e9a6e07cae4bfef7c0f8143142")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmdmsg)))
  "Returns md5sum for a message object of type 'cmdmsg"
  "4f54d6e9a6e07cae4bfef7c0f8143142")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmdmsg>)))
  "Returns full string definition for message of type '<cmdmsg>"
  (cl:format cl:nil "Header header~%float64 vl~%float64 vr~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmdmsg)))
  "Returns full string definition for message of type 'cmdmsg"
  (cl:format cl:nil "Header header~%float64 vl~%float64 vr~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmdmsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmdmsg>))
  "Converts a ROS message object to a list"
  (cl:list 'cmdmsg
    (cl:cons ':header (header msg))
    (cl:cons ':vl (vl msg))
    (cl:cons ':vr (vr msg))
))

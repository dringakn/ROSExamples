; Auto-generated. Do not edit!


(cl:in-package fourth_proj-srv)


;//! \htmlinclude addsrv-request.msg.html

(cl:defclass <addsrv-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0))
)

(cl:defclass addsrv-request (<addsrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <addsrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'addsrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fourth_proj-srv:<addsrv-request> is deprecated: use fourth_proj-srv:addsrv-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <addsrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-srv:a-val is deprecated.  Use fourth_proj-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <addsrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-srv:b-val is deprecated.  Use fourth_proj-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <addsrv-request>) ostream)
  "Serializes a message object of type '<addsrv-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <addsrv-request>) istream)
  "Deserializes a message object of type '<addsrv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<addsrv-request>)))
  "Returns string type for a service object of type '<addsrv-request>"
  "fourth_proj/addsrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addsrv-request)))
  "Returns string type for a service object of type 'addsrv-request"
  "fourth_proj/addsrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<addsrv-request>)))
  "Returns md5sum for a message object of type '<addsrv-request>"
  "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'addsrv-request)))
  "Returns md5sum for a message object of type 'addsrv-request"
  "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<addsrv-request>)))
  "Returns full string definition for message of type '<addsrv-request>"
  (cl:format cl:nil "float64 a~%float64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'addsrv-request)))
  "Returns full string definition for message of type 'addsrv-request"
  (cl:format cl:nil "float64 a~%float64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <addsrv-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <addsrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'addsrv-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude addsrv-response.msg.html

(cl:defclass <addsrv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:float
    :initform 0.0))
)

(cl:defclass addsrv-response (<addsrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <addsrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'addsrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fourth_proj-srv:<addsrv-response> is deprecated: use fourth_proj-srv:addsrv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <addsrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fourth_proj-srv:result-val is deprecated.  Use fourth_proj-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <addsrv-response>) ostream)
  "Serializes a message object of type '<addsrv-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <addsrv-response>) istream)
  "Deserializes a message object of type '<addsrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'result) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<addsrv-response>)))
  "Returns string type for a service object of type '<addsrv-response>"
  "fourth_proj/addsrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addsrv-response)))
  "Returns string type for a service object of type 'addsrv-response"
  "fourth_proj/addsrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<addsrv-response>)))
  "Returns md5sum for a message object of type '<addsrv-response>"
  "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'addsrv-response)))
  "Returns md5sum for a message object of type 'addsrv-response"
  "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<addsrv-response>)))
  "Returns full string definition for message of type '<addsrv-response>"
  (cl:format cl:nil "float64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'addsrv-response)))
  "Returns full string definition for message of type 'addsrv-response"
  (cl:format cl:nil "float64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <addsrv-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <addsrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'addsrv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'addsrv)))
  'addsrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'addsrv)))
  'addsrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addsrv)))
  "Returns string type for a service object of type '<addsrv>"
  "fourth_proj/addsrv")
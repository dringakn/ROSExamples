;; Auto-generated. Do not edit!


(when (boundp 'fourth_proj::cmdmsg)
  (if (not (find-package "FOURTH_PROJ"))
    (make-package "FOURTH_PROJ"))
  (shadow 'cmdmsg (find-package "FOURTH_PROJ")))
(unless (find-package "FOURTH_PROJ::CMDMSG")
  (make-package "FOURTH_PROJ::CMDMSG"))

(in-package "ROS")
;;//! \htmlinclude cmdmsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fourth_proj::cmdmsg
  :super ros::object
  :slots (_header _vl _vr ))

(defmethod fourth_proj::cmdmsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vl __vl) 0.0)
    ((:vr __vr) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vl (float __vl))
   (setq _vr (float __vr))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vl
   (&optional __vl)
   (if __vl (setq _vl __vl)) _vl)
  (:vr
   (&optional __vr)
   (if __vr (setq _vr __vr)) _vr)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _vl
    8
    ;; float64 _vr
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _vl
       (sys::poke _vl (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vr
       (sys::poke _vr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _vl
     (setq _vl (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vr
     (setq _vr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fourth_proj::cmdmsg :md5sum-) "4f54d6e9a6e07cae4bfef7c0f8143142")
(setf (get fourth_proj::cmdmsg :datatype-) "fourth_proj/cmdmsg")
(setf (get fourth_proj::cmdmsg :definition-)
      "Header header
float64 vl
float64 vr

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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :fourth_proj/cmdmsg "4f54d6e9a6e07cae4bfef7c0f8143142")



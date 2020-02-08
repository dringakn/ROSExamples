;; Auto-generated. Do not edit!


(when (boundp 'fourth_proj::sensmsg)
  (if (not (find-package "FOURTH_PROJ"))
    (make-package "FOURTH_PROJ"))
  (shadow 'sensmsg (find-package "FOURTH_PROJ")))
(unless (find-package "FOURTH_PROJ::SENSMSG")
  (make-package "FOURTH_PROJ::SENSMSG"))

(in-package "ROS")
;;//! \htmlinclude sensmsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fourth_proj::sensmsg
  :super ros::object
  :slots (_header _front _left _right ))

(defmethod fourth_proj::sensmsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:front __front) 0.0)
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _front (float __front))
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:front
   (&optional __front)
   (if __front (setq _front __front)) _front)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _front
    8
    ;; float64 _left
    8
    ;; float64 _right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _front
       (sys::poke _front (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left
       (sys::poke _left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right
       (sys::poke _right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _front
     (setq _front (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left
     (setq _left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right
     (setq _right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fourth_proj::sensmsg :md5sum-) "21187edbb23f50ee9495d83e3795ee9f")
(setf (get fourth_proj::sensmsg :datatype-) "fourth_proj/sensmsg")
(setf (get fourth_proj::sensmsg :definition-)
      "Header header
float64 front
float64 left
float64 right
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



(provide :fourth_proj/sensmsg "21187edbb23f50ee9495d83e3795ee9f")



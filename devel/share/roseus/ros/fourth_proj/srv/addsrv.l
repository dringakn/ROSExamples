;; Auto-generated. Do not edit!


(when (boundp 'fourth_proj::addsrv)
  (if (not (find-package "FOURTH_PROJ"))
    (make-package "FOURTH_PROJ"))
  (shadow 'addsrv (find-package "FOURTH_PROJ")))
(unless (find-package "FOURTH_PROJ::ADDSRV")
  (make-package "FOURTH_PROJ::ADDSRV"))
(unless (find-package "FOURTH_PROJ::ADDSRVREQUEST")
  (make-package "FOURTH_PROJ::ADDSRVREQUEST"))
(unless (find-package "FOURTH_PROJ::ADDSRVRESPONSE")
  (make-package "FOURTH_PROJ::ADDSRVRESPONSE"))

(in-package "ROS")





(defclass fourth_proj::addsrvRequest
  :super ros::object
  :slots (_a _b ))

(defmethod fourth_proj::addsrvRequest
  (:init
   (&key
    ((:a __a) 0.0)
    ((:b __b) 0.0)
    )
   (send-super :init)
   (setq _a (float __a))
   (setq _b (float __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; float64 _a
    8
    ;; float64 _b
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b
       (sys::poke _b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b
     (setq _b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass fourth_proj::addsrvResponse
  :super ros::object
  :slots (_result ))

(defmethod fourth_proj::addsrvResponse
  (:init
   (&key
    ((:result __result) 0.0)
    )
   (send-super :init)
   (setq _result (float __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; float64 _result
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _result
       (sys::poke _result (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _result
     (setq _result (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass fourth_proj::addsrv
  :super ros::object
  :slots ())

(setf (get fourth_proj::addsrv :md5sum-) "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(setf (get fourth_proj::addsrv :datatype-) "fourth_proj/addsrv")
(setf (get fourth_proj::addsrv :request) fourth_proj::addsrvRequest)
(setf (get fourth_proj::addsrv :response) fourth_proj::addsrvResponse)

(defmethod fourth_proj::addsrvRequest
  (:response () (instance fourth_proj::addsrvResponse :init)))

(setf (get fourth_proj::addsrvRequest :md5sum-) "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(setf (get fourth_proj::addsrvRequest :datatype-) "fourth_proj/addsrvRequest")
(setf (get fourth_proj::addsrvRequest :definition-)
      "float64 a
float64 b
---
float64 result

")

(setf (get fourth_proj::addsrvResponse :md5sum-) "6e7c4f317ab6ba70dfa7c335adfdc1c7")
(setf (get fourth_proj::addsrvResponse :datatype-) "fourth_proj/addsrvResponse")
(setf (get fourth_proj::addsrvResponse :definition-)
      "float64 a
float64 b
---
float64 result

")



(provide :fourth_proj/addsrv "6e7c4f317ab6ba70dfa7c335adfdc1c7")



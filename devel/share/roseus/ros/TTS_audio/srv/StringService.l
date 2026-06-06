;; Auto-generated. Do not edit!


(when (boundp 'TTS_audio::StringService)
  (if (not (find-package "TTS_AUDIO"))
    (make-package "TTS_AUDIO"))
  (shadow 'StringService (find-package "TTS_AUDIO")))
(unless (find-package "TTS_AUDIO::STRINGSERVICE")
  (make-package "TTS_AUDIO::STRINGSERVICE"))
(unless (find-package "TTS_AUDIO::STRINGSERVICEREQUEST")
  (make-package "TTS_AUDIO::STRINGSERVICEREQUEST"))
(unless (find-package "TTS_AUDIO::STRINGSERVICERESPONSE")
  (make-package "TTS_AUDIO::STRINGSERVICERESPONSE"))

(in-package "ROS")





(defclass TTS_audio::StringServiceRequest
  :super ros::object
  :slots (_data ))

(defmethod TTS_audio::StringServiceRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass TTS_audio::StringServiceResponse
  :super ros::object
  :slots (_result ))

(defmethod TTS_audio::StringServiceResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass TTS_audio::StringService
  :super ros::object
  :slots ())

(setf (get TTS_audio::StringService :md5sum-) "7229f1b0de733d7956dd646adcbf7e06")
(setf (get TTS_audio::StringService :datatype-) "TTS_audio/StringService")
(setf (get TTS_audio::StringService :request) TTS_audio::StringServiceRequest)
(setf (get TTS_audio::StringService :response) TTS_audio::StringServiceResponse)

(defmethod TTS_audio::StringServiceRequest
  (:response () (instance TTS_audio::StringServiceResponse :init)))

(setf (get TTS_audio::StringServiceRequest :md5sum-) "7229f1b0de733d7956dd646adcbf7e06")
(setf (get TTS_audio::StringServiceRequest :datatype-) "TTS_audio/StringServiceRequest")
(setf (get TTS_audio::StringServiceRequest :definition-)
      "string data
---
string result

")

(setf (get TTS_audio::StringServiceResponse :md5sum-) "7229f1b0de733d7956dd646adcbf7e06")
(setf (get TTS_audio::StringServiceResponse :datatype-) "TTS_audio/StringServiceResponse")
(setf (get TTS_audio::StringServiceResponse :definition-)
      "string data
---
string result

")



(provide :TTS_audio/StringService "7229f1b0de733d7956dd646adcbf7e06")



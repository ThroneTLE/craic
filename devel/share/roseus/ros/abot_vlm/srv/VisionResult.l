;; Auto-generated. Do not edit!


(when (boundp 'abot_vlm::VisionResult)
  (if (not (find-package "ABOT_VLM"))
    (make-package "ABOT_VLM"))
  (shadow 'VisionResult (find-package "ABOT_VLM")))
(unless (find-package "ABOT_VLM::VISIONRESULT")
  (make-package "ABOT_VLM::VISIONRESULT"))
(unless (find-package "ABOT_VLM::VISIONRESULTREQUEST")
  (make-package "ABOT_VLM::VISIONRESULTREQUEST"))
(unless (find-package "ABOT_VLM::VISIONRESULTRESPONSE")
  (make-package "ABOT_VLM::VISIONRESULTRESPONSE"))

(in-package "ROS")





(defclass abot_vlm::VisionResultRequest
  :super ros::object
  :slots (_result ))

(defmethod abot_vlm::VisionResultRequest
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

(defclass abot_vlm::VisionResultResponse
  :super ros::object
  :slots (_success ))

(defmethod abot_vlm::VisionResultResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass abot_vlm::VisionResult
  :super ros::object
  :slots ())

(setf (get abot_vlm::VisionResult :md5sum-) "5a308924dc8fc0f626a1724933b52b70")
(setf (get abot_vlm::VisionResult :datatype-) "abot_vlm/VisionResult")
(setf (get abot_vlm::VisionResult :request) abot_vlm::VisionResultRequest)
(setf (get abot_vlm::VisionResult :response) abot_vlm::VisionResultResponse)

(defmethod abot_vlm::VisionResultRequest
  (:response () (instance abot_vlm::VisionResultResponse :init)))

(setf (get abot_vlm::VisionResultRequest :md5sum-) "5a308924dc8fc0f626a1724933b52b70")
(setf (get abot_vlm::VisionResultRequest :datatype-) "abot_vlm/VisionResultRequest")
(setf (get abot_vlm::VisionResultRequest :definition-)
      "string result
---
bool success

")

(setf (get abot_vlm::VisionResultResponse :md5sum-) "5a308924dc8fc0f626a1724933b52b70")
(setf (get abot_vlm::VisionResultResponse :datatype-) "abot_vlm/VisionResultResponse")
(setf (get abot_vlm::VisionResultResponse :definition-)
      "string result
---
bool success

")



(provide :abot_vlm/VisionResult "5a308924dc8fc0f626a1724933b52b70")



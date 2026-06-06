;; Auto-generated. Do not edit!


(when (boundp 'abot_vlm::LLMQuery)
  (if (not (find-package "ABOT_VLM"))
    (make-package "ABOT_VLM"))
  (shadow 'LLMQuery (find-package "ABOT_VLM")))
(unless (find-package "ABOT_VLM::LLMQUERY")
  (make-package "ABOT_VLM::LLMQUERY"))
(unless (find-package "ABOT_VLM::LLMQUERYREQUEST")
  (make-package "ABOT_VLM::LLMQUERYREQUEST"))
(unless (find-package "ABOT_VLM::LLMQUERYRESPONSE")
  (make-package "ABOT_VLM::LLMQUERYRESPONSE"))

(in-package "ROS")





(defclass abot_vlm::LLMQueryRequest
  :super ros::object
  :slots (_query ))

(defmethod abot_vlm::LLMQueryRequest
  (:init
   (&key
    ((:query __query) "")
    )
   (send-super :init)
   (setq _query (string __query))
   self)
  (:query
   (&optional __query)
   (if __query (setq _query __query)) _query)
  (:serialization-length
   ()
   (+
    ;; string _query
    4 (length _query)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _query
       (write-long (length _query) s) (princ _query s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _query
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _query (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass abot_vlm::LLMQueryResponse
  :super ros::object
  :slots (_result ))

(defmethod abot_vlm::LLMQueryResponse
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

(defclass abot_vlm::LLMQuery
  :super ros::object
  :slots ())

(setf (get abot_vlm::LLMQuery :md5sum-) "40ece397ad679f27203bff340007bd45")
(setf (get abot_vlm::LLMQuery :datatype-) "abot_vlm/LLMQuery")
(setf (get abot_vlm::LLMQuery :request) abot_vlm::LLMQueryRequest)
(setf (get abot_vlm::LLMQuery :response) abot_vlm::LLMQueryResponse)

(defmethod abot_vlm::LLMQueryRequest
  (:response () (instance abot_vlm::LLMQueryResponse :init)))

(setf (get abot_vlm::LLMQueryRequest :md5sum-) "40ece397ad679f27203bff340007bd45")
(setf (get abot_vlm::LLMQueryRequest :datatype-) "abot_vlm/LLMQueryRequest")
(setf (get abot_vlm::LLMQueryRequest :definition-)
      "string query
---
string result

")

(setf (get abot_vlm::LLMQueryResponse :md5sum-) "40ece397ad679f27203bff340007bd45")
(setf (get abot_vlm::LLMQueryResponse :datatype-) "abot_vlm/LLMQueryResponse")
(setf (get abot_vlm::LLMQueryResponse :definition-)
      "string query
---
string result

")



(provide :abot_vlm/LLMQuery "40ece397ad679f27203bff340007bd45")



; Auto-generated. Do not edit!


(cl:in-package abot_vlm-srv)


;//! \htmlinclude VisionResult-request.msg.html

(cl:defclass <VisionResult-request> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass VisionResult-request (<VisionResult-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionResult-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionResult-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name abot_vlm-srv:<VisionResult-request> is deprecated: use abot_vlm-srv:VisionResult-request instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <VisionResult-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader abot_vlm-srv:result-val is deprecated.  Use abot_vlm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionResult-request>) ostream)
  "Serializes a message object of type '<VisionResult-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionResult-request>) istream)
  "Deserializes a message object of type '<VisionResult-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionResult-request>)))
  "Returns string type for a service object of type '<VisionResult-request>"
  "abot_vlm/VisionResultRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionResult-request)))
  "Returns string type for a service object of type 'VisionResult-request"
  "abot_vlm/VisionResultRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionResult-request>)))
  "Returns md5sum for a message object of type '<VisionResult-request>"
  "5a308924dc8fc0f626a1724933b52b70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionResult-request)))
  "Returns md5sum for a message object of type 'VisionResult-request"
  "5a308924dc8fc0f626a1724933b52b70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionResult-request>)))
  "Returns full string definition for message of type '<VisionResult-request>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionResult-request)))
  "Returns full string definition for message of type 'VisionResult-request"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionResult-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionResult-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionResult-request
    (cl:cons ':result (result msg))
))
;//! \htmlinclude VisionResult-response.msg.html

(cl:defclass <VisionResult-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VisionResult-response (<VisionResult-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionResult-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionResult-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name abot_vlm-srv:<VisionResult-response> is deprecated: use abot_vlm-srv:VisionResult-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VisionResult-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader abot_vlm-srv:success-val is deprecated.  Use abot_vlm-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionResult-response>) ostream)
  "Serializes a message object of type '<VisionResult-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionResult-response>) istream)
  "Deserializes a message object of type '<VisionResult-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionResult-response>)))
  "Returns string type for a service object of type '<VisionResult-response>"
  "abot_vlm/VisionResultResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionResult-response)))
  "Returns string type for a service object of type 'VisionResult-response"
  "abot_vlm/VisionResultResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionResult-response>)))
  "Returns md5sum for a message object of type '<VisionResult-response>"
  "5a308924dc8fc0f626a1724933b52b70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionResult-response)))
  "Returns md5sum for a message object of type 'VisionResult-response"
  "5a308924dc8fc0f626a1724933b52b70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionResult-response>)))
  "Returns full string definition for message of type '<VisionResult-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionResult-response)))
  "Returns full string definition for message of type 'VisionResult-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionResult-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionResult-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionResult-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VisionResult)))
  'VisionResult-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VisionResult)))
  'VisionResult-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionResult)))
  "Returns string type for a service object of type '<VisionResult>"
  "abot_vlm/VisionResult")
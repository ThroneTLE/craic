; Auto-generated. Do not edit!


(cl:in-package TTS_audio-srv)


;//! \htmlinclude StringService-request.msg.html

(cl:defclass <StringService-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass StringService-request (<StringService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<StringService-request> is deprecated: use TTS_audio-srv:StringService-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <StringService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:data-val is deprecated.  Use TTS_audio-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringService-request>) ostream)
  "Serializes a message object of type '<StringService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringService-request>) istream)
  "Deserializes a message object of type '<StringService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringService-request>)))
  "Returns string type for a service object of type '<StringService-request>"
  "TTS_audio/StringServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringService-request)))
  "Returns string type for a service object of type 'StringService-request"
  "TTS_audio/StringServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringService-request>)))
  "Returns md5sum for a message object of type '<StringService-request>"
  "7229f1b0de733d7956dd646adcbf7e06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringService-request)))
  "Returns md5sum for a message object of type 'StringService-request"
  "7229f1b0de733d7956dd646adcbf7e06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringService-request>)))
  "Returns full string definition for message of type '<StringService-request>"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringService-request)))
  "Returns full string definition for message of type 'StringService-request"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StringService-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude StringService-response.msg.html

(cl:defclass <StringService-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass StringService-response (<StringService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<StringService-response> is deprecated: use TTS_audio-srv:StringService-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <StringService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:result-val is deprecated.  Use TTS_audio-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringService-response>) ostream)
  "Serializes a message object of type '<StringService-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringService-response>) istream)
  "Deserializes a message object of type '<StringService-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringService-response>)))
  "Returns string type for a service object of type '<StringService-response>"
  "TTS_audio/StringServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringService-response)))
  "Returns string type for a service object of type 'StringService-response"
  "TTS_audio/StringServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringService-response>)))
  "Returns md5sum for a message object of type '<StringService-response>"
  "7229f1b0de733d7956dd646adcbf7e06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringService-response)))
  "Returns md5sum for a message object of type 'StringService-response"
  "7229f1b0de733d7956dd646adcbf7e06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringService-response>)))
  "Returns full string definition for message of type '<StringService-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringService-response)))
  "Returns full string definition for message of type 'StringService-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringService-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StringService-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StringService)))
  'StringService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StringService)))
  'StringService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringService)))
  "Returns string type for a service object of type '<StringService>"
  "TTS_audio/StringService")
; Auto-generated. Do not edit!


(cl:in-package abot_vlm-srv)


;//! \htmlinclude LLMQuery-request.msg.html

(cl:defclass <LLMQuery-request> (roslisp-msg-protocol:ros-message)
  ((query
    :reader query
    :initarg :query
    :type cl:string
    :initform ""))
)

(cl:defclass LLMQuery-request (<LLMQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LLMQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LLMQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name abot_vlm-srv:<LLMQuery-request> is deprecated: use abot_vlm-srv:LLMQuery-request instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <LLMQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader abot_vlm-srv:query-val is deprecated.  Use abot_vlm-srv:query instead.")
  (query m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LLMQuery-request>) ostream)
  "Serializes a message object of type '<LLMQuery-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'query))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'query))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LLMQuery-request>) istream)
  "Deserializes a message object of type '<LLMQuery-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'query) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'query) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LLMQuery-request>)))
  "Returns string type for a service object of type '<LLMQuery-request>"
  "abot_vlm/LLMQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMQuery-request)))
  "Returns string type for a service object of type 'LLMQuery-request"
  "abot_vlm/LLMQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LLMQuery-request>)))
  "Returns md5sum for a message object of type '<LLMQuery-request>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LLMQuery-request)))
  "Returns md5sum for a message object of type 'LLMQuery-request"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LLMQuery-request>)))
  "Returns full string definition for message of type '<LLMQuery-request>"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LLMQuery-request)))
  "Returns full string definition for message of type 'LLMQuery-request"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LLMQuery-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LLMQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LLMQuery-request
    (cl:cons ':query (query msg))
))
;//! \htmlinclude LLMQuery-response.msg.html

(cl:defclass <LLMQuery-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass LLMQuery-response (<LLMQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LLMQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LLMQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name abot_vlm-srv:<LLMQuery-response> is deprecated: use abot_vlm-srv:LLMQuery-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <LLMQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader abot_vlm-srv:result-val is deprecated.  Use abot_vlm-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LLMQuery-response>) ostream)
  "Serializes a message object of type '<LLMQuery-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LLMQuery-response>) istream)
  "Deserializes a message object of type '<LLMQuery-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LLMQuery-response>)))
  "Returns string type for a service object of type '<LLMQuery-response>"
  "abot_vlm/LLMQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMQuery-response)))
  "Returns string type for a service object of type 'LLMQuery-response"
  "abot_vlm/LLMQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LLMQuery-response>)))
  "Returns md5sum for a message object of type '<LLMQuery-response>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LLMQuery-response)))
  "Returns md5sum for a message object of type 'LLMQuery-response"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LLMQuery-response>)))
  "Returns full string definition for message of type '<LLMQuery-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LLMQuery-response)))
  "Returns full string definition for message of type 'LLMQuery-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LLMQuery-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LLMQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LLMQuery-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LLMQuery)))
  'LLMQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LLMQuery)))
  'LLMQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LLMQuery)))
  "Returns string type for a service object of type '<LLMQuery>"
  "abot_vlm/LLMQuery")
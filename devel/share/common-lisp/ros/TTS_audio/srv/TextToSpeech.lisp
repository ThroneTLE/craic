; Auto-generated. Do not edit!


(cl:in-package TTS_audio-srv)


;//! \htmlinclude TextToSpeech-request.msg.html

(cl:defclass <TextToSpeech-request> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass TextToSpeech-request (<TextToSpeech-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TextToSpeech-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TextToSpeech-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<TextToSpeech-request> is deprecated: use TTS_audio-srv:TextToSpeech-request instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <TextToSpeech-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:text-val is deprecated.  Use TTS_audio-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TextToSpeech-request>) ostream)
  "Serializes a message object of type '<TextToSpeech-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TextToSpeech-request>) istream)
  "Deserializes a message object of type '<TextToSpeech-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TextToSpeech-request>)))
  "Returns string type for a service object of type '<TextToSpeech-request>"
  "TTS_audio/TextToSpeechRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TextToSpeech-request)))
  "Returns string type for a service object of type 'TextToSpeech-request"
  "TTS_audio/TextToSpeechRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TextToSpeech-request>)))
  "Returns md5sum for a message object of type '<TextToSpeech-request>"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TextToSpeech-request)))
  "Returns md5sum for a message object of type 'TextToSpeech-request"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TextToSpeech-request>)))
  "Returns full string definition for message of type '<TextToSpeech-request>"
  (cl:format cl:nil "string text  # 输入的文本~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TextToSpeech-request)))
  "Returns full string definition for message of type 'TextToSpeech-request"
  (cl:format cl:nil "string text  # 输入的文本~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TextToSpeech-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TextToSpeech-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TextToSpeech-request
    (cl:cons ':text (text msg))
))
;//! \htmlinclude TextToSpeech-response.msg.html

(cl:defclass <TextToSpeech-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass TextToSpeech-response (<TextToSpeech-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TextToSpeech-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TextToSpeech-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<TextToSpeech-response> is deprecated: use TTS_audio-srv:TextToSpeech-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TextToSpeech-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:success-val is deprecated.  Use TTS_audio-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TextToSpeech-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:message-val is deprecated.  Use TTS_audio-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TextToSpeech-response>) ostream)
  "Serializes a message object of type '<TextToSpeech-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TextToSpeech-response>) istream)
  "Deserializes a message object of type '<TextToSpeech-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TextToSpeech-response>)))
  "Returns string type for a service object of type '<TextToSpeech-response>"
  "TTS_audio/TextToSpeechResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TextToSpeech-response)))
  "Returns string type for a service object of type 'TextToSpeech-response"
  "TTS_audio/TextToSpeechResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TextToSpeech-response>)))
  "Returns md5sum for a message object of type '<TextToSpeech-response>"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TextToSpeech-response)))
  "Returns md5sum for a message object of type 'TextToSpeech-response"
  "191f0a61ba3a929ce6cac14343f74651")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TextToSpeech-response>)))
  "Returns full string definition for message of type '<TextToSpeech-response>"
  (cl:format cl:nil "bool success  # 服务是否成功~%string message  # 返回的消息~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TextToSpeech-response)))
  "Returns full string definition for message of type 'TextToSpeech-response"
  (cl:format cl:nil "bool success  # 服务是否成功~%string message  # 返回的消息~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TextToSpeech-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TextToSpeech-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TextToSpeech-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TextToSpeech)))
  'TextToSpeech-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TextToSpeech)))
  'TextToSpeech-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TextToSpeech)))
  "Returns string type for a service object of type '<TextToSpeech>"
  "TTS_audio/TextToSpeech")
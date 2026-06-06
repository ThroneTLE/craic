; Auto-generated. Do not edit!


(cl:in-package TTS_audio-srv)


;//! \htmlinclude TTS-request.msg.html

(cl:defclass <TTS-request> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass TTS-request (<TTS-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TTS-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TTS-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<TTS-request> is deprecated: use TTS_audio-srv:TTS-request instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <TTS-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:text-val is deprecated.  Use TTS_audio-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TTS-request>) ostream)
  "Serializes a message object of type '<TTS-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TTS-request>) istream)
  "Deserializes a message object of type '<TTS-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TTS-request>)))
  "Returns string type for a service object of type '<TTS-request>"
  "TTS_audio/TTSRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTS-request)))
  "Returns string type for a service object of type 'TTS-request"
  "TTS_audio/TTSRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TTS-request>)))
  "Returns md5sum for a message object of type '<TTS-request>"
  "4027c3e5e88791dcb539699a421751c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TTS-request)))
  "Returns md5sum for a message object of type 'TTS-request"
  "4027c3e5e88791dcb539699a421751c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TTS-request>)))
  "Returns full string definition for message of type '<TTS-request>"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TTS-request)))
  "Returns full string definition for message of type 'TTS-request"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TTS-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TTS-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TTS-request
    (cl:cons ':text (text msg))
))
;//! \htmlinclude TTS-response.msg.html

(cl:defclass <TTS-response> (roslisp-msg-protocol:ros-message)
  ((audio_file_path
    :reader audio_file_path
    :initarg :audio_file_path
    :type cl:string
    :initform ""))
)

(cl:defclass TTS-response (<TTS-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TTS-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TTS-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TTS_audio-srv:<TTS-response> is deprecated: use TTS_audio-srv:TTS-response instead.")))

(cl:ensure-generic-function 'audio_file_path-val :lambda-list '(m))
(cl:defmethod audio_file_path-val ((m <TTS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TTS_audio-srv:audio_file_path-val is deprecated.  Use TTS_audio-srv:audio_file_path instead.")
  (audio_file_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TTS-response>) ostream)
  "Serializes a message object of type '<TTS-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'audio_file_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'audio_file_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TTS-response>) istream)
  "Deserializes a message object of type '<TTS-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'audio_file_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'audio_file_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TTS-response>)))
  "Returns string type for a service object of type '<TTS-response>"
  "TTS_audio/TTSResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTS-response)))
  "Returns string type for a service object of type 'TTS-response"
  "TTS_audio/TTSResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TTS-response>)))
  "Returns md5sum for a message object of type '<TTS-response>"
  "4027c3e5e88791dcb539699a421751c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TTS-response)))
  "Returns md5sum for a message object of type 'TTS-response"
  "4027c3e5e88791dcb539699a421751c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TTS-response>)))
  "Returns full string definition for message of type '<TTS-response>"
  (cl:format cl:nil "string audio_file_path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TTS-response)))
  "Returns full string definition for message of type 'TTS-response"
  (cl:format cl:nil "string audio_file_path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TTS-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'audio_file_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TTS-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TTS-response
    (cl:cons ':audio_file_path (audio_file_path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TTS)))
  'TTS-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TTS)))
  'TTS-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTS)))
  "Returns string type for a service object of type '<TTS>"
  "TTS_audio/TTS")
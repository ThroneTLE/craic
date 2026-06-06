; Auto-generated. Do not edit!


(cl:in-package robot_slam-msg)


;//! \htmlinclude NavCmd.msg.html

(cl:defclass <NavCmd> (roslisp-msg-protocol:ros-message)
  ((start_nav
    :reader start_nav
    :initarg :start_nav
    :type cl:boolean
    :initform cl:nil)
   (target_x
    :reader target_x
    :initarg :target_x
    :type cl:float
    :initform 0.0)
   (target_y
    :reader target_y
    :initarg :target_y
    :type cl:float
    :initform 0.0)
   (target_yaw
    :reader target_yaw
    :initarg :target_yaw
    :type cl:float
    :initform 0.0)
   (robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform ""))
)

(cl:defclass NavCmd (<NavCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_slam-msg:<NavCmd> is deprecated: use robot_slam-msg:NavCmd instead.")))

(cl:ensure-generic-function 'start_nav-val :lambda-list '(m))
(cl:defmethod start_nav-val ((m <NavCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_slam-msg:start_nav-val is deprecated.  Use robot_slam-msg:start_nav instead.")
  (start_nav m))

(cl:ensure-generic-function 'target_x-val :lambda-list '(m))
(cl:defmethod target_x-val ((m <NavCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_slam-msg:target_x-val is deprecated.  Use robot_slam-msg:target_x instead.")
  (target_x m))

(cl:ensure-generic-function 'target_y-val :lambda-list '(m))
(cl:defmethod target_y-val ((m <NavCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_slam-msg:target_y-val is deprecated.  Use robot_slam-msg:target_y instead.")
  (target_y m))

(cl:ensure-generic-function 'target_yaw-val :lambda-list '(m))
(cl:defmethod target_yaw-val ((m <NavCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_slam-msg:target_yaw-val is deprecated.  Use robot_slam-msg:target_yaw instead.")
  (target_yaw m))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <NavCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_slam-msg:robot_id-val is deprecated.  Use robot_slam-msg:robot_id instead.")
  (robot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavCmd>) ostream)
  "Serializes a message object of type '<NavCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_nav) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavCmd>) istream)
  "Deserializes a message object of type '<NavCmd>"
    (cl:setf (cl:slot-value msg 'start_nav) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavCmd>)))
  "Returns string type for a message object of type '<NavCmd>"
  "robot_slam/NavCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavCmd)))
  "Returns string type for a message object of type 'NavCmd"
  "robot_slam/NavCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavCmd>)))
  "Returns md5sum for a message object of type '<NavCmd>"
  "49a73690f439fc687c4e207d9765b972")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavCmd)))
  "Returns md5sum for a message object of type 'NavCmd"
  "49a73690f439fc687c4e207d9765b972")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavCmd>)))
  "Returns full string definition for message of type '<NavCmd>"
  (cl:format cl:nil "# NavCmd.msg - 导航指令消息~%bool start_nav       # 是否启动导航（true=启动，false=停止）~%float64 target_x     # 目标点 x 坐标（map 坐标系）~%float64 target_y     # 目标点 y 坐标（map 坐标系）~%float64 target_yaw   # 目标点偏航角（弧度）~%string robot_id      # 目标机器人 ID（区分发给 A/B，可选）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavCmd)))
  "Returns full string definition for message of type 'NavCmd"
  (cl:format cl:nil "# NavCmd.msg - 导航指令消息~%bool start_nav       # 是否启动导航（true=启动，false=停止）~%float64 target_x     # 目标点 x 坐标（map 坐标系）~%float64 target_y     # 目标点 y 坐标（map 坐标系）~%float64 target_yaw   # 目标点偏航角（弧度）~%string robot_id      # 目标机器人 ID（区分发给 A/B，可选）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavCmd>))
  (cl:+ 0
     1
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'robot_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'NavCmd
    (cl:cons ':start_nav (start_nav msg))
    (cl:cons ':target_x (target_x msg))
    (cl:cons ':target_y (target_y msg))
    (cl:cons ':target_yaw (target_yaw msg))
    (cl:cons ':robot_id (robot_id msg))
))

;; Auto-generated. Do not edit!


(when (boundp 'nav_command::NavCmd)
  (if (not (find-package "NAV_COMMAND"))
    (make-package "NAV_COMMAND"))
  (shadow 'NavCmd (find-package "NAV_COMMAND")))
(unless (find-package "NAV_COMMAND::NAVCMD")
  (make-package "NAV_COMMAND::NAVCMD"))

(in-package "ROS")
;;//! \htmlinclude NavCmd.msg.html


(defclass nav_command::NavCmd
  :super ros::object
  :slots (_start_nav _target_x _target_y _target_yaw _robot_id ))

(defmethod nav_command::NavCmd
  (:init
   (&key
    ((:start_nav __start_nav) nil)
    ((:target_x __target_x) 0.0)
    ((:target_y __target_y) 0.0)
    ((:target_yaw __target_yaw) 0.0)
    ((:robot_id __robot_id) "")
    )
   (send-super :init)
   (setq _start_nav __start_nav)
   (setq _target_x (float __target_x))
   (setq _target_y (float __target_y))
   (setq _target_yaw (float __target_yaw))
   (setq _robot_id (string __robot_id))
   self)
  (:start_nav
   (&optional __start_nav)
   (if __start_nav (setq _start_nav __start_nav)) _start_nav)
  (:target_x
   (&optional __target_x)
   (if __target_x (setq _target_x __target_x)) _target_x)
  (:target_y
   (&optional __target_y)
   (if __target_y (setq _target_y __target_y)) _target_y)
  (:target_yaw
   (&optional __target_yaw)
   (if __target_yaw (setq _target_yaw __target_yaw)) _target_yaw)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:serialization-length
   ()
   (+
    ;; bool _start_nav
    1
    ;; float64 _target_x
    8
    ;; float64 _target_y
    8
    ;; float64 _target_yaw
    8
    ;; string _robot_id
    4 (length _robot_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start_nav
       (if _start_nav (write-byte -1 s) (write-byte 0 s))
     ;; float64 _target_x
       (sys::poke _target_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_y
       (sys::poke _target_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_yaw
       (sys::poke _target_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _robot_id
       (write-long (length _robot_id) s) (princ _robot_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start_nav
     (setq _start_nav (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _target_x
     (setq _target_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_y
     (setq _target_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_yaw
     (setq _target_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _robot_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get nav_command::NavCmd :md5sum-) "49a73690f439fc687c4e207d9765b972")
(setf (get nav_command::NavCmd :datatype-) "nav_command/NavCmd")
(setf (get nav_command::NavCmd :definition-)
      "# 导航指令消息
bool start_nav       # 是否启动导航（true=启动，false=停止）
float64 target_x     # 目标点 x 坐标（map 坐标系）
float64 target_y     # 目标点 y 坐标（map 坐标系）
float64 target_yaw   # 目标点偏航角（弧度）
string robot_id      # 目标机器人 ID（区分发给 A/B）

")



(provide :nav_command/NavCmd "49a73690f439fc687c4e207d9765b972")



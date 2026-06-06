
(cl:in-package :asdf)

(defsystem "robot_slam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NavCmd" :depends-on ("_package_NavCmd"))
    (:file "_package_NavCmd" :depends-on ("_package"))
  ))
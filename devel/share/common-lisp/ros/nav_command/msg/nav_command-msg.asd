
(cl:in-package :asdf)

(defsystem "nav_command-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NavCmd" :depends-on ("_package_NavCmd"))
    (:file "_package_NavCmd" :depends-on ("_package"))
  ))
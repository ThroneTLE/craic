
(cl:in-package :asdf)

(defsystem "abot_vlm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LLMQuery" :depends-on ("_package_LLMQuery"))
    (:file "_package_LLMQuery" :depends-on ("_package"))
    (:file "VisionResult" :depends-on ("_package_VisionResult"))
    (:file "_package_VisionResult" :depends-on ("_package"))
  ))
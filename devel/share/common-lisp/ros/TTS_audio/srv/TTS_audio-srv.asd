
(cl:in-package :asdf)

(defsystem "TTS_audio-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StringService" :depends-on ("_package_StringService"))
    (:file "_package_StringService" :depends-on ("_package"))
    (:file "TTS" :depends-on ("_package_TTS"))
    (:file "_package_TTS" :depends-on ("_package"))
    (:file "TextToSpeech" :depends-on ("_package_TextToSpeech"))
    (:file "_package_TextToSpeech" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'TTS_audio::TextToSpeech)
  (if (not (find-package "TTS_AUDIO"))
    (make-package "TTS_AUDIO"))
  (shadow 'TextToSpeech (find-package "TTS_AUDIO")))
(unless (find-package "TTS_AUDIO::TEXTTOSPEECH")
  (make-package "TTS_AUDIO::TEXTTOSPEECH"))
(unless (find-package "TTS_AUDIO::TEXTTOSPEECHREQUEST")
  (make-package "TTS_AUDIO::TEXTTOSPEECHREQUEST"))
(unless (find-package "TTS_AUDIO::TEXTTOSPEECHRESPONSE")
  (make-package "TTS_AUDIO::TEXTTOSPEECHRESPONSE"))

(in-package "ROS")





(defclass TTS_audio::TextToSpeechRequest
  :super ros::object
  :slots (_text ))

(defmethod TTS_audio::TextToSpeechRequest
  (:init
   (&key
    ((:text __text) "")
    )
   (send-super :init)
   (setq _text (string __text))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass TTS_audio::TextToSpeechResponse
  :super ros::object
  :slots (_success _message ))

(defmethod TTS_audio::TextToSpeechResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass TTS_audio::TextToSpeech
  :super ros::object
  :slots ())

(setf (get TTS_audio::TextToSpeech :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get TTS_audio::TextToSpeech :datatype-) "TTS_audio/TextToSpeech")
(setf (get TTS_audio::TextToSpeech :request) TTS_audio::TextToSpeechRequest)
(setf (get TTS_audio::TextToSpeech :response) TTS_audio::TextToSpeechResponse)

(defmethod TTS_audio::TextToSpeechRequest
  (:response () (instance TTS_audio::TextToSpeechResponse :init)))

(setf (get TTS_audio::TextToSpeechRequest :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get TTS_audio::TextToSpeechRequest :datatype-) "TTS_audio/TextToSpeechRequest")
(setf (get TTS_audio::TextToSpeechRequest :definition-)
      "string text  # 输入的文本
---
bool success  # 服务是否成功
string message  # 返回的消息

")

(setf (get TTS_audio::TextToSpeechResponse :md5sum-) "191f0a61ba3a929ce6cac14343f74651")
(setf (get TTS_audio::TextToSpeechResponse :datatype-) "TTS_audio/TextToSpeechResponse")
(setf (get TTS_audio::TextToSpeechResponse :definition-)
      "string text  # 输入的文本
---
bool success  # 服务是否成功
string message  # 返回的消息

")



(provide :TTS_audio/TextToSpeech "191f0a61ba3a929ce6cac14343f74651")



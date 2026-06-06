;; Auto-generated. Do not edit!


(when (boundp 'TTS_audio::TTS)
  (if (not (find-package "TTS_AUDIO"))
    (make-package "TTS_AUDIO"))
  (shadow 'TTS (find-package "TTS_AUDIO")))
(unless (find-package "TTS_AUDIO::TTS")
  (make-package "TTS_AUDIO::TTS"))
(unless (find-package "TTS_AUDIO::TTSREQUEST")
  (make-package "TTS_AUDIO::TTSREQUEST"))
(unless (find-package "TTS_AUDIO::TTSRESPONSE")
  (make-package "TTS_AUDIO::TTSRESPONSE"))

(in-package "ROS")





(defclass TTS_audio::TTSRequest
  :super ros::object
  :slots (_text ))

(defmethod TTS_audio::TTSRequest
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

(defclass TTS_audio::TTSResponse
  :super ros::object
  :slots (_audio_file_path ))

(defmethod TTS_audio::TTSResponse
  (:init
   (&key
    ((:audio_file_path __audio_file_path) "")
    )
   (send-super :init)
   (setq _audio_file_path (string __audio_file_path))
   self)
  (:audio_file_path
   (&optional __audio_file_path)
   (if __audio_file_path (setq _audio_file_path __audio_file_path)) _audio_file_path)
  (:serialization-length
   ()
   (+
    ;; string _audio_file_path
    4 (length _audio_file_path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _audio_file_path
       (write-long (length _audio_file_path) s) (princ _audio_file_path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _audio_file_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _audio_file_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass TTS_audio::TTS
  :super ros::object
  :slots ())

(setf (get TTS_audio::TTS :md5sum-) "4027c3e5e88791dcb539699a421751c8")
(setf (get TTS_audio::TTS :datatype-) "TTS_audio/TTS")
(setf (get TTS_audio::TTS :request) TTS_audio::TTSRequest)
(setf (get TTS_audio::TTS :response) TTS_audio::TTSResponse)

(defmethod TTS_audio::TTSRequest
  (:response () (instance TTS_audio::TTSResponse :init)))

(setf (get TTS_audio::TTSRequest :md5sum-) "4027c3e5e88791dcb539699a421751c8")
(setf (get TTS_audio::TTSRequest :datatype-) "TTS_audio/TTSRequest")
(setf (get TTS_audio::TTSRequest :definition-)
      "string text
---
string audio_file_path

")

(setf (get TTS_audio::TTSResponse :md5sum-) "4027c3e5e88791dcb539699a421751c8")
(setf (get TTS_audio::TTSResponse :datatype-) "TTS_audio/TTSResponse")
(setf (get TTS_audio::TTSResponse :definition-)
      "string text
---
string audio_file_path

")



(provide :TTS_audio/TTS "4027c3e5e88791dcb539699a421751c8")



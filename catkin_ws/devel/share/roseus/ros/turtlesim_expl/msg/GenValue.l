;; Auto-generated. Do not edit!


(when (boundp 'turtlesim_expl::GenValue)
  (if (not (find-package "TURTLESIM_EXPL"))
    (make-package "TURTLESIM_EXPL"))
  (shadow 'GenValue (find-package "TURTLESIM_EXPL")))
(unless (find-package "TURTLESIM_EXPL::GENVALUE")
  (make-package "TURTLESIM_EXPL::GENVALUE"))

(in-package "ROS")
;;//! \htmlinclude GenValue.msg.html


(defclass turtlesim_expl::GenValue
  :super ros::object
  :slots (_value _intrusion ))

(defmethod turtlesim_expl::GenValue
  (:init
   (&key
    ((:value __value) 0)
    ((:intrusion __intrusion) "")
    )
   (send-super :init)
   (setq _value (round __value))
   (setq _intrusion (string __intrusion))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:intrusion
   (&optional __intrusion)
   (if __intrusion (setq _intrusion __intrusion)) _intrusion)
  (:serialization-length
   ()
   (+
    ;; uint8 _value
    1
    ;; string _intrusion
    4 (length _intrusion)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _value
       (write-byte _value s)
     ;; string _intrusion
       (write-long (length _intrusion) s) (princ _intrusion s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _intrusion
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _intrusion (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get turtlesim_expl::GenValue :md5sum-) "fff21b1ce4d892d68e646aa7a0d9dac4")
(setf (get turtlesim_expl::GenValue :datatype-) "turtlesim_expl/GenValue")
(setf (get turtlesim_expl::GenValue :definition-)
      "uint8 value
string intrusion
")



(provide :turtlesim_expl/GenValue "fff21b1ce4d892d68e646aa7a0d9dac4")



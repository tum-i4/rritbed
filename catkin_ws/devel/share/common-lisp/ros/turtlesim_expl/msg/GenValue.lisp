; Auto-generated. Do not edit!


(cl:in-package turtlesim_expl-msg)


;//! \htmlinclude GenValue.msg.html

(cl:defclass <GenValue> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0)
   (intrusion
    :reader intrusion
    :initarg :intrusion
    :type cl:string
    :initform ""))
)

(cl:defclass GenValue (<GenValue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenValue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenValue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtlesim_expl-msg:<GenValue> is deprecated: use turtlesim_expl-msg:GenValue instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GenValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlesim_expl-msg:value-val is deprecated.  Use turtlesim_expl-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'intrusion-val :lambda-list '(m))
(cl:defmethod intrusion-val ((m <GenValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlesim_expl-msg:intrusion-val is deprecated.  Use turtlesim_expl-msg:intrusion instead.")
  (intrusion m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenValue>) ostream)
  "Serializes a message object of type '<GenValue>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'intrusion))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'intrusion))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenValue>) istream)
  "Deserializes a message object of type '<GenValue>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'intrusion) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'intrusion) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenValue>)))
  "Returns string type for a message object of type '<GenValue>"
  "turtlesim_expl/GenValue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenValue)))
  "Returns string type for a message object of type 'GenValue"
  "turtlesim_expl/GenValue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenValue>)))
  "Returns md5sum for a message object of type '<GenValue>"
  "fff21b1ce4d892d68e646aa7a0d9dac4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenValue)))
  "Returns md5sum for a message object of type 'GenValue"
  "fff21b1ce4d892d68e646aa7a0d9dac4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenValue>)))
  "Returns full string definition for message of type '<GenValue>"
  (cl:format cl:nil "uint8 value~%string intrusion~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenValue)))
  "Returns full string definition for message of type 'GenValue"
  (cl:format cl:nil "uint8 value~%string intrusion~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenValue>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'intrusion))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenValue>))
  "Converts a ROS message object to a list"
  (cl:list 'GenValue
    (cl:cons ':value (value msg))
    (cl:cons ':intrusion (intrusion msg))
))

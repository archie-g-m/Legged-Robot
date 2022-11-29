; Auto-generated. Do not edit!


(cl:in-package legged_robot-msg)


;//! \htmlinclude walk_msg.msg.html

(cl:defclass <walk_msg> (roslisp-msg-protocol:ros-message)
  ((heading
    :reader heading
    :initarg :heading
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0))
)

(cl:defclass walk_msg (<walk_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <walk_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'walk_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name legged_robot-msg:<walk_msg> is deprecated: use legged_robot-msg:walk_msg instead.")))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <walk_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader legged_robot-msg:heading-val is deprecated.  Use legged_robot-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <walk_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader legged_robot-msg:speed-val is deprecated.  Use legged_robot-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <walk_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader legged_robot-msg:time-val is deprecated.  Use legged_robot-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <walk_msg>) ostream)
  "Serializes a message object of type '<walk_msg>"
  (cl:let* ((signed (cl:slot-value msg 'heading)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <walk_msg>) istream)
  "Deserializes a message object of type '<walk_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'heading) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<walk_msg>)))
  "Returns string type for a message object of type '<walk_msg>"
  "legged_robot/walk_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'walk_msg)))
  "Returns string type for a message object of type 'walk_msg"
  "legged_robot/walk_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<walk_msg>)))
  "Returns md5sum for a message object of type '<walk_msg>"
  "68f7308a667b38e6bfb90d635c4c4f55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'walk_msg)))
  "Returns md5sum for a message object of type 'walk_msg"
  "68f7308a667b38e6bfb90d635c4c4f55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<walk_msg>)))
  "Returns full string definition for message of type '<walk_msg>"
  (cl:format cl:nil "int32 heading~%int32 speed~%int32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'walk_msg)))
  "Returns full string definition for message of type 'walk_msg"
  (cl:format cl:nil "int32 heading~%int32 speed~%int32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <walk_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <walk_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'walk_msg
    (cl:cons ':heading (heading msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':time (time msg))
))

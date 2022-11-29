;; Auto-generated. Do not edit!


(when (boundp 'legged_robot::walk_msg)
  (if (not (find-package "LEGGED_ROBOT"))
    (make-package "LEGGED_ROBOT"))
  (shadow 'walk_msg (find-package "LEGGED_ROBOT")))
(unless (find-package "LEGGED_ROBOT::WALK_MSG")
  (make-package "LEGGED_ROBOT::WALK_MSG"))

(in-package "ROS")
;;//! \htmlinclude walk_msg.msg.html


(defclass legged_robot::walk_msg
  :super ros::object
  :slots (_heading _speed _time ))

(defmethod legged_robot::walk_msg
  (:init
   (&key
    ((:heading __heading) 0)
    ((:speed __speed) 0)
    ((:time __time) 0)
    )
   (send-super :init)
   (setq _heading (round __heading))
   (setq _speed (round __speed))
   (setq _time (round __time))
   self)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; int32 _heading
    4
    ;; int32 _speed
    4
    ;; int32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _heading
       (write-long _heading s)
     ;; int32 _speed
       (write-long _speed s)
     ;; int32 _time
       (write-long _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _heading
     (setq _heading (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _time
     (setq _time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get legged_robot::walk_msg :md5sum-) "68f7308a667b38e6bfb90d635c4c4f55")
(setf (get legged_robot::walk_msg :datatype-) "legged_robot/walk_msg")
(setf (get legged_robot::walk_msg :definition-)
      "int32 heading
int32 speed
int32 time
")



(provide :legged_robot/walk_msg "68f7308a667b38e6bfb90d635c4c4f55")


